#include "ACCVehicle.h"
#include "ACCVehicleController.h"

ACCVehicleController::ACCVehicleController(const ACCVehicle* acc_vehicle,
	bool verbose) : VehicleController(acc_vehicle, verbose),
	acc_vehicle{acc_vehicle} {}

void ACCVehicleController::add_vissim_controller()
{
	vissim_controller = VissimLongitudinalController(acc_vehicle,
		vissim_colors);
	available_controllers[ALCType::vissim] = &vissim_controller;
}

void ACCVehicleController::add_origin_lane_controllers()
{
	if (verbose) std::clog << "Creating origin lane controllers.\n";

	origin_lane_controller = std::make_unique<RealLongitudinalController>(
		acc_vehicle, orig_lane_colors, long_controllers_verbose);
	origin_lane_controller->create_velocity_controller(
		desired_velocity_controller_gains, velocity_filter_gain);
	origin_lane_controller->create_gap_controller(
		autonomous_real_following_gains, connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain);

	/* Note: the end of lane controller could have special vel control gains
	but we want to see how the ego vehicle responds to a stopped vehicle. */
	end_of_lane_controller = RealLongitudinalController(
		acc_vehicle, end_of_lane_colors, long_controllers_verbose);
	end_of_lane_controller.create_velocity_controller(
		desired_velocity_controller_gains, velocity_filter_gain);
	end_of_lane_controller.create_gap_controller(
		autonomous_real_following_gains, connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain);
	/* The end of the lane is seen as a stopped vehicle. We set some
	time headway to that "vehicle". */
	activate_end_of_lane_controller(time_headway_to_end_of_lane);

	available_controllers[ALCType::origin_lane] = origin_lane_controller.get();
	available_controllers[ALCType::end_of_lane] = &end_of_lane_controller;
}

void ACCVehicleController::activate_end_of_lane_controller(double time_headway)
{
	end_of_lane_controller.reset_time_headway_filter(time_headway);
	end_of_lane_controller.set_desired_time_headway(time_headway);
}

double ACCVehicleController::get_vissim_desired_acceleration()
{
	/* We need to ensure the velocity filter keeps active
	while VISSIM has control of the car to guarantee a smooth
	movement when taking back control */
	if (acc_vehicle->has_leader())
	{
		origin_lane_controller->update_leader_velocity_filter(
			acc_vehicle->get_leader()->compute_velocity(
				acc_vehicle->get_velocity()));
	}

	active_longitudinal_controller_type = ALCType::vissim;
	return vissim_controller.compute_desired_acceleration(
		nullptr, 0);
}

bool ACCVehicleController::get_origin_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	if (verbose) std::clog << "Origin lane controller" << std::endl;
	possible_accelerations[ALCType::origin_lane] =
		origin_lane_controller->compute_desired_acceleration(
			acc_vehicle->get_leader(), acc_vehicle->get_desired_velocity());

	return true;
}

bool ACCVehicleController::get_end_of_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	/* When the lane change direction equals the preferred relative
	lane, it means the vehicle is moving into the destination lane.
	At this point, we don't want to use this controller anymore. */
	bool is_lane_changing = acc_vehicle->get_preferred_relative_lane()
		== acc_vehicle->get_active_lane_change_direction();
	bool is_about_to_start_lane_change =
		acc_vehicle->get_preferred_relative_lane()
		== acc_vehicle->get_lane_change_direction();
	if (!is_lane_changing && !is_about_to_start_lane_change
		&& (acc_vehicle->get_lane_end_distance() > -1))
		// -1 because end of lane dist can get small negative values
	{
		if (verbose)
		{
			std::clog << "End of lane controller"
				<< std::endl;
		}
		/* We simulate a stopped vehicle at the end of
		the lane to force the vehicle to stop before the end of
		the lane. */
		NearbyVehicle virtual_vehicle =
			create_virtual_stopped_vehicle(*acc_vehicle);

		LongitudinalController::State old_state =
			end_of_lane_controller.get_state();
		/* To avoid sudden high decelerations */
		if (old_state
			== LongitudinalController::State::uninitialized)
		{
			end_of_lane_controller.reset_leader_velocity_filter(
				acc_vehicle->get_velocity());
		}
		double desired_acceleration =
			end_of_lane_controller.compute_desired_acceleration(
				&virtual_vehicle, acc_vehicle->get_desired_velocity());

		/* This controller is only active when it's at vehicle
		following mode */
		if (end_of_lane_controller.get_state()
			== LongitudinalController::State::vehicle_following)
		{
			if (verbose)
			{
				std::clog << "active EOL controller. Old state is "
					<< LongitudinalController::state_to_string(old_state)
					<< std::endl;
			}
			possible_accelerations[ALCType::end_of_lane] =
				desired_acceleration;
			is_active = true;
		}
	}
	return is_active;
}

double ACCVehicleController::get_desired_time_headway_gap_to_leader() const
{
	return origin_lane_controller->get_desired_time_headway_gap();
}

void ACCVehicleController::implement_add_internal_controllers()
{
	if (verbose) std::clog << "Adding ACC vehicle controllers\n";

	add_vissim_controller();
	add_origin_lane_controllers();
}

double ACCVehicleController::implement_get_desired_acceleration()
{
	if (acc_vehicle->has_lane_change_intention() ||
		acc_vehicle->is_lane_changing())
	{
		return get_vissim_desired_acceleration();
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void ACCVehicleController::implement_update_origin_lane_controller(
	const NearbyVehicle& real_leader)
{
	origin_lane_controller_time_headway =
		origin_lane_controller->get_current_time_headway();
	double safe_h = acc_vehicle->compute_current_desired_time_headway(
		real_leader);
	double current_h = origin_lane_controller->get_current_time_headway();
	double comf_h = find_comfortable_time_headway(real_leader,
		origin_lane_controller->get_standstill_distance());
	double new_h = std::max(current_h, std::min(comf_h, safe_h));

	if (verbose)
	{
		std::clog << "Resetting orig lane ctrl (real leader) h_r = "
			<< new_h << " and setting desired value to "
			<< safe_h << std::endl;
	}

	origin_lane_controller->reset_time_headway_filter(new_h);
	origin_lane_controller->set_desired_time_headway(safe_h);
	origin_lane_controller->connect_gap_controller(
		real_leader.is_connected());
}

double ACCVehicleController::implement_get_desired_time_headway_gap(
	const NearbyVehicle& nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	if (nearby_vehicle.is_ahead()
		&& nearby_vehicle.get_relative_lane() == RelativeLane::same)
	{
		time_headway_gap =
			get_desired_time_headway_gap_to_leader();
	}
	else
	{
		time_headway_gap = -1.0;
	}
	return time_headway_gap;
}
