#include "LongAVController.h"
#include "LongitudinallyAutonomousVehicle.h"

LongAVController::LongAVController(
	const LongitudinallyAutonomousVehicle& longitudinal_av, bool verbose)
	: VehicleController(longitudinal_av, verbose), 
	longitudinal_av{ &longitudinal_av }
{
	add_vissim_controller();
	add_origin_lane_controllers();
}

double LongAVController::get_vissim_desired_acceleration(
	const EgoVehicle& ego_vehicle)
{
	/* We need to ensure the velocity filter keeps active
	while VISSIM has control of the car to guarantee a smooth
	movement when taking back control */
	if (ego_vehicle.has_leader())
	{
		origin_lane_controller.update_leader_velocity_filter(
			ego_vehicle.get_leader()->compute_velocity(
				ego_vehicle.get_velocity()));
	}

	active_longitudinal_controller_type = ALCType::vissim;
	return vissim_controller.compute_desired_acceleration(
		ego_vehicle, nullptr, 0);
}

bool LongAVController::get_origin_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	if (verbose) std::clog << "Origin lane controller" << std::endl;
	possible_accelerations[ALCType::origin_lane] =
		origin_lane_controller.compute_desired_acceleration(
			*longitudinal_av, longitudinal_av->get_leader().get(),
			longitudinal_av->get_desired_velocity());

	return true;
}

bool LongAVController::get_end_of_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;
	const EgoVehicle& ego_vehicle = *longitudinal_av;

	/* When the lane change direction equals the preferred relative
	lane, it means the vehicle is moving into the destination lane.
	At this point, we don't want to use this controller anymore. */
	bool is_lane_changing = ego_vehicle.get_preferred_relative_lane()
		== ego_vehicle.get_active_lane_change_direction();
	bool is_about_to_start_lane_change =
		ego_vehicle.get_preferred_relative_lane()
		== ego_vehicle.get_lane_change_direction();
	if (!is_lane_changing && !is_about_to_start_lane_change
		&& (ego_vehicle.get_lane_end_distance() > -1))
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
			create_virtual_stopped_vehicle();

		SwitchedLongitudinalController::State old_state =
			end_of_lane_controller.get_state();
		/* To avoid sudden high decelerations */
		if (old_state
			== SwitchedLongitudinalController::State::uninitialized)
		{
			end_of_lane_controller.reset_leader_velocity_filter(
				ego_vehicle.get_velocity());
		}
		double desired_acceleration =
			end_of_lane_controller.compute_desired_acceleration(
				ego_vehicle, &virtual_vehicle,
				ego_vehicle.get_desired_velocity());

		/* This controller is only active when it's at vehicle
		following mode */
		if (end_of_lane_controller.get_state()
			== SwitchedLongitudinalController::State::vehicle_following)
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

double LongAVController::implement_compute_desired_acceleration()
{
	if (longitudinal_av->has_lane_change_intention() ||
		longitudinal_av->is_lane_changing())
	{
		return get_vissim_desired_acceleration(*longitudinal_av);
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void LongAVController::add_vissim_controller()
{
	vissim_controller = VissimLongitudinalController(
		vissim_colors);
}

void LongAVController::add_origin_lane_controllers()
{
	if (verbose) std::clog << "Creating origin lane controllers."
		<< std::endl;
	origin_lane_controller = RealLongitudinalController(
		*longitudinal_av,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		orig_lane_colors, long_controllers_verbose);
	/* Note: the end of lane controller could have special vel control gains
	but we want to see how the ego vehicle responds to a stopped vehicle. */
	end_of_lane_controller = RealLongitudinalController(
		*longitudinal_av,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		end_of_lane_colors, long_controllers_verbose);
	/* The end of the lane is seen as a stopped vehicle. We set some
	time headway to that "vehicle". */
	activate_end_of_lane_controller(time_headway_to_end_of_lane);
}

void LongAVController::activate_origin_lane_controller(
	double time_headway, bool is_leader_connected)
{
	origin_lane_controller.reset_leader_velocity_filter(
		longitudinal_av->get_velocity());
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	origin_lane_controller.reset_time_headway_filter(time_headway);
	update_origin_lane_controller(time_headway, is_leader_connected);
}

NearbyVehicle LongAVController::create_virtual_stopped_vehicle()
{
	const EgoVehicle& ego_vehicle = *longitudinal_av;

	NearbyVehicle virtual_vehicle = NearbyVehicle(1, RelativeLane::same, 1);
	virtual_vehicle.set_relative_velocity(
		ego_vehicle.get_velocity());
	virtual_vehicle.set_distance(
		ego_vehicle.get_lane_end_distance());
	virtual_vehicle.set_length(0.0);
	return virtual_vehicle;
}