#include "AVController.h"
#include "AutonomousVehicle.h"

AVController::AVController(const AutonomousVehicle& av, bool verbose)
	: LongAVController(av, verbose), autonomous_vehicle{ &av }
{
	add_lane_change_adjustment_controller();
}

bool AVController::get_destination_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;
	//double origin_lane_reference_velocity;

	bool end_of_lane_controller_is_active =
		end_of_lane_controller.get_state()
		== SwitchedLongitudinalController::State::vehicle_following;

	/* We only activate if we want to merge behding ld
	or if the end of the lane is close */
	/*if (autonomous_vehicle.merge_behind_destination_lane_leader()
		|| (autonomous_vehicle.has_destination_lane_leader()
			&& end_of_lane_controller_is_active))*/
			/* [Jan 23, 2023] TO DO: changes to reflect the addition of
			virtual leader. However, this piece of code is not clean. We should
			be able to get the virtual leader directly from autonomous_vehicle */
	std::shared_ptr<const NearbyVehicle> virtual_leader = nullptr;
	if (autonomous_vehicle->has_virtual_leader())
	{
		virtual_leader =
			autonomous_vehicle->get_virtual_leader();
	}
	else if (autonomous_vehicle->has_destination_lane_leader()
		&& end_of_lane_controller_is_active)
	{
		virtual_leader =
			autonomous_vehicle->get_destination_lane_leader();
	}

	if (virtual_leader != nullptr)
	{
		if (verbose)
		{
			std::clog << "Dest. lane controller"
				<< std::endl;
		}
		/*std::shared_ptr<const NearbyVehicle> virtual_leader =
			autonomous_vehicle.get_virtual_leader();*/
		double ego_velocity = autonomous_vehicle->get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			*virtual_leader);

		if (verbose)
		{
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity))
		{
			destination_lane_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				*autonomous_vehicle, virtual_leader.get(), reference_velocity);
		is_active = true;
	}
	return is_active;
}

double AVController::implement_compute_desired_acceleration()
{
	if (autonomous_vehicle->is_vissim_controlling_lane_change()
		&& (autonomous_vehicle->has_lane_change_intention()
			|| autonomous_vehicle->is_lane_changing()))
	{
		return get_vissim_desired_acceleration(*autonomous_vehicle);
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);
	get_destination_lane_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void AVController::add_lane_change_adjustment_controller()
{
	if (verbose) std::clog << "Creating lane change adjustment controller."
		<< std::endl;
	destination_lane_controller = VirtualLongitudinalController(
		*autonomous_vehicle,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		dest_lane_colors, long_controllers_verbose);
}