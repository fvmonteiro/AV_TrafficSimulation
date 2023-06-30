#include "CAVController.h"
#include "ConnectedAutonomousVehicle.h"

CAVController::CAVController(const ConnectedAutonomousVehicle& cav,
	bool verbose) : AVController(cav, verbose), connected_av{ &cav }
{
	add_cooperative_lane_change_controller();
}

bool CAVController::get_cooperative_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	if (connected_av->is_cooperating_to_generate_gap())
	{
		if (verbose)
		{
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		const NearbyVehicle* assisted_vehicle =
			connected_av->get_assisted_vehicle().get();
		double ego_velocity = connected_av->get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			*assisted_vehicle);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the gap generating controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::cooperative_gap_generation)
			&& gap_generating_controller.is_outdated(ego_velocity))
		{
			gap_generating_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::cooperative_gap_generation] =
			gap_generating_controller.compute_desired_acceleration(
				*connected_av, assisted_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}

double CAVController::implement_compute_desired_acceleration()
{
	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);
	get_destination_lane_desired_acceleration(possible_accelerations);
	get_cooperative_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void CAVController::add_cooperative_lane_change_controller()
{
	if (verbose) std::clog << "Creating cooperative lane change controller."
		<< std::endl;
	gap_generating_controller = VirtualLongitudinalController(
		*connected_av,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		gap_generation_colors, long_controllers_verbose);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here*/
	gap_generating_controller.connect_gap_controller(true);
}
