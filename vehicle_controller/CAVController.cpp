#include "CAVController.h"
#include "ConnectedAutonomousVehicle.h"

CAVController::CAVController(const ConnectedAutonomousVehicle& cav,
	bool verbose) : AVController(verbose)
{
	if (verbose)
	{
		std::clog << "Creating CAV control manager\n";
	}
	add_origin_lane_controllers(cav);
	add_lane_change_adjustment_controller(cav);
	add_cooperative_lane_change_controller(cav);
}

double CAVController::get_desired_acceleration(
	const ConnectedAutonomousVehicle& cav)
{
	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(cav,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(cav,
		possible_accelerations);
	get_destination_lane_desired_acceleration(cav,
		possible_accelerations);
	get_cooperative_desired_acceleration(cav,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void CAVController::add_cooperative_lane_change_controller(
	const ConnectedAutonomousVehicle& cav)
{
	if (verbose) std::clog << "Creating cooperative lane change controller."
		<< std::endl;
	gap_generating_controller = VirtualLongitudinalController(
		cav,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		gap_generation_colors, long_controllers_verbose);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here*/
	gap_generating_controller.connect_gap_controller(true);

	available_controllers[ALCType::cooperative_gap_generation] =
		&gap_generating_controller;
}

void CAVController::update_gap_generation_controller(double ego_velocity,
	double time_headway)
{
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	gap_generating_controller.reset_time_headway_filter(time_headway);
	gap_generating_controller.set_desired_time_headway(time_headway);
	gap_generating_controller.reset_leader_velocity_filter(
		ego_velocity);
}


bool CAVController::get_cooperative_desired_acceleration(
	const ConnectedAutonomousVehicle& cav,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	if (cav.is_cooperating_to_generate_gap())
	{
		if (verbose)
		{
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		const NearbyVehicle* assisted_vehicle =
			cav.get_assisted_vehicle();
		double ego_velocity = cav.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *assisted_vehicle);

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
				cav, assisted_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}