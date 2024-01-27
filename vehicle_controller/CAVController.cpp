#include "CAVController.h"
#include "ConnectedAutonomousVehicle.h"
#include "VirtualLongitudinalController.h"

CAVController::CAVController(const ConnectedAutonomousVehicle* cav,
	bool verbose) : AVController(cav, verbose), cav{cav} {}

void CAVController::update_gap_generation_controller(double ego_velocity,
	double time_headway)
{
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	gap_generating_controller->reset_time_headway_filter(time_headway);
	gap_generating_controller->set_desired_time_headway(time_headway);
	gap_generating_controller->reset_leader_velocity_filter(
		ego_velocity);
}

void CAVController::add_cooperative_lane_change_controller()
{
	if (verbose) std::clog << "Creating coop. lane change controller.\n";

	gap_generating_controller = 
		std::make_shared<VirtualLongitudinalController>(
			cav, gap_generation_colors, long_controllers_verbose);
	gap_generating_controller->create_velocity_controller(
		adjustment_velocity_controller_gains, velocity_filter_gain);
	gap_generating_controller->create_gap_controller(
		autonomous_virtual_following_gains, connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here */
	gap_generating_controller->connect_gap_controller(true);

	available_controllers[ALCType::cooperative_gap_generation] =
		gap_generating_controller.get();
}

bool CAVController::get_cooperative_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	if (cav->is_cooperating_to_generate_gap())
	{
		if (verbose)
		{
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		const NearbyVehicle* assisted_vehicle =
			cav->get_assisted_vehicle().get();
		double reference_velocity = determine_low_velocity_reference(
			*assisted_vehicle);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the gap generating controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::cooperative_gap_generation)
			&& gap_generating_controller->is_velocity_reference_outdated())
		{
			gap_generating_controller->reset_velocity_controller(
				cav->get_velocity());
		}

		possible_accelerations[ALCType::cooperative_gap_generation] =
			gap_generating_controller->compute_desired_acceleration(
				assisted_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}

void CAVController::implement_add_internal_controllers()
{
	if (verbose) std::clog << "Creating CAV controlers\n";

	add_origin_lane_controllers();
	add_lane_change_adjustment_controller();
	add_cooperative_lane_change_controller();
	lateral_controller = LateralController(verbose);
}

double CAVController::implement_get_desired_acceleration(
)
{
	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);
	get_destination_lane_desired_acceleration(possible_accelerations);
	get_cooperative_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}
