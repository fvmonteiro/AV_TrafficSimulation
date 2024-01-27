#include "SimpleLongitudinalController.h"
#include "EgoVehicle.h"

SimpleLongitudinalController::SimpleLongitudinalController(
	const EgoVehicle* ego_vehicle,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) 
	: SwitchedLongitudinalController(ego_vehicle, state_to_color_map, verbose)
{
	if (verbose) std::clog << "Created simple long. controller\n";
}

double SimpleLongitudinalController::get_max_accepted_brake() const
{
	return max_brake;
}

void SimpleLongitudinalController::determine_controller_state(
	const NearbyVehicle* leader, double reference_velocity,
	double gap_control_input)
{
	std::string message;
	if (leader == nullptr) // no vehicle ahead
	{
		state = State::velocity_control;
		message = "No leader. ";
	}
	else if (leader->compute_velocity(ego_vehicle->get_velocity())
				> ego_vehicle->get_desired_velocity())
	{
		state = State::velocity_control;
		message = "v_leader > v_ff. ";
	}
	else if (state == State::vehicle_following
		&& leader->get_id() == previous_leader_id)
	{
		state = State::vehicle_following;
		message = "locked in veh following. ";
	}
	else
	{
		double gap = ego_vehicle->compute_gap_to_a_leader(leader);
		double ego_velocity = ego_vehicle->get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_threshold = gap_controller.compute_time_headway_gap(
			gap_controller.get_desired_time_headway(),
			ego_vehicle->get_velocity())
			+ (threshold_parameter 
				* std::max(ego_velocity - leader_velocity, 0.));
		if (gap > gap_threshold)
		{
			state = State::velocity_control;
			message = "gap > gap_thresh. ";
		}
		else
		{
			state = State::vehicle_following;
			message = "gap <= gap_thresh. ";
			previous_leader_id = leader->get_id();
		}
	}

	if (verbose) std::clog << message << "State: " << state_to_string(state);
}

bool SimpleLongitudinalController::implement_is_velocity_reference_outdated(
) const
{
	return ego_vehicle->get_velocity()
		> velocity_controller.get_reference_value();
}