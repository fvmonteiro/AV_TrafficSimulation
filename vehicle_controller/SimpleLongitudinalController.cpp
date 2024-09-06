#include "EgoVehicle.h"
#include "Platoon.h"
#include "SimpleLongitudinalController.h"

SimpleLongitudinalController::SimpleLongitudinalController(
	const EgoVehicle* ego_vehicle,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) 
	: SwitchedLongitudinalController(ego_vehicle, state_to_color_map, verbose)
{
	if (verbose) std::cout << "Created simple long. controller\n";
}

double SimpleLongitudinalController::get_max_accepted_brake() const
{
	return max_brake;
}

void SimpleLongitudinalController::determine_controller_state(
	const NearbyVehicle* leader, double reference_velocity)
{
	std::string message;
	if (leader == nullptr) // no vehicle ahead
	{
		state = State::velocity_control;
		message = "No leader. ";
	}
	else if (ego_vehicle->is_in_a_platoon()
		&& (ego_vehicle->get_platoon()
			->is_vehicle_id_in_platoon(leader->get_id())))
	{
		state = State::vehicle_following;
		message = "In platoon. ";
	}
	else if (ego_vehicle->compute_nearby_vehicle_velocity(*leader)
				> reference_velocity*1.1)
	{
		double v_leader = 
			ego_vehicle->compute_nearby_vehicle_velocity(*leader);
		state = State::velocity_control;
		message = "v_leader (" + std::to_string(v_leader) 
			+ ") > v_ff (" + std::to_string(1.1*reference_velocity) + ").";
	}
	else if (state == State::vehicle_following
		&& leader->get_id() == previous_leader_id)
	{
		state = State::vehicle_following;
		message = "Locked in veh following. ";
	}
	else
	{
		double gap = ego_vehicle->compute_gap_to_a_leader(*leader);
		double ego_velocity = ego_vehicle->get_velocity();
		double leader_velocity = 
			ego_vehicle->compute_nearby_vehicle_velocity(*leader);
		double gap_threshold = gap_controller.compute_time_headway_gap(
			gap_controller.get_desired_time_headway(),
			ego_vehicle->get_velocity())
			+ (threshold_parameter 
				* std::max(ego_velocity - leader_velocity, 0.));
		if (gap > gap_threshold)
		{
			state = State::velocity_control;
			message = "gap (" + std::to_string(gap) 
				+ ") > gap_thresh (" + std::to_string(gap_threshold) + ").";
		}
		else
		{
			state = State::vehicle_following;
			message = "gap (" + std::to_string(gap)
				+ ") <= gap_thresh (" + std::to_string(gap_threshold) + ").";
			previous_leader_id = leader->get_id();
		}
	}

	if (verbose) std::cout << message << "State: " 
		<< state_to_string(state) << "\n";
}

bool SimpleLongitudinalController::implement_is_velocity_reference_outdated(
) const
{
	return ego_vehicle->get_velocity()
		> velocity_controller.get_reference_value();
}