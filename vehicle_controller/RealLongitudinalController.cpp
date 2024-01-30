/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of xxxx-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "RealLongitudinalController.h"
#include "EgoVehicle.h"

RealLongitudinalController::RealLongitudinalController(
	const EgoVehicle* ego_vehicle,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) 
	: SwitchedLongitudinalController(ego_vehicle, state_to_color_map, verbose)
{
	if (verbose) std::clog << "Created real longitudinal controller\n";
}

double RealLongitudinalController::get_max_accepted_brake() const
{
	return ego_vehicle->get_max_brake();
}

void RealLongitudinalController::determine_controller_state(
	const NearbyVehicle* leader, double reference_velocity) 
{
	if (leader == nullptr) // no vehicle ahead
	{ 
		state = State::velocity_control;
		
		if (verbose)
		{
			std::clog << "No leader id"
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}
	}
	else 
	{
		double gap = ego_vehicle->compute_gap_to_a_leader(leader);
		double ego_velocity = ego_vehicle->get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_control_input =
			gap_controller.compute_desired_acceleration(*ego_vehicle, leader);
		double gap_threshold = compute_gap_threshold_1(gap,
			reference_velocity - ego_velocity, gap_control_input);
		if (state == State::vehicle_following) 
		{
			gap_threshold += hysteresis_bias;
		}

		bool is_gap_small = gap < gap_threshold;
		bool is_leader_too_fast =
			leader_velocity > (reference_velocity + reference_velocity_margin);
		if (is_gap_small && !is_leader_too_fast)
		{
			state = State::vehicle_following;
		}
		else 
		{
			state = State::velocity_control;
		}

		if (verbose) 
		{
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}
	}
}

bool RealLongitudinalController::implement_is_velocity_reference_outdated(
) const
{
	return ego_vehicle->get_velocity()
		> velocity_controller.get_reference_value();
}