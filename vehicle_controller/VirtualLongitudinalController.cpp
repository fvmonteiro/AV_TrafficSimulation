/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of xxxx-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

//#include <algorithm>
#include "VirtualLongitudinalController.h"
#include "EgoVehicle.h"

//VirtualLongitudinalController::VirtualLongitudinalController() :
//	SwitchedLongitudinalController() {}

VirtualLongitudinalController::VirtualLongitudinalController(
	const EgoVehicle* ego_vehicle,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	SwitchedLongitudinalController(ego_vehicle, state_to_color_map, verbose) 
{
	if (verbose) std::clog << "Created virtual longitudinal controller\n";
}

double VirtualLongitudinalController::get_max_accepted_brake()
{
	return ego_vehicle->get_comfortable_brake();
}

void VirtualLongitudinalController::determine_controller_state(
	const NearbyVehicle* leader, double reference_velocity, 
	double gap_control_input) 
{
	if (leader == nullptr) 
	{ // no vehicle ahead
		/*If there's no leader this controller should not be active */
		if (verbose) std::clog << "\tno leader" << std::endl;
		state = State::uninitialized;
	}
	else 
	{		
		double gap = ego_vehicle->compute_gap_to_a_leader(leader);
		double ego_velocity = ego_vehicle->get_velocity();
		double gap_threshold = compute_gap_threshold_1(gap,
			0/*reference_velocity - ego_velocity*/, gap_control_input);

		if (state == State::vehicle_following) {
			gap_threshold -= hysteresis_bias;
		}

		if ((gap > gap_threshold) 
			|| (ego_velocity < std::min(reference_velocity, 5.0))) 
		{
			state = State::vehicle_following;
		}
		else 
		{
			state = State::velocity_control;
		}

		if (verbose) {
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}
	}
}

bool VirtualLongitudinalController::is_active() const 
{
	return state != State::uninitialized;
}

bool VirtualLongitudinalController::is_outdated() const 
{
	return ego_vehicle->get_velocity() 
		< velocity_controller.get_reference_value();
}