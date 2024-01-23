/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

//#include <algorithm>
#include "VirtualLongitudinalController.h"
#include "EgoVehicle.h"

//VirtualLongitudinalController::VirtualLongitudinalController() :
//	SwitchedLongitudinalController() {}

VirtualLongitudinalController::VirtualLongitudinalController(
	const EgoVehicle& ego_vehicle,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	SwitchedLongitudinalController(velocity_controller_gains,
		autonomous_gains, connected_gains,
		velocity_filter_gain, time_headway_filter_gain,
		ego_vehicle.get_comfortable_brake(), 
		ego_vehicle.get_comfortable_acceleration(),
		ego_vehicle.get_simulation_time_step(), 
		state_to_color_map, verbose) 
{
	if (verbose) 
	{
		std::clog << "Created virtual longitudinal controller" << std::endl;
	}
}

//VirtualLongitudinalController::VirtualLongitudinalController(
//	const EgoVehicle& ego_vehicle,
//	VelocityControllerGains velocity_controller_gains,
//	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
//	double velocity_filter_gain, double time_headway_filter_gain) :
//	VirtualLongitudinalController(ego_vehicle,
//		velocity_controller_gains, autonomous_gains, connected_gains,
//		velocity_filter_gain, time_headway_filter_gain, false) {}

void VirtualLongitudinalController::determine_controller_state(
	const EgoVehicle& ego_vehicle, const NearbyVehicle* leader,
	double reference_velocity, double gap_control_input) 
{
	if (leader == nullptr) 
	{ // no vehicle ahead
		/*If there's no leader this controller should not be active */
		if (verbose) std::clog << "\tno leader" << std::endl;
		state = State::uninitialized;
	}
	else 
	{
		/*double ego_velocity = ego_vehicle.get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double velocity_error = compute_velocity_error(
			ego_velocity, leader_velocity);
		double gap_threshold = gap_controller.compute_gap_threshold(
			reference_velocity, velocity_error,
			ego_vehicle.get_acceleration(), leader->get_acceleration()
		);*/
		
		double gap = ego_vehicle.compute_gap_to_a_leader(leader);
		double ego_velocity = ego_vehicle.get_velocity();
		double gap_threshold = compute_gap_threshold(gap,
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

bool VirtualLongitudinalController::is_outdated(
	double ego_velocity) const 
{
	return ego_velocity < velocity_controller.get_reference_value();
}