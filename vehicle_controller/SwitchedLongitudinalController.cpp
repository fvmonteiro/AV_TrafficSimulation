/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include <iostream>

#include "EgoVehicle.h"
#include "SwitchedLongitudinalController.h"

SwitchedLongitudinalController::SwitchedLongitudinalController(
	const VelocityControllerGains& velocity_controller_gains,
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double filter_brake_limit, double comfortable_acceleration,
	double simulation_time_step,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	LongitudinalController(state_to_color_map, verbose)
	/*autonomous_gains{ autonomous_gains },
	connected_gains{ connected_gains }*/
{
	velocity_controller = VelocityController(simulation_time_step,
		velocity_controller_gains, velocity_filter_gain, 
		comfortable_acceleration, filter_brake_limit);
	gap_controller = GapController(simulation_time_step, autonomous_gains,
		connected_gains, velocity_filter_gain, time_headway_filter_gain,
		comfortable_acceleration, filter_brake_limit, verbose);
}

void SwitchedLongitudinalController::connect_gap_controller(bool is_connected)
{
	gap_controller.set_connexion(is_connected);
}

void SwitchedLongitudinalController::smooth_start_leader_velocity_filter()
{
	gap_controller.ask_for_smooth_start();
}

void SwitchedLongitudinalController::reset_leader_velocity_filter(
	double reset_velocity)
{
	gap_controller.reset_velocity_filter(reset_velocity);
}

void SwitchedLongitudinalController::reset_time_headway_filter(
	double time_headway)
{
	gap_controller.reset_time_headway_filter(time_headway);
}

double SwitchedLongitudinalController::get_desired_time_headway() const
{
	return gap_controller.get_desired_time_headway();
}

double SwitchedLongitudinalController::get_current_time_headway() const
{
	return gap_controller.get_current_time_headway();
}

double SwitchedLongitudinalController::get_time_headway_gap(
	double time_headway, double velocity) 
{
	return gap_controller.compute_time_headway_gap(time_headway, velocity);
}

double SwitchedLongitudinalController::get_desired_time_headway_gap(
	double ego_velocity/*, bool has_lane_change_intention*/) 
{
	return gap_controller.get_desired_time_headway_gap(ego_velocity/*,
		has_lane_change_intention*/);
}

double SwitchedLongitudinalController::get_desired_gap(double ego_velocity)
{
	return gap_controller.get_desired_gap(ego_velocity);
	/*return get_time_headway_gap(time_headway_filter.get_current_value(),
		ego_velocity);*/
}

void SwitchedLongitudinalController::set_desired_time_headway(
	double time_headway)
{
	gap_controller.set_desired_time_headway(time_headway);
}

double SwitchedLongitudinalController::compute_gap_threshold(double gap,
	double diff_to_velocity_reference, double gap_control_input) 
{
	/* Threshold is computed such that, at the switch, the vehicle following
	input is greater or equal to kg*h*(Vf - v) > 0. */
	double h = gap_controller.get_desired_time_headway();
	double kg = gap_controller.get_gap_error_gain();
	/* Line below is equivalent to the paper's equation but using
	only variables easily available to the code */
	return gap + h * (diff_to_velocity_reference) - gap_control_input / kg;
	/* Other threshold options:
	- VISSIM's maximum gap: 250
	- Worst-case: h*vf + d + (kv*vf)/kg = (h + kv/kg)*vf + d*/
}

double SwitchedLongitudinalController::implement_get_gap_error() const
{
	return gap_controller.get_gap_error();
}

double SwitchedLongitudinalController::implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference) 
{
	double desired_acceleration;
	State old_state = state;
	double gap_control_input = 
		gap_controller.compute_desired_acceleration(
			ego_vehicle, leader);
	
	determine_controller_state(ego_vehicle, leader, velocity_reference,
		gap_control_input);

	switch (state)
	{
	case State::uninitialized:
		desired_acceleration = 0;
		break;
	case State::vehicle_following:
	{
		desired_acceleration = gap_control_input;
		break;
	}
	case State::velocity_control:
	{
		if (old_state != State::velocity_control) 
		{
			velocity_controller.smooth_reset(ego_vehicle, 
				velocity_reference);
		}
		desired_acceleration = 
			velocity_controller.compute_acceleration(ego_vehicle, 
				velocity_reference);
		break;
	}
	default:
		std::clog << "Unknown controller state!" << std::endl;
		std::clog << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle.get_vissim_acceleration();
		break;
	}

	if (verbose) {
		std::clog << "\tu=" << desired_acceleration << std::endl;
	}

	return desired_acceleration;
}

//void SwitchedLongitudinalController::compute_max_risk_to_leader(bool is_lane_changing) {
//	double time_headway = get_safe_time_headway();
//	max_risk_to_leader = std::sqrt(
//		2 * time_headway * ego_max_brake * free_flow_velocity);
//	if (verbose) std::clog << "max risk to leader="
//		<< max_risk_to_leader << std::endl;
//}

void SwitchedLongitudinalController::reset_velocity_controller(
	double reset_velocity) {
	velocity_controller.reset_filter(reset_velocity);
	velocity_controller.reset_error_integrator();
}