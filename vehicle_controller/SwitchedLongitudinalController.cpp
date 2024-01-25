/*==========================================================================*/
/* SwitchedLongitudinalController.h											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of xxxx-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include <iostream>

#include "EgoVehicle.h"
#include "SwitchedLongitudinalController.h"

SwitchedLongitudinalController::SwitchedLongitudinalController(
	const EgoVehicle* ego_vehicle,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	LongitudinalController(ego_vehicle, state_to_color_map, verbose)
{}

void SwitchedLongitudinalController::create_velocity_controller(
	const VelocityControllerGains& velocity_controller_gains,
	double velocity_filter_gain)
{
	velocity_controller = VelocityController(
		ego_vehicle->get_simulation_time_step(),
		velocity_controller_gains, velocity_filter_gain,
		ego_vehicle->get_comfortable_acceleration(),
		get_max_accepted_brake());
}

void SwitchedLongitudinalController::create_gap_controller(
	const AutonomousGains& autonomous_gains, 
	const ConnectedGains& connected_gains, double velocity_filter_gain,
	double time_headway_filter_gain)
{
	gap_controller = GapController(ego_vehicle->get_simulation_time_step(),
		autonomous_gains, connected_gains, velocity_filter_gain,
		time_headway_filter_gain,
		ego_vehicle->get_comfortable_acceleration(), 
		get_max_accepted_brake(), verbose);
}

bool SwitchedLongitudinalController::is_initialized() const
{
	return state != State::uninitialized;
}

bool SwitchedLongitudinalController::is_velocity_reference_outdated() const
{
	return implement_is_velocity_reference_outdated();
}

void SwitchedLongitudinalController::reset_velocity_controller(
	double reset_velocity) {
	velocity_controller.reset_filter(reset_velocity);
	velocity_controller.reset_error_integrator();
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
	double time_headway, double velocity) const
{
	return gap_controller.compute_time_headway_gap(time_headway, velocity);
}

double SwitchedLongitudinalController::get_desired_time_headway_gap(
	) const
{
	return gap_controller.get_desired_time_headway_gap(
		ego_vehicle->get_velocity());
}

double SwitchedLongitudinalController::get_standstill_distance() const
{
	return gap_controller.get_standstill_distance();
}

double SwitchedLongitudinalController::get_desired_gap()
const
{
	return gap_controller.get_desired_gap(ego_vehicle->get_velocity());
}

void SwitchedLongitudinalController::set_desired_time_headway(
	double time_headway)
{
	gap_controller.set_desired_time_headway(time_headway);
}

void SwitchedLongitudinalController::update_leader_velocity_filter(
	double leader_velocity)
{
	gap_controller.update_leader_velocity_filter(leader_velocity);
	//leader_velocity_filter.apply_filter(leader_velocity);
}

double SwitchedLongitudinalController::compute_gap_threshold_1(double gap,
	double diff_to_velocity_reference, double gap_control_input) const
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
	const NearbyVehicle* leader, double velocity_reference) 
{
	double desired_acceleration;
	State old_state = state;
	double gap_control_input = 
		gap_controller.compute_desired_acceleration(*ego_vehicle, leader);
	
	determine_controller_state(leader, velocity_reference,
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
			velocity_controller.smooth_reset(*ego_vehicle, 
				velocity_reference);
		}
		desired_acceleration = 
			velocity_controller.compute_acceleration(*ego_vehicle, 
				velocity_reference);
		break;
	}
	default:
		std::clog << "Unknown controller state!" << std::endl;
		std::clog << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle->get_vissim_acceleration();
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
