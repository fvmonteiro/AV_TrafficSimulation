/*==========================================================================*/
/*  LongitudinalController.h    											*/
/*  Adaptive Cruise controller using the constant time headway policy       */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <iostream>
#include "LongitudinalController.h"
#include "Vehicle.h"

LongitudinalController::LongitudinalController(
	const Vehicle& ego_vehicle, bool verbose) {

	if (verbose) {
		std::clog << "Creating longitudinal controller with "
			<< "simulation time step " << ego_vehicle.get_sampling_interval()
			<< std::endl;
	}

	this->verbose = verbose;
	this->simulation_time_step = ego_vehicle.get_sampling_interval();
	this->velocity_filter = VelocityFilter(ego_vehicle.get_comfortable_acceleration(),
		ego_vehicle.get_max_brake(), simulation_time_step, verbose);
}

LongitudinalController::LongitudinalController(const Vehicle& ego_vehicle) 
	: LongitudinalController(ego_vehicle, false){}

double LongitudinalController::compute_desired_gap(double velocity_ego) { 
	return h * velocity_ego + d;
}

double LongitudinalController::compute_gap_error(double gap, double reference_gap) {
	return std::min(max_gap_error, gap - reference_gap);
};

double LongitudinalController::compute_velocity_error(double velocity_ego, 
	double velocity_reference) {
	return velocity_reference - velocity_ego;
};

void LongitudinalController::set_controller_state(
	const Vehicle& ego_vehicle, const NearbyVehicle& leader) {

	if (leader.get_current_id() <= 0) { // no vehicle ahead
		state = State::velocity_control;
	}
	else {
		double ego_velocity = ego_vehicle.get_current_velocity();
		double leader_velocity = ego_vehicle.get_current_velocity()
			- leader.get_current_relative_velocity();
		double gap_threshold = compute_gap_threshold(
			ego_vehicle.get_desired_velocity(), compute_velocity_error(
				ego_velocity, leader_velocity));
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}
		if ((leader.get_current_distance() < (gap_threshold))
			&& (leader_velocity < ego_vehicle.get_desired_velocity())) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}
	}
}

double LongitudinalController::compute_gap_threshold(double desired_velocity, 
	double velocity_error) {
	return h * desired_velocity + d - 1 / kg * (kv * velocity_error);
	//return 250;
}

double LongitudinalController::compute_vehicle_following_input(double gap_error,
	double velocity_error) {
	return kg * gap_error + kv * velocity_error;
}

/* PID velocity controller. The derivative of the velocity error equals the 
ego vehicle acceleration since the desired speed is constant. */
double LongitudinalController::compute_velocity_control_input(double velocity_error, 
	double ego_acceleration) {
	velocity_error_integral += velocity_error * simulation_time_step;
	return ki * velocity_error_integral 
		+ kp * velocity_error 
		- kd * ego_acceleration;
}

double LongitudinalController::compute_desired_acceleration(
	const Vehicle& ego_vehicle,	const NearbyVehicle& leader) {
	
	double gap;
	double gap_reference;
	double gap_error;
	double velocity_reference;
	double filtered_velocity_reference;
	double velocity_error;
	double desired_acceleration;
	double ego_velocity = ego_vehicle.get_current_velocity();
	double leader_velocity = ego_velocity
		- leader.get_current_relative_velocity();
	State old_state = state;

	set_controller_state(ego_vehicle, leader);
	if (old_state != state) {
		velocity_filter.reset(ego_velocity);
	}

	switch (state)
	{
	case State::vehicle_following:
		gap = ego_vehicle.compute_gap(leader);
		gap_reference = compute_desired_gap(ego_velocity);
		gap_error = compute_gap_error(gap, gap_reference);
		velocity_reference = leader_velocity; 
		filtered_velocity_reference = velocity_filter.filter_velocity(
			leader_velocity);
		velocity_error = compute_velocity_error(
			ego_velocity, filtered_velocity_reference);
		desired_acceleration = compute_vehicle_following_input(
			gap_error, velocity_error);
		break;
	case State::velocity_control:
		if (old_state != State::velocity_control) {
			reset_velocity_error_integrator();
		}
		velocity_reference = ego_vehicle.get_desired_velocity(); 
		filtered_velocity_reference = velocity_filter.filter_velocity(
			ego_vehicle.get_desired_velocity());
		velocity_error = compute_velocity_error(
			ego_velocity, filtered_velocity_reference);
		desired_acceleration = compute_velocity_control_input(velocity_error, 
			ego_vehicle.get_current_acceleration());
		break;
	default:
		std::cerr << "Unknown controller state for vehicle:" << std::endl;
		std::cerr << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
		break;
	}

	/*if (verbose) {
		std::clog << ego_vehicle.get_current_time() << "; " << velocity_reference
			<< "; " << filtered_velocity_reference << std::endl;
	}*/

	return desired_acceleration;
}

void LongitudinalController::reset_velocity_error_integrator() {
	velocity_error_integral = 0;
}

/* Convert the State enum to string. Used mostly for printing while
debugging */
std::string LongitudinalController::state_to_string(State state) {
	switch (state)
	{
	case LongitudinalController::State::uninitialized:
		return "uninitialized";
	case LongitudinalController::State::velocity_control:
		return "velocity_control";
	case LongitudinalController::State::vehicle_following:
		return "vehicle_following";
	default:
		return "unknown longitudinal controller state";
	}
}