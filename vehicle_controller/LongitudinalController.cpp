/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include <iostream>
#include "LongitudinalController.h"
#include "Vehicle.h"

LongitudinalController::LongitudinalController(const Vehicle& ego_vehicle,
	double max_brake, double desired_velocity, double filter_brake_limit,
	bool verbose)
	: simulation_time_step{ ego_vehicle.get_sampling_interval() },
	ego_vehicle_max_brake{ max_brake },
	ego_vehicle_desired_velocity{ desired_velocity },
	verbose{ verbose } {

	double max_jerk = ego_vehicle.get_max_jerk();
	double comfortable_acceleration =
		ego_vehicle.get_comfortable_acceleration();
	double brake_delay = ego_vehicle.get_brake_delay();

	compute_safe_gap_parameters(max_jerk, comfortable_acceleration,
		ego_vehicle_max_brake, brake_delay);
	this->velocity_filter = VelocityFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step);

	if (verbose) {
		std::clog << "Created base longitudinal controller with "
			<< "max brake=" << ego_vehicle_max_brake
			<< ", desired velocity=" << ego_vehicle_desired_velocity
			<< std::endl;
	}
}

//LongitudinalController::LongitudinalController(const Vehicle& ego_vehicle,
//	bool is_used_for_lane_change, bool verbose) 
//	: simulation_time_step{ ego_vehicle.get_sampling_interval() },
//	ego_vehicle_desired_velocity{ ego_vehicle.get_desired_velocity() },
//	verbose{ verbose } {
//
//	//this->verbose = verbose;
//	//this->simulation_time_step = ego_vehicle.get_sampling_interval();
//
//	// getting ego vehicle parameters
//	//this->ego_vehicle_desired_velocity = ego_vehicle.get_desired_velocity();
//	double filter_braking_limit;
//	if (is_used_for_lane_change) {
//		this->ego_vehicle_max_brake = ego_vehicle.get_max_brake() / 2;
//		filter_braking_limit = ego_vehicle.get_comfortable_brake();
//	}
//	else {
//		this->ego_vehicle_max_brake = ego_vehicle.get_max_brake();
//		filter_braking_limit = ego_vehicle.get_max_brake();
//	}
//	double max_jerk = ego_vehicle.get_max_jerk();
//	double comfortable_acceleration = 
//		ego_vehicle.get_comfortable_acceleration();
//	double brake_delay = ego_vehicle.get_brake_delay();
//
//	compute_safe_gap_parameters(max_jerk, comfortable_acceleration, 
//		ego_vehicle_max_brake, brake_delay);
//	this->velocity_filter = VelocityFilter(comfortable_acceleration,
//		filter_braking_limit, simulation_time_step);
//
//	if (verbose) {
//		std::clog << "Created longitudinal controller with "
//			<< "simulation time step " << ego_vehicle.get_sampling_interval()
//			<< ", used for lane change? " << is_used_for_lane_change
//			<< ", lambda1=" << ego_vehicle_lambda_1
//			<< ", lambda0=" << ego_vehicle_lambda_0
//			<< std::endl;
//	}
//}
//
//LongitudinalController::LongitudinalController(const Vehicle& ego_vehicle,
//	bool is_used_for_lane_change) : 
//	LongitudinalController(ego_vehicle, is_used_for_lane_change, false){}

double LongitudinalController::compute_exact_collision_free_gap(
	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle) {

	double follower_lambda_0, follower_lambda_1;
	double v_follower, v_leader;
	double brake_follower, brake_leader;
	double delta_v = other_vehicle.get_relative_velocity();
	if (other_vehicle.is_ahead()) {
		follower_lambda_0 = ego_vehicle_lambda_0;
		follower_lambda_1 = ego_vehicle_lambda_1;
		v_follower = ego_vehicle.get_current_velocity();
		v_leader = v_follower - delta_v;
		brake_follower = ego_vehicle.get_max_brake();
		brake_leader = other_vehicle.get_max_brake();
	}
	else {
		follower_lambda_0 = other_vehicle.get_lambda_0();
		follower_lambda_1 = other_vehicle.get_lambda_1();
		v_leader = ego_vehicle.get_current_velocity();
		v_follower = v_leader - delta_v;
		brake_follower = other_vehicle.get_max_brake();
		brake_leader = ego_vehicle.get_max_brake();
	}

	double stop_time_follower = (v_follower + follower_lambda_1) / brake_follower;
	double stop_time_leader = v_leader / brake_leader;
	double collision_free_gap;

	if (stop_time_follower >= stop_time_leader) {
		collision_free_gap =
			std::pow(v_follower + follower_lambda_1, 2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader + follower_lambda_0;
	}
	else if (brake_leader < brake_follower) {
		collision_free_gap =
			std::pow(delta_v - follower_lambda_1, 2) / 2 / (brake_follower - brake_leader)
			+ follower_lambda_0;
	}
	else {
		collision_free_gap = 0.0;
	}
	return collision_free_gap;
}

double LongitudinalController::compute_time_headway_gap(double time_headway,
	double velocity) {
	/* TODO: for now we assumed the standstill distance (d) is the same for 
	all cases. */
	return time_headway * velocity + d;
}

double LongitudinalController::compute_desired_gap(double velocity_ego) {
	return compute_time_headway_gap(h, velocity_ego);
}

double LongitudinalController::compute_gap_error(double gap, double reference_gap) {
	return std::min(max_gap_error, gap - reference_gap);
};

double LongitudinalController::compute_velocity_error(double velocity_ego, 
	double velocity_reference) {
	return velocity_reference - velocity_ego;
};

//void LongitudinalController::determine_controller_state(
//	const Vehicle& ego_vehicle, const NearbyVehicle* leader) {
//
//	if (leader == nullptr) { // no vehicle ahead
//		state = State::velocity_control;
//	}
//	else {
//		double ego_velocity = ego_vehicle.get_current_velocity();
//		double leader_velocity = leader->compute_velocity(ego_velocity);
//		double gap_threshold = compute_gap_threshold(
//			ego_vehicle.get_desired_velocity(), compute_velocity_error(
//				ego_velocity, leader_velocity));
//		if (state == State::vehicle_following) {
//			gap_threshold += hysteresis_bias;
//		}
//		if ((leader->get_distance() < (gap_threshold))
//			&& (leader_velocity < ego_vehicle.get_desired_velocity())) {
//			state = State::vehicle_following;
//		}
//		else {
//			state = State::velocity_control;
//		}
//	}
//}

double LongitudinalController::compute_gap_threshold(double desired_velocity, 
	double velocity_error) {
	/* Threshold is computed such that, at the switch, the vehicle following 
	input is greater or equal to kg*h*(Vf - v) > 0. */
	return h * desired_velocity + d - 1 / kg * (kv * velocity_error);
	/* Other threshold options: 
	- VISSIM's maximum gap: 250 
	- Worst-case: h*vf + d + (kv*vf)/kg = (h + kv/kg)*vf + d*/
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
	const Vehicle& ego_vehicle,	const NearbyVehicle* leader) {
	
	double gap, gap_reference, gap_error;
	double velocity_reference, filtered_velocity_reference;
	double velocity_error;
	double desired_acceleration;
	double leader_velocity;
	double ego_velocity = ego_vehicle.get_current_velocity();
	State old_state = state;

	determine_controller_state(ego_velocity, leader);
	if (old_state != state) {
		velocity_filter.reset(ego_velocity);
	}

	switch (state)
	{
	case State::uninitialized:
		desired_acceleration = 0;
		break;
	case State::vehicle_following:
		gap = ego_vehicle.compute_gap(leader);
		gap_reference = compute_desired_gap(ego_velocity);
		gap_error = compute_gap_error(gap, gap_reference);
		leader_velocity =  leader->compute_velocity(ego_velocity);
		velocity_reference = leader_velocity; 
		filtered_velocity_reference = velocity_filter.filter_velocity(
			velocity_reference);
		velocity_error = compute_velocity_error(
			ego_velocity, filtered_velocity_reference);
		desired_acceleration = compute_vehicle_following_input(
			gap_error, velocity_error);
		break;
	case State::velocity_control:
		if (old_state != State::velocity_control) {
			reset_velocity_error_integrator();
		}
		velocity_reference = ego_vehicle_desired_velocity; 
		filtered_velocity_reference = velocity_filter.filter_velocity(
			velocity_reference);
		velocity_error = compute_velocity_error(
			ego_velocity, filtered_velocity_reference);
		desired_acceleration = compute_velocity_control_input(velocity_error, 
			ego_vehicle.get_current_acceleration());
		break;
	default:
		std::clog << "Unknown controller state!" << std::endl;
		std::clog << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
		break;
	}

	return desired_acceleration;
}

//double LongitudinalController::compute_desired_acceleration(
//	const Vehicle& ego_vehicle, const NearbyVehicle& leader) {
//	return compute_desired_acceleration(ego_vehicle, &leader);
//}

void LongitudinalController::reset_velocity_error_integrator() {
	velocity_error_integral = 0;
}

void LongitudinalController::update_time_headway(
	const Vehicle& ego_vehicle, double leader_max_brake) {

	h = compute_time_headway(ego_vehicle_desired_velocity,
		ego_vehicle_max_brake, leader_max_brake,
		ego_vehicle_lambda_1, rho);

}

/* Protected and private methods ------------------------------------------ */

void LongitudinalController::compute_safe_gap_parameters(double max_jerk,
	double comfortable_acceleration, double maximum_braking,
	double brake_delay) {
	double j_ego = max_jerk;
	double a_ego = comfortable_acceleration;
	double b_ego = maximum_braking;
	double tau_d = brake_delay;
	double tau_j = (a_ego + b_ego) / j_ego;
	ego_vehicle_lambda_0 = -(a_ego + b_ego)
		* (std::pow(tau_d, 2) + tau_d * tau_j + std::pow(tau_j, 2) / 3);
	ego_vehicle_lambda_1 = (a_ego + b_ego) * (tau_d + tau_j / 2);
}

double LongitudinalController::compute_time_headway(double free_flow_velocity,
	double follower_max_brake, double leader_max_brake,
	double lambda_1, double rho) {

	double time_headway;
	double gamma = leader_max_brake / follower_max_brake;
	double gamma_threshold = (1 - rho) * free_flow_velocity
		/ (free_flow_velocity + lambda_1);

	if (gamma < gamma_threshold) {
		// case where ego brakes harder
		time_headway = (std::pow(rho, 2) * free_flow_velocity / 2 + rho * lambda_1)
			/ ((1 - gamma) * follower_max_brake);
	}
	else if (gamma >= std::pow(1 - rho, 2)) {
		time_headway = ((1 - std::pow(1 - rho, 2) / gamma) * free_flow_velocity / 2
			+ lambda_1) / follower_max_brake;
	}
	else {
		time_headway = lambda_1 / follower_max_brake;
	}

	return time_headway;
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