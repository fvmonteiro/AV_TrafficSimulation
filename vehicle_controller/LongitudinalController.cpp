/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include <iostream>
#include "LongitudinalController.h"
#include "EgoVehicle.h"

LongitudinalController::LongitudinalController(const EgoVehicle& ego_vehicle,
	double max_brake, double reference_velocity, double filter_brake_limit,
	bool verbose)
	: simulation_time_step{ ego_vehicle.get_sampling_interval() },
	ego_max_brake{ max_brake },
	ego_reference_velocity{ reference_velocity },
	free_flow_velocity{ ego_vehicle.get_desired_velocity() },
	verbose{ verbose } {

	//double max_jerk = ego_vehicle.get_max_jerk();
	//double brake_delay = ego_vehicle.get_brake_delay();*/

	/*compute_safe_gap_parameters(max_jerk, comfortable_acceleration,
		ego_vehicle_max_brake, brake_delay);*/
	double comfortable_acceleration =
		ego_vehicle.get_comfortable_acceleration();
	this->velocity_filter = VelocityFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step);

	if (verbose) {
		std::clog << "Created base longitudinal controller with "
			<< "max brake=" << ego_max_brake
			<< ", free flow velocity=" << free_flow_velocity
			<< ", reference velocity=" << ego_reference_velocity
			<< std::endl;
	}
}

//double LongitudinalController::compute_exact_collision_free_gap(
//	double ego_velocity, const NearbyVehicle& other_vehicle) {
//
//	double follower_lambda_0, follower_lambda_1;
//	double v_follower, v_leader;
//	double brake_follower, brake_leader;
//	double delta_v = other_vehicle.get_relative_velocity();
//	if (other_vehicle.is_ahead()) {
//		follower_lambda_0 = ego_vehicle_lambda_0;
//		follower_lambda_1 = ego_vehicle_lambda_1;
//		v_follower = ego_velocity;
//		v_leader = v_follower - delta_v;
//		brake_follower = ego_vehicle_max_brake;
//		brake_leader = other_vehicle.get_max_brake();
//	}
//	else {
//		follower_lambda_0 = other_vehicle.get_lambda_0();
//		follower_lambda_1 = other_vehicle.get_lambda_1();
//		v_leader = ego_velocity;
//		v_follower = v_leader - delta_v;
//		brake_follower = other_vehicle.get_max_brake();
//		brake_leader = ego_vehicle_max_brake;
//	}
//
//	double stop_time_follower = (v_follower + follower_lambda_1) / brake_follower;
//	double stop_time_leader = v_leader / brake_leader;
//	double collision_free_gap;
//
//	if (stop_time_follower >= stop_time_leader) {
//		collision_free_gap =
//			std::pow(v_follower + follower_lambda_1, 2) / 2 / brake_follower
//			- std::pow(v_leader, 2) / 2 / brake_leader + follower_lambda_0;
//	}
//	else if (brake_leader < brake_follower) {
//		collision_free_gap =
//			std::pow(delta_v - follower_lambda_1, 2) / 2 / (brake_follower - brake_leader)
//			+ follower_lambda_0;
//	}
//	else {
//		collision_free_gap = 0.0;
//	}
//	return collision_free_gap;
//}

//double LongitudinalController::compute_collision_severity_risk(
//	const Vehicle& ego_vehicle, const NearbyVehicle& leader) {
//
//	double brake_delay = ego_vehicle.get_brake_delay();
//	double comfortable_acceleration = 
//		ego_vehicle.get_comfortable_acceleration();
//	double jerk_delay = (comfortable_acceleration + ego_vehicle_max_brake)
//		/ ego_vehicle.get_max_jerk();
//	double ego_vel = ego_vehicle.get_current_velocity();
//
//	double leader_max_brake = leader.get_max_brake();
//	double relative_vel = leader.get_relative_velocity();
//	double leader_vel = leader.compute_velocity(ego_vel);
//
//	double gamma = leader_max_brake / ego_vehicle_max_brake;
//	double gamma_threshold = leader_vel / (ego_vel + ego_vehicle_lambda_1);
//	
//	std::vector<double> gap_thresholds(4);
//	gap_thresholds[0] = brake_delay
//		* (brake_delay * (comfortable_acceleration + leader_max_brake) / 2
//			+ relative_vel);
//	gap_thresholds[1] = (brake_delay + jerk_delay)
//		* (ego_vehicle_lambda_1 + relative_vel
//			- (brake_delay + jerk_delay) 
//			* (ego_vehicle_max_brake - leader_max_brake) / 2)
//		+ ego_vehicle_lambda_0;
//	gap_thresholds[2] = leader_vel / leader_max_brake
//		* (ego_vehicle_lambda_1 + relative_vel
//			- leader_vel / leader_max_brake
//			* (ego_vehicle_max_brake - leader_max_brake) / 2)
//		+ ego_vehicle_lambda_0;
//	gap_thresholds[3] = compute_exact_collision_free_gap(ego_vel, leader);
//
//	double gap{ ego_vehicle.compute_gap(leader) };
//	double collision_severity = 0;
//	if (gap < gap_thresholds[0]) {
//		collision_severity = std::pow(relative_vel, 2) 
//			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
//	}
//	else if (gap < gap_thresholds[1]) {
//		/* The solution for this case requires solving a 3rd degree equation.
//		To avoid that, we will approximate it as the mean of the previous
//		and following case. We will also record how often this case 
//		happens to see if it's important to properly code the solution. */
//		collision_severity = std::pow(relative_vel, 2)
//			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
//		collision_severity += std::pow(relative_vel + ego_vehicle_lambda_1, 2)
//			+ 2 * (leader_max_brake - ego_vehicle_max_brake)
//			* (gap - ego_vehicle_lambda_0);
//		collision_severity /= 2;
//		std::clog << "t=" << ego_vehicle.get_current_time()
//			<< ", id=" << ego_vehicle.get_id()
//			<< ", collision severity complicated case" 
//			<< std::endl;
//	}
//	else if (((gamma >= gamma_threshold) && (gap < gap_thresholds[2]))
//		|| (gamma < gamma_threshold) && (gap < gap_thresholds[3])) {
//		collision_severity = std::pow(relative_vel + ego_vehicle_lambda_1, 2)
//			+ 2 * (leader_max_brake - ego_vehicle_max_brake) 
//			* (gap - ego_vehicle_lambda_0);
//	}
//	else if ((gamma >= gamma_threshold) && (gap < gap_thresholds[3])) {
//		collision_severity = std::pow(ego_vel + ego_vehicle_lambda_1, 2)
//			- 2 * ego_vehicle_max_brake
//			* (std::pow(leader_vel, 2) / 2 / leader_max_brake 
//				+ gap - ego_vehicle_lambda_0);
//	}
//	return collision_severity;
//}

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

double LongitudinalController::compute_gap_threshold(double free_flow_velocity, 
	double velocity_error) {
	/* Threshold is computed such that, at the switch, the vehicle following 
	input is greater or equal to kg*h*(Vf - v) > 0. */
	return h * free_flow_velocity + d - 1 / kg * (kv * velocity_error);
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
	const EgoVehicle& ego_vehicle,	const NearbyVehicle* leader) {
	
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

	if (verbose) {
		std::clog << "\tstate="<< state_to_string(state)
			<< std::endl;
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

		if (verbose) {
			std::clog << "\tleader id = " << leader->get_id()
				<< ", eg=" << gap_error
				<< ", ev=" << velocity_error
				<< std::endl;
		}

		desired_acceleration = compute_vehicle_following_input(
			gap_error, velocity_error);
		break;
	case State::velocity_control:
		if (old_state != State::velocity_control) {
			reset_velocity_error_integrator();
		}
		velocity_reference = ego_reference_velocity; 
		filtered_velocity_reference = velocity_filter.filter_velocity(
			velocity_reference);
		velocity_error = compute_velocity_error(
			ego_velocity, filtered_velocity_reference);

		if (verbose) {
			std::clog << "\tev=" << velocity_error
				<< std::endl;
		}

		desired_acceleration = compute_velocity_control_input(velocity_error, 
			ego_vehicle.get_current_acceleration());
		break;
	default:
		std::clog << "Unknown controller state!" << std::endl;
		std::clog << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
		break;
	}

	if (verbose) {
		std::clog << "\tu=" << desired_acceleration << std::endl;
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

void LongitudinalController::update_safe_time_headway(
	double lambda_1, double leader_max_brake) {
	h = compute_safe_time_headway(free_flow_velocity,
		ego_max_brake, leader_max_brake,
		lambda_1, rho);
}

void LongitudinalController::update_time_headway_with_risk(
	double lambda_1, double leader_max_brake) {
	if (verbose) std::clog << "Updating time headway from " << h;
	h = compute_time_headway_with_risk(free_flow_velocity,
		ego_max_brake, leader_max_brake,
		lambda_1, rho, accepted_risk);
	if (verbose) std::clog << " to " << h << std::endl;
}

/* Protected and private methods ------------------------------------------ */

void LongitudinalController::set_vehicle_following_gains(
	double kg, double kv) {
	this->kg = kg;
	this->kv = kv;
}

double LongitudinalController::compute_safe_time_headway(
	double free_flow_velocity, double follower_max_brake,
	double leader_max_brake, double lambda_1, double rho,
	double accepted_risk) {

	return compute_time_headway_with_risk(free_flow_velocity,
		follower_max_brake, leader_max_brake, lambda_1,
		rho, 0.0);
}

double LongitudinalController::compute_time_headway_with_risk(double free_flow_velocity,
	double follower_max_brake, double leader_max_brake,
	double lambda_1, double rho, double accepted_risk) {

	double time_headway;
	double gamma = leader_max_brake / follower_max_brake;
	double gamma_threshold = (1 - rho) * free_flow_velocity
		/ (free_flow_velocity + lambda_1);
	double risk_term = std::pow(accepted_risk, 2) / 2 / free_flow_velocity;

	if (gamma < gamma_threshold) {
		// case where ego brakes harder
		time_headway =
			(std::pow(rho, 2) * free_flow_velocity / 2
				+ rho * lambda_1 - risk_term)
			/ ((1 - gamma) * follower_max_brake);
	}
	else if (gamma >= std::pow(1 - rho, 2)) {
		time_headway =
			((1 - std::pow(1 - rho, 2) / gamma) * free_flow_velocity / 2
				+ lambda_1 - risk_term)
			/ follower_max_brake;
	}
	else {
		time_headway = (lambda_1 - risk_term) / follower_max_brake;
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