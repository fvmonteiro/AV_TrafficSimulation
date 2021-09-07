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

LongitudinalController::LongitudinalController(
	const VehicleParameters& ego_parameters,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double max_brake, double filter_brake_limit, bool verbose)
	: simulation_time_step{ ego_parameters.sampling_interval },
	velocity_controller_gains{ velocity_controller_gains },
	autonomous_gains{ autonomous_gains },
	connected_gains{ connected_gains },
	ego_max_brake{ max_brake },
	free_flow_velocity{ ego_parameters.desired_velocity },
	verbose{ verbose } {

	double comfortable_acceleration =
		ego_parameters.comfortable_acceleration;
	this->leader_velocity_filter = VelocityFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step);
	this->desired_velocity_filter = VelocityFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step);

	if (verbose) {
		std::clog << "Created base longitudinal controller with "
			<< "max brake=" << ego_max_brake
			<< ", free flow velocity=" << free_flow_velocity
			<< ", autonomous kg=" << this->autonomous_gains.kg
			<< ", connected kg=" << this->connected_gains.kg
			<< std::endl;
	}
}

//void LongitudinalController::set_vehicle_following_gains(
//	AutonomousGains gains) {
//	this->autonomous_gains = gains;
//}
//
//void LongitudinalController::set_vehicle_following_gains(
//	ConnectedGains gains) {
//	this->connected_gains = gains;
//}
//
//void LongitudinalController::set_velocity_controller_gains(
//	VelocityControllerGains gains) {
//	this->velocity_controller_gains = gains;
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

double LongitudinalController::compute_gap_error(
	double gap, double reference_gap) {
	if (is_connected) {
		return std::min(max_gap_error_connected, gap - reference_gap);
	}
	else {
		return std::min(max_gap_error, gap - reference_gap);
	}
	//return gap - reference_gap;
};

double LongitudinalController::compute_velocity_error(double velocity_ego, 
	double velocity_reference) {
	return velocity_reference - velocity_ego;
}

double LongitudinalController::estimate_gap_error_derivative(
	double velocity_error, double acceleration) {
	return velocity_error - h * acceleration;
}

double LongitudinalController::compute_acceleration_error(
	double acceleration_ego, double acceleration_reference) {
	return acceleration_reference - acceleration_ego;
}

double LongitudinalController::compute_gap_threshold(double free_flow_velocity, 
	double velocity_error) {
	/* Threshold is computed such that, at the switch, the vehicle following 
	input is greater or equal to kg*h*(Vf - v) > 0. */
	double kg = autonomous_gains.kg;
	double kv = autonomous_gains.kv;
	return h * free_flow_velocity + d - 1 / kg * (kv * velocity_error);
	/* Other threshold options: 
	- VISSIM's maximum gap: 250 
	- Worst-case: h*vf + d + (kv*vf)/kg = (h + kv/kg)*vf + d*/
}

double LongitudinalController::compute_gap_threshold(double free_flow_velocity,
	double velocity_error, double gap_error_derivative, 
	double acceleration_error) {
	/* Threshold is computed such that, at the switch, the vehicle following
	input is greater or equal to kg*h*(Vf - v) > 0. */
	double kg = connected_gains.kg;
	double kv = connected_gains.kv;
	double kgd = connected_gains.kgd;
	double ka = connected_gains.ka;
	return h * free_flow_velocity + d - 1 / kg * (kv * velocity_error 
		+ kgd * gap_error_derivative + ka * acceleration_error);
	/* Other threshold options:
	- VISSIM's maximum gap: 250
	- Worst-case: */
}

double LongitudinalController::compute_vehicle_following_input(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& leader) {

	double ego_velocity = ego_vehicle.get_velocity();
	double gap = ego_vehicle.compute_gap(leader);
	double gap_reference = compute_desired_gap(ego_velocity);
	double gap_error = compute_gap_error(gap, gap_reference);
	double velocity_reference = leader.compute_velocity(ego_velocity);
	double filtered_velocity_reference =
		leader_velocity_filter.filter_velocity(velocity_reference);
	double velocity_error = compute_velocity_error(
		ego_velocity, filtered_velocity_reference);

	if (verbose) {
		std::clog << "\tleader id = " << leader.get_id()
			<< ", eg=" << gap - gap_reference
			<< ", sat(eg)=" << gap_error
			<< ", ev=" << velocity_error;
	}

	double desired_acceleration;
	if (is_connected) {
		double ego_acceleration = ego_vehicle.get_acceleration();
		double gap_error_derivative = estimate_gap_error_derivative(
			velocity_error, ego_acceleration);
		double acceleration_error = compute_acceleration_error(
			ego_acceleration, leader.get_acceleration());
		desired_acceleration = connected_gains.kg * gap_error
			+ connected_gains.kv * velocity_error
			+ connected_gains.kgd * gap_error_derivative
			+ connected_gains.ka * acceleration_error;

		if (verbose) {
			std::clog << ", eg_dot=" << gap_error_derivative
				<< ", ea=" << acceleration_error
				<< std::endl;
		}

	}
	else {
		desired_acceleration = autonomous_gains.kg * gap_error
			+ autonomous_gains.kv * velocity_error;

		if (verbose) {
			std::clog << std::endl;
		}

	}

	return desired_acceleration;
}

double LongitudinalController::compute_vehicle_following_input(
	double gap_error, double velocity_error) {
	return autonomous_gains.kg * gap_error 
		+ autonomous_gains.kv * velocity_error;
}

double LongitudinalController::compute_vehicle_following_input(
	double gap_error, double velocity_error, double gap_error_derivative,
	double acceleration_error) {
	return connected_gains.kg * gap_error 
		+ connected_gains.kv * velocity_error 
		+ connected_gains.kgd * gap_error_derivative 
		+ connected_gains.ka * acceleration_error;
}

/* PID velocity controller. The derivative of the velocity error equals the 
ego vehicle acceleration since the desired speed is constant. */
double LongitudinalController::compute_velocity_control_input(
	const EgoVehicle& ego_vehicle, double velocity_reference) {
	
	double ego_velocity = ego_vehicle.get_velocity();
	double ego_acceleration = ego_vehicle.get_acceleration();
	//double velocity_reference = ego_reference_velocity;
	double filtered_velocity_reference =
		desired_velocity_filter.filter_velocity(velocity_reference);
	double velocity_error = compute_velocity_error(
		ego_velocity, filtered_velocity_reference);
	double acceleration_error = compute_acceleration_error(
		ego_acceleration, 0);

	/* We avoid integral windup by only deactivating the integral gain when the
	input is 'large' */
	double comfortable_acceleration = 
		ego_vehicle.get_comfortable_acceleration();
	velocity_error_integral += velocity_error * simulation_time_step;
	double desired_acceleration = 
		velocity_controller_gains.kp * velocity_error 
		+ velocity_controller_gains.kd * acceleration_error
		+ velocity_controller_gains.ki * velocity_error_integral;
	if (desired_acceleration > comfortable_acceleration / 2) {
		desired_acceleration -= 
			velocity_controller_gains.ki * velocity_error_integral;
		reset_velocity_error_integrator();
	}

	if (verbose) {
		std::clog << "\tref=" << velocity_reference
			<< ", filtered=" << filtered_velocity_reference
			<< ", vel=" << ego_velocity
			<< ", ev=" << velocity_error
			<< ", ea=" << acceleration_error
			<< std::endl;
	}

	return desired_acceleration;
}
//double LongitudinalController::compute_velocity_control_input(
//	double velocity_error, double acceleration_error, 
//	double comfortable_acceleration) {
//	/* We avoid integral windup by only deactivating the integral gain when the
//	input is 'large' */
//	velocity_error_integral += velocity_error * simulation_time_step;
//	double u = kp * velocity_error + kd * acceleration_error 
//		+ ki * velocity_error_integral;
//	if (u > comfortable_acceleration / 2) {
//		u -= ki * velocity_error_integral;
//		reset_velocity_error_integrator();
//	}
//	
//	return u;
//}

double LongitudinalController::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference) {
	
	double desired_acceleration;
	State old_state = state;

	determine_controller_state(ego_vehicle, leader, velocity_reference);

	switch (state)
	{
	case State::uninitialized:
		desired_acceleration = 0;
		break;
	case State::vehicle_following:
	{
		desired_acceleration = compute_vehicle_following_input(
			ego_vehicle, *leader);
		break;
	}
	case State::velocity_control:
	{
		if (old_state != State::velocity_control) {
			double ego_velocity = ego_vehicle.get_velocity();
			/* The smooth velocity leads the vel. controller to output a 
			desired acceleration close to the previous step desired
			acceleration. */
			double smooth_velocity = (ego_vehicle.get_desired_acceleration()
				- velocity_controller_gains.kd * ego_vehicle.get_acceleration())
				/ velocity_controller_gains.kp + ego_velocity;
			/* We use the smooth velocity only when that helps the vehicle
			achieve the velocity reference faster. This happens either when
			the ego vel is lesser than both reference and smooth vel or when
			the ego vel is greater than both reference and smooth vel. */
			double reset_velocity;
			if ((ego_velocity < velocity_reference) 
				 == (ego_velocity < smooth_velocity)) {
				reset_velocity = smooth_velocity;
			}
			else {
				reset_velocity = ego_velocity;
			}
			desired_velocity_filter.reset(reset_velocity);
			reset_velocity_error_integrator();
		}

		desired_acceleration = compute_velocity_control_input(ego_vehicle,
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

void LongitudinalController::reset_velocity_error_integrator() {
	velocity_error_integral = 0;
}

void LongitudinalController::reset_accepted_risks() {
	accepted_risk_to_leader = initial_risk;
}

void LongitudinalController::compute_max_risk_to_leader() {
	max_risk_to_leader = std::sqrt(
		2 * h * ego_max_brake * free_flow_velocity);
	if (verbose) std::clog << "max risk to leader=" 
		<< max_risk_to_leader << std::endl;
}

void LongitudinalController::update_safe_time_headway(
	double lambda_1, double new_leader_max_brake) {
	h = compute_time_headway_with_risk(free_flow_velocity,
		ego_max_brake, new_leader_max_brake,
		lambda_1, rho, 0);
}

void LongitudinalController::update_time_headway(
	double lambda_1, double new_leader_max_brake) {
	
	if (verbose) {
		std::clog << "Updating time headway from " << h;
	}
	
	h = compute_time_headway_with_risk(free_flow_velocity,
		ego_max_brake, new_leader_max_brake,
		lambda_1, rho, accepted_risk_to_leader);
	
	if (verbose) std::clog << " to " << h << std::endl;
}

//void LongitudinalController::update_time_headway_with_new_risk(
//	double lambda_1) {
//	if (verbose) std::clog << "New risk. Updating time headway from " << h;
//
//	h = compute_time_headway_with_risk(free_flow_velocity,
//		ego_max_brake, leader_max_brake,
//		lambda_1, rho, accepted_risk_to_leader);
//	
//	if (verbose) std::clog << " to " << h << std::endl;
//}

void LongitudinalController::reset_leader_velocity_filter(
	double ego_velocity) {
	leader_velocity_filter.reset(ego_velocity);
}

void LongitudinalController::reset_desired_velocity_filter(
	double ego_velocity) {
	desired_velocity_filter.reset(ego_velocity);
}

/* Protected and private methods ------------------------------------------ */

//double LongitudinalController::compute_safe_time_headway(
//	double free_flow_velocity, double follower_max_brake,
//	double leader_max_brake, double lambda_1, double rho) {
//
//	return compute_time_headway_with_risk(free_flow_velocity,
//		follower_max_brake, leader_max_brake, lambda_1,
//		rho, 0.0);
//}

double LongitudinalController::compute_time_headway_with_risk(
	double free_flow_velocity,
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