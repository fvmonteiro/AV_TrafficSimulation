#include <algorithm>
#include <cmath>

#include "VehicleFollowingController.h"

VehicleFollowingController::VehicleFollowingController(
	double simulation_time_step, double kg, double kv, 
	double comfortable_acceleration, double filter_brake_limit)
	: simulation_time_step{ simulation_time_step },
	kg{ kg }, kv{ kv }, is_connected{ false },
	leader_velocity_filter{ VariationLimitedFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step) } {}

VehicleFollowingController::VehicleFollowingController(
	double simulation_time_step, double kg, double kv,
	double kd, double ka, double comfortable_acceleration,
	double filter_brake_limit)
	: simulation_time_step{ simulation_time_step },
	kg{ kg }, kv{ kv }, kd{ kd }, ka{ ka }, is_connected{ true },
	leader_velocity_filter{ VariationLimitedFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step) } {}

void VehicleFollowingController::compute_time_headway_with_risk(
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

	h = time_headway;
}

double VehicleFollowingController::compute_time_headway_gap(double time_headway,
	double velocity) {
	/* TODO: for now we assumed the standstill distance (d) is the same for
	all cases. */
	return time_headway * velocity + d;
}

double VehicleFollowingController::compute_gap_error(
	double gap, double reference_gap) {
	return std::min(max_gap_error, gap - reference_gap);
};

double VehicleFollowingController::compute_velocity_error(double velocity_ego,
	double velocity_reference) {
	return velocity_reference - velocity_ego;
}

double VehicleFollowingController::estimate_gap_error_derivative(
	double velocity_error, double acceleration) {
	return velocity_error - h * acceleration;
}

double VehicleFollowingController::compute_acceleration_error(
	double acceleration_ego, double acceleration_reference) {
	return acceleration_reference - acceleration_ego;
}

double VehicleFollowingController::compute_input(
	double gap, double ego_velocity, double leader_velocity) {
	double gap_reference = compute_time_headway_gap(h, ego_velocity);
	double gap_error = compute_gap_error(gap, gap_reference);
	double velocity_reference = leader_velocity_filter.apply_filter(
		leader_velocity);
	double velocity_error = compute_velocity_error(ego_velocity, 
		velocity_reference);
	return kg * gap_error + kv * velocity_error;
}

//double VehicleFollowingController::compute_input(
//	double gap_error, double velocity_error, double gap_error_derivative,
//	double acceleration_error) {
//	return kg * gap_error + kv * velocity_error
//		+ kd * gap_error_derivative + ka * acceleration_error;
//}

double VehicleFollowingController::compute_gap_threshold(
	double free_flow_velocity, double ego_velocity, double leader_velocity) {
	/* Threshold is computed such that, at the switch, the vehicle following
	input is greater or equal to kg*h*(Vf - v) > 0. */
	return h * free_flow_velocity + d 
		- (kv * compute_velocity_error(ego_velocity, leader_velocity)) / kg;
	/* Other threshold options:
	- VISSIM's maximum gap: 250
	- Worst-case: h*vf + d + (kv*vf)/kg = (h + kv/kg)*vf + d*/
}

void VehicleFollowingController::reset_leader_velocity_filter(
	double ego_velocity) {
	leader_velocity_filter.reset(ego_velocity);
}