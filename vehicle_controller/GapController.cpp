#include <algorithm>
#include <cmath>
#include <iostream>

#include "GapController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"

GapController::GapController(double simulation_time_step, 
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose)
	: //simulation_time_step{ simulation_time_step },
	autonomous_gains{ autonomous_gains }, 
	connected_gains{ connected_gains },
	velocity_filter{ VariationLimitedFilter(velocity_filter_gain, 
		comfortable_acceleration, filter_brake_limit,
		simulation_time_step) },
	time_headway_filter { VariationLimitedFilter(time_headway_filter_gain,
		100, -100, simulation_time_step)},
	verbose{ verbose } {}

GapController::GapController(double simulation_time_step,
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double comfortable_acceleration, double filter_brake_limit)
	: GapController(simulation_time_step, autonomous_gains, 
		connected_gains, velocity_filter_gain, time_headway_filter_gain,
		comfortable_acceleration, filter_brake_limit, 
		false) {}

//void GapController::compute_time_headway_with_risk(
//	double free_flow_velocity,
//	double follower_max_brake, double leader_max_brake,
//	double lambda_1, double rho, double accepted_risk) {
//
//	double time_headway;
//	double gamma = leader_max_brake / follower_max_brake;
//	double gamma_threshold = (1 - rho) * free_flow_velocity
//		/ (free_flow_velocity + lambda_1);
//	double risk_term = std::pow(accepted_risk, 2) / 2 / free_flow_velocity;
//
//	if (gamma < gamma_threshold) {
//		// case where ego brakes harder
//		time_headway =
//			(std::pow(rho, 2) * free_flow_velocity / 2
//				+ rho * lambda_1 - risk_term)
//			/ ((1 - gamma) * follower_max_brake);
//	}
//	else if (gamma >= std::pow(1 - rho, 2)) {
//		time_headway =
//			((1 - std::pow(1 - rho, 2) / gamma) * free_flow_velocity / 2
//				+ lambda_1 - risk_term)
//			/ follower_max_brake;
//	}
//	else {
//		time_headway = (lambda_1 - risk_term) / follower_max_brake;
//	}
//
//	time_headway = time_headway;
//}

double GapController::get_safe_time_headway() const
{
	return desired_time_headway;
}

double GapController::get_current_time_headway() const
{
	return time_headway_filter.get_current_value();
}

double GapController::compute_time_headway_gap(double time_headway,
	double velocity) const
{
	/* TODO: for now we assumed the standstill distance (d) is the same for
	all cases. */
	return time_headway * velocity + standstill_distance;
}

double GapController::get_safe_time_headway_gap(
	double ego_velocity, bool has_lane_change_intention) const
{
	return compute_time_headway_gap(get_safe_time_headway(), ego_velocity);
}

double GapController::get_desired_gap(double ego_velocity)
{
	return compute_time_headway_gap(time_headway_filter.get_current_value(),
		ego_velocity);
}

double GapController::compute_desired_gap(double velocity_ego)
{
	double desired_time_headway = get_safe_time_headway();
	double time_headway = time_headway_filter.apply_filter(
		desired_time_headway);

	if (verbose) {
		std::clog << "desired h " << desired_time_headway
			<< ", filtered h " << time_headway
			<< std::endl;
	}

	return compute_time_headway_gap(time_headway, velocity_ego);
}

double GapController::compute_gap_error(
	double gap, double reference_gap) const
{
	double upper_limit = is_connected ?
		max_gap_error_connected : max_gap_error;
	return std::min(upper_limit, gap - reference_gap);
};

double GapController::compute_velocity_error(double velocity_ego,
	double velocity_reference) const
{
	return velocity_reference - velocity_ego;
}

double GapController::estimate_gap_error_derivative(
	double velocity_error, double acceleration) const
{
	/* TODO [Oct 29, 2021]: should be changed to e_v - h.a - dh/dt.v */
	double time_headway = get_safe_time_headway();
	return velocity_error - time_headway * acceleration;
}

double GapController::compute_acceleration_error(
	double acceleration_ego, double acceleration_reference) const
{
	return acceleration_reference - acceleration_ego;
}

double GapController::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle, 
	const std::shared_ptr<NearbyVehicle> leader)
{
	if (leader == nullptr)
	{
		return ego_vehicle.get_comfortable_acceleration();
	}
	
	double ego_velocity = ego_vehicle.get_velocity();
	double gap = ego_vehicle.compute_gap(leader);
	double gap_reference = compute_desired_gap(ego_velocity);
	double gap_error = compute_gap_error(gap, gap_reference);
	double velocity_reference = leader->compute_velocity(ego_velocity);
	double filtered_velocity_reference =
		velocity_filter.apply_filter(velocity_reference);
	double velocity_error = compute_velocity_error(
		ego_velocity, filtered_velocity_reference);

	if (verbose) {
		std::clog << "\t[Gap controller]\n\t" 
			<< "leader id = " << leader->get_id()
			<< ", eg=" << gap - gap_reference
			<< ", sat(eg)=" << gap_error
			<< ", ev=" << velocity_error
			<< ", vl=" << velocity_reference
			<< ", vl_hat=" << filtered_velocity_reference;
	}

	double desired_acceleration;
	if (is_connected)
	{
		desired_acceleration = compute_connected_input(gap_error, 
			velocity_error, ego_vehicle.get_acceleration(), 
			leader->get_acceleration());
	}
	else
	{
		desired_acceleration = compute_autonomous_input(gap_error, 
			velocity_error);
	}

	if (verbose) {
		std::clog << std::endl;
	}

	return desired_acceleration;
}

double GapController::compute_autonomous_input(
	double gap_error, double velocity_error)
{
	return autonomous_gains.kg * gap_error 
		+ autonomous_gains.kv * velocity_error;
}

double GapController::compute_connected_input(
	double gap_error, double velocity_error, double ego_acceleration,
	double leader_acceleration) 
{
	double gap_error_derivative = estimate_gap_error_derivative(
		velocity_error, ego_acceleration);
	double acceleration_error = compute_acceleration_error(
		ego_acceleration, leader_acceleration);

	if (verbose) {
		std::clog << ", eg_dot=" << gap_error_derivative
			<< ", ea=" << acceleration_error;
	}

	return connected_gains.kg * gap_error 
		+ connected_gains.kv * velocity_error
		+ connected_gains.kgd * gap_error_derivative 
		+ connected_gains.ka * acceleration_error;
}

//double GapController::compute_gap_threshold(double free_flow_velocity,
//	double velocity_error, double ego_acceleration,
//	double leader_acceleration) const
//{
//	/* Threshold is computed such that, at the switch, the vehicle following
//	input is greater or equal to kg*h*(Vf - v) > 0. */
//	double time_headway = get_safe_time_headway();
//	if (is_connected)
//	{
//		double kg = connected_gains.kg;
//		double kv = connected_gains.kv;
//		double kgd = connected_gains.kgd;
//		double ka = connected_gains.ka;
//		double gap_error_derivative = estimate_gap_error_derivative(
//			velocity_error, ego_acceleration);
//		double acceleration_error = compute_acceleration_error(
//			ego_acceleration, leader_acceleration);
//
//		if (verbose)
//		{
//			std::clog << "\tCopmuting thresh old\n\t"
//				<< "h=" << time_headway
//				<< ", vf=" << free_flow_velocity
//				<< ", (u/kg - eg) =" << 1 / kg * (kv * velocity_error
//					+ kgd * gap_error_derivative + ka * acceleration_error) << std::endl;
//		}
//
//		return time_headway * free_flow_velocity + standstill_distance
//			- 1 / kg * (kv * velocity_error 
//				+ kgd * gap_error_derivative + ka * acceleration_error);
//	}
//	else
//	{
//		double kg = autonomous_gains.kg;
//		double kv = autonomous_gains.kv;
//		return time_headway * free_flow_velocity + standstill_distance
//			- 1 / kg * (kv * velocity_error);
//	}
//	/* Other threshold options:
//	- VISSIM's maximum gap: 250
//	- Worst-case: h*vf + d + (kv*vf)/kg = (h + kv/kg)*vf + d*/
//}

void GapController::reset_time_headway_filter(double time_headway)
{
	time_headway_filter.reset(time_headway);
}

void GapController::reset_velocity_filter(
	double ego_velocity) 
{
	velocity_filter.reset(ego_velocity);
}

void GapController::update_leader_velocity_filter(
	double leader_velocity)
{
	velocity_filter.apply_filter(leader_velocity);
}