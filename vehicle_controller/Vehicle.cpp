#include <iomanip>
//#include <iostream>

#include "Vehicle.h"

Vehicle::Vehicle(long id) : id{ id } {}

Vehicle::Vehicle(long id, VehicleType type, double brake_delay) :
	id{ id }, type{ type }, brake_delay{ brake_delay } 
{
	set_category(static_cast<int>(type) / 100);
}

void Vehicle::set_category(long category) 
{
	/* We only need to set the category once, but VISSIM passes the
	category every time step. */
	if (this->category == VehicleCategory::undefined) {
		this->category = VehicleCategory(category);
		switch (this->category) {
		case VehicleCategory::truck:
			this->max_brake = TRUCK_MAX_BRAKE;
			this->max_jerk = TRUCK_MAX_JERK;
			break;
		case VehicleCategory::car:
			this->max_brake = CAR_MAX_BRAKE;
			this->max_jerk = CAR_MAX_JERK;
			break;
		default: // shouldn't happen but assume car if any other category
			this->max_brake = CAR_MAX_BRAKE;
			this->max_jerk = CAR_MAX_JERK;
			break;
		}
	}
}

/* TODO: we want to avoid methods using VehicleType.
Let's instead deal with this through polymorphism */
//bool Vehicle::is_connected() const {
//	return (type == VehicleType::connected_car 
//		|| type == VehicleType::traffic_light_cacc_car
//		|| type == VehicleType::platoon_car);
//}

bool Vehicle::has_lane_change_intention() const 
{
	return desired_lane_change_direction != RelativeLane::same;
}

double Vehicle::compute_max_risk(double leader_max_brake,
	double follower_max_brake, double desired_velocity, double rho)
{
	double gamma = leader_max_brake / follower_max_brake;
	double max_risk = std::sqrt(
		((1 - std::pow(1 - rho, 2) / gamma) * desired_velocity / 2
			+ get_lambda_1()) * 2 * desired_velocity
	);
	return max_risk;
}

void Vehicle::compute_safe_gap_parameters()
{
	lambda_0 = compute_lambda_0(max_jerk, comfortable_acceleration, 
		max_brake, brake_delay);
	lambda_1 = compute_lambda_1(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
}

double Vehicle::compute_lambda_0(double max_jerk,
	double comf_accel, double max_brake,
	double brake_delay) const
{
	double jerk_time = (comf_accel + max_brake) / max_jerk;
	double result = -(comf_accel + max_brake) 
		* (std::pow(brake_delay, 2) + brake_delay * jerk_time
			+ std::pow(jerk_time, 2) / 3);
	return result;
	//lambda_1 = (comf_accel + max_brake) * (tau_d + tau_j / 2);
}

double Vehicle::compute_lambda_1(double max_jerk,
	double comf_accel, double max_brake,
	double brake_delay) const
{
	double jerk_time = (comf_accel + max_brake) / max_jerk;
	double result = (comf_accel + max_brake) 
		* (brake_delay + jerk_time / 2);
	return result;
}

double Vehicle::compute_time_headway_with_risk(
	double free_flow_velocity,
	double follower_max_brake, double leader_max_brake,
	double lambda_1, double rho, double accepted_risk) const
{
	double time_headway;
	double gamma = leader_max_brake / follower_max_brake;
	double gamma_threshold = (1 - rho) * free_flow_velocity
		/ (free_flow_velocity + lambda_1);
	double risk_term = std::pow(accepted_risk, 2) / 2 / free_flow_velocity;

	if (gamma < gamma_threshold) 
	{
		// case where ego brakes harder
		time_headway =
			(std::pow(rho, 2) * free_flow_velocity / 2
				+ rho * lambda_1 - risk_term)
			/ ((1 - gamma) * follower_max_brake);
	}
	else if (gamma >= std::pow(1 - rho, 2)) 
	{
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