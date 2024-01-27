#include <iomanip>
//#include <iostream>

#include "Vehicle.h"

bool operator== (const Vehicle& vehicle1, const Vehicle& vehicle2)
{
	return vehicle1.get_id() == vehicle2.get_id();
}

bool operator!= (const Vehicle& vehicle1, const Vehicle& vehicle2)
{
	return !(vehicle1 == vehicle2);
}

Vehicle::Vehicle(long id) : id{ id } {}

Vehicle::Vehicle(long id, VehicleType type, double brake_delay) :
	id{ id }, type{ type }, brake_delay{ brake_delay } 
{
	set_category(static_cast<int>(type) / 100);
}

void Vehicle::set_category(long category) 
{
	set_category(VehicleCategory(category));
}

void Vehicle::set_category(VehicleCategory category)
{
	/* We only need to set the category once, but VISSIM passes the
	category every time step. */
	if (this->category == VehicleCategory::undefined)
	{
		this->category = category;
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

bool Vehicle::is_a_connected_type(VehicleType vehicle_type) const 
{
	return (vehicle_type == VehicleType::connected_car
		|| vehicle_type == VehicleType::traffic_light_calc_car
		|| vehicle_type == VehicleType::platoon_car
		|| vehicle_type == VehicleType::no_lane_change_connected_car);
}

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
	double a_lambda_1, double rho, double accepted_risk) const
{
	double time_headway;
	double gamma = leader_max_brake / follower_max_brake;
	double gamma_threshold = (1 - rho) * free_flow_velocity
		/ (free_flow_velocity + a_lambda_1);
	double risk_term = std::pow(accepted_risk, 2) / 2 / free_flow_velocity;

	if (gamma < gamma_threshold) 
	{
		// case where ego brakes harder
		time_headway =
			(std::pow(rho, 2) * free_flow_velocity / 2
				+ rho * a_lambda_1 - risk_term)
			/ ((1 - gamma) * follower_max_brake);
	}
	else if (gamma >= std::pow(1 - rho, 2)) 
	{
		time_headway =
			((1 - std::pow(1 - rho, 2) / gamma) * free_flow_velocity / 2
				+ a_lambda_1 - risk_term)
			/ follower_max_brake;
	}
	else {
		time_headway = (a_lambda_1 - risk_term) / follower_max_brake;
	}

	std::clog << "Computing some h."
		<< "\n\td_l=" << leader_max_brake << ", d_f=" << follower_max_brake
		<< "\n\tV_f=" << free_flow_velocity << ", rho=" << rho
		<< ", lambda1=" << a_lambda_1
		<< "\n\tgamma=" << gamma << ", Gamma=" << gamma_threshold
		<< "\n\t\th=" << time_headway << "\n";
	return time_headway;
}

double Vehicle::compute_risky_gap(double v_follower,
	double v_leader, double brake_follower, double brake_leader,
	double lambda_0, double lambda_1, double accepted_risk) const
{
	double accepted_risk_2 = std::pow(accepted_risk, 2);
	double stop_time_follower = (v_follower + lambda_1)
		/ brake_follower;
	double stop_time_leader = v_leader / brake_leader;
	double accepted_gap;
	if (stop_time_follower >= stop_time_leader)
	{
		accepted_gap =
			(std::pow(v_follower + lambda_1, 2)
				- accepted_risk_2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader
			+ lambda_0;
	}
	else if (brake_follower > brake_leader)
	{
		accepted_gap =
			(std::pow(v_follower - v_leader + lambda_1, 2)
				- accepted_risk_2) / 2 / (brake_follower - brake_leader)
			+ lambda_0;
	}
	else
	{
		accepted_gap = 0.0;
	}
	return accepted_gap;
}
