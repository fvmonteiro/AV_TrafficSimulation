#include <iomanip>
//#include <iostream>

#include "Vehicle.h"

void Vehicle::set_category(long category) {
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

bool Vehicle::is_connected() const {
	return type == VehicleType::connected_car;
}

bool Vehicle::has_lane_change_intention() const {
	return desired_lane_change_direction != RelativeLane::same;
}

//RelativeLane Vehicle::get_opposite_relative_lane(
//	const RelativeLane& relative_lane) const {
//	switch (relative_lane)
//	{
//	case RelativeLane::right_right:
//		return RelativeLane::left_left;
//	case RelativeLane::right:
//		return RelativeLane::left;
//	case RelativeLane::same:
//		return RelativeLane::same;
//	case RelativeLane::left:
//		return RelativeLane::right;
//	case RelativeLane::left_left:
//		return RelativeLane::right_right;
//	default:
//		break;
//	}
//	return RelativeLane::same;
//}

double Vehicle::compute_lambda_0(double max_jerk,
	double comf_accel, double max_brake,
	double brake_delay) {
	double jerk_time = (comf_accel + max_brake) / max_jerk;
	double result = -(comf_accel + max_brake) 
		* (std::pow(brake_delay, 2) + brake_delay * jerk_time
			+ std::pow(jerk_time, 2) / 3);
	return result;
	//lambda_1 = (comf_accel + max_brake) * (tau_d + tau_j / 2);
}

double Vehicle::compute_lambda_1(double max_jerk,
	double comf_accel, double max_brake,
	double brake_delay) {
	double jerk_time = (comf_accel + max_brake) / max_jerk;
	double result = (comf_accel + max_brake) 
		* (brake_delay + jerk_time / 2);
	return result;
}