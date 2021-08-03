#include <iomanip>
//#include <iostream>

#include "Vehicle.h"

void Vehicle::set_type(VehicleType type) {
	// we only need to set the type once
	switch (this->type) {
	case VehicleType::undefined:
		this->type = type;
		break;
	default: // no changes
		break;
	}
}

void Vehicle::set_type(long type) {
	// we only need to set the type once
	switch (this->type) {
	case VehicleType::undefined:
		set_type(VehicleType(type));
		break;
	default: // no changes
		break;
	}
}

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