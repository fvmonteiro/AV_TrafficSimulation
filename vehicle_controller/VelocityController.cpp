#include "VelocityController.h"

VelocityController::VelocityController(double simulation_time_step,
	double comfortable_acceleration,
	double filter_brake_limit) :
	simulation_time_step{ simulation_time_step },
	desired_velocity_filter{ VariationLimitedFilter(comfortable_acceleration,
		filter_brake_limit, simulation_time_step)} {}

void VelocityController::reset_filter(double ego_velocity) {
	desired_velocity_filter.reset(ego_velocity);
}

void VelocityController::reset_error_integrator() {
	error_integral = 0;
}

double VelocityController::compute_input(double velocity_error, 
	double acceleration_error,
	double comfortable_acceleration) {
	/* We avoid integral windup by only deactivating the integral gain when the
	input is 'large' */
	error_integral += velocity_error * simulation_time_step;
	double u = kp * velocity_error + kd * acceleration_error
		+ ki * error_integral;
	if (u > comfortable_acceleration / 2) {
		u -= ki * error_integral;
		reset_error_integrator();
	}

	return u;
}