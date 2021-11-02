#pragma once

#include "VariationLimitedFilter.h"

class VelocityController
{
public:
	VelocityController() = default;
	VelocityController(double simulation_time_step, 
		double comfortable_acceleration,
		double filter_brake_limit);

	void reset_filter(double ego_velocity);
	void reset_error_integrator();

	double compute_input(double velocity_error, double acceleration_error,
		double comfortable_acceleration);

	VariationLimitedFilter desired_velocity_filter;

private:
	double simulation_time_step;
	/* Gains
	(computed in Matlab. Should be computed here?) */
	double ki{ 0.03 }; //0.07
	double kp{ 0.5 };
	double kd{ 0.1 }; // 0.1

	double error_integral{ 0.0 };
};

