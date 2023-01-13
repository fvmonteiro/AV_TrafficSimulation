#pragma once

#include "VariationLimitedFilter.h"

// Forward declaration
class EgoVehicle;

struct VelocityControllerGains {
	double kp{ 0.0 };
	double kd{ 0.0 };
	double ki{ 0.0 };
};

class VelocityController
{
public:
	VelocityController() = default;
	VelocityController(double simulation_time_step,
		const VelocityControllerGains& gains,
		double filter_gain,
		double comfortable_acceleration,
		double filter_brake_limit, bool verbose);
	VelocityController(double simulation_time_step,
		const VelocityControllerGains& gains,
		double filter_gain,
		double comfortable_acceleration,
		double filter_brake_limit);

	double get_reference_value() const { 
		return desired_velocity_filter.get_current_value(); 
	};

	void reset_filter(double reset_velocity);
	void reset_error_integrator();

	/* Chooses an initial value for the velocity filter that leads to a
	desired acceleration close to the one at the previous step */
	void smooth_reset(const EgoVehicle& ego_vehicle,
		double velocity_reference);
	double compute_acceleration(const EgoVehicle& ego_vehicle,
		double velocity_reference);

private:
	VariationLimitedFilter desired_velocity_filter;

	double simulation_time_step{ 0.1 };
	/* Gains */
	VelocityControllerGains gains;
	//double ki{ 0.03 }; //0.07
	//double kp{ 0.5 };
	//double kd{ 0.1 }; // 0.1

	double error_integral{ 0.0 };

	bool verbose{ false };
};