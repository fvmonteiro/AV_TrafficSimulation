#include "EgoVehicle.h"
#include "VelocityController.h"

VelocityController::VelocityController(double simulation_time_step,
	const VelocityControllerGains& gains, double filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose) :
	simulation_time_step{ simulation_time_step }, gains{ gains },
	desired_velocity_filter{ VariationLimitedFilter(filter_gain, 
		comfortable_acceleration, filter_brake_limit, simulation_time_step)},
	verbose{ verbose } {}

VelocityController::VelocityController(double simulation_time_step,
	const VelocityControllerGains& gains, double filter_gain,
	double comfortable_acceleration, double filter_brake_limit) :
	VelocityController(simulation_time_step, gains, filter_gain,
		comfortable_acceleration, filter_brake_limit, false) {}

void VelocityController::reset_filter(double ego_velocity) 
{
	desired_velocity_filter.reset(ego_velocity);
}

void VelocityController::reset_error_integrator() 
{
	error_integral = 0;
}

void VelocityController::smooth_reset(const EgoVehicle& ego_vehicle,
	double velocity_reference)
{
	double ego_velocity = ego_vehicle.get_velocity();
	/* The smooth velocity leads the vel. controller to output a
	desired acceleration close to the desired acceleration in
	the previous step. */
	double smooth_velocity = 
		(ego_vehicle.get_desired_acceleration()
		- gains.kd * ego_vehicle.get_acceleration()) 
		/ gains.kp + ego_velocity;
	/* We use the smooth velocity only when that helps the vehicle
	achieve the velocity reference faster. This happens either when:
	the ego vel is already lesser (or greater) than both reference
	and smooth vel */
	double reset_velocity;
	if ((ego_velocity < velocity_reference)
		== (ego_velocity < smooth_velocity)) {
		reset_velocity = smooth_velocity;
	}
	else {
		reset_velocity = ego_velocity;
	}
	reset_filter(reset_velocity);
	reset_error_integrator();
}

double VelocityController::compute_acceleration(const EgoVehicle& ego_vehicle,
	double velocity_reference) 
{
	double ego_velocity = ego_vehicle.get_velocity();
	double ego_acceleration = ego_vehicle.get_acceleration();
	double filtered_velocity_reference =
		desired_velocity_filter.apply_filter(velocity_reference);
	double velocity_error = filtered_velocity_reference - ego_velocity;
	double acceleration_error = -ego_acceleration;

	/* We avoid integral windup by only deactivating the integral gain when the
	input is 'large' */
	double comfortable_acceleration =
		ego_vehicle.get_comfortable_acceleration();
	error_integral += velocity_error * simulation_time_step;
	double desired_acceleration =
		gains.kp * velocity_error
		+ gains.kd * acceleration_error
		+ gains.ki * error_integral;
	if (desired_acceleration > comfortable_acceleration / 2) 
	{
		desired_acceleration -= gains.ki * error_integral;
		reset_error_integrator();
	}

	if (verbose) 
	{
		std::clog << "\tref=" << velocity_reference
			<< ", filtered=" << filtered_velocity_reference
			<< ", vel=" << ego_velocity
			<< ", ev=" << velocity_error
			<< ", ea=" << acceleration_error
			<< std::endl;
	}

	return desired_acceleration;
}