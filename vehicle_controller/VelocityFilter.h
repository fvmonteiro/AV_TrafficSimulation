/*==========================================================================*/
/*  VelocityFilter.h    													*/
/* Implements an acceleration limiter to filter the desired velocity (either*/ 
/* the free flow or the leading vehicle's) to smooth the control input		*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

class VelocityFilter {
public:
	VelocityFilter() = default;
	VelocityFilter(double max_acceleration, double min_acceleration,
		double time_step, bool verbose);
	VelocityFilter(double max_acceleration, double min_acceleration, 
		double time_step);

	double get_current_value() const { return current_value; };
	
	void reset(double initial_value);
	double filter_velocity(double new_velocity);

private:
	double time_step{ 0.1 };
	double current_value{ -1 }; /* negative value can be used as a flag for 
								uninitialized filter */

	double max_acceleration{ 3.0 }; // [m/s^2]
	double min_acceleration{ -8.0 }; // [m/s^2]
	double max_variation_per_time_step; // maximum acceleration per time step
	double min_variation_per_time_step; // minimum acceleration per time step
	double gain{ 10.0 };
	double alpha{ 0.0 }; /* constant used in the discrete approximation of
						 the first order actuator dynamics */
	bool verbose = false;
};