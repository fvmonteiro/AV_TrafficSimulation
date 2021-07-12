/*==========================================================================*/
/*  VelocityFilter.cpp    													*/
/* Implements an acceleration limiter to filter the desired velocity (either*/
/* the free flow or the leading vehicle's) to smooth the control input		*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <cmath>
#include <iostream>

#include "VelocityFilter.h"

//VelocityFilter::VelocityFilter(double time_step) {
//	this->time_step = time_step;
//}

VelocityFilter::VelocityFilter(double max_acceleration, 
	double min_acceleration, double time_step, bool verbose) {

	if (verbose) {
		std::clog << "Creating velocity filter with "
			<< "max acceleration = " << max_acceleration
			<< "; min acceleration = " << -std::abs(min_acceleration)
			<< "; time step = " << time_step
			<< std::endl;
	}

	this->verbose = verbose;
	this->time_step = time_step;
	this->max_acceleration = max_acceleration;
	this->min_acceleration = -std::abs(min_acceleration); /* minimum 
	acceleration (maximum braking) is sometimes given in absolute value,
	so we have to make sure to get a negative value here*/
	this->max_variation_per_time_step = this->max_acceleration 
		* this->time_step;
	this->min_variation_per_time_step = this->min_acceleration 
		* this->time_step;
}

VelocityFilter::VelocityFilter(double max_acceleration,
	double min_acceleration, double time_step) 
	: VelocityFilter(max_acceleration,
		min_acceleration, time_step, false) {}

void VelocityFilter::reset(double initial_value) {
	/*if (verbose) {
		std::clog << "------- Filter reset. Init value = "
			<< initial_value << " -------" << std::endl;
	}*/
	this->previous_value = initial_value;
}

double VelocityFilter::filter_velocity(double new_velocity) {
	/* Filter equations:
	v_f is the filtered velocity, v_r is the reference velocity
	Continuous:
	dot{v_f} = a_max,		 if p(v_r - v_f) > a_max
			 = a_min,		 if p(v_r - v_f) < a_min
			 = p(v_r - v_f), otherwise
	Discrete approximation:
	v_f(k+1) = v_f(k) + delta*a_max,
						if (1-alpha)(v_r(k+1) - v_f(k)) > a_max*delta
			 = v_f(k) + delta*a_min,
						if (1-alpha)(v_r(k+1) - v_f(k)) < a_min*delta
			 = v_f(k) + (1 - alpha)*(v_r(k+1) - v_f(k)), 
						otherwise
	where alpha = exp(-p*delta), and delta is the sampling interval
	*/

	double filtered_variation;
	double alpha = std::exp(-gain * time_step);
	double variation = new_velocity - previous_value;
	if ((1 - alpha) * variation > max_variation_per_time_step) {
		filtered_variation = max_variation_per_time_step;
	}
	else if ((1 - alpha) * variation < min_variation_per_time_step) {
		filtered_variation = min_variation_per_time_step;
	}
	else {
		filtered_variation = (1 - alpha) * variation;
	}
	double reference_velocity = previous_value + filtered_variation;

	/*if (verbose) {
		std::clog << "[f] previous value = " << previous_value
			<< "; variation = " << variation << "; gain  = " << gain 
			<< "; time_step = " << time_step << "; alpha = " 
			<< alpha << "; saturated variation = " << filtered_variation 
			<< "; new_value = " << reference_velocity << std::endl;
	}*/

	previous_value = reference_velocity;
	return reference_velocity;
}