/*==========================================================================*/
/*  VelocityFilter.cpp    													*/
/* Implements an acceleration limiter to filter the desired velocity (either*/
/* the free flow or the leading vehicle's) to smooth the control input		*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <cmath>
#include <iostream>

#include "VariationLimitedFilter.h"

//VelocityFilter::VelocityFilter(double time_step) {
//	this->time_step = time_step;
//}

VariationLimitedFilter::VariationLimitedFilter(double max_variation,
	double min_variation, double time_step, bool verbose) :
	max_variation_per_second{ max_variation },
	min_variation_per_second{ -std::abs(min_variation) }, /* minimum
	acceleration (maximum braking) is sometimes given in absolute value,
	so we have to make sure to get a negative value here*/
	time_step{ time_step },
	verbose{ verbose } {

	if (verbose) {
		std::clog << "Creating velocity filter with "
			<< "max acceleration = " << max_variation
			<< "; min acceleration = " << -std::abs(min_variation)
			<< "; time step = " << time_step
			<< std::endl;
	}

	this->max_variation_per_time_step = this->max_variation_per_second 
		* this->time_step;
	this->min_variation_per_time_step = this->min_variation_per_second 
		* this->time_step;
}

VariationLimitedFilter::VariationLimitedFilter(double max_variation,
	double min_variation, double time_step) 
	: VariationLimitedFilter(max_variation,
		min_variation, time_step, false) {}

void VariationLimitedFilter::reset(double initial_value) {
	/*if (verbose) {
		std::clog << "------- Filter reset. Init value = "
			<< initial_value << " -------" << std::endl;
	}*/
	if (!is_initialized) {
		is_initialized = true;
		compute_alpha();
	}
	this->current_value = initial_value;
}

void VariationLimitedFilter::set_gain(double new_gain) {
	this->gain = new_gain;
	compute_alpha();
}

double VariationLimitedFilter::apply_filter(double new_value) {
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
	double variation = new_value - current_value;
	if ((1 - alpha) * variation > max_variation_per_time_step) {
		filtered_variation = max_variation_per_time_step;
	}
	else if ((1 - alpha) * variation < min_variation_per_time_step) {
		filtered_variation = min_variation_per_time_step;
	}
	else {
		filtered_variation = (1 - alpha) * variation;
	}
	double filtered_value = current_value + filtered_variation;

	/*if (verbose) {
		std::clog << "[f] previous value = " << previous_value
			<< "; variation = " << variation << "; gain  = " << gain 
			<< "; time_step = " << time_step << "; alpha = " 
			<< alpha << "; saturated variation = " << filtered_variation 
			<< "; new_value = " << reference_velocity << std::endl;
	}*/

	current_value = filtered_value;
	return filtered_value;
}

void VariationLimitedFilter::compute_alpha() {
	alpha = std::exp(-gain * time_step);
}
