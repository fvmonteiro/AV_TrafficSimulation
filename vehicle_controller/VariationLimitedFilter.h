/*==========================================================================*/
/*  VelocityFilter.h    													*/
/* Implements a first order filter with the option of limiting the maximum  */
/* variation to smooth the control input									*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

class VariationLimitedFilter {
public:
	VariationLimitedFilter() = default;
	VariationLimitedFilter(double max_variation, double min_variation,
		double time_step, bool verbose);
	VariationLimitedFilter(double max_variation, double min_variation, 
		double time_step);

	double get_current_value() const { return current_value; };
	bool get_is_initialized() const { return this->is_initialized; };
	void set_gain(double new_gain) { this->gain = new_gain; };
	
	void reset(double initial_value);
	double apply_filter(double new_velocity);

private:
	bool is_initialized{ false };
	double time_step{ 0.1 };
	double current_value{ -1 }; /* negative value can be used as a flag for 
								uninitialized filter */

	double max_variation_per_second{ +0.0 }; // max positive variation
	double min_variation_per_second{ -0.0 }; // max negative variation
	double max_variation_per_time_step{ +0.0 };
	double min_variation_per_time_step{ -0.0 };
	double gain{ 10.0 };
	double alpha{ 0.0 }; /* constant used in the discrete approximation of
						 the first order actuator dynamics */
	bool verbose = false;
};