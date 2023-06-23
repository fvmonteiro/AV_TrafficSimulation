#pragma once

#include <memory>

class EgoVehicle;
class NearbyVehicle;

class SafetyCriticalGapController
{
public:
	SafetyCriticalGapController() = default;

	double get_gap_error() const {
		return gap_error;
	}

	double compute_safe_acceleration(const EgoVehicle& ego_vehicle,
		const NearbyVehicle* leader);

private:
	double rho, gamma; // class K function parameters
	double gap_error;

	double compute_finite_time_cbf_class_k_function(double cbf_value);
	/* Computes and stores the current gap error value */
	void compute_gap_error(const EgoVehicle& ego_vehicle,
		const NearbyVehicle* leader);

	// PLACE HOLDERS: not sure how to get the proper values here
	double get_time_headway() { 
		return 1.0; 
	}
	double get_standstill_distance() { 
		return 1.0;
	}
	double get_max_brake() { 
		return 6;
	}
};

