#pragma once

#include "VariationLimitedFilter.h"

/* Constant time headway based controller */
class VehicleFollowingController
{
public:
	VehicleFollowingController() = default;
	VehicleFollowingController(double simulation_time_step, double kg, 
		double kv, double comfortable_acceleration,
		double filter_brake_limit);
	VehicleFollowingController(double simulation_time_step, double kg,
		double kv, double kd, double ka, double comfortable_acceleration,
		double filter_brake_limit);

	double get_h() const { return h; };
	double get_d() const { return d; };

	void compute_time_headway_with_risk(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho, double accepted_risk);

	/* Computes constant time headway following distance */
	double compute_time_headway_gap(double time_headway, double velocity);

	/* Computes gap minus reference gap and upper bounds it
	with max_gap_error. */
	double compute_gap_error(double gap, double reference_gap);

	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego, double velocity_leader);

	double estimate_gap_error_derivative(
		double velocity_error, double acceleration);

	double compute_acceleration_error(
		double acceleration_ego, double acceleration_reference);

	double compute_input(double gap, double ego_velocity,
		double leader_velocity);

	/*double compute_input(double gap_error,
		double velocity_error, double gap_error_derivative,
		double acceleration_error);*/

	double compute_gap_threshold(double free_flow_velocity,
		double ego_velocity, double leader_velocity);

	void reset_leader_velocity_filter(double ego_velocity);

private:
	double simulation_time_step{ 0.01 };

	/* Desired gap parameters */
	double h{ 0.0 }; /* time headway [s] */
	double d{ 1.0 }; /* standstill distance [m] */

	/* Vehicle following gains
	(computed in Matlab. Should be computed here?) */
	double kg{ 0.4 }; // relative to gap error.
	double kv{ 1.0 }; // relative to velocity error.
	double kd{ 0.0 }; // relative to gap error derivative
	double ka{ 0.0 }; // relative to acceleration error

	/* Other controller parameters */
	double max_gap_error{ 3.0 }; // maximum positive gap error in meters
	bool is_connected{ false };

	VariationLimitedFilter leader_velocity_filter;
};

