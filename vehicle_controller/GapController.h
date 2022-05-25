#pragma once

#include "VariationLimitedFilter.h"

class EgoVehicle;
class NearbyVehicle;

struct AutonomousGains {
	double kg{ 0.0 }; // relative to gap error.
	double kv{ 0.0 }; // relative to velocity error.
};

struct ConnectedGains {
	double kg{ 0.0 }; // relative to gap error.
	double kv{ 0.0 }; // relative to velocity error.
	double kgd{ 0.0 }; // relative to gap error derivative
	double ka{ 0.0 }; // relative to acceleration error
};

/* Constant time headway based controller */
class GapController
{
public:
	GapController() = default;
	GapController(double simulation_time_step, 
		const AutonomousGains& autonomous_gains,
		const ConnectedGains& connected_gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double comfortable_acceleration, double filter_brake_limit,
		bool verbose);
	GapController(double simulation_time_step,
		const AutonomousGains& autonomous_gains,
		const ConnectedGains& connected_gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double comfortable_acceleration, double filter_brake_limit);

	double get_time_headway() const { return time_headway; };
	double get_standstill_distance() const { return standstill_distance; };
	double get_gap_error_gain() const {
		return is_connected ? connected_gains.kg : autonomous_gains.kg;
	};

	void set_desired_time_headway(double time_headway) {
		this->desired_time_headway = time_headway;
	};
	void set_connexion(bool is_conneced) {
		this->is_connected = is_conneced;
	};

	/* Returns the desired (final) time headway. */
	double get_safe_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;

	/* Computes the time headway following distance */
	double compute_time_headway_gap(double time_headway, 
		double velocity) const;
	/* Computes the time headway following distance using the safe
	value for time headway*/
	double get_safe_time_headway_gap(double ego_velocity,
		bool has_lane_change_intention) const;
	double get_desired_gap(double ego_velocity);

	double compute_desired_acceleration(const EgoVehicle& ego_vehicle, 
		const std::shared_ptr<NearbyVehicle> leader);

	void reset_time_headway_filter(double time_headway);
	void reset_velocity_filter(double ego_velocity);
	void update_leader_velocity_filter(double leader_velocity);

private:
	VariationLimitedFilter velocity_filter;
	VariationLimitedFilter time_headway_filter;

	//double simulation_time_step{ 0.1 };
	/* Desired gap parameters */
	double desired_time_headway{ 1.0 };
	double time_headway{ 1.0 }; /* time headway [s] */
	double standstill_distance{ 1.0 }; /* standstill distance [m] */

	/* Vehicle following gains */
	AutonomousGains autonomous_gains;
	ConnectedGains connected_gains;

	/* Other controller parameters */
	double max_gap_error{ 30.0 }; // maximum positive gap error in meters
	double max_gap_error_connected{ 20.0 }; /* maximum positive gap 
											error in meters */
	bool is_connected{ false };

	bool verbose{ false };

	double compute_autonomous_input(double gap_error, double velocity_error);

	double compute_connected_input(double gap_error,
		double velocity_error, double ego_acceleration,
		double leader_acceleration);

	/* Updates the time headway and computes desired gap */
	double compute_desired_gap(double ego_velocity);

	/* Computes gap minus reference gap and upper bounds it
	with max_gap_error. */
	double compute_gap_error(double gap, double reference_gap) const;

	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego,
		double velocity_leader) const;

	double estimate_gap_error_derivative(
		double velocity_error, double acceleration) const;

	double compute_acceleration_error(
		double acceleration_ego, double acceleration_reference) const;
};

