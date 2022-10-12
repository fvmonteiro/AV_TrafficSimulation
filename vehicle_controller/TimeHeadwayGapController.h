#pragma once
#include "GapController.h"

class TimeHeadwayGapController :
    public GapController
{
public:
	TimeHeadwayGapController() = default;

	TimeHeadwayGapController(double simulation_time_step,
		const AutonomousGains& autonomous_gains,
		const ConnectedGains& connected_gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double comfortable_acceleration, double filter_brake_limit,
		bool verbose);

	double get_time_headway() const { return time_headway; };
	void set_desired_time_headway(double time_headway) {
		this->desired_time_headway = time_headway;
	};

	/* Returns the desired (final) time headway. */
	double get_desired_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;
	double get_gap_error_gain() const;

	/* Computes the time headway following distance */
	double compute_time_headway_gap(double time_headway,
		double velocity) const;
	/* Computes the time headway following distance using the safe
	value for time headway */
	double get_desired_time_headway_gap(double ego_velocity) const;

	void reset_time_headway_filter(double time_headway);

private:
	/* Vehicle following gains */
	AutonomousGains autonomous_gains;
	ConnectedGains connected_gains;
	VariationLimitedFilter time_headway_filter;
	/* Desired gap parameters */
	double desired_time_headway{ 1.0 };
	double time_headway{ 1.0 }; /* time headway [s] */

	double implement_get_desired_gap(double ego_velocity) override;
	double compute_desired_gap(const EgoVehicle& ego_vehicle) override;
	double compute_autonomous_input(double gap_error, 
		double velocity_error) override;
	double compute_connected_input(
		double gap_error, double velocity_error, double ego_acceleration,
		double leader_acceleration) override;

	double estimate_gap_error_derivative(
		double velocity_error, double acceleration) const;
};

