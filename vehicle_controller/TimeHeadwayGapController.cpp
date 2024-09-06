#include <iostream>

#include "EgoVehicle.h"
#include "TimeHeadwayGapController.h"

TimeHeadwayGapController::TimeHeadwayGapController(
	double simulation_time_step,
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose) :
	GapController(simulation_time_step, velocity_filter_gain,
		comfortable_acceleration, filter_brake_limit, verbose),
	autonomous_gains{ autonomous_gains },
	connected_gains{ connected_gains },
	time_headway_filter{ VariationLimitedFilter(time_headway_filter_gain,
		100, -100, simulation_time_step) } {}

double TimeHeadwayGapController::get_desired_time_headway() const
{
	return desired_time_headway;
}

double TimeHeadwayGapController::get_current_time_headway() const
{
	return time_headway_filter.get_current_value();
}

double TimeHeadwayGapController::get_gap_error_gain() const {
	return get_is_connected() ? connected_gains.kg : autonomous_gains.kg;
};

double TimeHeadwayGapController::compute_time_headway_gap(double time_headway,
	double velocity) const
{
	/* TODO: for now we assumed the standstill distance (d) is the same for
	all cases. */
	return time_headway * velocity + get_standstill_distance();
}

double TimeHeadwayGapController::get_desired_time_headway_gap(
	double ego_velocity /*, bool has_lane_change_intention */) const
{
	return compute_time_headway_gap(get_desired_time_headway(), ego_velocity);
}

void TimeHeadwayGapController::reset_time_headway_filter(double time_headway)
{
	//if (verbose)
	//{
	//	std::clog << "\t[Gap controller] Reset h=" << time_headway << std::endl;
	//}
	time_headway_filter.reset(time_headway);
}

double TimeHeadwayGapController::implement_get_desired_gap(
	double ego_velocity)
{
	return compute_time_headway_gap(time_headway_filter.get_current_value(),
		ego_velocity);
}

double TimeHeadwayGapController::compute_desired_gap(
	const EgoVehicle& ego_vehicle)
{
	double ego_velocity = ego_vehicle.get_velocity();
	double desired_time_headway = get_desired_time_headway();
	double time_headway = time_headway_filter.apply_filter(
		desired_time_headway);
	return compute_time_headway_gap(time_headway, ego_velocity);
}

double TimeHeadwayGapController::compute_autonomous_input(
	double gap_error, double velocity_error)
{
	return autonomous_gains.kg * gap_error
		+ autonomous_gains.kv * velocity_error;
}

double TimeHeadwayGapController::compute_connected_input(
	double gap_error, double velocity_error, double ego_acceleration,
	double leader_acceleration)
{
	double gap_error_derivative = estimate_gap_error_derivative(
		velocity_error, ego_acceleration);
	double acceleration_error = compute_acceleration_error(
		ego_acceleration, leader_acceleration);

	if (verbose) {
		std::clog << ", eg_dot=" << gap_error_derivative
			<< ", ea=" << acceleration_error;
	}

	return connected_gains.kg * gap_error
		+ connected_gains.kv * velocity_error
		+ connected_gains.kgd * gap_error_derivative
		+ connected_gains.ka * acceleration_error;
}

double TimeHeadwayGapController::estimate_gap_error_derivative(
	double velocity_error, double acceleration) const
{
	/* TODO [Oct 29, 2021]: should be changed to e_v - h.a - dh/dt.v */
	double time_headway = get_desired_time_headway();
	return velocity_error - time_headway * acceleration;
}