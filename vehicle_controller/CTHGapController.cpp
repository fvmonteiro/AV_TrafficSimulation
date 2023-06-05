#include <algorithm>
#include <cmath>
#include <iostream>

#include "CTHGapController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"

CTHGapController::CTHGapController(double simulation_time_step,
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose)
	: autonomous_gains{ autonomous_gains },
	connected_gains{ connected_gains },
	velocity_filter{ VariationLimitedFilter(velocity_filter_gain,
		comfortable_acceleration, filter_brake_limit,
		simulation_time_step) },
	time_headway_filter { VariationLimitedFilter(time_headway_filter_gain,
		100, -100, simulation_time_step)},
	verbose{ verbose } {}

CTHGapController::CTHGapController(double simulation_time_step,
	const AutonomousGains& autonomous_gains,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double comfortable_acceleration, double filter_brake_limit)
	: CTHGapController(simulation_time_step, autonomous_gains,
		connected_gains, velocity_filter_gain, time_headway_filter_gain,
		comfortable_acceleration, filter_brake_limit,
		false) {}

double CTHGapController::get_desired_time_headway() const
{
	return desired_time_headway;
}

double CTHGapController::get_current_time_headway() const
{
	return time_headway_filter.get_current_value();
}

double CTHGapController::compute_time_headway_gap(double time_headway,
	double velocity) const
{
	/* TODO: for now we assumed the standstill distance (d) is the same for
	all cases. */
	return time_headway * velocity + standstill_distance;
}

double CTHGapController::get_desired_time_headway_gap(
	double ego_velocity /*, bool has_lane_change_intention */) const
{
	return compute_time_headway_gap(get_desired_time_headway(), ego_velocity);
}

double CTHGapController::get_desired_gap(double ego_velocity) const
{
	return compute_time_headway_gap(time_headway_filter.get_current_value(),
		ego_velocity);
}

double CTHGapController::compute_desired_gap(double velocity_ego)
{
	double desired_time_headway = get_desired_time_headway();
	double time_headway = time_headway_filter.apply_filter(
		desired_time_headway);
	return compute_time_headway_gap(time_headway, velocity_ego);
}

double CTHGapController::compute_gap_error(
	double gap, double reference_gap) const
{
	double upper_limit = is_connected ?
		max_gap_error_connected : max_gap_error;
	return std::min(upper_limit, gap - reference_gap);
};

double CTHGapController::compute_velocity_error(double velocity_ego,
	double velocity_reference) const
{
	return velocity_reference - velocity_ego;
}

double CTHGapController::estimate_gap_error_derivative(
	double velocity_error, double acceleration) const
{
	/* TODO [Oct 29, 2021]: should be changed to e_v - h.a - dh/dt.v */
	double time_headway = get_desired_time_headway();
	return velocity_error - time_headway * acceleration;
}

double CTHGapController::compute_acceleration_error(
	double acceleration_ego, double acceleration_reference) const
{
	return acceleration_reference - acceleration_ego;
}

double CTHGapController::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::shared_ptr<const NearbyVehicle> leader)
{
	if (leader == nullptr)
	{
		return ego_vehicle.get_comfortable_acceleration();
	}

	double ego_velocity = ego_vehicle.get_velocity();
	double gap = ego_vehicle.compute_gap_to_a_leader(leader);
	double gap_reference = compute_desired_gap(ego_velocity);
	this->gap_error = compute_gap_error(gap, gap_reference);
	double velocity_reference = leader->compute_velocity(ego_velocity);
	/* The velocity error is set to zero when we want a "smooth start" */
	double filtered_velocity_reference = should_perform_smooth_start ?
		ego_velocity : velocity_filter.apply_filter(velocity_reference);
	double velocity_error = compute_velocity_error(
		ego_velocity, filtered_velocity_reference);

	if (verbose)
	{
		std::clog << "\t[Gap controller]\n\t"
			<< "leader id = " << leader->get_id()
			<< ", h = " << get_current_time_headway()
			<< ", eg=" << gap - gap_reference
			<< ", sat(eg)=" << gap_error
			<< ", ev=" << velocity_error
			<< ", vl=" << velocity_reference
			<< ", vl_hat=" << filtered_velocity_reference;
	}

	double desired_acceleration;
	if (is_connected)
	{
		desired_acceleration = compute_connected_input(gap_error,
			velocity_error, ego_vehicle.get_acceleration(),
			leader->get_acceleration());
	}
	else
	{
		desired_acceleration = compute_autonomous_input(gap_error,
			velocity_error);
	}

	if (verbose)
	{
		std::clog << " => a_d=" << desired_acceleration << std::endl;
	}

	//if (should_perform_smooth_start) // TODO: not being used [Nov 1, 2022]
	//{
	//	should_perform_smooth_start = false;
	//	if (verbose) std::clog << "\t[Gap controller]"
	//		<< " restarting leader vel filter.\n";
	//	double current_accel = ego_vehicle.get_acceleration();
	//	double reset_vel;
	//	/* [Oct 26, 22] Playing safe for now: we only use the "smooth"
	//	reset velocity if that helps braking. */
	//	if ((current_accel < 0) && (velocity_reference < ego_velocity))
	//	{
	//		/* Set the filtered leader velocity to a value that yields
	//		the desired acceleration of the previous step */
	//		double vel_gain = is_connected ?
	//			connected_gains.kv : autonomous_gains.kv;
	//		double smooth_vel = (current_accel - desired_acceleration)
	//			/ vel_gain + ego_velocity;
	//		/* Be careful with the line below if we move it out of
	//		this condition */
	//		reset_vel = std::max(smooth_vel, velocity_reference);
	//		if (verbose)
	//		{
	//			std::clog << "accel=" << current_accel
	//				<< "des. accel=" << desired_acceleration
	//				<< std::endl;
	//		}
	//	}
	//	else
	//	{
	//		reset_vel = ego_velocity;
	//	}
	//	velocity_filter.reset(reset_vel);
	//	return compute_desired_acceleration(ego_vehicle, leader);
	//}

	return desired_acceleration;
}

double CTHGapController::compute_autonomous_input(
	double gap_error, double velocity_error)
{
	return autonomous_gains.kg * gap_error
		+ autonomous_gains.kv * velocity_error;
}

double CTHGapController::compute_connected_input(
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

void CTHGapController::reset_time_headway_filter(double time_headway)
{
	//if (verbose)
	//{
	//	std::clog << "\t[Gap controller] Reset h=" << time_headway << std::endl;
	//}
	time_headway_filter.reset(time_headway);
}

void CTHGapController::reset_velocity_filter(
	double ego_velocity)
{
	velocity_filter.reset(ego_velocity);
}

void CTHGapController::update_leader_velocity_filter(
	double leader_velocity)
{
	velocity_filter.apply_filter(leader_velocity);
}
