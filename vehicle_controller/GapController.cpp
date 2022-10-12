#include <algorithm>
#include <cmath>
#include <iostream>

#include "GapController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"

GapController::GapController(double simulation_time_step, 
	double velocity_filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose) : 
	velocity_filter{ VariationLimitedFilter(velocity_filter_gain, 
		comfortable_acceleration, filter_brake_limit,
		simulation_time_step) },
	verbose{ verbose } {}

double GapController::get_desired_gap(double ego_velocity)
{
	return implement_get_desired_gap(ego_velocity);
}

double GapController::compute_gap_error(
	double gap, double reference_gap) const
{
	double upper_limit = is_connected ?
		max_gap_error_connected : max_gap_error;
	return std::min(upper_limit, gap - reference_gap);
};

double GapController::compute_velocity_error(double velocity_ego,
	double velocity_reference) const
{
	return velocity_reference - velocity_ego;
}

double GapController::compute_acceleration_error(
	double acceleration_ego, double acceleration_reference) const
{
	return acceleration_reference - acceleration_ego;
}

double GapController::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle, 
	const std::shared_ptr<NearbyVehicle> leader)
{
	if (leader == nullptr)
	{
		return ego_vehicle.get_comfortable_acceleration();
	}
	
	double ego_velocity = ego_vehicle.get_velocity();
	double gap = ego_vehicle.compute_gap(leader);
	double gap_reference = compute_desired_gap(ego_vehicle);
	double gap_error = compute_gap_error(gap, gap_reference);
	double velocity_reference = leader->compute_velocity(ego_velocity);
	double filtered_velocity_reference =
		velocity_filter.apply_filter(velocity_reference);
	double velocity_error = compute_velocity_error(
		ego_velocity, filtered_velocity_reference);

	//if (verbose) {
	//	std::clog << "\t[Gap controller]\n\t" 
	//		<< "leader id = " << leader->get_id()
	//		<< ", h = " << get_current_time_headway()
	//		<< ", eg=" << gap - gap_reference
	//		<< ", sat(eg)=" << gap_error
	//		<< ", ev=" << velocity_error
	//		<< ", vl=" << velocity_reference
	//		<< ", vl_hat=" << filtered_velocity_reference;
	//}

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

	if (verbose) {
		std::clog << std::endl;
	}

	return desired_acceleration;
}

void GapController::reset_velocity_filter(
	double ego_velocity) 
{
	velocity_filter.reset(ego_velocity);
}

void GapController::update_leader_velocity_filter(
	double leader_velocity)
{
	velocity_filter.apply_filter(leader_velocity);
}