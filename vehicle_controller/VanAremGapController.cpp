#include <algorithm>
#include <cmath>

#include "EgoVehicle.h"
#include "VanAremGapController.h"

VanAremGapController::VanAremGapController(
	double simulation_time_step,
	const ConnectedGains& connected_gains,
	double velocity_filter_gain,
	double comfortable_acceleration, double filter_brake_limit,
	bool verbose) :
	GapController(simulation_time_step, velocity_filter_gain,
		comfortable_acceleration, filter_brake_limit, verbose),
	simulation_time_step{simulation_time_step},
	connected_gains{ connected_gains }
{
	set_connexion(true);
}

double VanAremGapController::implement_get_desired_gap(double ego_velocity)
{
	return current_desired_gap;
}

double VanAremGapController::compute_desired_gap(
	const EgoVehicle& ego_vehicle)
{
	double ego_velocity = ego_vehicle.get_velocity();
	double gap_system = ego_velocity * ego_vehicle.get_brake_delay();
	double gap_safe = gap_system ;
	if (ego_vehicle.has_leader())
	{
		std::shared_ptr<NearbyVehicle> leader = ego_vehicle.get_leader();
		double leader_vel = leader->compute_velocity(ego_velocity);
		gap_safe = std::pow(leader_vel, 2) / 2
			* (1 / ego_vehicle.get_max_brake() - 1 / leader->get_max_brake());
	}
	std::vector<double> candidates{ gap_min, gap_system, gap_safe };
	current_desired_gap = std::min(gap_min, std::min(gap_system, gap_safe));
	return current_desired_gap;
}

double VanAremGapController::compute_connected_input(double gap_error,
	double velocity_error, double ego_acceleration,
	double leader_acceleration)
{
	return connected_gains.kg * gap_error
		+ connected_gains.kv * velocity_error
		+ connected_gains.ka * leader_acceleration;
}

double VanAremGapController::compute_autonomous_input(double gap_error,
	double velocity_error)
{
	return 100.0;
}