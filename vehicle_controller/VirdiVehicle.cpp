#include "VirdiVehicle.h"

VirdiVehicle::VirdiVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	ConnectedAutonomousVehicle(id, VehicleType::virdi_car, desired_velocity,
		simulation_time_step, creation_time, verbose) 
{
	if (verbose)
	{
		std::clog << "Virdi Vehicle created" << std::endl;
	}
}

void VirdiVehicle::find_relevant_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_requests();
}

double VirdiVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	return controller.get_virdi_desired_acceleration(*this);
}

bool VirdiVehicle::can_start_lane_change()
{
	double min_headway = 0.5;
	if (verbose) std::clog << "Deciding lane change" << std::endl;

	bool gap_ahead_is_safe = !has_destination_lane_leader() 
		|| (compute_gap(get_destination_lane_leader()) > min_headway);
	bool gap_behind_is_safe = !has_destination_lane_follower()
		|| (compute_gap(get_destination_lane_follower()) > min_headway);
	bool leader_speed_is_safe = !has_destination_lane_leader()
		|| (get_destination_lane_leader()->compute_velocity(get_velocity()));

	if (verbose)
	{
		std::clog 
			<< ", [dest lane] gap ahead is safe? " << gap_ahead_is_safe
			<< ", [dest_lane] gap behind is safe? " << gap_behind_is_safe
			<< std::endl;
	}

	return gap_ahead_is_safe && gap_behind_is_safe;
}

double VirdiVehicle::compute_accepted_lane_change_gap(
	std::shared_ptr<NearbyVehicle> nearby_vehicle)
{
	double min_headway = 0.5;
	return min_headway;
}

bool VirdiVehicle::is_lane_change_gap_safe(
	std::shared_ptr<NearbyVehicle>& nearby_vehicle)
{
	if (nearby_vehicle == nullptr) return true;
	double min_headway = 0.5;
	bool is_headway_safe = compute_gap(nearby_vehicle) 
		>= compute_accepted_lane_change_gap(nearby_vehicle);
	double ego_vel = get_velocity();
	double nv_vel = nearby_vehicle->compute_velocity(ego_vel);
	double tau = get_sampling_interval();
	bool is_speed_safe;
	if (nearby_vehicle->is_ahead())
	{
		double alpha = -VIRDI_MIN_ACCEL * tau
			+ VIRDI_MAX_JERK * std::pow(tau, 2) / 2;
		is_speed_safe = nv_vel - alpha >= ego_vel;
	}
	else
	{
		double beta = VIRDI_MAX_ACCEL * tau
			+ VIRDI_MAX_JERK * std::pow(tau, 2) / 2;
		is_speed_safe = nv_vel + beta <= ego_vel;
	}
	return is_headway_safe && is_speed_safe;
}
