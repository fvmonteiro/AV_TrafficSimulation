#include "VirdiVehicle.h"

VirdiVehicle::VirdiVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, VehicleType::virdi_car, desired_velocity,
		CONNECTED_BRAKE_DELAY, false, false, simulation_time_step,
		creation_time, verbose) 
{
	if (verbose)
	{
		std::clog << "Virdi Vehicle created" << std::endl;
	}
}

double VirdiVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_acc_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

bool VirdiVehicle::can_start_lane_change()
{
	return get_relative_target_lane() != RelativeLane::same;
}