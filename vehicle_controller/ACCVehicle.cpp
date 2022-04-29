#include "ACCVehicle.h"

double ACCVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_acc_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

bool ACCVehicle::can_start_lane_change()
{
	return get_relative_target_lane() != RelativeLane::same;
}
