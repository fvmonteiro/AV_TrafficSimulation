#include "ACCVehicle.h"

double ACCVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_acc_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

//double ACCVehicle::compute_lane_changing_desired_time_headway(
//	const NearbyVehicle& nearby_vehicle) const
//{
//	return compute_time_headway_with_risk(get_desired_velocity(),
//		get_max_brake(), nearby_vehicle.get_max_brake(),
//		get_lambda_1(), get_rho(), 0);
//}

bool ACCVehicle::can_start_lane_change()
{
	return get_relative_target_lane() != RelativeLane::same;
}
