#include "TrafficLightALCVehicle.h"

bool TrafficLightALCVehicle::implement_has_next_traffic_light() const {
	return next_traffic_light_id != 0;
}

void TrafficLightALCVehicle::set_traffic_light_information(
	int traffic_light_id, double distance)
{
	if (implement_has_next_traffic_light()
		&& (traffic_light_id != next_traffic_light_id))
	{
		time_crossed_last_traffic_light = get_time();
	}
	next_traffic_light_id = traffic_light_id;
	distance_to_next_traffic_light = distance;
}

double TrafficLightALCVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller.get_desired_acceleration(*this, traffic_lights);
	return a_desired_acceleration;
}

void TrafficLightALCVehicle::set_desired_lane_change_direction()
{
	desired_lane_change_direction = RelativeLane::same;
}

bool TrafficLightALCVehicle::implement_can_start_lane_change()
{
	return false;
}