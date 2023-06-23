#include "TrafficLightALCVehicle.h"
#include "TrafficLightLongAVController.h"

TrafficLightALCVehicle::TrafficLightALCVehicle(long id,
	double desired_velocity, double simulation_time_step,
	double creation_time, bool verbose) :
	TrafficLightALCVehicle(id, VehicleType::traffic_light_alc_car,
		desired_velocity, false, simulation_time_step, creation_time,
		verbose)
{
	controller = std::make_unique<TrafficLightLongAVController>(
		TrafficLightLongAVController(*this, verbose));
}

bool TrafficLightALCVehicle::implement_has_next_traffic_light() const 
{
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
	controller->set_traffic_lights(traffic_lights);
	double a_desired_acceleration =
		controller->compute_desired_acceleration();
	return a_desired_acceleration;
}

void TrafficLightALCVehicle::set_desired_lane_change_direction()
{
	/* Do nothing: for now these vehicles never change lanes */
	//desired_lane_change_direction = RelativeLane::same;
}

bool TrafficLightALCVehicle::implement_check_lane_change_gaps()
{
	return false;
}
