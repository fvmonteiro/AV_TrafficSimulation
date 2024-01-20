#include "ACCVehicle.h"

ACCVehicle::ACCVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, VehicleType::acc_car, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, false, false,
		simulation_time_step, creation_time, verbose)
{
	if (verbose)
	{
		std::clog << "[ACCVehicle] constructor done" << std::endl;
	}
}

double ACCVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller->get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

bool ACCVehicle::implement_check_lane_change_gaps()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}
