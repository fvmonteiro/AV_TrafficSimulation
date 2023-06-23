#include "LongitudinallyAutonomousVehicle.h"
#include "LongAVController.h"

LongitudinallyAutonomousVehicle::LongitudinallyAutonomousVehicle(
	long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, VehicleType::long_autonomous_car, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, false, false,
		simulation_time_step, creation_time, verbose)
{
	this->controller = std::make_unique<VehicleController>(
		LongAVController(*this, verbose));

	if (verbose)
	{
		std::clog << "[LongitudinalAV] constructor done" << std::endl;
	}
}

double LongitudinallyAutonomousVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller->get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

bool LongitudinallyAutonomousVehicle::implement_check_lane_change_gaps()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}
