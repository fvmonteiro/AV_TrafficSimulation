#include "LongitudinallyAutonomousVehicle.h"
#include "LongAVController.h"

LongitudinallyAutonomousVehicle::LongitudinallyAutonomousVehicle(
	long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	LongitudinallyAutonomousVehicle(id, VehicleType::long_autonomous_car,
		desired_velocity, false, simulation_time_step, creation_time, verbose)
{
	this->controller = std::make_unique<LongAVController>(
		LongAVController(*this, verbose));

	if (verbose)
	{
		std::clog << "[LongitudinalAV] constructor done" << std::endl;
	}
}

LongitudinallyAutonomousVehicle::LongitudinallyAutonomousVehicle(
	long id, VehicleType type, double desired_velocity,
	bool is_connected, double simulation_time_step,
	double creation_time, bool verbose) :
	EgoVehicle(id, type, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, is_connected,
		simulation_time_step, creation_time, verbose) {};

double LongitudinallyAutonomousVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller->compute_desired_acceleration();
	return consider_vehicle_dynamics(a_desired_acceleration);
}

bool LongitudinallyAutonomousVehicle::implement_check_lane_change_gaps()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}
