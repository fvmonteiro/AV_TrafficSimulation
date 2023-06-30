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

void LongitudinallyAutonomousVehicle::update_leader(
	std::shared_ptr<const NearbyVehicle>& old_leader)
{
	if (has_leader())
	{
		double new_leader_max_brake = get_leader()->get_max_brake();
		bool is_new_leader_connected = get_leader()->is_connected();
		if (old_leader == nullptr)
		{
			controller->activate_origin_lane_controller(
				compute_current_desired_time_headway(*get_leader()),
				is_new_leader_connected);
		}
		else if ((std::abs(new_leader_max_brake
			- old_leader->get_max_brake()) > 0.5)
			|| (get_leader()->get_type() != old_leader->get_type()))
		{
			controller->update_origin_lane_controller(
				compute_current_desired_time_headway(*get_leader()),
				is_new_leader_connected
			);
		}
	}
}

bool LongitudinallyAutonomousVehicle::implement_check_lane_change_gaps()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}
