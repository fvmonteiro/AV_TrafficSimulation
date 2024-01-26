#include "ACCVehicle.h"

ACCVehicle::ACCVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	ACCVehicle(id, VehicleType::acc_car, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, false, false,
		simulation_time_step, creation_time, verbose)
{
	if (verbose) std::clog << "[ACCVehicle] created" << std::endl;
}

void ACCVehicle::set_controller(ACCVehicleController* acc_controller)
{
	this->acc_vehicle_controller = acc_controller;
	EgoVehicle::set_controller(acc_controller);
}

void ACCVehicle::implement_create_controller()
{
	this->controller_exclusive = ACCVehicleController(this, is_verbose());
	controller_exclusive.add_internal_controllers();
	this->set_controller(&controller_exclusive);
}

//double ACCVehicle::implement_compute_desired_acceleration(
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	double a_desired_acceleration =
//		acc_vehicle_controller->get_desired_acceleration();
//	return apply_low_level_dynamics(a_desired_acceleration);
//}

bool ACCVehicle::implement_check_lane_change_gaps()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}

void ACCVehicle::implement_prepare_to_start_long_adjustments()
{
	set_has_completed_lane_change(false);
	update_time_headway_to_leader();
}

void ACCVehicle::implement_prepare_to_restart_lane_keeping(
	bool was_lane_change_successful)
{
	set_has_completed_lane_change(was_lane_change_successful);
	reset_lane_change_waiting_time();
	update_time_headway_to_leader();
	reset_origin_lane_velocity_controller();
}