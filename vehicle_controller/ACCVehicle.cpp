#include "ACCVehicle.h"

ACCVehicle::ACCVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, VehicleType::acc_car, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, false, false,
		simulation_time_step, creation_time, verbose)
{
	controller.add_vissim_controller();
	controller.add_origin_lane_controllers(*this);
	if (verbose)
	{
		std::clog << "[ACCVehicle] constructor done" << std::endl;
	}
}

double ACCVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller.get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

//double ACCVehicle::compute_lane_changing_desired_time_headway(
//	const NearbyVehicle& nearby_vehicle) const
//{
//	return compute_time_headway_with_risk(get_desired_velocity(),
//		get_max_brake(), nearby_vehicle.get_max_brake(),
//		get_lambda_1(), get_rho(), 0);
//}

void ACCVehicle::set_desired_lane_change_direction()
{
	desired_lane_change_direction = get_vissim_lane_suggestion();
}

bool ACCVehicle::can_start_lane_change()
{
	return get_vissim_lane_suggestion() != RelativeLane::same;
}
