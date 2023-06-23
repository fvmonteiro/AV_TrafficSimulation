#include "EgoVehicle.h"
#include "NearbyVehicle.h"
#include "SafetyCriticalGapController.h"

double SafetyCriticalGapController::compute_safe_acceleration(
	const EgoVehicle& ego_vehicle,
	const NearbyVehicle* leader)
{
	double ego_vel = ego_vehicle.get_velocity();
	double leader_vel = leader->compute_velocity(ego_vel);
	double max_brake = get_max_brake();
	compute_gap_error(ego_vehicle, leader);

	double leader_accel;
	if (ego_vehicle.get_is_connected() && leader->is_connected())
	{
		leader_accel = leader->get_acceleration();
	}
	{
		leader_accel = -leader->get_max_brake();
	}
	return (leader_vel - ego_vel 
		+ leader_vel * leader_accel / leader->get_max_brake()
		+ compute_finite_time_cbf_class_k_function(gap_error))
		* max_brake / (get_time_headway() * max_brake + ego_vel);
}

double SafetyCriticalGapController::compute_finite_time_cbf_class_k_function(
	double cbf_value)
{
	int sign = (cbf_value > 0) - (cbf_value < 0);
	return gamma * sign * std::pow(std::abs(cbf_value), rho);
}

void SafetyCriticalGapController::compute_gap_error(
	const EgoVehicle& ego_vehicle, 
	const NearbyVehicle* leader)
{
	double gap = ego_vehicle.compute_gap_to_a_leader(leader);
	double ego_vel = ego_vehicle.get_velocity();
	double leader_vel = leader->compute_velocity(ego_vel);
	double max_brake = get_max_brake();
	double safe_gap = get_time_headway() * ego_vel + get_standstill_distance()
		+ (std::pow(ego_vel, 2) - std::pow(leader_vel, 2))
		/ 2 / max_brake;
	gap_error = gap - safe_gap;
}
