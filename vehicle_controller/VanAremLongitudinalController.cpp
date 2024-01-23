
#include "EgoVehicle.h"
#include "VanAremLongitudinalController.h"

VanAremLongitudinalController::VanAremLongitudinalController(
	const VelocityControllerGains& velocity_controller_gains,
	const ConnectedGains& gap_controller_gains, double max_brake,
	double max_jerk,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose)
	: LongitudinalController(state_to_color_map, verbose),
	velocity_controller_gains(velocity_controller_gains),
	gap_controller_gains(gap_controller_gains),
	ego_max_brake(max_brake), max_jerk_per_interval(max_jerk) {}


double VanAremLongitudinalController::implement_get_gap_error() const
{
	return gap_error;
}

double VanAremLongitudinalController::implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle, const NearbyVehicle* leader,
	double velocity_reference)
{
	double des_accel_vel = compute_velocity_control_acceleration(
		velocity_reference, ego_vehicle.get_velocity());
	double des_accel_gap = compute_gap_control_acceleration(
			ego_vehicle, leader);
	double des_accel;
	if (des_accel_vel < des_accel_gap)
	{
		state = State::velocity_control;
		des_accel = des_accel_vel;
	}
	else
	{
		state = State::vehicle_following;
		des_accel = des_accel_gap;
	}

	double current_accel = ego_vehicle.get_acceleration();
	double filtered_accel = apply_jerk_limit(current_accel, des_accel);

	//if (verbose)
	//{
	//	std::clog << "a_vel=" << des_accel_vel
	//		<< ", a_gap=" << des_accel_gap
	//		<< ", a=" << des_accel
	//		<< ", sat(a)" << filtered_accel << std::endl;
	//}

	return filtered_accel;
}

double VanAremLongitudinalController::compute_desired_gap(
	double ego_velocity, const NearbyVehicle& leader) const
{
	double leader_max_brake = leader.get_max_brake();
	double reaction_time = leader.is_connected() ?
		0.5 : 1.4;
	double reaction_time_gap = ego_velocity * reaction_time;
	double safe_gap = std::pow(ego_velocity, 2) / 2
		* (1 / ego_max_brake - 1 / leader_max_brake);
	return std::max(std::max(min_gap, reaction_time_gap), safe_gap);
}

double VanAremLongitudinalController::compute_gap_control_acceleration(
	const EgoVehicle& ego_vehicle, const NearbyVehicle* leader)
{
	if (leader == nullptr)
	{
		gap_error = MAX_DISTANCE;
		return INFINITY;
	}

	double gap = ego_vehicle.compute_gap_to_a_leader(leader);
	double ego_velocity = ego_vehicle.get_velocity();
	double gap_reference = compute_desired_gap(ego_velocity, *leader);
	gap_error = gap - gap_reference;
	double vel_error = -leader->get_relative_velocity();
	double leader_accel = leader->get_acceleration();
	return gap_controller_gains.ka * leader_accel
		+ gap_controller_gains.kv * vel_error
		+ gap_controller_gains.kg * gap_error;
}

double VanAremLongitudinalController::compute_velocity_control_acceleration(
	double velocity_reference, double ego_velocity) const
{
	return velocity_controller_gains.kp 
		* (velocity_reference - ego_velocity);
}

double VanAremLongitudinalController::apply_jerk_limit(
	double current_accel, double des_accel)
{
	double new_accel = des_accel;
	if (new_accel - current_accel > max_jerk_per_interval)
	{
		new_accel = current_accel + max_jerk_per_interval;
	}
	else if (new_accel - current_accel < -max_jerk_per_interval)
	{
		new_accel = current_accel - max_jerk_per_interval;
	}
	return new_accel;
}