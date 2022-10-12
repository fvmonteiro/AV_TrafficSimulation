#include "EgoVehicle.h"
#include "TalebpourALC.h"


TalebpourALC::TalebpourALC(
	const EgoVehicle& ego_vehicle,
	double velocity_controller_gain,
	ConnectedGains connected_gains,
	double velocity_filter_gain, bool verbose)
{
	double simulation_time_step = ego_vehicle.get_sampling_interval();
	double comfortable_acceleration =
		ego_vehicle.get_comfortable_acceleration();
	double filter_brake_limit = ego_vehicle.get_max_brake();
	velocity_controller = VelocityController(simulation_time_step,
		VelocityControllerGains{ velocity_controller_gain, 0.0, 0.0 },
		velocity_filter_gain, comfortable_acceleration, filter_brake_limit);
	gap_controller = VanAremGapController(
		simulation_time_step, connected_gains, velocity_filter_gain,
		comfortable_acceleration, filter_brake_limit, verbose);

	if (verbose)
	{
		std::clog << "Created Talebpour longitudinal controller" << std::endl;
	}
}

double TalebpourALC::compute_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double velocity_reference)
{
	double veh_following_accel = gap_controller.compute_desired_acceleration(
		ego_vehicle, leader);
	double max_safe_speed = compute_max_safe_speed(ego_vehicle, leader);
	double vel_control_accel = velocity_controller.compute_acceleration(
		ego_vehicle, max_safe_speed);
	return std::min(veh_following_accel, vel_control_accel);
}

double TalebpourALC::compute_max_safe_speed(const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader)
{
	double ego_velocity = ego_vehicle.get_velocity();
	if (leader == nullptr)
	{
		return MAX_DISTANCE;
	}
	double leader_velocity = leader->compute_velocity(ego_velocity);
	double delta_xn = leader->get_distance()
		+ ego_velocity * ego_vehicle.get_brake_delay()
		+ std::pow(leader_velocity, 2) / 2 / leader->get_max_brake();
	double delta_x = std::min(delta_xn, MAX_DISTANCE);
	return std::sqrt(2 * ego_vehicle.get_max_brake() * delta_x);
}
