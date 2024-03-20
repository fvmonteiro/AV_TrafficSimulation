#include "Platoon.h"
#include "PlatoonVehicleController.h"
#include "PlatoonVehicle.h"

PlatoonVehicleController::PlatoonVehicleController(
	const PlatoonVehicle* platoon_vehicle, bool verbose) 
	: CAVController(platoon_vehicle, verbose), 
	platoon_vehicle{platoon_vehicle} {}

void PlatoonVehicleController::implement_add_internal_controllers()
{
	if (verbose) std::cout << "Creating Platoon Vehicle controllers\n";
	
	// To decide lane change safety
	lateral_controller = LateralController(verbose);

	real_leader_controller = std::make_shared<SimpleLongitudinalController>(
		platoon_vehicle, real_leader_controller_colors, verbose);
	real_leader_controller->set_max_brake(
		platoon_vehicle->get_max_brake());
	real_leader_controller->create_velocity_controller(
		platoon_vehicle_velocity_gains, platoon_velocity_filter_gain);
	real_leader_controller->create_gap_controller(
		platoon_vehicle_autonomous_gains, platoon_vehicle_connected_gains,
		platoon_velocity_filter_gain, platoon_time_headway_filter_gain);

	virtual_leader_controller = std::make_shared<SimpleLongitudinalController>(
		platoon_vehicle, virtual_leader_controller_colors, verbose);
	virtual_leader_controller->set_max_brake(
		platoon_vehicle->get_comfortable_brake());
	virtual_leader_controller->create_velocity_controller(
		platoon_vehicle_velocity_gains, platoon_velocity_filter_gain);
	virtual_leader_controller->create_gap_controller(
		platoon_vehicle_autonomous_gains, platoon_vehicle_connected_gains,
		platoon_velocity_filter_gain, platoon_time_headway_filter_gain);

	/* We need to populate the pointers used by parent classes 
	[Jan 26] Still checking which *have* to be set. */
	origin_lane_controller = real_leader_controller;
	destination_lane_controller = virtual_leader_controller;
	gap_generating_controller = virtual_leader_controller;
	
	available_controllers[ALCType::real_leader] = 
		real_leader_controller.get();
	available_controllers[ALCType::virtual_leader] =
		virtual_leader_controller.get();
	
}

double PlatoonVehicleController::implement_get_desired_acceleration()
{
	std::unordered_map<ALCType, double> possible_accelerations;

	possible_accelerations[ALCType::real_leader] = 
		real_leader_controller->compute_desired_acceleration(
			platoon_vehicle->get_leader().get(),
			platoon_vehicle->get_desired_velocity_from_platoon());

	if (platoon_vehicle->has_virtual_leader())
	{
		possible_accelerations[ALCType::virtual_leader] = 
			virtual_leader_controller->compute_desired_acceleration(
				platoon_vehicle->get_virtual_leader().get(),
				platoon_vehicle->get_desired_velocity());
	}

	return choose_minimum_acceleration(possible_accelerations);
}

void PlatoonVehicleController::implement_update_origin_lane_controller(
	const NearbyVehicle& real_leader)
{
	origin_lane_controller_time_headway =
		real_leader_controller->get_current_time_headway();
	double h_safe = platoon_vehicle->compute_current_desired_time_headway(
		real_leader);
	double h_ref = h_safe + platoon_vehicle->get_time_headway_margin();
	lateral_controller.set_time_headway_to_leader(h_safe);
	real_leader_controller->reset_time_headway_filter(h_ref);
	real_leader_controller->set_desired_time_headway(h_ref);
	real_leader_controller->connect_gap_controller(
		real_leader.is_connected());
}

void PlatoonVehicleController::implement_update_destination_lane_controller(
	const NearbyVehicle& virtual_leader)
{
	/*double safe_h = platoon_vehicle->compute_current_desired_time_headway(
		virtual_leader);*/
	double h_safe = platoon_vehicle->decide_safe_time_headway(virtual_leader);
	double h_ref = h_safe + platoon_vehicle->get_time_headway_margin();
	virtual_leader_controller->reset_time_headway_filter(h_ref);
	virtual_leader_controller->set_desired_time_headway(h_ref);
	virtual_leader_controller->connect_gap_controller(
		virtual_leader.is_connected());
}

double PlatoonVehicleController
::compute_destination_lane_follower_time_headway(
	NearbyVehicle& dest_lane_follower)
{
	return platoon_vehicle->decide_safe_time_headway(dest_lane_follower);
}
