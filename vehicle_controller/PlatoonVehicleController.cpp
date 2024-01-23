#include "PlatoonVehicleController.h"
#include "PlatoonVehicle.h"

PlatoonVehicleController::PlatoonVehicleController(
	const PlatoonVehicle* platoon_vehicle, bool verbose) 
	: CAVController(platoon_vehicle, verbose), 
	platoon_vehicle{platoon_vehicle} {}

double PlatoonVehicleController::get_desired_acceleration(
	const PlatoonVehicle& platoon_vehicle)
{
	// TODO major: define computation
	/*std::unordered_map<ALCType, double> possible_accelerations;

	double real_leader_accel =
		real_leader_controller->compute_desired_acceleration(
			platoon_vehicle, platoon_vehicle.get_leader(),
			platoon_vehicle.get_desired_velocity_from_platoon());
	real_leader_accel = std::min(
		std::max(real_leader_accel, -platoon_vehicle.get_max_brake()),
		platoon_vehicle.get_comfortable_acceleration());
	possible_accelerations[ALCType::real_leader] = real_leader_accel;

	double virtual_leader_accel =
		virtual_leader_controller.compute_desired_acceleration(
			platoon_vehicle, platoon_vehicle.get_leader(),
			platoon_vehicle.get_desired_velocity());
	virtual_leader_accel = std::min(
		std::max(real_leader_accel, -platoon_vehicle.get_comfortable_brake()),
		platoon_vehicle.get_comfortable_acceleration());
	possible_accelerations[ALCType::virtual_leader] = virtual_leader_accel;

	return choose_minimum_acceleration(possible_accelerations);*/
	return 0.0;
}

void PlatoonVehicleController::update_origin_lane_controller(
	const PlatoonVehicle& platoon_vehicle, const NearbyVehicle& real_leader)
{
	//origin_lane_controller_time_headway =
	//	real_leader_controller->get_current_time_headway();
	double safe_h = platoon_vehicle.compute_current_desired_time_headway(
		real_leader);
	if (verbose)
	{
		std::clog << "Setting real leader controller h_r = "
			<< safe_h << std::endl;
	}
	//origin_lane_controller.reset_time_headway_filter(new_h);
	lateral_controller.set_time_headway_to_leader(safe_h);
	real_leader_controller->set_desired_time_headway(safe_h);
	real_leader_controller->connect_gap_controller(
		real_leader.is_connected());
}

bool PlatoonVehicleController
::get_destination_lane_desired_acceleration_when_in_platoon(
	const PlatoonVehicle& platoon_vehicle,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	/* TODO: reorganize
	Almost copy of get_destination_lane_desired_acceleration
	*/
	bool is_active = false;
	if (platoon_vehicle.has_virtual_leader()
		/*&& platoon_vehicle.can_start_adjustment_to_virtual_leader()*/)
	{
		const NearbyVehicle* virtual_leader =
			platoon_vehicle.get_virtual_leader();
		double ego_velocity = platoon_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *virtual_leader);

		if (verbose)
		{
			std::clog << "Dest. lane controller" << std::endl;
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity))
		{
			destination_lane_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				platoon_vehicle, virtual_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

void PlatoonVehicleController::implement_add_internal_controllers()
{
	if (verbose) std::clog << "Creating Platoon Vehicle controllers\n";

	// TODO major: create controllers 
	/*origin_lane_controller = RealLongitudinalController(platoon_vehicle,
		platoon_vehicle_velocity_gains, platoon_vehicle_autonomous_gains,
		platoon_vehicle_connected_gains, platoon_velocity_filter_gain,
		platoon_time_headway_filter_gain, in_platoon_colors,
		long_controllers_verbose);
	real_leader_controller = &origin_lane_controller;

	virtual_leader_controller = RealLongitudinalController(platoon_vehicle,
		platoon_vehicle_velocity_gains, platoon_vehicle_autonomous_gains,
		platoon_vehicle_connected_gains, platoon_velocity_filter_gain,
		platoon_time_headway_filter_gain, in_platoon_colors,
		long_controllers_verbose);


	available_controllers[ALCType::real_leader] = real_leader_controller;
	available_controllers[ALCType::virtual_leader] =
		&virtual_leader_controller;*/
}