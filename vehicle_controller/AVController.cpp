#include "AVController.h"
#include "AutonomousVehicle.h"

AVController::AVController(const AutonomousVehicle* autonomous_vehicle,
	bool verbose) : ACCVehicleController(autonomous_vehicle, verbose),
	autonomous_vehicle {autonomous_vehicle} {}

void AVController::activate_destination_lane_controller(
	const NearbyVehicle& virtual_leader)
{
	//destination_lane_controller.smooth_start_leader_velocity_filter();
	double leader_velocity = virtual_leader.compute_velocity(
		autonomous_vehicle->get_velocity());
	destination_lane_controller.reset_leader_velocity_filter(leader_velocity);

	/* [Mar 6] Test: setting the initial value equal the current
	origin lane time headway value. In the update function, we
	set the desired value to the lane changing one. */
	//destination_lane_controller.reset_time_headway_filter(time_headway
	//	/*origin_lane_controller.get_current_time_headway()*/
	//);
	update_destination_lane_controller(virtual_leader);
}

void AVController::update_destination_lane_controller(
	const NearbyVehicle& virtual_leader)
{
	double old_leader_id = autonomous_vehicle->get_old_leader_id();
	double safe_h = autonomous_vehicle->compute_current_desired_time_headway(
		virtual_leader);
	double new_h;
	if (old_leader_id == virtual_leader.get_id())
	{
		new_h = std::min(safe_h, origin_lane_controller_time_headway);
	}
	else
	{
		double current_h =
			destination_lane_controller.get_current_time_headway();
		double comf_h = find_comfortable_time_headway(virtual_leader,
			destination_lane_controller.get_standstill_distance());
		new_h = std::max(current_h, std::min(comf_h, safe_h));
	}
	destination_lane_controller.reset_time_headway_filter(new_h);

	if (verbose)
	{
		std::clog << "Resetting dest lane ctrl (virtual leader) h_r = "
			<< new_h << " and setting desired value to " << safe_h
			<< std::endl;
	}

	/* [Mar 6, 23] TODO: still not sure what's the best way to
	initialize the gap controller's velocity filter */
	//destination_lane_controller.smooth_start_leader_velocity_filter();
	destination_lane_controller.set_desired_time_headway(safe_h);
	destination_lane_controller.connect_gap_controller(
		virtual_leader.is_connected());
	//destination_lane_controller.reset_leader_velocity_filter(ego_velocity);
}

void AVController::update_destination_lane_follower_parameters(
	NearbyVehicle& dest_lane_follower)
{
	dest_lane_follower.compute_safe_gap_parameters();
	lateral_controller.set_destination_lane_follower_parameters(
		dest_lane_follower.get_lambda_0(), dest_lane_follower.get_lambda_1());
}

void AVController::update_destination_lane_follower_time_headway(
	bool are_vehicles_connected, NearbyVehicle& dest_lane_follower)
{
	double dest_lane_follower_time_headway = are_vehicles_connected ?
		dest_lane_follower.get_h_to_incoming_vehicle()
		: dest_lane_follower.estimate_desired_time_headway(
			autonomous_vehicle->get_max_brake(), 0);

	if (verbose)
	{
		std::clog << "\tUpdating fd's h."
			<< " Are vehs connected ? " << are_vehicles_connected
			<< ", h_f=" << dest_lane_follower_time_headway << "\n";
	}
	lateral_controller.set_destination_lane_follower_time_headway(
		dest_lane_follower_time_headway);
}

void AVController::update_destination_lane_leader_time_headway(
	double time_headway)
{
	if (verbose)
	{
		std::clog << "Setting dest lane leader h_r = "
			<< time_headway << std::endl;
	}
	lateral_controller.set_time_headway_to_destination_lane_leader(
		time_headway);
}

double AVController::compute_accepted_lane_change_gap(
	const NearbyVehicle& nearby_vehicle, double accepted_risk)
{
	double veh_following_gap = lateral_controller.compute_time_headway_gap(
		autonomous_vehicle->get_velocity(), nearby_vehicle, accepted_risk);
	double gap_variation = lateral_controller.compute_transient_gap(
		*autonomous_vehicle, nearby_vehicle, false);

	if (verbose)
	{
		std::clog << "\tnv id " << nearby_vehicle.get_id()
			<< ": delta g_lc = " << gap_variation
			<< ", g_vf = " << veh_following_gap
			<< "; g_lc = " << veh_following_gap + gap_variation << std::endl;
	}

	return veh_following_gap + gap_variation;
}

double AVController::compute_accepted_lane_change_gap_exact(
	const NearbyVehicle& nearby_vehicle, 
	std::pair<double, double> ego_safe_lane_changing_params,
	double accepted_risk)
{
	double veh_following_gap =
		lateral_controller.compute_vehicle_following_gap_for_lane_change(
			*autonomous_vehicle, nearby_vehicle, 
			ego_safe_lane_changing_params,
			accepted_risk);
	double gap_variation = lateral_controller.compute_transient_gap(
		*autonomous_vehicle, nearby_vehicle, false);

	if (verbose)
	{
		std::clog << "\tnv id " << nearby_vehicle.get_id()
			<< ": delta g_lc = " << gap_variation
			<< ", g_vf = " << veh_following_gap
			<< "; g_lc = " << veh_following_gap + gap_variation << std::endl;
	}

	return veh_following_gap + gap_variation;
}

double AVController::get_gap_variation_during_lane_change(
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) const
{
	return lateral_controller.compute_transient_gap(
		*autonomous_vehicle, nearby_vehicle, will_accelerate);
}

bool AVController::is_in_free_flow_at_origin_lane() const
{
	return origin_lane_controller->get_state()
		== SwitchedLongitudinalController::State::velocity_control;
}

void AVController::add_lane_change_adjustment_controller()
{
	if (verbose) std::clog << "Creating lane change adjustment controller\n";

	destination_lane_controller = VirtualLongitudinalController(
		autonomous_vehicle, dest_lane_colors, long_controllers_verbose);
	destination_lane_controller.create_velocity_controller(
		adjustment_velocity_controller_gains, velocity_filter_gain);
	destination_lane_controller.create_gap_controller(
		autonomous_virtual_following_gains, connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain);
	available_controllers[ALCType::destination_lane] =
		&destination_lane_controller;
}

bool AVController::get_destination_lane_desired_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	bool end_of_lane_controller_is_active =
		end_of_lane_controller.get_state()
		== SwitchedLongitudinalController::State::vehicle_following;

	const NearbyVehicle* virtual_leader =
		autonomous_vehicle->get_virtual_leader();
	if (virtual_leader != nullptr)
	{
		if (verbose)
		{
			std::clog << "Dest. lane controller"
				<< std::endl;
		}
		/*std::shared_ptr<const NearbyVehicle> virtual_leader =
			autonomous_vehicle.get_virtual_leader();*/
		double reference_velocity = determine_low_velocity_reference(
			*virtual_leader);

		if (verbose)
		{
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::destination_lane)
			&& destination_lane_controller.is_outdated())
		{
			destination_lane_controller.reset_velocity_controller(
				autonomous_vehicle->get_velocity());
		}

		possible_accelerations[ALCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				virtual_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

double AVController::determine_low_velocity_reference(const NearbyVehicle& nearby_vehicle) const
{
	double ego_velocity = autonomous_vehicle->get_velocity();
	double leader_velocity =
		nearby_vehicle.compute_velocity(ego_velocity);
	/* The fraction of the leader speed has to vary. Otherwise, vehicles
	take too long to create safe gaps at low speeds*/
	double vel_fraction;
	if (leader_velocity < 5 / 3.6)
	{
		vel_fraction = 0;
	}
	else if (leader_velocity < 40 / 3.6)
	{
		vel_fraction = 0.5;
	}
	else
	{
		vel_fraction = 0.8;
	}
	double reference_velocity = std::min(
		leader_velocity * vel_fraction,
		ego_velocity);

	if (verbose)
	{
		std::clog << "\tDetermining vel ref. v_l=" << leader_velocity
			<< ", v_ref=" << reference_velocity << std::endl;
	}

	return reference_velocity;
}

void AVController::implement_add_internal_controllers()
{
	if (verbose) std::clog << "Creating AV controllers\n";

	add_vissim_controller();
	add_origin_lane_controllers();
	add_lane_change_adjustment_controller();
	lateral_controller = LateralController(verbose);
}

double AVController::implement_get_desired_acceleration()
{
	if (autonomous_vehicle->is_vissim_controlling_lane_change()
		&& (autonomous_vehicle->has_lane_change_intention()
			|| autonomous_vehicle->is_lane_changing()))
	{
		return get_vissim_desired_acceleration();
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);
	get_destination_lane_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

void AVController::implement_update_origin_lane_controller(
	const NearbyVehicle& real_leader)
{
	/* Vehicles always update the leader before
	updating others */
	long old_virtual_leader_id = autonomous_vehicle->get_virtual_leader_id();
	origin_lane_controller_time_headway =
		origin_lane_controller->get_current_time_headway();
	double safe_h = autonomous_vehicle->compute_current_desired_time_headway(
		real_leader);
	double new_h;
	if (old_virtual_leader_id == real_leader.get_id())
	{
		// Use the time headway of to the virtual leader
		new_h = std::min(safe_h,
			destination_lane_controller.get_current_time_headway());
	}
	else
	{
		double current_h = 
			origin_lane_controller->get_current_time_headway();
		double comf_h = find_comfortable_time_headway(real_leader,
			origin_lane_controller->get_standstill_distance());
		new_h = std::max(current_h, std::min(comf_h, safe_h));
	}

	if (verbose)
	{
		std::clog << "Resetting orig lane ctrl (real leader) h_r = "
			<< new_h << " and setting desired value to "
			<< safe_h << std::endl;
	}

	lateral_controller.set_time_headway_to_leader(safe_h);
	origin_lane_controller->reset_time_headway_filter(new_h);
	origin_lane_controller->set_desired_time_headway(safe_h);
	origin_lane_controller->connect_gap_controller(
		real_leader.is_connected());
}

double AVController::implement_get_desired_time_headway_gap(
	const NearbyVehicle& nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	//double ego_velocity = autonomous_vehicle->get_velocity();
	if (nearby_vehicle.is_ahead())
	{
		if (nearby_vehicle.get_relative_lane() == RelativeLane::same)
		{
			time_headway_gap =
				get_desired_time_headway_gap_to_leader();
		}
		else
		{
			time_headway_gap =
				destination_lane_controller.get_desired_time_headway_gap();
		}
	}
	else
	{
		double dest_lane_follower_time_headway =
			lateral_controller.get_destination_lane_follower_time_headway();
		time_headway_gap =
			destination_lane_controller.get_time_headway_gap(
				dest_lane_follower_time_headway,
				nearby_vehicle.compute_velocity(
					autonomous_vehicle->get_velocity()));
	}

	return time_headway_gap;
}