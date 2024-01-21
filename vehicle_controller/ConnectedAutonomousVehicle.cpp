#include "ConnectedAutonomousVehicle.h"

ConnectedAutonomousVehicle::ConnectedAutonomousVehicle(
	long id, VehicleType type, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	AutonomousVehicle(id, type, desired_velocity,
		true, simulation_time_step, creation_time, verbose)
	//original_desired_velocity(desired_velocity)
{
	compute_connected_safe_gap_parameters();
	if (verbose)
	{
		std::clog << "lambda 1 connected = " << lambda_1_connected
			<< ", lambda 0 connected = " << lambda_0_connected
			<< ", lambda 1 lc connected = " << lambda_1_lane_change_connected
			<< ", lambda 0 lc connected = " << lambda_0_lane_change_connected
			<< "\n[ConnectedAutonomousVehicle] constructor done" << std::endl;
	}
}

bool ConnectedAutonomousVehicle::is_cooperating_to_generate_gap() const {
	return has_assisted_vehicle();
	/* Function seems unnecessary, but we might perform other checks here */
}

std::shared_ptr<NearbyVehicle> ConnectedAutonomousVehicle
::create_nearby_vehicle_from_another(
	const ConnectedAutonomousVehicle& cav, long nv_id) const
{
	const auto& source_nv = get_nearby_vehicle_by_id(cav.get_id());
	
	if (source_nv != nullptr
		&& cav.get_nearby_vehicle_by_id(nv_id) != nullptr)
	{
		NearbyVehicle new_nv(*cav.get_nearby_vehicle_by_id(nv_id));
		new_nv.offset_from_another(*source_nv);
		return std::make_shared<NearbyVehicle>(new_nv);
	}
	else
	{
		return nullptr;
	}
}

std::shared_ptr<NearbyVehicle> ConnectedAutonomousVehicle::
implement_get_assisted_vehicle() const
{
	return assisted_vehicle;
}

//bool ConnectedAutonomousVehicle::implement_check_lane_change_gaps()
//{
//	//if (verbose) std::clog << "Deciding lane change" << std::endl;
//	
//	/* We don't move between platoon vehicles 
//	[Feb 9, 2023] No longer needed after creation of CAV no LC class*/
//	//if (has_destination_lane_leader()
//	//	&& get_destination_lane_leader()->get_type() == VehicleType::platoon_car
//	//	&& has_destination_lane_follower()
//	//	&& get_destination_lane_follower()->get_type() == VehicleType::platoon_car)
//	//{
//	//	return false;
//	//}
//
//	lane_change_gaps_safety.orig_lane_leader_gap =
//		is_lane_change_gap_safe(get_leader());
//	lane_change_gaps_safety.dest_lane_leader_gap =
//		is_lane_change_gap_safe(get_destination_lane_leader());
//	/* Besides the regular safety conditions, we add the case
//	where the dest lane follower has completely stopped to give room
//	to the lane changing vehicle */
//	lane_change_gaps_safety.dest_lane_follower_gap =
//		is_lane_change_gap_safe(get_destination_lane_follower())
//		|| ((get_destination_lane_follower()->
//			compute_velocity(get_velocity()) <= 1.0)
//			&& (get_destination_lane_follower()->get_distance() <= -2.0));
//	lane_change_gaps_safety.no_conflict =
//		!has_lane_change_conflict();
//
//	return lane_change_gaps_safety.is_lane_change_safe();
//}

long ConnectedAutonomousVehicle::implement_get_lane_change_request() const
{
	return lane_change_request;
}

void ConnectedAutonomousVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_requests();
	create_lane_change_request();
}

void ConnectedAutonomousVehicle::find_cooperation_requests()
{
	//set_desired_velocity(original_desired_velocity);
	bool should_increase_vel = false;

	std::shared_ptr<NearbyVehicle> old_assisted_vehicle =
		std::move(assisted_vehicle);

	for (auto const& id_veh_pair : get_nearby_vehicles())
	{
		auto const& nearby_vehicle = id_veh_pair.second;
		// Wants to merge in front of me?
		if (nearby_vehicle->get_lane_change_request_veh_id() == get_id())
		{
			if (assisted_vehicle == nullptr
				|| (nearby_vehicle->get_relative_position()
					< assisted_vehicle->get_relative_position()))
			{
				assisted_vehicle = nearby_vehicle;
			}
		}
		// Wants to merge behind me?
		else if (nearby_vehicle->get_destination_lane_leader_id() == get_id())
		{
			//set_desired_velocity(MAX_VELOCITY);
			should_increase_vel = true;
		}
	}
	set_max_desired_velocity(should_increase_vel);
	deal_with_close_and_slow_assited_vehicle();
	update_assisted_vehicle(old_assisted_vehicle);
}

std::shared_ptr<NearbyVehicle> ConnectedAutonomousVehicle
::define_virtual_leader() const
{
	/* By default, we try to merge behind the current destination
	lane leader. */
	std::shared_ptr<NearbyVehicle> nv = get_modifiable_dest_lane_leader();

	if (try_to_overtake_destination_lane_leader())
	{
		nv = nullptr;
	}
	else if (was_my_cooperation_request_accepted())
	{
		/* We don't merge behind a vehicle that is 
		braking to make space for us. */
		int cooperating_vehicle_relative_position =
			get_nearby_vehicle_by_id(lane_change_request)
			->get_relative_position();
		if (cooperating_vehicle_relative_position == 1)
		{
			nv = get_destination_lane_leader_leader();
		}
		else if (cooperating_vehicle_relative_position > 1)
		{
			nv = nullptr;
		}
	}
	return nv;
}

void ConnectedAutonomousVehicle::create_lane_change_request()
{
	// Only requests help for mandatory maneuvers
	//if (get_preferred_relative_lane() != RelativeLane::same)
	if (has_lane_change_intention())
		lane_change_request = get_destination_lane_follower_id();
	else
		lane_change_request = 0;
}

bool ConnectedAutonomousVehicle::was_my_cooperation_request_accepted() const
{
	return false;
	if (lane_change_request != 0)
	{
		std::shared_ptr<NearbyVehicle> cooperating_vehicle =
			get_nearby_vehicle_by_id(lane_change_request);
		if (cooperating_vehicle != nullptr
			&& cooperating_vehicle->get_assisted_vehicle_id() == get_id())
		{
			return true;
		}
	}
	return false;
}

void ConnectedAutonomousVehicle::deal_with_close_and_slow_assited_vehicle()
{
	/* We need to account for case where the ego vehicle is too
	close to the vehicle asking to move in and the vehicle asking to move in
	is very slow. The only solution would be for the ego vehicle to
	go backwards, which would lead to a deadlock situation. */
	if (has_assisted_vehicle()
		&& (assisted_vehicle->compute_velocity(get_velocity()) < 1)
		&& compute_gap_to_a_leader(assisted_vehicle) < 1)
	{
		assisted_vehicle = nullptr;
	}
}

void ConnectedAutonomousVehicle::update_destination_lane_follower(
	const std::shared_ptr<NearbyVehicle>& old_follower)
{
	if (has_destination_lane_follower())
	{
		std::shared_ptr<NearbyVehicle>& dest_lane_follower =
			get_modifiable_dest_lane_follower();
		cav_controller->update_destination_lane_follower_time_headway(
			get_max_brake(), dest_lane_follower->is_connected(),
			*dest_lane_follower);
		if ((old_follower == nullptr )
			|| (old_follower->get_id() != dest_lane_follower->get_id()))
		{
			cav_controller->update_destination_lane_follower_parameters(
				*dest_lane_follower);
			//dest_lane_follower_lambda_0 =
			//	get_destination_lane_follower()->get_lambda_0();
			//dest_lane_follower_lambda_1 =
			//	get_destination_lane_follower()->get_lambda_1();
		}
	}
}

void ConnectedAutonomousVehicle::update_assisted_vehicle(
	const std::shared_ptr<NearbyVehicle>& old_assisted_vehicle)
{
	if (has_assisted_vehicle())
	{
		if ((old_assisted_vehicle == nullptr)
			|| (old_assisted_vehicle->get_id() != assisted_vehicle->get_id()))
		{
			double h_to_assisted_vehicle = std::max(0.0,
				compute_vehicle_following_time_headway(
					*assisted_vehicle, 0
					/* assisted_vehicle->get_max_lane_change_risk_to_follower()*/
				));
			cav_controller->update_gap_generation_controller(
				get_velocity(), h_to_assisted_vehicle);
		}
	}
}

void ConnectedAutonomousVehicle::set_max_desired_velocity(
	bool should_increase)
{
	bool is_at_max = get_desired_velocity() == MAX_VELOCITY;
	if (should_increase && !is_at_max)
	{
		original_desired_velocity = get_desired_velocity();
		set_desired_velocity(MAX_VELOCITY);
	}
	else if (!should_increase && is_at_max)
	{
		set_desired_velocity(original_desired_velocity);
	}
}

double ConnectedAutonomousVehicle::get_lambda_1(
	bool is_leader_connected) const
{
	return is_leader_connected ?
		lambda_1_connected : Vehicle::get_lambda_1();
}

double ConnectedAutonomousVehicle::get_lambda_1_lane_change(
	bool is_leader_connected) const
{
	return get_lane_changing_safe_gap_parameters(is_leader_connected).second;
}

std::pair<double, double> ConnectedAutonomousVehicle
::get_lane_changing_safe_gap_parameters(bool is_leader_connected) const
{
	return is_leader_connected ?
		std::make_pair(lambda_0_lane_change_connected,
			lambda_1_lane_change_connected)
		: EgoVehicle::get_lane_changing_safe_gap_parameters();
}

void ConnectedAutonomousVehicle::set_controller(CAVController* a_controller)
{
	this->cav_controller = a_controller;
	AutonomousVehicle::set_controller(a_controller);
}

void ConnectedAutonomousVehicle::set_assisted_vehicle_by_id(
	long assisted_vehicle_id)
{
	std::shared_ptr<NearbyVehicle> old_assisted_vehicle =
		std::move(assisted_vehicle);
	assisted_vehicle = get_nearby_vehicle_by_id(assisted_vehicle_id);
	//deal_with_close_and_slow_assited_vehicle();
	update_assisted_vehicle(old_assisted_vehicle);
}

double ConnectedAutonomousVehicle::
compute_vehicle_following_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	/*double current_lambda_1 = get_lambda_1(nearby_vehicle.is_connected());
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), 0);*/
	return compute_vehicle_following_time_headway(nearby_vehicle, 0);
}

double ConnectedAutonomousVehicle::
compute_vehicle_following_time_headway(
	const NearbyVehicle& nearby_vehicle,
	double nv_max_lane_change_risk) const
{
	double current_lambda_1 = get_lambda_1(nearby_vehicle.is_connected());
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), nv_max_lane_change_risk);
}

double ConnectedAutonomousVehicle::compute_accepted_lane_change_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	if (nearby_vehicle == nullptr) return 0.0;

	double accepted_gap;
	double accepted_risk = 0.0;

	if (verbose)
	{
		std::clog << "\tUsing linear overestimation? "
			<< use_linear_lane_change_gap << "\n";
	}

	if (use_linear_lane_change_gap)
	{
		accepted_gap = cav_controller->compute_accepted_lane_change_gap(*this,
			*nearby_vehicle, accepted_risk);
	}
	else
	{
		accepted_gap = cav_controller->compute_accepted_lane_change_gap_exact(
			*this, *nearby_vehicle, 
			get_lane_changing_safe_gap_parameters(
				nearby_vehicle->is_connected()),
			accepted_risk);
	}

	return std::max(accepted_gap, 1.0);
}

double ConnectedAutonomousVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1 =
		get_lambda_1_lane_change(nearby_vehicle.is_connected());
	double h_lc = compute_time_headway_with_risk(get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), 0/*get_accepted_risk_to_leaders()*/);
	return h_lc;
	/* We only allow the lane changing headway towards the leaders to be below
	the safe vehicle following headway if a risky headway to the future
	follower is also allowed */
	/*if (get_accepted_risk_to_follower() <= 0)
	{
		return std::max(h_lc,
			compute_vehicle_following_safe_time_headway(nearby_vehicle));
	}
	else
	{
		return h_lc;
	}*/
}

void ConnectedAutonomousVehicle::implement_create_controller()
{
	this->controller_exclusive = std::make_unique<CAVController>(
		*this, is_verbose());
	this->set_controller(controller_exclusive.get());
}

double ConnectedAutonomousVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	if (verbose) std::clog << "[CAV] get_desired_acceleration" << std::endl;
	double a_desired_acceleration =
		cav_controller->get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

void ConnectedAutonomousVehicle::compute_connected_safe_gap_parameters()
{
	lambda_0_connected =
		compute_lambda_0(max_jerk, comfortable_acceleration,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_connected =
		compute_lambda_1(max_jerk, comfortable_acceleration,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_0_lane_change_connected =
		compute_lambda_0(max_jerk, comfortable_acceleration,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
	lambda_1_lane_change_connected =
		compute_lambda_1(max_jerk, comfortable_acceleration,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
}
