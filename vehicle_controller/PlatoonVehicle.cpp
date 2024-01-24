
#include "PlatoonVehicle.h"
#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"

PlatoonVehicle::PlatoonVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	PlatoonVehicle(id, VehicleType::platoon_car,
		desired_velocity, AUTONOMOUS_BRAKE_DELAY, true, true,
		simulation_time_step, creation_time, verbose)
{
	if (verbose) std::clog << "[PlatoonVehicle] constructor done\n";
}

PlatoonVehicle::PlatoonVehicle(long id, VehicleType type,
	double desired_velocity, double brake_delay,
	bool is_lane_change_autonomous, bool is_connected,
	double simulation_time_step, double creation_time, bool verbose)
	: ConnectedAutonomousVehicle(id, type, desired_velocity, brake_delay,
		is_lane_change_autonomous, is_connected,
		simulation_time_step, creation_time, verbose) 
{
	compute_platoon_safe_gap_parameters();
	if (verbose)
	{
		std::clog << "lambda 1 platoon = " << lambda_1_platoon
			<< ", lambda 0 platoon = " << lambda_0_platoon
			<< ", lambda 1 lc platoon = " << lambda_1_lane_change_platoon
			<< ", lambda 0 lc platoon = " << lambda_0_lane_change_platoon
			<< std::endl;
	}
};

PlatoonVehicle::~PlatoonVehicle()
{
	if (verbose) 
	{
		std::clog << "[PlatoonVehicle] Destructor. t=" << get_current_time()
			<< ", veh " << get_id() << std::endl;
	}
}

bool PlatoonVehicle::is_platoon_leader() const
{
	return !is_in_a_platoon() || (platoon->get_leader_id() == get_id());
}

bool PlatoonVehicle::is_last_platoon_vehicle() const
{
	return !is_in_a_platoon() || (platoon->get_last_veh_id() == get_id());
}

bool PlatoonVehicle::has_finished_adjusting_time_headway() const
{
	double safe_time_headway = get_safe_time_headway();
	bool has_time_headway_transition_ended =
		(std::abs(safe_time_headway
			- get_current_desired_time_headway())
			/ safe_time_headway) < 0.1;
	return has_time_headway_transition_ended || !has_leader();
}

bool PlatoonVehicle::has_finished_increasing_gap() const
{
	/* When we increase the reference gap, the gap error becomes negative.
	The maneuver is considered to be done when the gap error is close 
	to becoming non-negative. */
	bool is_gap_error_non_negative = true;
	if (has_leader())
	{
		double safe_gap = compute_time_headway_gap(get_leader());
		double gap = compute_gap_to_a_leader(get_leader());
		double gap_error = gap - safe_gap;
		is_gap_error_non_negative = gap_error / safe_gap > (-0.05);
	}
	return has_finished_adjusting_time_headway()
		&& is_gap_error_non_negative;
}

bool PlatoonVehicle::has_finished_closing_gap() const
{
	/* When we decrease the reference gap, the gap error becomes positive.
	The maneuver is considered to be done when the gap error is close
	to zero. */

	bool is_gap_error_small = true;
	if (has_leader())
	{
		double safe_gap = compute_time_headway_gap(get_leader());
		double gap = compute_gap_to_a_leader(get_leader());
		double gap_error = gap - safe_gap;
		is_gap_error_small = gap_error / safe_gap < 0.05;
	}
	return has_finished_adjusting_time_headway() && is_gap_error_small;
	//bool is_gap_error_small = get_gap_error() < gap_error_threshold;
	//return has_finished_adjusting_time_headway()
	//	&& (is_gap_error_small || is_platoon_leader());
}

double PlatoonVehicle::get_desired_velocity_from_platoon() const
{
	if (is_in_a_platoon() && is_platoon_leader()
		&& has_lane_change_intention() && get_platoon()->has_lane_change_started())
	{
		return get_platoon()->get_destination_lane_leader()
			->compute_velocity(get_velocity());
	}
	return get_desired_velocity();
}

const VehicleState* PlatoonVehicle::get_preceding_vehicle_state() const
{
	const auto& leader = platoon->get_preceding_vehicle(get_id());
	return leader == nullptr ? nullptr : leader->get_state();
}

long PlatoonVehicle::get_preceding_vehicle_id() const
{
	const auto& leader = platoon->get_preceding_vehicle(get_id());
	return leader == nullptr ? 0 : leader->get_id();
}

const PlatoonVehicle*
PlatoonVehicle::get_preceding_vehicle_in_platoon() const
{
	return platoon->get_preceding_vehicle(get_id());
}

const VehicleState*
PlatoonVehicle::get_following_vehicle_state() const
{
	const auto& follower = platoon->get_following_vehicle(get_id());
	return follower == nullptr ? nullptr : follower->get_state();
}

long PlatoonVehicle::get_following_vehicle_id() const
{
	const auto& follower = platoon->get_following_vehicle(get_id());
	return follower == nullptr ? 0 : follower->get_id();
}

long PlatoonVehicle::get_suitable_destination_lane_leader_id() const
{
	return get_is_space_suitable_for_lane_change() ?
		get_destination_lane_leader_id() : 0;
}

const PlatoonVehicle*
PlatoonVehicle::get_following_vehicle_in_platoon() const
{
	return get_platoon()->get_following_vehicle(get_id());
}

NearbyVehicle* PlatoonVehicle::define_virtual_leader_when_alone() const
{
	
	/* Note: this is a copy of 
	ConnectedAutonomousVehicle::define_virtual_leader() */
	//return ConnectedAutonomousVehicle::define_virtual_leader();

	/* By default, we try to merge behind the current destination
	lane leader. */
	NearbyVehicle* nv = get_modifiable_dest_lane_leader();
	
	/* The choice of min overtaking rel vel ensure that we only overtake
	if the veh is in free-flow mode and the dest lane leader is almost 
	stopped. */
	double min_vel = 1.5; // = 5.4 km/h
	double min_overtaking_rel_vel = get_desired_velocity() - min_vel;
	if (try_to_overtake_destination_lane_leader(min_overtaking_rel_vel))
	{
		nv = nullptr;
	}
	else if (was_my_cooperation_request_accepted())
	{
		/* We must avoid trying to merge behind a vehicle that is
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

bool PlatoonVehicle::is_vehicle_in_sight(long nearby_vehicle_id) const
{
	return get_nearby_vehicle_by_id(nearby_vehicle_id) != nullptr;
}

bool PlatoonVehicle::can_start_lane_change()
{
	bool is_safe = check_lane_change_gaps();
	bool is_my_turn;
	if (is_in_a_platoon())
	{
		//if (is_platoon_leader())
		//{
		//	get_platoon()->set_possible_maneuver_initial_states();
		//}
		is_my_turn = get_platoon()->can_vehicle_start_lane_change(get_id());
	}
	else
	{
		is_my_turn = true;
	}
	return is_safe && is_my_turn;
}

void PlatoonVehicle::implement_create_controller() {
	this->controller_exclusive = 
		std::make_unique<PlatoonVehicleController>(this, is_verbose());
	controller_exclusive->add_internal_controllers();
	set_controller(controller_exclusive.get());
}

//double PlatoonVehicle::implement_compute_desired_acceleration(
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	double a_desired_acceleration =
//		platoon_vehicle_controller->get_desired_acceleration();
//	return a_desired_acceleration;
//	//return consider_vehicle_dynamics(a_desired_acceleration);
//}

double PlatoonVehicle::compute_vehicle_following_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1;
	double rho;
	if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	{
		if (verbose) std::clog << "Leader identified as platoon veh.\n";
		current_lambda_1 = lambda_1_platoon;
		rho = in_platoon_rho;
	}
	else
	{
		current_lambda_1 = ConnectedAutonomousVehicle::get_lambda_1(
			nearby_vehicle.is_connected());
		rho = get_rho();
	}

	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, rho, 0);
}

double PlatoonVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1;
	double rho;
	if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	{
		current_lambda_1 = lambda_1_platoon;
		/*Not lambda_1_lane_change_platoon because we are not interessed 
		in this detail of safety for platoons */
		rho = in_platoon_rho;
	}
	else
	{
		current_lambda_1 = ConnectedAutonomousVehicle::
			get_lambda_1(nearby_vehicle.is_connected());
			/* Not get_lambda_1_lane_change(nearby_vehicle.is_connected());
			for the same reason as above. */
		rho = get_rho();
	}
	double h_lc = compute_time_headway_with_risk(get_desired_velocity(),
		/*get_lane_change_max_brake()*/
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, rho, 0);
	return h_lc;
}

void PlatoonVehicle::create_lane_change_request()
{
	if (is_in_a_platoon())
	{
		lane_change_request =
			platoon->create_lane_change_request_for_vehicle(*this);
	}
	else
	{
		lane_change_request = 0;
	}
}

bool PlatoonVehicle::was_my_cooperation_request_accepted() const
{
	if (lane_change_request != 0)
	{
		/* lane_change_request might be in the nearby vehicle
		list of some other platoon vehicle, but not ours */
		NearbyVehicle* cooperating_vehicle =
			get_nearby_vehicle_by_id(lane_change_request);
		if (cooperating_vehicle != nullptr)
		{
			long id_of_vehicle_being_assisted =
				cooperating_vehicle->get_assisted_vehicle_id();
			if (platoon->is_vehicle_id_in_platoon(id_of_vehicle_being_assisted))
			{
				return true;
			}
		}
	}
	return false;
}

NearbyVehicle* PlatoonVehicle::choose_virtual_leader()
{
	return is_in_a_platoon() ? 
		get_platoon()->define_virtual_leader(*this)
		: define_virtual_leader_when_alone();
}

void PlatoonVehicle::set_controller(PlatoonVehicleController* a_controller)
{
	platoon_vehicle_controller = a_controller;
	ConnectedAutonomousVehicle::set_controller(a_controller);
}

double PlatoonVehicle::compute_accepted_lane_change_gap(
	const NearbyVehicle* nearby_vehicle) const
{
	/* TODO [must check if working]: use simple lane keeping reference gaps */
	return compute_reference_vehicle_following_gap(nearby_vehicle);
}

void PlatoonVehicle::set_desired_lane_change_direction()
{
	bool should_change_lane = (get_link() == MAIN_LINK_NUMBER)
		&& (get_lane() == 1);
	desired_lane_change_direction = should_change_lane ?
		RelativeLane::left : RelativeLane::same;
}

void PlatoonVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_request_from_platoon();
	create_lane_change_request();
}

double PlatoonVehicle::compute_reference_vehicle_following_gap(
	const NearbyVehicle* nearby_vehicle) const
{
	return nearby_vehicle == nullptr ? 
		0.0 : platoon_vehicle_controller->get_lateral_controller()
		.compute_time_headway_gap(get_velocity(), *nearby_vehicle, 0.0);
}

void PlatoonVehicle::find_cooperation_request_from_platoon()
{
	/*if (is_in_a_platoon())
	{
		long assisted_vehicle_id =
			get_platoon()->get_assisted_vehicle_id(get_id());
		set_assisted_vehicle_by_id(assisted_vehicle_id);
	}*/
	long assisted_vehicle_id = 0;
	if (is_in_a_platoon())
	{
		for (auto&& id_veh_pair : get_nearby_vehicles())
		{
			NearbyVehicle* nearby_vehicle = id_veh_pair.second.get();
			if (platoon->is_vehicle_id_in_platoon(nearby_vehicle->get_id())
				&& (nearby_vehicle->get_lane_change_request_veh_id() 
					== get_id()))
			{
				assisted_vehicle_id = nearby_vehicle->get_id();
			}
		}
	}
	set_assisted_vehicle_by_id(assisted_vehicle_id);
}

bool PlatoonVehicle::implement_analyze_platoons(
	std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
	long new_platoon_id, int platoon_lc_strategy)
{
	bool new_platoon_created = false;
	bool am_in_a_platoon = is_in_a_platoon();
	bool leader_is_in_a_platoon = 
		has_leader() && get_leader()->is_in_a_platoon();
	bool may_join_leader_platoon =
		leader_is_in_a_platoon && (get_leader()->get_distance() < 60);

	/*std::shared_ptr<PlatoonVehicle> pointer_to_me_my_type =
		std::dynamic_pointer_cast<PlatoonVehicle>(pointer_to_me);*/

	if (!am_in_a_platoon && !may_join_leader_platoon)
	{
		// Create platoon
		create_platoon(new_platoon_id, platoon_lc_strategy);
		new_platoon_created = true;
	}
	else if (!am_in_a_platoon && may_join_leader_platoon)
	{
		// Join the platoon of the vehicle ahead
		long leader_platoon_id = get_leader()->get_platoon_id();
		add_myself_to_leader_platoon(platoons.at(leader_platoon_id));
	}
	else if (am_in_a_platoon && may_join_leader_platoon)
	{
		// Leave my platoon and join the platoon of the vehicle ahead
		long current_platoon_id = get_platoon_id();
		long leader_platoon_id = get_leader()->get_platoon_id();
		if (current_platoon_id != leader_platoon_id)
		{
			// remove myself
			platoon->remove_vehicle_by_id(get_id(), false);
			// add myself to leader platoon
			add_myself_to_leader_platoon(platoons.at(leader_platoon_id));
			// delete my old platoon if it is empty
			if (platoons.at(current_platoon_id)->is_empty())
			{
				platoons.erase(current_platoon_id);
			}
		}
	}
	else // am_in_a_platoon && !may_join_leader_platoon
	{
		if (verbose)
		{
			std::clog << "\t[PlatoonVehicle] Might leave platoon\n";
		}
		if (platoon->can_vehicle_leave_platoon(*this))
		{
			if (verbose)
			{
				std::clog << "\tt=" << get_current_time() 
					<< " id=" << get_id() 
					<< ": leaving platoon " << platoon->get_id() << "\n";
			}
			platoon->remove_vehicle_by_id(get_id(), false);
			//platoon.reset();
			create_platoon(new_platoon_id, platoon_lc_strategy);
			new_platoon_created = true;
		}
	}

	return new_platoon_created;
}

double PlatoonVehicle::apply_low_level_dynamics(
	double unfiltered_acceleration)
{
	return unfiltered_acceleration;
}

const Platoon* PlatoonVehicle::implement_get_platoon() const
{
	return platoon.get();
}

std::shared_ptr<Platoon> PlatoonVehicle::implement_share_platoon() const
{
	return platoon;
}

void PlatoonVehicle::pass_this_to_state()
{
	state->set_ego_vehicle(this);
}

void PlatoonVehicle::create_platoon(long platoon_id, 
	int platoon_lc_strategy)
{
	if (verbose)
	{
		std::clog << "Veh id " << get_id()
			<< ". Creating platoon id " << platoon_id << std::endl;
	}

	platoon = std::make_shared<Platoon>(platoon_id, 
		platoon_lc_strategy, this);
}

void PlatoonVehicle::add_myself_to_leader_platoon(
	std::shared_ptr<Platoon> leader_platoon)
{
	// Add my pointer to the platoon vehicles list
	leader_platoon->add_last_vehicle(this);
	// Update my platoon pointer
	platoon = leader_platoon;
	// Update my desired velocity
	set_desired_velocity(
		platoon->get_platoon_leader()->get_desired_velocity());
	/* Possibly more stuff:
	 Decrease desired time headway?
	 Deactivate velocity control?
	 Deactivate cooperation (so no one cuts in) ?
	*/
	alone_time = 0.0;
}

void PlatoonVehicle::compute_platoon_safe_gap_parameters()
{
	lambda_0_platoon =
		compute_lambda_0(max_jerk, in_platoon_comf_accel,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_platoon =
		compute_lambda_1(max_jerk, in_platoon_comf_accel,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_0_lane_change_platoon =
		compute_lambda_0(max_jerk, in_platoon_comf_accel,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
	lambda_1_lane_change_platoon =
		compute_lambda_1(max_jerk, in_platoon_comf_accel,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
}
