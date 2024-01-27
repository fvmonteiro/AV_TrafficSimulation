
#include "PlatoonVehicle.h"
#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonVehicleState.h"

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
		double safe_gap = compute_time_headway_gap(get_leader().get());
		double gap = compute_gap_to_a_leader(get_leader().get());
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
		double safe_gap = compute_time_headway_gap(get_leader().get());
		double gap = compute_gap_to_a_leader(get_leader().get());
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

std::shared_ptr<NearbyVehicle> PlatoonVehicle
::define_virtual_leader_when_alone() const
{
	return ConnectedAutonomousVehicle::choose_behind_whom_to_move();
}

bool PlatoonVehicle::can_start_lane_change()
{
	if (verbose) std::clog << "Can start lane change?\n";
	bool is_safe = check_lane_change_gaps();
	if (verbose) std::clog << "Gaps are safe? " 
		<< (is_safe ? "yes" : "no") << "\n";

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

	if (verbose) std::clog << "Start LC? "
		<< (is_my_turn ? "yes" : "no") << "\n";

	return is_safe && is_my_turn;
}

void PlatoonVehicle::add_another_as_nearby_vehicle(
	const PlatoonVehicle& platoon_vehicle)
{
	if (!is_vehicle_in_sight(platoon_vehicle.get_id()))
	{
		int rel_lane = platoon_vehicle.get_lane() - get_lane();
		double distance = 
			platoon_vehicle.get_distance_traveled() - get_distance_traveled();
		int rel_position = distance > 0 ? 3 : -3;
		double rel_velocity = 
			get_velocity() - platoon_vehicle.get_velocity();
		NearbyVehicle new_nv(platoon_vehicle.get_id(), rel_lane, rel_position);
		new_nv.set_distance(distance);
		new_nv.set_relative_velocity(rel_velocity);
		new_nv.set_length(platoon_vehicle.get_length());
		new_nv.set_category(platoon_vehicle.get_category());
		new_nv.set_type(platoon_vehicle.get_type(), get_type());
		add_nearby_vehicle(new_nv);
	}
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
		const NearbyVehicle* cooperating_vehicle =
			get_nearby_vehicle_by_id(lane_change_request).get();
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

void PlatoonVehicle::implement_prepare_to_start_long_adjustments()
{
	set_has_completed_lane_change(false);
}

void PlatoonVehicle::implement_prepare_to_restart_lane_keeping(
	bool was_lane_change_successful)
{
	set_has_completed_lane_change(was_lane_change_successful);
	set_lane_change_direction(RelativeLane::same);
	reset_lane_change_waiting_time();
}

std::shared_ptr<NearbyVehicle> PlatoonVehicle::choose_behind_whom_to_move() const
{
	if (is_in_a_platoon())
	{
		long vl_id = get_platoon()->define_desired_destination_lane_leader_id(
			get_id());
		return get_nearby_vehicle_by_id(vl_id);
	}
	else
	{
		return ConnectedAutonomousVehicle::choose_behind_whom_to_move();
	}
}

void PlatoonVehicle::set_controller(
	std::shared_ptr<PlatoonVehicleController> a_controller)
{
	platoon_vehicle_controller = a_controller;
	ConnectedAutonomousVehicle::set_controller(a_controller);
}

double PlatoonVehicle::compute_accepted_lane_change_gap(
	const NearbyVehicle* nearby_vehicle) const
{
	/* TODO [must check if working]: use simple lane keeping reference gaps */
	double safe_gap;
	if (nearby_vehicle == nullptr) 
	{
		safe_gap = 0.0;
	}
	else
	{
		safe_gap = platoon_vehicle_controller->get_lateral_controller()
			.compute_time_headway_gap(get_velocity(), *nearby_vehicle, 0.0);
	}
	return safe_gap;
	//return compute_reference_vehicle_following_gap(nearby_vehicle);
}

void PlatoonVehicle::set_desired_lane_change_direction()
{
	bool should_change_lane = (get_link() == MAIN_LINK_NUMBER)
		&& (get_lane() == 1);
	desired_lane_change_direction = should_change_lane ?
		RelativeLane::left : RelativeLane::same;
}

void PlatoonVehicle::implement_create_controller()
{
	set_controller(std::make_shared<PlatoonVehicleController>(
		this, is_verbose()));
	/*this->controller_exclusive =
		std::make_unique<PlatoonVehicleController>(this, is_verbose());
	controller_exclusive->add_internal_controllers();
	set_controller(controller_exclusive.get());*/
}

void PlatoonVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_request_from_platoon();

	std::shared_ptr<NearbyVehicle> desired_leader = 
		choose_behind_whom_to_move();
	if (desired_leader != nullptr && has_assisted_vehicle())
	{
		std::clog << "==== WARNING (unexpected behavior) ===="
			<<"[PlatoonVehicle] too many possible virtual leaders: "
			<< "both a desired dest lane leader and an assisted veh.\n";
	}
	else if (desired_leader != nullptr)
	{
		set_virtual_leader(desired_leader);
	}
	else if (has_assisted_vehicle())
	{
		set_virtual_leader(get_assisted_vehicle());
	}
	else
	{
		set_virtual_leader(nullptr);
	}
	
	if (verbose)
	{
		std::clog << "\tleader=" << get_leader_id()
			<< ", dest.lane leader=" << get_destination_lane_leader_id()
			<< ", dest.lane foll.=" << get_destination_lane_follower_id()
			<< ", assited veh=" << get_assisted_veh_id()
			<< ", virtual leader=" << get_virtual_leader_id() << "\n";
	}
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
	if (is_in_a_platoon())
	{
		long assisted_vehicle_id =
			get_platoon()->get_assisted_vehicle_id(get_id());
		set_assisted_vehicle_by_id(assisted_vehicle_id);
	}
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
		/*if (verbose) std::clog << "\t[PlatoonVehicle] May leave platoon\n";*/

		if (platoon->can_vehicle_leave_platoon(*this))
		{
			if (verbose)
			{
				std::clog << "\tt=" << get_current_time() 
					<< " id=" << get_id() 
					<< ": leaving platoon " << platoon->get_id() << "\n";
			}
			platoon->remove_vehicle_by_id(get_id(), false);
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
		platoon_lc_strategy, this, verbose);

	if (verbose) std::clog << "platoon created\n";
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
