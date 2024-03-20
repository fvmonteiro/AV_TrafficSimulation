
#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonVehicle.h"
#include "PlatoonVehicleState.h"

PlatoonVehicle::PlatoonVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	PlatoonVehicle(id, VehicleType::platoon_car,
		desired_velocity, AUTONOMOUS_BRAKE_DELAY, true, true,
		simulation_time_step, creation_time, verbose)
{
	if (verbose) std::cout << "[PlatoonVehicle] constructor done\n";
}

PlatoonVehicle::PlatoonVehicle(long id, VehicleType type,
	double desired_velocity, double brake_delay,
	bool is_lane_change_autonomous, bool is_connected,
	double simulation_time_step, double creation_time, bool verbose)
	: ConnectedAutonomousVehicle(id, type, desired_velocity, brake_delay,
		is_lane_change_autonomous, is_connected,
		simulation_time_step, creation_time, verbose)
{
	if (verbose) std::cout << "(platoon vehicles don't use lambda params)\n";
	//compute_platoon_safe_gap_parameters();
	//if (verbose)
	//{
	//	std::cout << "lambda 1 platoon = " << lambda_1_platoon
	//		<< ", lambda 0 platoon = " << lambda_0_platoon
	//		<< ", lambda 1 lc platoon = " << lambda_1_lane_change_platoon
	//		<< ", lambda 0 lc platoon = " << lambda_0_lane_change_platoon
	//		<< std::endl;
	//}
};

PlatoonVehicle::~PlatoonVehicle()
{
	if (verbose) 
	{
		std::cout << "[PlatoonVehicle] Destructor. t=" << get_current_time()
			<< ", veh " << get_id() << std::endl;
	}
}

double PlatoonVehicle::decide_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	if (is_in_a_platoon())
	{
		return platoon->is_vehicle_id_in_platoon(nearby_vehicle.get_id()) ?
			SAFE_PLATOON_TIME_HEADWAY : SAFE_TIME_HEADWAY;
	}
	else
	{
		/* This code will only be run at the first simulation step 
		when platoons haven't been set yet. */
		return nearby_vehicle.is_connected() ?
			SAFE_PLATOON_TIME_HEADWAY : SAFE_TIME_HEADWAY;
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

long PlatoonVehicle::get_suitable_destination_lane_leader_id() const
{
	return get_is_space_suitable_for_lane_change() ?
		get_destination_lane_leader_id() : 0;
}

double PlatoonVehicle::get_desired_velocity_from_platoon() const
{
	/* The goal here is to prevent the platoon from "speeding ahead" 
	while some other platoon vehicles are decelerating while adjusting 
	for a gap.	
	We can control the entire platoon by limiting only its leader's 
	speed. */
	if (is_in_a_platoon() && is_platoon_leader()
		&& has_lane_change_intention() 
		&& get_platoon()->has_lane_change_intention())
	{
		const NearbyVehicle* nv =
			get_platoon()->get_destination_lane_leader();
		/* The the platoon's dest lane leader only equals the platoon's 
		leader virtual leader when the platoon leader is the first vehicle
		to change lanes. In this case, we don't need to limit its speed. */
		return nv != nullptr && nv->get_id() != get_virtual_leader_id() ?
			compute_nearby_vehicle_velocity(*nv)
			: get_desired_velocity();
	}
	return get_desired_velocity();
}

bool PlatoonVehicle::can_start_lane_change()
{
	if (verbose) std::cout << "Can start lane change?\n";
	bool is_safe = are_surrounding_gaps_safe_for_lane_change();
	if (verbose) std::cout << "Gaps are safe? "
		<< boolean_to_string(is_safe) << "\n";

	bool is_my_turn;
	if (is_in_a_platoon())
	{
		is_my_turn = get_platoon()->can_vehicle_start_lane_change(get_id());
	}
	else
	{
		is_my_turn = true;
	}

	if (verbose) std::cout << "Start LC? "
		<< boolean_to_string(is_my_turn) << "\n";

	return is_safe && is_my_turn;
}

//bool PlatoonVehicle::is_at_right_lane_change_gap() const
//{
//	bool is_dest_lane_leader_correct =
//		get_virtual_leader_id() == get_destination_lane_leader_id();
//	bool is_dest_lane_follower_correct;
//	if (is_in_a_platoon())
//	{
//		long coop_id = platoon->get_cooperating_vehicle_id();
//		is_dest_lane_follower_correct =
//			coop_id == 0 || coop_id == get_destination_lane_follower_id();
//	}
//	else
//	{
//		is_dest_lane_follower_correct = true;
//	}
//	return (is_dest_lane_leader_correct && is_dest_lane_follower_correct);
//}

void PlatoonVehicle::add_another_as_nearby_vehicle(
	const PlatoonVehicle& platoon_vehicle)
{
	if (!is_vehicle_in_sight(platoon_vehicle.get_id()))
	{
		int rel_lane = platoon_vehicle.get_lane() - get_lane();
		/* The platoon leader has pos. 0 and the last veh has pos. N.
		position_diff > 0: other is behind */
		int position_diff =
			platoon->get_vehicle_position_in_platoon(platoon_vehicle.get_id())
			- platoon->get_vehicle_position_in_platoon(get_id());
		int rel_position = position_diff < 0 ? 3 : -3;
		/* Approximate distance to other vehicle. */
		double h = SAFE_PLATOON_TIME_HEADWAY + TIME_HEADWAY_MARGIN;
		double safe_gap = h * get_velocity() + 1.0;
		double distance = -position_diff * safe_gap;
		double rel_velocity =
			get_velocity() - platoon_vehicle.get_velocity();
		NearbyVehicle new_nv(platoon_vehicle.get_id(), rel_lane, rel_position);
		new_nv.set_distance(distance);
		new_nv.set_relative_velocity(rel_velocity);
		new_nv.set_length(platoon_vehicle.get_length());
		new_nv.set_category(platoon_vehicle.get_category());
		new_nv.set_type(platoon_vehicle.get_type(), get_type());
		add_nearby_vehicle(new_nv);

		if (verbose) std::cout << "\t\tAdding another to my nv list "
			<< "pos diff = " << position_diff
			<< "\nNew nv: " << new_nv << "\n";
	}
}

void PlatoonVehicle::give_up_lane_change()
{
	if (verbose) std::cout << "t=" << get_current_time()
		<< ", id=" << get_id() << ": giving up lane change\n";
	has_lane_change_failed = true;
}

double PlatoonVehicle::get_free_flow_intra_platoon_gap() const
{
	return (SAFE_PLATOON_TIME_HEADWAY + TIME_HEADWAY_MARGIN)
		* get_desired_velocity() + 1.0; 
}

const VehicleState* PlatoonVehicle::get_preceding_vehicle_state() const
{
	const auto& leader = platoon->get_preceding_vehicle(get_id());
	return leader == nullptr ? nullptr : leader->get_state();
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
	//double current_lambda_1;
	//double rho;
	//if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	//{
	//	if (verbose) std::cout << "Leader identified as platoon veh.\n";
	//	current_lambda_1 = lambda_1_platoon;
	//	rho = in_platoon_rho;
	//}
	//else
	//{
	//	current_lambda_1 = ConnectedAutonomousVehicle::get_lambda_1(
	//		nearby_vehicle.is_connected());
	//	rho = get_rho();
	//}

	//return compute_time_headway_with_risk(get_desired_velocity(),
	//	get_max_brake(), nearby_vehicle.get_max_brake(),
	//	current_lambda_1, rho, 0);
	return decide_safe_time_headway(nearby_vehicle);
}

double PlatoonVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	//double current_lambda_1;
	//double rho;
	//if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	//{
	//	current_lambda_1 = lambda_1_platoon;
	//	/*Not lambda_1_lane_change_platoon because we are not interessed 
	//	in this detail of safety for platoons */
	//	rho = in_platoon_rho;
	//}
	//else
	//{
	//	current_lambda_1 = ConnectedAutonomousVehicle::
	//		get_lambda_1(nearby_vehicle.is_connected());
	//		/* Not get_lambda_1_lane_change(nearby_vehicle.is_connected());
	//		for the same reason as above. */
	//	rho = get_rho();
	//}
	//double h_lc = compute_time_headway_with_risk(get_desired_velocity(),
	//	/*get_lane_change_max_brake()*/
	//	get_max_brake(), nearby_vehicle.get_max_brake(),
	//	current_lambda_1, rho, 0);
	//return h_lc;
	return decide_safe_time_headway(nearby_vehicle);
}

void PlatoonVehicle::create_lane_change_request()
{
	if (is_in_a_platoon())
	{
		lane_change_request =
			platoon->create_lane_change_request_for_vehicle(get_id());
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
	platoon->receive_lane_change_intention_signal();
}

void PlatoonVehicle::implement_prepare_to_restart_lane_keeping(
	bool was_lane_change_successful)
{
	set_has_completed_lane_change(was_lane_change_successful);
	set_lane_change_direction(RelativeLane::same);
	reset_lane_change_waiting_time();
	platoon->receive_lane_keeping_signal();
}

std::shared_ptr<NearbyVehicle> PlatoonVehicle::choose_behind_whom_to_move() 
const
{
	if (verbose) std::cout << "Choosing behind whom to move\n";
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
	const NearbyVehicle* nearby_vehicle, double lane_change_speed) const
{
	
	if (nearby_vehicle == nullptr) return 0.0;
	
	double safe_gap = platoon_vehicle_controller->get_lateral_controller()
		.compute_time_headway_gap(lane_change_speed, *nearby_vehicle, 0.0);
	if (!nearby_vehicle->is_on_same_lane())
	{
		double leader_vel, leader_brake, follower_vel, follower_brake;
		if (nearby_vehicle->is_ahead())
		{
			follower_vel = lane_change_speed;
			follower_brake = get_max_brake();
			leader_vel = compute_nearby_vehicle_velocity(*nearby_vehicle);
			leader_brake = nearby_vehicle->get_max_brake();
		}
		else
		{
			follower_vel = compute_nearby_vehicle_velocity(*nearby_vehicle);
			follower_brake = nearby_vehicle->get_max_brake();
			leader_vel = lane_change_speed;
			leader_brake = get_max_brake();
		}
		double rel_vel_term = std::pow(follower_vel, 2) / 2 / follower_brake
			- std::pow(leader_vel, 2) / 2 / leader_brake;
		safe_gap += std::max(0., rel_vel_term);
	}
	
	return safe_gap;
}

void PlatoonVehicle::set_desired_lane_change_direction()
{
	bool should_change_lane = (get_link() == MAIN_LINK_NUMBER)
		&& (get_lane() == 1);
	desired_lane_change_direction = 
		should_change_lane && !has_lane_change_failed ?
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
		has_lane_change_intention() ? 
		choose_behind_whom_to_move() : nullptr;
	if (desired_leader != nullptr && has_assisted_vehicle())
	{
		std::cout << "==== WARNING (unexpected behavior) ===="
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

	if (has_lane_change_intention()) 
	{
		check_lane_change_gaps();
		check_adjacent_space_suitability();
	}

	if (verbose)
	{
		std::cout << "Surrounding vehicles:\n"
			<< "\tleader=" << get_leader_id()
			<< ", dest.lane leader=" << get_destination_lane_leader_id()
			<< ", dest.lane foll.=" << get_destination_lane_follower_id()
			<< ", assisted veh=" << get_assisted_veh_id()
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

void PlatoonVehicle::check_adjacent_space_suitability()
{
	if (verbose) std::cout << "Checking gap feasibility\n";

	double margin = 0.0;
	is_space_suitable_for_lane_change =
		lane_change_gaps_safety.is_lane_change_safe();
	if (!is_space_suitable_for_lane_change)
	{
		double gap_to_ld = compute_gap_to_a_leader(
			get_destination_lane_leader().get());
		double gap_to_fd = compute_gap_to_a_follower(
			get_destination_lane_follower().get());
		double dest_lane_leader_vel = compute_nearby_vehicle_velocity(
			get_destination_lane_leader().get(), INFINITY);
		/* [March 18] Running tests here - be careful not to insert new bugs */
		double lc_speed = std::min(get_velocity(), dest_lane_leader_vel);
		double safe_gap_to_ld_after_decel = compute_accepted_lane_change_gap(
			get_destination_lane_leader().get(), lc_speed);
		double safe_gap_to_fd_after_decel = compute_accepted_lane_change_gap(
			get_destination_lane_follower().get(), lc_speed);
		double min_gap_to_decelerate = (
			(std::pow(get_velocity(), 2) - std::pow(dest_lane_leader_vel, 2))
			/ 2 / get_comfortable_brake()
			);
		double existing_free_space = gap_to_ld + gap_to_fd;
		double needed_space = safe_gap_to_fd_after_decel
			+ safe_gap_to_ld_after_decel;
		is_space_suitable_for_lane_change =
			lane_change_gaps_safety.dest_lane_follower_gap
			&& gap_to_ld >= min_gap_to_decelerate
			&& (existing_free_space + margin >= needed_space);
		if (verbose)
		{
			std::cout << "\t- Safe to fd? "
				<< boolean_to_string(lane_change_gaps_safety.dest_lane_follower_gap)
				<< "\n\t- gap to ld = " << gap_to_ld
				<< ", min_gap_to_decel = " << min_gap_to_decelerate
				<< "\n\t- total space = " << existing_free_space + margin
				<< ", needed gap = " << needed_space
				<< "\n";
		}
	}
	if (verbose)
	{
		std::cout << "\tIs adjacent gap suitable? "
			<< boolean_to_string(is_space_suitable_for_lane_change) << "\n";
	}
}

void PlatoonVehicle::find_cooperation_request_from_platoon()
{
	if (is_in_a_platoon() && !get_platoon()->is_lane_change_done())
	{
		long assisted_vehicle_id =
			get_platoon()->get_assisted_vehicle_id(get_id());
		if (assisted_vehicle_id > 0 
			&& !is_vehicle_in_sight(assisted_vehicle_id))
		{
			add_another_as_nearby_vehicle(
				*platoon->get_vehicle_by_id(assisted_vehicle_id));
		}
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
		/* In discretionary lane change scenarios, we will never split
		* platoons. In the worst-case, the vehicles in the platoon will 
		* just never complete lane changes.
		* If mandatory lane change scenarios are included, splitting becomes
		* necessary.
		*/

		/*if (verbose) std::cout << "\t[PlatoonVehicle] May leave platoon\n";*/
		//if (platoon->can_vehicle_leave_platoon(*this))
		//{
		//	if (verbose)
		//	{
		//		std::cout << "\tt=" << get_current_time() 
		//			<< " id=" << get_id() 
		//			<< ": leaving platoon " << platoon->get_id() << "\n";
		//	}
		//	platoon->remove_vehicle_by_id(get_id(), false);
		//	create_platoon(new_platoon_id, platoon_lc_strategy);
		//	new_platoon_created = true;
		//}
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
		std::cout << "Veh id " << get_id()
			<< ". Creating platoon id " << platoon_id << std::endl;
	}

	platoon = std::make_shared<Platoon>(platoon_id, 
		platoon_lc_strategy, this, verbose);

	if (verbose) std::cout << "platoon created\n";
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
