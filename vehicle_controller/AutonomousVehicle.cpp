#include "AutonomousVehicle.h"

AutonomousVehicle::AutonomousVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose)
	: AutonomousVehicle(id, VehicleType::autonomous_car, 
		AUTONOMOUS_BRAKE_DELAY, desired_velocity, true, false,
		simulation_time_step, creation_time, verbose)
{
	if (verbose) std::clog << "[AutonomousVehicle] created" <<std::endl;
}

bool AutonomousVehicle::has_destination_lane_leader_leader() const
{
	return destination_lane_leader_leader != nullptr;
}

long AutonomousVehicle::get_destination_lane_leader_leader_id() const
{
	return has_destination_lane_leader_leader() ?
		destination_lane_leader_leader->get_id() : 0;
}

bool AutonomousVehicle::has_virtual_leader() const
{
	return virtual_leader != nullptr;
}

bool AutonomousVehicle::are_surrounding_gaps_safe_for_lane_change() const
{
	return lane_change_gaps_safety.is_lane_change_safe();
}

bool AutonomousVehicle::get_is_space_suitable_for_lane_change() const
{
	return is_space_suitable_for_lane_change;
}

LaneChangeGapsSafety AutonomousVehicle::get_lane_change_gaps_safety() const
{
	return lane_change_gaps_safety;
}

double AutonomousVehicle::get_dest_follower_time_headway() const
{
	return av_controller->get_lateral_controller().
		get_destination_lane_follower_time_headway();
}

double AutonomousVehicle::compute_transient_gap(const NearbyVehicle* nearby_vehicle)
{
	double transient_gap = 0.0;
	if (nearby_vehicle != nullptr)
	{
		transient_gap = av_controller->get_lateral_controller().
			compute_transient_gap(*this, *nearby_vehicle, false);
	}
	return transient_gap;
}

void AutonomousVehicle::set_virtual_leader(
	NearbyVehicle* new_virtual_leader)
{
	NearbyVehicle* old_virtual_lane_leader = virtual_leader;
	virtual_leader = new_virtual_leader;
	update_virtual_leader(old_virtual_lane_leader);
}

//void AutonomousVehicle::set_destination_lane_follower_by_id(
//	long new_follower_id)
//{
//	// Note: this makes the destination_lane_leader = nullptr
//	std::shared_ptr<NearbyVehicle> old_dest_lane_foll =
//		std::move(destination_lane_follower);
//	destination_lane_follower = get_nearby_vehicle_by_id(new_follower_id);
//	update_destination_lane_follower(old_dest_lane_foll);
//}

void AutonomousVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
}

void AutonomousVehicle::find_destination_lane_vehicles()
{
	/* Note: move operation "empties" the current pointer values */
	NearbyVehicle* old_dest_lane_follower = destination_lane_follower;
	NearbyVehicle* old_dest_lane_leader = destination_lane_leader;
	destination_lane_follower = nullptr;
	destination_lane_leader = nullptr;
	destination_lane_leader_leader = nullptr;
	if (has_lane_change_intention())
	{
		for (auto const& id_veh_pair : get_nearby_vehicles())
		{
			NearbyVehicle* nearby_vehicle = id_veh_pair.second.get();
			if (is_destination_lane_follower(*nearby_vehicle))
			{
				destination_lane_follower = nearby_vehicle;
			}
			else if (is_destination_lane_leader(*nearby_vehicle))
			{
				destination_lane_leader = nearby_vehicle;
			}
			else if (is_leader_of_destination_lane_leader(*nearby_vehicle))
			{
				destination_lane_leader_leader = nearby_vehicle;
			}
		}
	}

	update_destination_lane_follower(old_dest_lane_follower);
	update_destination_lane_leader(old_dest_lane_leader);
	NearbyVehicle* vl = choose_virtual_leader();
	set_virtual_leader(vl);
}

NearbyVehicle* AutonomousVehicle::choose_virtual_leader()
{
	if (try_to_overtake_destination_lane_leader())
	{
		return nullptr;
	}
	else
	{
		return destination_lane_leader;
	}
}

//bool AutonomousVehicle::is_destination_lane_leader_stuck() const
//{
//	/* To avoid deadlocks, we try to overtake the destination lane leader 
//	in case	it is stopped and has no leader. This situation means that the
//	dest lane leader is not changing lanes because we are too close to it.*/
//	if (has_destination_lane_leader()
//		&& (destination_lane_leader->compute_velocity(get_velocity()) < 0.1)
//		&& (destination_lane_leader_leader == nullptr))
//	{
//		/*destination_lane_follower = destination_lane_leader;
//		destination_lane_leader = nullptr;*/
//		return true;
//	}
//	return false;
//}

bool AutonomousVehicle::try_to_overtake_destination_lane_leader(
	double min_rel_vel) const
{
	if (!has_destination_lane_leader()) return true;

	double ego_velocity = get_velocity();
	double dest_lane_leader_vel =
		get_destination_lane_leader()->compute_velocity(ego_velocity);
	double origin_lane_desired_velocity =
		(av_controller->is_in_free_flow_at_origin_lane() || !has_leader()) ?
		get_desired_velocity()
		: get_leader()->compute_velocity(ego_velocity);

	/* We say the dest lane leader is stuck if it is not moving even
	though it has no leader. This can happen when vehicles in different
	lanes trying to "switch" lanes. */
	bool is_dest_lane_leader_stuck =
		dest_lane_leader_vel < 0.1
		&& (!has_destination_lane_leader_leader());
	bool is_desired_vel_higher = dest_lane_leader_vel
		< (origin_lane_desired_velocity - min_rel_vel);

	if (verbose)
	{
		std::clog << "\tv_o=" << origin_lane_desired_velocity
			<< ", v_d=" << dest_lane_leader_vel 
			<< ", is_desired_vel_higher? " << is_desired_vel_higher 
			<< ", dest lane leader has leader? "
			<< has_destination_lane_leader_leader()
			<< ", is dest lane leader stuck? " 
			<< is_dest_lane_leader_stuck <<"\n";
	}

	/* [Jan 24, 2023] Note: maybe the first condition is redundant. */
	return is_dest_lane_leader_stuck || is_desired_vel_higher;
}

bool AutonomousVehicle::try_to_overtake_destination_lane_leader() const
{
	return try_to_overtake_destination_lane_leader(
		min_overtaking_rel_vel);
}

bool AutonomousVehicle::
try_to_overtake_destination_lane_leader_based_on_time() const
{
	if (!has_destination_lane_leader()) return true;

	double ego_vel = get_velocity();
	double dest_lane_leader_vel =
		get_destination_lane_leader()->compute_velocity(ego_vel);
	double orig_lane_desired_vel =
		(av_controller->is_in_free_flow_at_origin_lane() || !has_leader()) ?
		get_desired_velocity()
		: get_leader()->compute_velocity(ego_vel);

	double relative_desired_velocity = orig_lane_desired_vel
		- dest_lane_leader_vel;
	double gap = compute_gap_to_a_leader(*get_destination_lane_leader());
	double overtaking_time = gap / relative_desired_velocity;

	/* We say the dest lane leader is stuck if it is not moving even
	though it has no leader. This can happen when vehicles in different
	lanes trying to "switch" lanes. */
	bool is_dest_lane_leader_stuck =
		dest_lane_leader_vel < 0.1
		&& (get_destination_lane_leader_leader() == nullptr);
	bool is_overtaking_time_acceptable = 
		overtaking_time < min_overtaking_time;

	if (verbose)
	{
		std::clog << "\tv_o=" << orig_lane_desired_vel
			<< ", v_d=" << dest_lane_leader_vel << "\n";
	}

	/* [Jan 24, 2023] Note: maybe the first condition is redundant. */
	return is_dest_lane_leader_stuck || is_overtaking_time_acceptable;
}

void AutonomousVehicle::update_destination_lane_follower(
	const NearbyVehicle* old_follower)
{
	if (has_destination_lane_follower())
	{
		get_is_connected();
		if (old_follower == nullptr
			|| (old_follower->get_category()
				!= destination_lane_follower->get_category()))
		{
			av_controller->update_destination_lane_follower_parameters(
				*destination_lane_follower);
			av_controller->update_destination_lane_follower_time_headway(
				false, *destination_lane_follower);
		}
	}
}

void AutonomousVehicle::update_destination_lane_leader(
	const NearbyVehicle* old_leader)
{
	if (has_destination_lane_leader())
	{
		double new_leader_max_brake = 
			destination_lane_leader->get_max_brake();
		if (old_leader == nullptr
			|| (std::abs(new_leader_max_brake
				- old_leader->get_max_brake()) > 0.5)
			|| (old_leader->get_type()
				!= destination_lane_leader->get_type()))
		{
			if (verbose)
			{
				std::clog << "\tUpdating the dest lane leader"
					<< std::endl;
			}
			av_controller->update_destination_lane_leader_time_headway(
				compute_lane_changing_desired_time_headway(
					*destination_lane_leader));
		}
	}
}

void AutonomousVehicle::update_virtual_leader(
	const NearbyVehicle* old_leader)
{
	if (has_virtual_leader())
	{
		if (verbose) std::clog << "\tHas a virtual leader\n";

		double new_leader_max_brake = get_virtual_leader()->get_max_brake();
		bool is_new_leader_connected = get_virtual_leader()->is_connected();
		if (old_leader == nullptr)
		{
			if (verbose) std::clog << "Activating dest lane ctrl.\n";

			av_controller->activate_destination_lane_controller(*get_virtual_leader());
		}
		else if ((std::abs(new_leader_max_brake
			- old_leader->get_max_brake()) > 0.5)
			|| (old_leader->get_type()
				!= get_virtual_leader()->get_type()))
		{
			if (verbose) std::clog << "Updating dest lane ctrl.\n";

			av_controller->update_destination_lane_controller(*get_virtual_leader());
		}
	}
}

double AutonomousVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double h_lc = compute_time_headway_with_risk(
		get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle.get_max_brake(),
		get_lambda_1_lane_change(), get_rho(), 0
		/*accepted_lane_change_risk_to_leaders*/
	);
	return h_lc;
	/* We only allow the lane changing headway towards the leaders to be below
	the safe vehicle following headway if a risky headway to the future
	follower is also allowed */
	/*if (accepted_lane_change_risk_to_follower <= 0)
	{
		return std::max(h_lc,
			compute_vehicle_following_safe_time_headway(nearby_vehicle));
	}
	else
	{
		return h_lc;
	}*/
}

/* TODO: not yet sure whether this function should belong in this class
or in some controller class */
double AutonomousVehicle::estimate_nearby_vehicle_time_headway(
	NearbyVehicle& nearby_vehicle)
{
	//double accepted_risk = accepted_lane_change_risk_to_follower;
	//// Condition below is just to avoid unnecessary computations
	//if (accepted_lane_change_risk_to_follower > 0)
	//{
	//	double max_risk =
	//		nearby_vehicle.estimate_max_accepted_risk_to_incoming_vehicle(
	//			get_desired_velocity(), max_brake, get_rho());
	//	accepted_risk = std::min(max_risk, accepted_lane_change_risk_to_follower);
	//	if (verbose)
	//	{
	//		std::clog << "Estimating follower's headway.\n" <<
	//			"ar = " << accepted_lane_change_risk_to_follower
	//			<< ", max_risk = " << max_risk << std::endl;
	//	}
	//}
	//return nearby_vehicle.estimate_desired_time_headway(get_desired_velocity(),
	//	max_brake, get_rho(), accepted_risk);
	return std::max(0.0,
		nearby_vehicle.estimate_desired_time_headway(max_brake, 0));
}

void AutonomousVehicle::set_controller(AVController* a_controller)
{
	this->av_controller = a_controller;
	EgoVehicle::set_controller(av_controller);
}

void AutonomousVehicle::implement_create_controller()
{
	this->controller_exclusive = AVController(this, is_verbose());
	controller_exclusive.add_internal_controllers();
	this->set_controller(&controller_exclusive);
}

//double AutonomousVehicle::implement_compute_desired_acceleration(
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	double a_desired_acceleration =
//		av_controller->get_desired_acceleration();
//	return apply_low_level_dynamics(a_desired_acceleration);
//}

bool AutonomousVehicle::give_lane_change_control_to_vissim() const
{
	return lane_change_waiting_time > max_lane_change_waiting_time;
}

bool AutonomousVehicle::implement_check_lane_change_gaps()
{
	if (give_lane_change_control_to_vissim())
	{
		return get_vissim_lane_suggestion() != RelativeLane::same;
	}

	double gap_to_lo, gap_to_ld, gap_to_fd;
	double safe_gap_to_lo, safe_gap_to_ld, safe_gap_to_fd;
	double margin = 0.1;


	if (verbose) std::clog << "Deciding lane changing safety\n"
			<< "\t- To orig lane leader:\n";

	gap_to_lo = compute_gap_to_a_leader(get_leader());  // possibly MAX_DIST
	safe_gap_to_lo = compute_accepted_lane_change_gap(
			get_leader()); // possibly 0.0
	lane_change_gaps_safety.orig_lane_leader_gap =
		gap_to_lo + margin >= safe_gap_to_lo;
	/*lane_change_gaps_safety.orig_lane_leader_gap = 
		is_lane_change_gap_safe(get_leader());*/

	if (verbose) std::clog << "\t- To dest lane leader:\n";

	gap_to_ld = compute_gap_to_a_leader(destination_lane_leader);
	safe_gap_to_ld = compute_accepted_lane_change_gap(
		destination_lane_leader);
	lane_change_gaps_safety.dest_lane_leader_gap =
		gap_to_ld + margin >= safe_gap_to_ld;
	/*lane_change_gaps_safety.dest_lane_leader_gap =
		is_lane_change_gap_safe(destination_lane_leader);*/
	
	if (verbose) std::clog << "\t- To dest lane follower:\n";
	
	/* Besides the regular safety conditions, we add the case
	where the dest lane follower has completely stopped to give room
	to the lane changing vehicle */
	gap_to_fd = compute_gap_to_a_follower(destination_lane_follower);
	safe_gap_to_fd = compute_accepted_lane_change_gap(
		destination_lane_follower);
	lane_change_gaps_safety.dest_lane_follower_gap = 
		(gap_to_fd + margin >= safe_gap_to_fd)
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -2.0));
	/*lane_change_gaps_safety.dest_lane_follower_gap = 
		is_lane_change_gap_safe(destination_lane_follower)
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -2.0)); */

	lane_change_gaps_safety.no_conflict =
		!has_lane_change_conflict();

	is_space_suitable_for_lane_change =
		(gap_to_ld + gap_to_fd + margin)
		>= (safe_gap_to_fd + safe_gap_to_ld + get_length());
	return lane_change_gaps_safety.is_lane_change_safe();
}

bool AutonomousVehicle::is_lane_change_gap_safe(
	const NearbyVehicle* nearby_vehicle) const
{
	if (nearby_vehicle == nullptr) return true;

	double margin = 0.1;
	double gap = nearby_vehicle->is_ahead() ?
		compute_gap_to_a_leader(nearby_vehicle)
		: compute_gap_to_a_follower(nearby_vehicle);

	if (verbose)
	{
		std::clog << "\tg=" << gap << "\n";
	}

	double accepted_gap = compute_accepted_lane_change_gap(
		nearby_vehicle);

	return gap + margin >= accepted_gap;
}

bool AutonomousVehicle::has_lane_change_conflict() const
{
	/* If there's no lane change intention, there's no conflict */
	if (!has_lane_change_intention()) return false;

	for (auto const& id_veh_pair : get_nearby_vehicles())
	{
		NearbyVehicle* nearby_vehicle = id_veh_pair.second.get();
		if (nearby_vehicle->is_lane_changing())
		{
			RelativeLane& nv_lane = nearby_vehicle->get_relative_lane();
			RelativeLane& nv_lc_direction = 
				nearby_vehicle->get_lane_change_direction();

			// Vehicles on the same lane
			if (nv_lane == RelativeLane::same)
			{
				if (nv_lc_direction == desired_lane_change_direction)
				{
					return true;
				}
			}
			// Vehicles on other lanes
			else
			{
				bool nv_moving_towards_ego =
					!nv_lane.on_same_side(nv_lc_direction);
				bool ego_moving_towards_nv =
					nv_lane.on_same_side(desired_lane_change_direction);
				if (nv_moving_towards_ego && ego_moving_towards_nv)
				{
					return true;
				}
			}
		}
	}
	return false;
}

double AutonomousVehicle::compute_accepted_lane_change_gap(
	const NearbyVehicle* nearby_vehicle) const
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
		accepted_gap = av_controller->compute_accepted_lane_change_gap(*nearby_vehicle,
				accepted_risk);
	}
	else
	{
		accepted_gap = av_controller->compute_accepted_lane_change_gap_exact(
			*nearby_vehicle, get_lane_changing_safe_gap_parameters(), accepted_risk);
	}

	return std::max(accepted_gap, 1.0);
}

double AutonomousVehicle::compute_time_headway_gap_for_lane_change(
	const NearbyVehicle& nearby_vehicle) const
{
	/* [Jan 23, 2023] Now that we have both destination_lane_leader
	and virtual_leader, this function is outdated. The controller stores 
	data from the virtual leader, so it should not be used to measure 
	safety towards the destination lane leader (which might be different 
	from the virtual leader) */
	if (verbose) std::clog << "[WARNING] USING OUTDATED METHOD\n";

	double accepted_time_headway_gap =
		av_controller->get_desired_time_headway_gap(nearby_vehicle);

	double accepted_risk = nearby_vehicle.is_ahead() ?
		accepted_lane_change_risk_to_leaders :
		accepted_lane_change_risk_to_follower;

	if (accepted_risk > 0)
	{
		double leader_max_brake, follower_max_brake, follower_lambda_1;
		if (nearby_vehicle.is_ahead())
		{
			leader_max_brake = nearby_vehicle.get_max_brake();
			follower_max_brake = get_lane_change_max_brake();
			follower_lambda_1 = get_lambda_1_lane_change();
		}
		else
		{
			leader_max_brake = get_max_brake();
			follower_max_brake = nearby_vehicle.get_max_brake();
			follower_lambda_1 = nearby_vehicle.get_lambda_1();
		}
		double rho = get_rho();
		double vf = get_desired_velocity();
		double gamma = leader_max_brake / follower_max_brake;
		double Gamma = (1 - rho) * vf / (vf + follower_lambda_1);

		double denominator = gamma >= Gamma ? 1 : (1 - gamma);
		denominator *= 2 * follower_max_brake;
		double risk_term = std::pow(accepted_risk, 2) / denominator;
		accepted_time_headway_gap -= risk_term;
	}

	return std::max(0.0, accepted_time_headway_gap);
}

//double AutonomousVehicle::compute_vehicle_following_gap_for_lane_change(
//	const NearbyVehicle& nearby_vehicle, double current_lambda_1) const
//{
//	double follower_lambda_0, follower_lambda_1, accepted_risk;
//	double v_follower, v_leader;
//	double brake_follower, brake_leader;
//	double ego_velocity = get_velocity();
//	double delta_v = nearby_vehicle.get_relative_velocity();
//	if (nearby_vehicle.is_ahead())
//	{
//		accepted_risk = accepted_lane_change_risk_to_leaders;
//		follower_lambda_0 = get_lambda_0();
//		follower_lambda_1 = current_lambda_1;
//		v_follower = ego_velocity;
//		v_leader = nearby_vehicle.compute_velocity(ego_velocity);
//		brake_follower = get_lane_change_max_brake();
//		brake_leader = nearby_vehicle.get_max_brake();
//	}
//	else
//	{
//		accepted_risk = accepted_lane_change_risk_to_follower;
//		follower_lambda_0 = dest_lane_follower_lambda_0;
//		follower_lambda_1 = dest_lane_follower_lambda_1;
//		v_leader = ego_velocity;
//		v_follower = nearby_vehicle.compute_velocity(ego_velocity);
//		brake_follower = nearby_vehicle.get_max_brake();
//		brake_leader = max_brake;
//		delta_v = -delta_v;
//	}
//
//	double accepted_risk_2 = std::pow(accepted_risk, 2);
//	double stop_time_follower = (v_follower + follower_lambda_1)
//		/ brake_follower;
//	double stop_time_leader = v_leader / brake_leader;
//
//	double accepted_gap;
//	if (stop_time_follower >= stop_time_leader)
//	{
//		accepted_gap =
//			(std::pow(v_follower + follower_lambda_1, 2)
//				- accepted_risk_2) / 2 / brake_follower
//			- std::pow(v_leader, 2) / 2 / brake_leader
//			+ follower_lambda_0;
//	}
//	else if (brake_follower > brake_leader)
//	{
//		accepted_gap =
//			(std::pow(delta_v + follower_lambda_1, 2)
//				- accepted_risk_2) / 2 / (brake_follower - brake_leader)
//			+ follower_lambda_0;
//	}
//	else
//	{
//		accepted_gap = 0.0;
//	}
//
//	if (verbose)
//	{
//		std::clog << "\tVeh following gap computation\n\t"
//			<< "vf=" << v_follower
//			<< ", lambda1=" << follower_lambda_1
//			<< ", df=" << brake_follower
//			<< ", vl=" << v_leader
//			<< ", dl=" << brake_leader
//			<< ", lambda 0=" << follower_lambda_0
//			<< "\n\tt_f=" << stop_time_follower
//			<< ", t_l=" << stop_time_leader
//			<< ", g_vf=" << accepted_gap
//			<< std::endl;
//	}
//
//	return std::max(0.0, accepted_gap);
//}

//void AutonomousVehicle::compute_lane_change_risks()
//{
//	if (!has_lane_change_intention()) return;
//
//	std::vector <std::shared_ptr<NearbyVehicle>> relevant_vehicles{
//		get_leader(),
//		get_destination_lane_leader(),
//		get_destination_lane_follower()
//	};
//
//	for (std::shared_ptr<NearbyVehicle> nv : relevant_vehicles)
//	{
//		if (verbose) std::clog << "is nullptr? " << (nv == nullptr) << std::endl;
//		if (nv != nullptr)
//		{
//			set_gap_variation_during_lane_change(nv->get_id(),
//				compute_gap_variation_during_lane_change(*nv));
//			set_collision_free_gap(nv->get_id(),
//				compute_collision_free_gap_during_lane_change(*nv));
//			if (verbose)
//			{
//				std::clog << "nv id = " << nv->get_id()
//					<< ", nv_vel = " << nv->compute_velocity(get_velocity())
//					<< "\n\tdelta g = " << get_gap_variation_to(nv)
//					<< ", glc* = " << get_collision_free_gap_to(nv)
//					<< std::endl;
//			}
//		}
//	}
//}

//double AutonomousVehicle::compute_vehicle_following_gap_for_lane_change(
//	const NearbyVehicle& nearby_vehicle) const
//{
//	return compute_vehicle_following_gap_for_lane_change(
//		nearby_vehicle, get_lambda_1_lane_change());
//}

double AutonomousVehicle::compute_gap_variation_during_lane_change(
	const NearbyVehicle& nearby_vehicle) const
{
	double relative_vel = nearby_vehicle.get_relative_velocity();
	double lc_time = 5.0;
	if (nearby_vehicle.is_on_same_lane())
	{
		return std::max(0.0, relative_vel * lc_time / 2);
	}
	if (nearby_vehicle.is_behind()) relative_vel *= -1;
	return std::max(relative_vel * lc_time / 2, relative_vel * lc_time);
}

NearbyVehicle* AutonomousVehicle::
implement_get_destination_lane_leader() const
{
	return destination_lane_leader;
}

NearbyVehicle* AutonomousVehicle::
implement_get_destination_lane_follower() const
{
	return destination_lane_follower;
}

long AutonomousVehicle::implement_get_virtual_leader_id() const
{
	return has_virtual_leader() ? get_virtual_leader()->get_id() : 0;
}

//void AutonomousVehicle::compute_lane_change_gap_parameters()
//{
//	lambda_1_lane_change = compute_lambda_1(max_jerk,
//		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
//}

void AutonomousVehicle::implement_set_accepted_lane_change_risk_to_leaders(
	double value)
{
	accepted_lane_change_risk_to_leaders = value;
}

void AutonomousVehicle::implement_set_accepted_lane_change_risk_to_follower(
	double value)
{
	accepted_lane_change_risk_to_follower = value;
}

void AutonomousVehicle::implement_set_use_linear_lane_change_gap(
	long value)
{
	use_linear_lane_change_gap = value > 0;
}

void AutonomousVehicle::reset_accepted_lane_change_risks(double time)
{
	accepted_lane_change_risk_to_leaders = 0;
	accepted_lane_change_risk_to_follower = 0;
	//lane_change_timer_start = time;
}

bool AutonomousVehicle::update_accepted_risk(double time)
{
	bool has_increased = false;

	/*if (((time - lane_change_timer_start) >= constant_risk_period)) {
		if ((accepted_risk_to_leader + delta_risk)
			< max_risk_to_leader) {
			lane_change_timer_start = time;
			accepted_risk_to_leader += delta_risk;
			has_increased = true;

			if (verbose) {
				std::clog << "\tleader risk updated to "
					<< accepted_risk_to_leader
					<< std::endl;
			}
		}
		if (accepted_risk_to_leader > intermediate_risk_to_leader
			&& (accepted_risk_to_follower + delta_risk)
			< max_risk_to_follower) {
			lane_change_timer_start = time;
			accepted_risk_to_follower += delta_risk;
			has_increased = true;

			if (verbose) {
				std::clog << "\tfollower risk updated to "
					<< accepted_risk_to_follower
					<< std::endl;
			}
		}
	}*/
	return has_increased;
}

double AutonomousVehicle::
compute_intermediate_risk_to_leader(double lambda_1,
	double lane_change_lambda_1, double max_brake_no_lane_change,
	double leader_max_brake)
{
	double h_no_lane_change = compute_time_headway_with_risk(
		get_desired_velocity(), get_max_brake(), leader_max_brake,
		get_lambda_1(), get_rho(), 0);
	double h_with_lane_change = compute_time_headway_with_risk(
		get_desired_velocity(), get_lane_change_max_brake(), leader_max_brake,
		get_lambda_1_lane_change(), get_rho(), 0);

	double intermediate_risk_to_leader = std::sqrt(2
		* (h_with_lane_change - h_no_lane_change)
		* get_max_brake() * get_desired_velocity());

	if (verbose)
	{
		std::clog << "h no lc=" << h_no_lane_change
			<< ", h with lc=" << h_with_lane_change
			<< ", mid risk to leader="
			<< intermediate_risk_to_leader << std::endl;
	}
	return intermediate_risk_to_leader;
}


void AutonomousVehicle::update_headways_with_risk(const EgoVehicle& ego_vehicle)
{
	/* This function has to be substantially changed to work. */

	//if (verbose) {
	//	std::clog << "Considering to increase risk" << std::endl;
	//}

	///* TODO: include difference to check whether vehicles are connected */

	//if (destination_lane_controller->update_accepted_risk(
	//	ego_vehicle.get_time(), ego_vehicle)) {

	//	if (ego_vehicle.has_destination_lane_leader()) {
	//		destination_lane_controller->update_time_headway(
	//			ego_parameters.lambda_1, ego_parameters.lambda_1_lane_change,
	//			ego_vehicle.get_destination_lane_leader()->get_max_brake());
	//	}
	//	/* TODO: should only enter this condition if follower_risk
	//	was changed
	//	And even so, we need to recompute the follower's lambda 1
	//	because we erase and rebuild the NearbyVehicle object every time.
	//	Maybe we should avoid this? */
	//	if (ego_vehicle.has_destination_lane_follower()) {
	//		std::shared_ptr<NearbyVehicle> dest_lane_follower =
	//			ego_vehicle.get_destination_lane_follower();

	//		if (ego_parameters.is_connected
	//			&& dest_lane_follower->is_connected()) {
	//			std::clog << "TODO: risk for connected vehicles not "
	//				<< "yet fully coded." << std::endl;
	//		}

	//		double estimated_follower_free_flow_velocity =
	//			ego_parameters.desired_velocity;
	//		double ego_max_brake = ego_parameters.max_brake;
	//		dest_lane_follower->compute_safe_gap_parameters();
	//		destination_lane_controller->estimate_follower_time_headway(
	//			*dest_lane_follower, ego_max_brake,
	//			estimated_follower_free_flow_velocity);
	//	}
	//}
}
