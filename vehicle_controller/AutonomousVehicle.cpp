#include "AutonomousVehicle.h"

AutonomousVehicle::AutonomousVehicle(long id, VehicleType type, 
	double desired_velocity, bool is_connected,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, type, desired_velocity,
		AUTONOMOUS_BRAKE_DELAY, true, false,
		simulation_time_step, creation_time, verbose)
{
	compute_lane_change_gap_parameters();
	controller.add_vissim_controller();
	controller.add_origin_lane_controllers(*this);
	controller.add_lane_change_adjustment_controller(*this);
	
	if (verbose)
	{
		std::clog << "lambda1_lc = " << get_lambda_1_lane_change() 
			<< "\n[AutonomousVehicle] constructor done" <<std::endl;
	}
}

//bool AutonomousVehicle::has_destination_lane_leader() const
//{
//	return get_destination_lane_leader() != nullptr;
//}
//
//bool AutonomousVehicle::has_destination_lane_follower() const
//{
//	return get_destination_lane_follower() != nullptr;
//}

bool AutonomousVehicle::has_lane_change_conflict() const 
{

	//if (verbose) std::clog << "checking conflicts" << std::endl;
	/* If there's no lane change intention, there's no conflict */
	if (!has_lane_change_intention()) return false;

	for (auto& nv : nearby_vehicles) 
	{
		if (nv->is_lane_changing()) 
		{
			RelativeLane& nv_lane = nv->get_relative_lane();
			RelativeLane& nv_lc_direction = nv->get_lane_change_direction();

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

bool AutonomousVehicle::is_destination_lane_follower(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();
	return nv_relative_lane == desired_lane_change_direction 
		&& nearby_vehicle.is_immediatly_behind();
}

bool AutonomousVehicle::is_destination_lane_leader(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();
	return nv_relative_lane == desired_lane_change_direction
		&& nearby_vehicle.is_immediatly_ahead();
}

bool AutonomousVehicle::is_leader_of_destination_lane_leader(
	const NearbyVehicle& nearby_vehicle)
{
	RelativeLane nv_relative_lane =
		nearby_vehicle.get_relative_lane();
	long relative_position =
		nearby_vehicle.get_relative_position();
	return (nv_relative_lane == desired_lane_change_direction
		&& relative_position == 2);
}

void AutonomousVehicle::implement_analyze_nearby_vehicles() 
{
	find_leader();
	find_destination_lane_vehicles();
}

void AutonomousVehicle::find_destination_lane_vehicles()
{
	std::shared_ptr<NearbyVehicle> old_dest_lane_follower =
		std::move(destination_lane_follower);
	std::shared_ptr<NearbyVehicle> old_dest_lane_leader =
		std::move(destination_lane_leader);
	bool dest_lane_leader_has_leader = false;
	if (has_lane_change_intention())
	{
		for (auto& nearby_vehicle : nearby_vehicles)
		{
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
				dest_lane_leader_has_leader = true;
			}
		}
	}

	deal_with_stopped_destination_lane_leader(dest_lane_leader_has_leader);
	update_destination_lane_follower(old_dest_lane_follower);
	update_destination_lane_leader(old_dest_lane_leader);
}

void AutonomousVehicle::deal_with_stopped_destination_lane_leader(
	bool dest_lane_leader_has_leader)
{
	/* To avoid deadlocks, we overtake the destination lane leader in case
	it is stopped and has no leader. This situation means that the dest
	lane leader is not moving because we are too close to it.*/
	if (has_destination_lane_leader()
		&& (destination_lane_leader->compute_velocity(get_velocity()) < 0.1)
		&& !dest_lane_leader_has_leader)
	{
		destination_lane_follower = destination_lane_leader;
		destination_lane_leader = nullptr;
	}
}

void AutonomousVehicle::update_destination_lane_follower(
	const std::shared_ptr<NearbyVehicle>& old_follower)
{
	if (has_destination_lane_follower())
	{
		if (old_follower == nullptr
			|| (old_follower->get_category()
				!= destination_lane_follower->get_category()))
		{
			controller.update_destination_lane_follower_time_headway(
				estimate_nearby_vehicle_time_headway(
					*destination_lane_follower));
			dest_lane_follower_lambda_0 =
				destination_lane_follower->get_lambda_0();
			dest_lane_follower_lambda_1 = 
				destination_lane_follower->get_lambda_1();
		}
	}
}

void AutonomousVehicle::update_destination_lane_leader(
	const std::shared_ptr<NearbyVehicle>& old_leader)
{
	if (has_destination_lane_leader())
	{
		double new_leader_max_brake = 
			destination_lane_leader->get_max_brake();
		bool is_new_leader_connected = 
			destination_lane_leader->is_connected();
		if (old_leader == nullptr)
		{
			double ego_vel = get_velocity();
			controller.activate_destination_lane_controller(ego_vel,
				destination_lane_leader->compute_velocity(ego_vel),
				compute_lane_changing_desired_time_headway(
					*destination_lane_leader), 
				is_new_leader_connected);
		}
		else if ((std::abs(new_leader_max_brake 
			- old_leader->get_max_brake()) > 0.5) 
			|| (old_leader->get_type() 
				!= destination_lane_leader->get_type()))
		{
			controller.update_destination_lane_controller(get_velocity(),
				compute_lane_changing_desired_time_headway(
					*destination_lane_leader), 
				is_new_leader_connected);
		}
	}
}

bool AutonomousVehicle::merge_behind_ld() const
{
	if (!has_destination_lane_leader()) return false;

	double origin_lane_reference_velocity;
	double ego_velocity = get_velocity();

	/* Get the possible max vel at the origin lane */
	if (controller.is_in_free_flow_at_origin_lane())
	{
		origin_lane_reference_velocity =
			get_desired_velocity();
	}
	else
	{
		origin_lane_reference_velocity = get_leader()
			->compute_velocity(ego_velocity);
	}

	return (get_destination_lane_leader()->compute_velocity(ego_velocity)
				> origin_lane_reference_velocity - min_overtaking_rel_vel);
}

double AutonomousVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double h_lc = compute_time_headway_with_risk(
		get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle.get_max_brake(),
		lambda_1_lane_change, get_rho(), 0/*accepted_lane_change_risk_to_leaders*/
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
		nearby_vehicle.estimate_desired_time_headway(MAX_VELOCITY,
			max_brake, get_rho(), 0/*accepted_lane_change_risk_to_follower*/));
}

double AutonomousVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller.get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

bool AutonomousVehicle::give_lane_change_control_to_vissim() const
{
	return lane_change_waiting_time > max_lane_change_waiting_time;
}

void AutonomousVehicle::set_desired_lane_change_direction()
{
	/* Both preferred_relative_lane and vissim_lane_suggestion indicate
	desire to change lanes. The former indicates preference due to
	routing (mandatory), so it takes precedence over the latter. */
	RelativeLane current_preferred_lane = get_preferred_relative_lane();
	if (current_preferred_lane.is_to_the_left())
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (current_preferred_lane.is_to_the_right())
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else if (get_vissim_lane_suggestion().is_to_the_left())
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (get_vissim_lane_suggestion().is_to_the_right())
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else
	{
		desired_lane_change_direction = RelativeLane::same;
	}
}

bool AutonomousVehicle::can_start_lane_change() 
{
	if (give_lane_change_control_to_vissim())
	{
		return get_vissim_lane_suggestion() != RelativeLane::same;
	}
	//if (!has_lane_change_intention())  // just to avoid computations
	//{
	//	return false;
	//}

	double margin = 0.1;
	//if (verbose) std::clog << "Deciding lane change" << std::endl;

	bool gap_same_lane_is_safe = is_lane_change_gap_safe(get_leader());
	bool gap_ahead_is_safe = is_lane_change_gap_safe(destination_lane_leader);
	/* Besides the regular safety conditions, we add the case
	where the dest lane follower has completely stopped to give room
	to the lane changing vehicle */
	bool gap_behind_is_safe = 
		is_lane_change_gap_safe(destination_lane_follower)
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -2.0));
	bool no_conflict = !has_lane_change_conflict();

	if (verbose) 
	{
		std::clog << "[orig lane] gap ahead is safe? " 
			<< gap_same_lane_is_safe
			<< ", [dest lane] gap ahead is safe? " << gap_ahead_is_safe
			<< ", [dest_lane] gap behind is safe? " << gap_behind_is_safe
			<< ", no conflict? " << no_conflict
			<< std::endl;
	}

	return gap_same_lane_is_safe && gap_ahead_is_safe
		&& gap_behind_is_safe && no_conflict;
}

bool AutonomousVehicle::is_lane_change_gap_safe(
	std::shared_ptr<NearbyVehicle>& nearby_vehicle)
{
	if (nearby_vehicle == nullptr) return true;
	double margin = 0.1;

	//set_gap_variation_during_lane_change(nearby_vehicle->get_id(),
	//	compute_gap_variation_during_lane_change(*nearby_vehicle));
	//set_collision_free_gap(nearby_vehicle->get_id(),
	//	compute_collision_free_gap_during_lane_change(*nearby_vehicle));

	return (compute_gap(nearby_vehicle) + margin)
		>= compute_accepted_lane_change_gap(nearby_vehicle);
}

double AutonomousVehicle::compute_accepted_lane_change_gap(
	std::shared_ptr<NearbyVehicle> nearby_vehicle)
{
	if (nearby_vehicle == nullptr) return 0.0;

	double accepted_vehicle_following_gap;
	
	// Only to be used during initial checks
	/*double gap1, gap2;
	gap1 = compute_time_headway_gap_for_lane_change(*nearby_vehicle);
	gap2 = compute_vehicle_following_gap_for_lane_change(*nearby_vehicle);*/

	if (use_linear_lane_change_gap)
	{
		accepted_vehicle_following_gap = 
			compute_time_headway_gap_for_lane_change(*nearby_vehicle);
	}
	else
	{
		accepted_vehicle_following_gap = 
			compute_vehicle_following_gap_for_lane_change(
				*nearby_vehicle);
	}

	double gap_variation_during_lc =
		controller.get_gap_variation_during_lane_change(*this,
			*nearby_vehicle, false);
	double accepted_gap = accepted_vehicle_following_gap 
		+ gap_variation_during_lc;
	//if (verbose)
	//{
	//	std::clog << "nv id " << nearby_vehicle->get_id()
	//		<< ": delta g_lc = " << gap_variation_during_lc
	//		//<< ", g_h = " << gap1
	//		//<< ", g_non-linear = " << gap2
	//		<< ", g_vf = " << accepted_vehicle_following_gap
	//		<< "; g_lc = " << accepted_gap << std::endl;
	//}

	return std::max(accepted_gap, 1.0);
}

double AutonomousVehicle::compute_time_headway_gap_for_lane_change(
	const NearbyVehicle& nearby_vehicle)
{
	double accepted_time_headway_gap =
		controller.get_desired_time_headway_gap(get_velocity(),
			nearby_vehicle);

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

double AutonomousVehicle::compute_vehicle_following_gap_for_lane_change(
	const NearbyVehicle& nearby_vehicle, double current_lambda_1) const
{
	double follower_lambda_0, follower_lambda_1, accepted_risk;
	double v_follower, v_leader;
	double brake_follower, brake_leader;
	double ego_velocity = get_velocity();
	double delta_v = nearby_vehicle.get_relative_velocity();
	if (nearby_vehicle.is_ahead())
	{
		accepted_risk = accepted_lane_change_risk_to_leaders;
		follower_lambda_0 = get_lambda_0();
		follower_lambda_1 = current_lambda_1;
		v_follower = ego_velocity;
		v_leader = nearby_vehicle.compute_velocity(ego_velocity);
		brake_follower = get_lane_change_max_brake();
		brake_leader = nearby_vehicle.get_max_brake();
	}
	else
	{
		accepted_risk = accepted_lane_change_risk_to_follower;
		follower_lambda_0 = dest_lane_follower_lambda_0;
		follower_lambda_1 = dest_lane_follower_lambda_1;
		v_leader = ego_velocity;
		v_follower = nearby_vehicle.compute_velocity(ego_velocity);
		brake_follower = nearby_vehicle.get_max_brake();
		brake_leader = max_brake;
	}

	double accepted_risk_2 = std::pow(accepted_risk, 2);
	double stop_time_follower = (v_follower + follower_lambda_1)
		/ brake_follower;
	double stop_time_leader = v_leader / brake_leader;

	//if (verbose)
	//{
	//	std::clog << "vf = " << v_follower
	//		<< ", lambda1 = " << follower_lambda_1
	//		<< ", df = " << brake_follower
	//		<< ", vl = " << v_leader
	//		<< ", dl = " << brake_leader
	//		<< ", lambda 0 = " << follower_lambda_0
	//		<< std::endl;
	//}

	double accepted_gap;
	if (stop_time_follower >= stop_time_leader)
	{
		accepted_gap =
			(std::pow(v_follower + follower_lambda_1, 2) 
				- accepted_risk_2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader 
			+ follower_lambda_0;
	}
	else if (brake_follower > brake_leader)
	{
		accepted_gap =
			(std::pow(delta_v - follower_lambda_1, 2)
				- accepted_risk_2) / 2 / (brake_follower - brake_leader)
			+ follower_lambda_0;
	}
	else
	{
		accepted_gap = 0.0;
	}
	return std::max(0.0, accepted_gap);
}

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

double AutonomousVehicle::compute_vehicle_following_gap_for_lane_change(
	const NearbyVehicle& nearby_vehicle) const
{
	return compute_vehicle_following_gap_for_lane_change(
		nearby_vehicle, lambda_1_lane_change);
}

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

std::shared_ptr<NearbyVehicle> AutonomousVehicle::
implement_get_destination_lane_leader() const
{
	return destination_lane_leader;
}

std::shared_ptr<NearbyVehicle> AutonomousVehicle::
implement_get_destination_lane_follower() const
{
	return destination_lane_follower;
}

void AutonomousVehicle::compute_lane_change_gap_parameters()
{
	lambda_1_lane_change = compute_lambda_1(max_jerk,
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
}

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
	/*if (verbose) 
	{
		std::clog << "\tt=" << time
			<< ", timer_start=" << lane_change_timer_start << std::endl;
	}*/

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

	//if (destination_lane_controller.update_accepted_risk(
	//	ego_vehicle.get_time(), ego_vehicle)) {

	//	if (ego_vehicle.has_destination_lane_leader()) {
	//		destination_lane_controller.update_time_headway(
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
	//		destination_lane_controller.estimate_follower_time_headway(
	//			*dest_lane_follower, ego_max_brake,
	//			estimated_follower_free_flow_velocity);
	//	}
	//}
}
