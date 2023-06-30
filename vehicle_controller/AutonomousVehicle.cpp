#include "AutonomousVehicle.h"
#include "AVController.h"

AutonomousVehicle::AutonomousVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) : AutonomousVehicle(id,
		VehicleType::autonomous_car, desired_velocity, false,
		simulation_time_step, creation_time, verbose) 
{
	this->controller = std::make_unique<AVController>(
		AVController(*this, verbose));
	if (verbose)
	{
		std::clog << "[AutonomousVehicle] constructor done\n";
	}
}

AutonomousVehicle::AutonomousVehicle(long id, VehicleType type,
	double desired_velocity, bool is_connected,
	double simulation_time_step, double creation_time,
	bool verbose) :
	LongitudinallyAutonomousVehicle(id, type, desired_velocity,
		false, simulation_time_step, creation_time, verbose)
{

	MUST ASSIGN NEW AV STATE HERE!

	compute_lane_change_gap_parameters();
	if (verbose)
	{
		std::clog << "lambda1_lc = " << get_lambda_1_lane_change() << "\n";
	}
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

bool AutonomousVehicle::are_all_lane_change_gaps_safe() const
{
	return lane_change_gaps_safety.is_lane_change_safe();
}

LaneChangeGapsSafety AutonomousVehicle::get_lane_change_gaps_safety() const
{
	return lane_change_gaps_safety;
}

void AutonomousVehicle::reset_state(
	std::unique_ptr<VehicleState> new_lane_keeping_state)
{
	// TODO: poor condition checking: hard coding variables
	if (new_lane_keeping_state->get_state_number() != 1)
	{
		std::clog << "[WARNING] t=" << get_time()
			<< ", veh " << get_id() << ": reseting vehicle state to "
			<< *new_lane_keeping_state
			<< ", which is not a lane keeping state." << std::endl;
	}

	set_lane_change_direction(RelativeLane::same);
	reset_lane_change_waiting_time();
	update_origin_lane_controller();
	set_state(std::move(new_lane_keeping_state));
}

bool AutonomousVehicle::is_lane_change_gap_safe(
	const NearbyVehicle* nearby_vehicle) const
{
	if (nearby_vehicle == nullptr) return true;
	double margin = 0.1;

	//set_gap_variation_during_lane_change(nearby_vehicle->get_id(),
	//	compute_gap_variation_during_lane_change(*nearby_vehicle));
	//set_collision_free_gap(nearby_vehicle->get_id(),
	//	compute_collision_free_gap_during_lane_change(*nearby_vehicle));
	double gap = nearby_vehicle->is_ahead() ?
		compute_gap_to_a_leader(nearby_vehicle)
		: compute_gap_to_a_follower(nearby_vehicle);
		
	if (verbose)
	{
		std::clog << "\tg=" << gap<< "\n";
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
		auto const& nv = id_veh_pair.second;
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

void AutonomousVehicle::set_virtual_leader(
	std::shared_ptr<NearbyVehicle> new_virtual_leader)
{
	// Note: this makes the virtual_leader = nullptr
	std::shared_ptr<NearbyVehicle> old_virtual_lane_leader =
		std::move(virtual_leader);
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
	/* Note: move operation "empties" the current pointer values */
	std::shared_ptr<NearbyVehicle> old_dest_lane_follower =
		std::move(destination_lane_follower);
	destination_lane_leader = nullptr;
	destination_lane_leader_leader = nullptr;
	if (has_lane_change_intention())
	{
		for (auto const& id_veh_pair : get_nearby_vehicles())
		{
			auto const& nearby_vehicle = id_veh_pair.second;
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
	std::shared_ptr<NearbyVehicle> vl = define_virtual_leader();
	set_virtual_leader(vl);
}

std::shared_ptr<NearbyVehicle> AutonomousVehicle::define_virtual_leader()
const
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
		(controller->is_in_free_flow_at_origin_lane() || !has_leader()) ?
		get_desired_velocity()
		: get_leader()->compute_velocity(ego_velocity);

	if (verbose)
	{
		std::clog << "v_o=" << origin_lane_desired_velocity
			<< ", v_d=" << dest_lane_leader_vel << "\n";
	}

	/* We say the dest lane leader is stuck if it is not moving even
	though it has no leader. This can happen when vehicles in different
	lanes trying to "switch" lanes. */
	bool is_dest_lane_leader_stuck =
		dest_lane_leader_vel < 0.1
		&& (get_destination_lane_leader_leader() == nullptr);
	bool is_desired_speed_higher = dest_lane_leader_vel
		< (origin_lane_desired_velocity - min_rel_vel);

	/* [Jan 24, 2023] Note: maybe the first condition is redundant. */
	return is_dest_lane_leader_stuck || is_desired_speed_higher;
}

bool AutonomousVehicle::try_to_overtake_destination_lane_leader() const
{
	return try_to_overtake_destination_lane_leader(min_overtaking_rel_vel);
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
			controller->update_destination_lane_follower_time_headway(
				estimate_nearby_vehicle_time_headway(
					*destination_lane_follower));
			dest_lane_follower_lambda_0 =
				destination_lane_follower->get_lambda_0();
			dest_lane_follower_lambda_1 =
				destination_lane_follower->get_lambda_1();
		}
	}
}

void AutonomousVehicle::update_virtual_leader(
	std::shared_ptr<const NearbyVehicle> old_leader)
{
	if (has_virtual_leader())
	{
		double new_leader_max_brake = get_virtual_leader()->get_max_brake();
		bool is_new_leader_connected = get_virtual_leader()->is_connected();
		if (old_leader == nullptr)
		{
			double ego_vel = get_velocity();
			controller->activate_destination_lane_controller(
				get_virtual_leader()->compute_velocity(ego_vel),
				compute_lane_changing_desired_time_headway(
					*get_virtual_leader()),
				is_new_leader_connected);
		}
		else if ((std::abs(new_leader_max_brake
			- old_leader->get_max_brake()) > 0.5)
			|| (old_leader->get_type()
				!= get_virtual_leader()->get_type()))
		{
			controller->update_destination_lane_controller(
				compute_lane_changing_desired_time_headway(
					*get_virtual_leader()),
				is_new_leader_connected);
		}
	}
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

void AutonomousVehicle::pass_this_to_state()
{
	state->set_ego_vehicle(this);
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
		controller->compute_desired_acceleration();
	return consider_vehicle_dynamics(a_desired_acceleration);
}

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

	if (verbose) 
	{
		std::clog << "Deciding lane changing safety\n"
			<< "\tTo orig lane leader:\n";
	}
	lane_change_gaps_safety.orig_lane_leader_gap = 
		is_lane_change_gap_safe(get_leader().get());
	if (verbose)
	{
		std::clog << "\tTo dest lane leader:\n";
	}
	lane_change_gaps_safety.dest_lane_leader_gap =
		is_lane_change_gap_safe(destination_lane_leader.get());
	if (verbose)
	{
		std::clog << "\tTo dest lane follower:\n";
	}
	/* Besides the regular safety conditions, we add the case
	where the dest lane follower has completely stopped to give room
	to the lane changing vehicle */
	lane_change_gaps_safety.dest_lane_follower_gap = 
		is_lane_change_gap_safe(destination_lane_follower.get())
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -2.0));
	lane_change_gaps_safety.no_conflict =
		!has_lane_change_conflict();

	return lane_change_gaps_safety.is_lane_change_safe();
}

double AutonomousVehicle::compute_accepted_lane_change_gap(
	const NearbyVehicle* nearby_vehicle) const
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
		controller->get_gap_variation_during_lane_change(
			*nearby_vehicle, false);
	double accepted_gap = accepted_vehicle_following_gap
		+ gap_variation_during_lc;
	if (verbose)
	{
		std::clog << "\tnv id " << nearby_vehicle->get_id()
			<< ": delta g_lc = " << gap_variation_during_lc
			<< ", g_vf = " << accepted_vehicle_following_gap
			<< "; g_lc = " << accepted_gap << std::endl;
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
	double accepted_time_headway_gap =
		controller->get_desired_time_headway_gap(
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

	if (verbose)
	{
		std::clog << "\tVeh following gap computation\n\t"
			<< "vf=" << v_follower
			<< ", lambda1=" << follower_lambda_1
			<< ", df=" << brake_follower
			<< ", vl=" << v_leader
			<< ", dl=" << brake_leader
			<< ", lambda 0=" << follower_lambda_0
			<< "\n\tt_f=" << stop_time_follower
			<< ", t_l=" << stop_time_leader
			<< ", g_vf=" << accepted_gap
			<< std::endl;
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

long AutonomousVehicle::implement_get_virtual_leader_id() const
{
	return has_virtual_leader() ? get_virtual_leader()->get_id() : 0;
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
