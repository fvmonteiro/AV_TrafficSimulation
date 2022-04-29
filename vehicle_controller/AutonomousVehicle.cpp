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
}

bool AutonomousVehicle::has_destination_lane_leader() const
{
	return destination_lane_leader != nullptr;
}

bool AutonomousVehicle::has_destination_lane_follower() const
{
	return destination_lane_follower != nullptr;
}

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

bool AutonomousVehicle::check_if_is_destination_lane_follower(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();

	//if (nv_relative_lane == desired_lane_change_direction
	//	&& nearby_vehicle->is_immediatly_behind()) {
	//	/*if (current_id != dest_lane_follower_id)
	//	{
	//		controller.update_follower_time_headway(*nearby_vehicle);
	//	}*/
	//	destination_lane_follower = nearby_vehicle;
	//}

	return nv_relative_lane == desired_lane_change_direction 
		&& nearby_vehicle.is_immediatly_behind();
}

bool AutonomousVehicle::check_if_is_destination_lane_leader(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();

	//if (nv_relative_lane == desired_lane_change_direction
	//	&& nearby_vehicle->is_immediatly_ahead()) {
	//	// Update parameters if new follower
	//	/*if (current_id != dest_lane_leader_id) {
	//		controller.update_destination_lane_leader(
	//			get_velocity(), *nearby_vehicle);
	//	}*/
	//	destination_lane_leader = nearby_vehicle;
	//}
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

//void AutonomousVehicle::save_other_relevant_nearby_vehicle_ids()
//{
//	dest_lane_leader_id = has_destination_lane_leader() ?
//		destination_lane_leader->get_id() : 0;
//	dest_lane_follower_id = has_destination_lane_follower() ?
//		destination_lane_follower->get_id() : 0;
//}

//void AutonomousVehicle::clear_other_relevant_nearby_vehicles()
//{
//	//destination_lane_leader = nullptr;
//	//destination_lane_follower = nullptr;
//}

void AutonomousVehicle::find_relevant_nearby_vehicles() 
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
	for (auto& nearby_vehicle : nearby_vehicles)
	{
		if (has_lane_change_intention())
		{
			if (check_if_is_destination_lane_follower(*nearby_vehicle))
			{
				destination_lane_follower = nearby_vehicle;
			}
			else if (check_if_is_destination_lane_leader(*nearby_vehicle))
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

	//if (verbose)
	//{
	//	if (has_destination_lane_follower()) 
	//		std::clog << "Dest lane foll id=" 
	//		<< destination_lane_follower->get_id();
	//	else std::clog << "No dest lane foll";
	//	std::clog << std::endl;
	//	if (has_destination_lane_leader())
	//		std::clog << "Dest lane leader id="
	//		<< destination_lane_leader->get_id();
	//	else std::clog << "No dest lane leader";
	//	std::clog << std::endl;
	//}
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
		//destination_lane_follower = std::move(destination_lane_leader);
		destination_lane_follower = destination_lane_leader;
		destination_lane_leader = nullptr;
	}
}

void AutonomousVehicle::update_destination_lane_follower(
	const std::shared_ptr<NearbyVehicle>& old_follower)
{
	long new_follower_id{ 0 };
	if (has_destination_lane_follower())
	{
		if (old_follower == nullptr
			|| (old_follower->get_category()
				!= destination_lane_follower->get_category()))
		{
			controller.update_destination_lane_follower_time_headway(
				estimate_nearby_vehicle_time_headway(
					*destination_lane_follower));
		}
	}
	//destination_lane_follower = new_follower;
	//dest_lane_follower_id = new_follower_id; /* possibly not needed anymore */

	//const double old_id = dest_lane_follower_id;
	//if (has_destination_lane_follower() &&
	//	destination_lane_follower->get_id() != old_id)
	//{
	//	controller.update_follower_time_headway(*destination_lane_follower);
	//	dest_lane_follower_id = destination_lane_follower->get_id();
	//}
}

void AutonomousVehicle::update_destination_lane_leader(
	const std::shared_ptr<NearbyVehicle>& old_leader)
{
	long new_leader_id{ 0 };
	if (has_destination_lane_leader())
	{
		double new_leader_max_brake = 
			destination_lane_leader->get_max_brake();
		bool is_new_leader_connected = 
			destination_lane_leader->is_connected();
		if (old_leader == nullptr)
		{
			controller.activate_destination_lane_controller(get_velocity(),
				compute_lane_changing_desired_time_headway(
					new_leader_max_brake, is_new_leader_connected), 
				is_new_leader_connected);
		}
		else if ((std::abs(new_leader_max_brake 
			- old_leader->get_max_brake()) > 0.5) 
			|| (old_leader->get_type() 
				!= destination_lane_leader->get_type()))
		{
			controller.update_destination_lane_controller(get_velocity(),
				compute_lane_changing_desired_time_headway(
					new_leader_max_brake, is_new_leader_connected), 
				is_new_leader_connected);
		}
	}
	//destination_lane_leader = new_leader;
	//dest_lane_leader_id = new_leader_id; /* possibly not needed anymore */

	/*const double old_id = dest_lane_leader_id;
	if (has_destination_lane_leader() &&
		destination_lane_leader->get_id() != old_id)
	{
		controller.update_destination_lane_leader(get_velocity(),
			*destination_lane_leader);
		dest_lane_follower_id = destination_lane_follower->get_id();
	}*/
}

/* TODO: not yet sure whether this function should belong in this class 
or in some controller class */
double AutonomousVehicle::estimate_nearby_vehicle_time_headway(
	NearbyVehicle& nearby_vehicle)
{
	nearby_vehicle.compute_safe_gap_parameters();
	return nearby_vehicle.estimate_desired_time_headway(get_desired_velocity(),
		max_brake, get_rho(), 0);
}

double AutonomousVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_av_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

bool AutonomousVehicle::give_lane_change_control_to_vissim() const
{
	return lane_change_waiting_time > max_lane_change_waiting_time;
}

bool AutonomousVehicle::can_start_lane_change() 
{
	if (give_lane_change_control_to_vissim())
	{
		return get_relative_target_lane() != RelativeLane::same;
	}
	if (!has_lane_change_intention())  // just to avoid computations
	{
		return false;
	}

	double margin = 0.1;
	if (verbose) std::clog << "Deciding lane change" << std::endl;

	bool gap_same_lane_is_safe = is_lane_change_gap_safe(get_leader());
	bool gap_ahead_is_safe = is_lane_change_gap_safe(destination_lane_leader);
	/* besides the regular safety conditions, we add the case
	where the dest lane follower has completely stopped to give room
	to the lane changing vehicle */
	bool gap_behind_is_safe = 
		is_lane_change_gap_safe(destination_lane_follower)
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -1.0));
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
	return (compute_gap(nearby_vehicle) + margin)
		>= compute_safe_lane_change_gap(nearby_vehicle);
}

double AutonomousVehicle::get_current_lambda_1(
	bool is_other_connected) const
{
	return has_lane_change_intention() ? lambda_1_lane_change : lambda_1;
}

void AutonomousVehicle::compute_lane_change_gap_parameters()
{
	lambda_1_lane_change = compute_lambda_1(max_jerk,
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
}