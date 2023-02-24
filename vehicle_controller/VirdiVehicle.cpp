#include "VirdiVehicle.h"

VirdiVehicle::VirdiVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	EgoVehicle(id, VehicleType::virdi_car, desired_velocity, 
		AUTONOMOUS_BRAKE_DELAY, true, true, simulation_time_step,
		creation_time, verbose)
{
	if (verbose)
	{
		std::clog << "[VirdiVehicle] constructor done" << std::endl;
	}
}

void VirdiVehicle::implement_create_controller() 
{
	this->controller = std::make_unique<ControlManager>(*this,
		is_verbose());
}

double VirdiVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller->get_desired_acceleration(*this);
	return a_desired_acceleration;
}

void VirdiVehicle::implement_analyze_nearby_vehicles()
{
	// Reset results from previous step
	set_leader_by_id(0); // Sets leader to nullptr
	destination_lane_follower = nullptr;
	destination_lane_leader = nullptr;
	assisted_vehicle = nullptr;
	lane_change_request = 0;

	for (auto const& id_veh_pair : get_nearby_vehicles())
	{
		// Real leader
		auto const& nearby_vehicle = id_veh_pair.second;
		if (check_if_is_leader(*nearby_vehicle))
		{
			set_leader_by_id(nearby_vehicle->get_id());
		}
		// Dest lane vehicles
		if (has_lane_change_intention())
		{
			if (is_destination_lane_follower(*nearby_vehicle))
			{
				destination_lane_follower = nearby_vehicle;
			}
			else if (is_destination_lane_leader(*nearby_vehicle))
			{
				destination_lane_leader = nearby_vehicle;
			}

		}
		// Assisted vehicle
		if (nearby_vehicle->get_lane_change_request_veh_id() == get_id())
		{
			if (assisted_vehicle == nullptr
				|| (nearby_vehicle->get_relative_position()
					< assisted_vehicle->get_relative_position()))
			{
				assisted_vehicle = nearby_vehicle;
			}
		}
	}
	// Create request
	if (get_preferred_relative_lane() != RelativeLane::same)
		lane_change_request = get_destination_lane_follower_id();

	if (verbose)
	{
		std::clog << "Leader id: " << get_leader_id()
			<< ". Dest lane leader id: "
			<< (has_destination_lane_leader() ?
				destination_lane_leader->get_id() : 0)
			<< ". Dest lane foll id: "
			<< (has_destination_lane_follower() ?
				destination_lane_follower->get_id() : 0)
			<< ". Assisted veh id: "
			<< (has_assisted_vehicle() ?
				assisted_vehicle->get_id() : 0) << std::endl;
	}
}

bool VirdiVehicle::implement_check_lane_change_gaps()
{
	// TODO: follows vissim if other vehicles not connected!
	/*if (give_lane_change_control_to_vissim())
	{
		return get_vissim_lane_suggestion() != RelativeLane::same;
	}*/

	// Virdi's paper does not address conflicts
	lane_change_gaps_safety.no_conflict = true;
	lane_change_gaps_safety.orig_lane_leader_gap =
		is_lane_change_gap_safe(get_leader());
	lane_change_gaps_safety.dest_lane_leader_gap =
		is_lane_change_gap_safe(destination_lane_leader);
	/* Besides the regular safety conditions, we add the case
	where the dest lane follower has completely stopped to give room
	to the lane changing vehicle */
	lane_change_gaps_safety.dest_lane_follower_gap =
		is_lane_change_gap_safe(destination_lane_follower)
		|| ((destination_lane_follower->
			compute_velocity(get_velocity()) <= 1.0)
			&& (destination_lane_follower->get_distance() <= -2.0));;
	return lane_change_gaps_safety.is_lane_change_safe();
}

long VirdiVehicle::implement_get_lane_change_request() const
{
	return lane_change_request;
}

double VirdiVehicle::compute_accepted_lane_change_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	return min_lane_change_gap;
}

bool VirdiVehicle::is_lane_change_gap_safe(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	if (nearby_vehicle == nullptr) return true;

	double gap, max_speed_variation;
	bool is_gap_safe, is_speed_safe;
	double tau = get_simulation_time_step();
	if (nearby_vehicle->is_ahead())
	{
		gap = compute_gap_to_a_leader(nearby_vehicle);
		max_speed_variation =
			-(nearby_vehicle->get_max_brake() * tau
				+ nearby_vehicle->get_max_jerk() * tau * tau / 2);
		is_speed_safe = 
			nearby_vehicle->get_relative_velocity() < max_speed_variation;
	}
	else
	{
		gap = compute_gap_to_a_follower(nearby_vehicle);
		max_speed_variation =
			COMFORTABLE_ACCELERATION * tau
			+ nearby_vehicle->get_max_jerk() * tau * tau / 2;
		is_speed_safe = 
			nearby_vehicle->get_relative_velocity() > max_speed_variation;
	}

	//if (verbose)
	//{
	//	std::clog << "To veh " << nearby_vehicle->get_id()
	//		<< (nearby_vehicle->is_ahead()?
	//			" (ahead)" : " (behind)") << ": g=" << gap
	//		<< ", rel_vel=" << nearby_vehicle->get_relative_velocity()
	//		<< ", thesh=" << max_speed_variation << std::endl;
	//}

	is_gap_safe = gap >= compute_accepted_lane_change_gap(nearby_vehicle);
	return is_gap_safe && is_speed_safe;
}

std::shared_ptr<NearbyVehicle> VirdiVehicle::
implement_get_destination_lane_leader() const
{
	return destination_lane_leader;
}

std::shared_ptr<NearbyVehicle> VirdiVehicle::
implement_get_destination_lane_follower() const
{
	return destination_lane_follower;
}

std::shared_ptr<NearbyVehicle> VirdiVehicle::
implement_get_assisted_vehicle() const
{
	return assisted_vehicle;
}

long VirdiVehicle::implement_get_virtual_leader_id() const
{
	return has_destination_lane_leader() ?
		destination_lane_leader->get_id() : 0;
}