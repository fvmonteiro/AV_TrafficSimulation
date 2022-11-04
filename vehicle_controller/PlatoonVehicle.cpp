
#include "PlatoonVehicle.h"
#include "Platoon.h"

PlatoonVehicle::PlatoonVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	ConnectedAutonomousVehicle(id, VehicleType::platoon_car,
		desired_velocity, simulation_time_step, creation_time,
		verbose),
	alone_desired_velocity{ desired_velocity }
{
	std::clog << "[PlatoonVehicle] constructor" << std::endl;
	//create_platoon();
}

PlatoonVehicle::~PlatoonVehicle()
{
	std::clog << "[PlatoonVehicle] destructor" << std::endl;
}

void PlatoonVehicle::set_desired_lane_change_direction()
{
	if (is_platoon_leader()
		|| (platoon->get_lane_change_strategy() 
			== Platoon::LaneChangeStrategy::none))
	{
		AutonomousVehicle::set_desired_lane_change_direction();
	}
	else
	{
		desired_lane_change_direction = platoon->
			get_platoon_leader()->get_desired_lane_change_direction();
	}
}

bool PlatoonVehicle::can_start_lane_change()
{
	bool individual_lane_change_is_safe =
		AutonomousVehicle::can_start_lane_change();
	if (is_in_a_platoon())
	{
		platoon->set_vehicle_lane_change_gaps_safe(get_id(),
			individual_lane_change_is_safe);
		return platoon->can_vehicle_start_lane_change(get_id());
	}
	return individual_lane_change_is_safe;
}

void PlatoonVehicle::implement_analyze_nearby_vehicles()
{
	// Find leader, dest lane leader and follower, and assisted vehicle
	if (is_platoon_leader())
	{
		find_leader();
	}
	else
	{
		set_leader_by_id()
	}
}

bool PlatoonVehicle::implement_analyze_platoons(
	std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
	std::shared_ptr<EgoVehicle> pointer_to_me, long new_platoon_id)
{
	bool new_platoon_created = false;
	bool am_in_a_platoon = is_in_a_platoon();
	bool leader_is_in_a_platoon = 
		has_leader() && get_leader()->is_in_a_platoon();
	bool may_join_leader_platoon =
		leader_is_in_a_platoon && (get_leader()->get_distance() < 60);

	std::shared_ptr<PlatoonVehicle> pointer_to_me_my_type =
		std::dynamic_pointer_cast<PlatoonVehicle>(pointer_to_me);

	if (!am_in_a_platoon && !may_join_leader_platoon)
	{
		// Create platoon
		if (alone_time >= max_time_looking_for_platoon)
		{
			create_platoon(new_platoon_id, pointer_to_me_my_type);
			alone_time = 0.0;
			new_platoon_created = true;
		}
		else
		{
			alone_time += get_sampling_interval();
		}
	}
	else if (!am_in_a_platoon && may_join_leader_platoon)
	{
		// Join the platoon of the vehicle ahead
		long leader_platoon_id = get_leader()->get_platoon_id();
		add_myself_to_leader_platoon(platoons.at(leader_platoon_id),
			pointer_to_me_my_type);
	}
	else if (am_in_a_platoon && may_join_leader_platoon)
	{
		// Leave my platoon and join the platoon of the vehicle ahead
		/* Open question : make my entire platoon merge or move only myself
		 to the platoon ahead? */
		long old_platoon_id = get_platoon_id();
		long leader_platoon_id = get_leader()->get_platoon_id();
		if (old_platoon_id != leader_platoon_id)
		{
			// remove myself
			platoon->remove_vehicle_by_id(get_id());
			// add myself to leader platoon
			add_myself_to_leader_platoon(platoons.at(leader_platoon_id),
				pointer_to_me_my_type);
			// delete my old platoon if it is empty
			if (platoons.at(old_platoon_id)->is_empty())
			{
				//std::clog << "Old platoon is empty" << std::endl;
				platoons.erase(old_platoon_id);
			}
		}
	}
	else // am_in_a_platoon && !may_join_leader_platoon
	{
		if (platoon->can_vehicle_leave_platoon(get_id()))
		{
			platoon->remove_vehicle_by_id(get_id());
			platoon.reset();
		}
	}

	return new_platoon_created;
}

void PlatoonVehicle::create_platoon(long platoon_id,
	std::shared_ptr<PlatoonVehicle> pointer_to_me)
{
	std::clog << "Veh id " << get_id()
		<< ". Creating platoon id " << platoon_id << std::endl;

	platoon = std::make_shared<Platoon>(platoon_id,
		pointer_to_me);

	std::clog << "Platoon " << platoon->get_id()
		<< " ptr count: " << platoon.use_count()
		<< " (before putting platoon in platoon map)"
		<< std::endl;
}

void PlatoonVehicle::add_myself_to_leader_platoon(
	std::shared_ptr<Platoon> leader_platoon,
	std::shared_ptr<PlatoonVehicle> pointer_to_me)
{
	// Add my pointer to the platoon vehicles list
	leader_platoon->add_last_vehicle(pointer_to_me);
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

bool PlatoonVehicle::is_platoon_leader()
{
	return is_in_a_platoon() ? platoon->get_leader_id() == get_id() : true;
}

//double PlatoonVehicle::compute_desired_acceleration(
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	double desired_acceleration = get_vissim_acceleration();
//		//controller.get_cav_desired_acceleration(*this);
//	return consider_vehicle_dynamics(desired_acceleration);
//}

//bool PlatoonVehicle::can_start_lane_change()
//{
//	return false;
//}
