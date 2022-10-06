
#include "PlatoonVehicle.h"
#include "Platoon.h"

PlatoonVehicle::PlatoonVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	ConnectedAutonomousVehicle(id, VehicleType::platoon_car,
		desired_velocity, simulation_time_step, creation_time,
		verbose) 
{
	std::clog << "[PlatoonVehicle] constructor" << std::endl;
	//create_platoon();
}

PlatoonVehicle::~PlatoonVehicle()
{
	std::clog << "[PlatoonVehicle] destructor" << std::endl;
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
		/* TODO: do we want to wait for a second before creating
		the single vehicle platoon? It could avoid creating too
		many platoons when there are lots of lane changes */
		create_platoon(new_platoon_id, pointer_to_me_my_type);
		new_platoon_created = true;
	}
	else if (!am_in_a_platoon && may_join_leader_platoon)
	{
		// Join the platoon of the vehicle ahead
		long leader_platoon_id = get_leader()->get_platoon_id();

		std::clog << "Veh id " << get_id()
			<< ". I didn't have a platoon and now am joining " 
			<< leader_platoon_id << std::endl;

		platoons.at(leader_platoon_id)->add_last_vehicle(
			pointer_to_me_my_type);
		platoon = platoons.at(leader_platoon_id);

		std::clog << "Platoon " << platoon->get_id()
			<< " # vehs: " << platoon->get_size()
			<< " ptr count: " << platoon.use_count()
			<< std::endl;
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
			std::clog << "Veh id " << get_id()
				<< " from platoon " << get_platoon_id()
				<< ", joining platoon " << leader_platoon_id
				<< std::endl;

			// remove myself
			platoon->remove_leader();
			// add myself to platoon ahead
			platoons.at(leader_platoon_id)->add_last_vehicle(
				pointer_to_me_my_type);
			// update my platoon pointer
			platoon = platoons.at(leader_platoon_id);
			// delete my old platoon if it is empty
			if (platoons.at(old_platoon_id)->is_empty())
			{
				std::clog << "Old platoon is empty" << std::endl;
				platoons.erase(old_platoon_id);
			}
		}
	}
	else // am_in_a_platoon && !may_join_leader_platoon
	{
		// I or my leader changed lanes, or someone cut in between us
		bool am_the_platoon_leader =
			platoon->get_leader_id() == get_id();
		if (!am_the_platoon_leader)
		{
			std::clog << "Veh id " << get_id()
				<< " leaving platoon " << platoon->get_id()
				<< std::endl;
			platoon->remove_vehicle_by_id(get_id());
			platoon = nullptr;
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
//void PlatoonVehicle::find_relevant_nearby_vehicles()
//{
//	/* WILL CHANGE */
//	find_leader();
//
//	/* We can only figure out if the leader changed or not after the
//	loop explores all nearby vehicles. Thus, we can only decide whether
//	to update the origin lane controller here. */
//	/*if (has_leader()
//		&& (leader->get_id() != old_leader_id))
//	{
//		controller.update_origin_lane_leader(
//			get_velocity(), old_leader_id != 0, *leader);
//	}*/
//
//	//update_nearby_vehicles();
//}

//void PlatoonVehicle::decide_platoon_to_join(
//	std::unordered_map<int, std::shared_ptr<Platoon>> platoons)
//{
//
//	if (has_leader()
//		&& platoons.find(get_leader()->get_id()) != platoons.end())
//	{
//		platoons[get_leader()->get_id()]->add_last_vehicle(
//			std::make_shared<PlatoonVehicle>(*this));
//	}
//	else
//	{
//		Platoon platoon = Platoon(
//			std::make_shared<PlatoonVehicle>(*this));
//		bool ok = platoons.insert({
//			get_id(), std::make_shared<Platoon>(platoon) }).second;
//		if (!ok)
//		{
//			std::clog << "Platoon " << get_id()
//				<< " already exists." << std::endl;
//		}
//	}
//}

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
