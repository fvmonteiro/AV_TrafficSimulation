#include <iostream>

#include "Platoon.h"
#include "PlatoonVehicle.h"

Platoon::Platoon(long id, std::shared_ptr<PlatoonVehicle> leader):
	id {id}
{
	add_leader(leader);
}

Platoon::~Platoon()
{
	std::clog << "Inside platoon " << id << " destructor\n" 
		<< "Vehs in platoon: " << vehicles.size()
		<< std::endl;
}

long Platoon::get_leader_id() const
{ 
	return is_empty() ? 0 : get_platoon_leader()->get_id();
}

std::shared_ptr<PlatoonVehicle> Platoon::get_platoon_leader() const
{
	return is_empty() ? nullptr : vehicles.at(leader_idx - 1);
}

long Platoon::get_last_veh_id() const
{
	return is_empty() ? 0 : vehicles.at(last_veh_idx)->get_id();
}

//void Platoon::set_id(long platoon_id)
//{
//	if (id == -1) 
//	{
//		id = platoon_id;
//	}
//	else
//	{
//		std::clog << "Platoon [" << id << "]"
//			<<"\n\tTrying to set the id of a platoon twice" << std::endl;
//	}
//}

bool Platoon::is_empty() const
{
	return vehicles.size() == 0;
}

void Platoon::add_leader(
	 std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	std::clog << "Platoon " << id << ": adding leader with id " 
		<< new_vehicle->get_id() << std::endl;;
	vehicles.insert({ leader_idx, new_vehicle });
	vehicle_id_to_position.insert({ new_vehicle->get_id(), leader_idx });
	leader_idx++;
}

void Platoon::add_last_vehicle(
	std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	std::clog << "Platoon " << id << ": adding last veh with id "
		<< new_vehicle->get_id() << std::endl;;
	last_veh_idx --;
	vehicles.insert({ last_veh_idx, new_vehicle });
	vehicle_id_to_position.insert({ new_vehicle->get_id(), last_veh_idx});
}

void Platoon::remove_leader()
{
	std::clog << "Platoon " << id << ": removing leader." << std::endl;
	vehicle_id_to_position.erase(get_leader_id());
	vehicles.erase(leader_idx);
	leader_idx --;
}

void Platoon::remove_last_vehicle()
{
	vehicle_id_to_position.erase(get_last_veh_id());
	vehicles.erase(last_veh_idx);
	last_veh_idx++;
}

void Platoon::remove_vehicle_by_id(long veh_id)
{
	if (vehicle_id_to_position.find(veh_id) !=
		vehicle_id_to_position.end())
	{
		vehicles.erase(vehicle_id_to_position.at(veh_id));
		vehicle_id_to_position.erase(veh_id);
	}
}

//Platoon Platoon::split_platoon(int platoon_position)
//{
//	if (platoon_position > vehicles.size())
//	{
//		std::clog << "[Splitting platoon]"
//			<< " Position index greater than platoon length." << std::endl;
//		return Platoon();
//	}
//
//	int platoon_idx = leader_idx - platoon_position + 1;
//	Platoon new_platoon = Platoon(vehicles[platoon_idx]);
//	for (int i = last_veh_idx; i < platoon_idx; i++)
//	{
//		/* TODO: must check if move will remove vehicles from this platoon */
//		new_platoon.add_last_vehicle(std::move(vehicles[i]));
//	}
//
//	return new_platoon;
//}

//Platoon Platoon::remove_a_vehicle(int platoon_position_idx)
//{
//	if (platoon_position_idx > vehicles.size())
//	{
//		std::clog << "[Removing vehicle from platoon]" 
//			<< " Position index greater than platoon length." << std::endl;
//		return Platoon();
//	}
//
//	Platoon new_platoon = split_platoon(platoon_position_idx);
//	remove_last_vehicle();
//	return new_platoon;
//}

//bool Platoon::merge_into_leading_platoon(
//	Platoon& other_platoon)
//{
//	leader_idx--;
//	for (int i = leader_idx; i >= last_veh_idx; i--)
//	{
//		other_platoon.add_last_vehicle(std::move(vehicles[i]));
//		vehicles[i]->get_platoon();
//	}
//	return is_empty();
//}