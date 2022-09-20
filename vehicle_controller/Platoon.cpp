#include <iostream>

#include "Platoon.h"
#include "PlatoonVehicle.h"

Platoon::Platoon(std::shared_ptr<PlatoonVehicle> vehicle):
	id {vehicle->get_id()}
{
	add_leader(vehicle);
}

Platoon::~Platoon()
{
	std::clog << "Inside platoon " << id << " destructor\n" 
		<< "Vehs in platoon: " << vehicles.size()
		<< std::endl;
}

int Platoon::get_leader_id() 
{ 
	return is_empty() ? 0 : vehicles[leader_idx]->get_id();
}

bool Platoon::is_empty()
{
	return vehicles.size() == 0;
}

void Platoon::add_leader(
	const std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	std::clog << "Platoon " << id << ": adding leader with id " 
		<< new_vehicle->get_id() << std::endl;;
	leader_idx ++;
	vehicles.insert({ leader_idx, new_vehicle });
}

void Platoon::add_last_vehicle(
	const std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	std::clog << "Platoon " << id << ": adding last veh with id "
		<< new_vehicle->get_id() << std::endl;;
	last_veh_idx --;
	vehicles.insert({ last_veh_idx, new_vehicle });
}

void Platoon::remove_leader()
{
	std::clog << "Platoon " << id << ": removing leader.";
	vehicles.erase(leader_idx);
	leader_idx --;
	if (!is_empty()) std::clog << "\n\tNew id: " 
		<< vehicles[leader_idx]->get_id();
}

void Platoon::remove_last_vehicle()
{
	vehicles.erase(last_veh_idx);
	last_veh_idx++;
}

Platoon Platoon::split_platoon(int platoon_position)
{
	if (platoon_position > vehicles.size())
	{
		std::clog << "[Splitting platoon]"
			<< " Position index greater than platoon length." << std::endl;
		return Platoon(0);
	}

	int platoon_idx = leader_idx - platoon_position + 1;
	Platoon new_platoon = Platoon(vehicles[platoon_idx]);
	for (int i = last_veh_idx; i < platoon_idx; i++)
	{
		/* TODO: must check if move will remove vehicles from this platoon */
		new_platoon.add_last_vehicle(std::move(vehicles[i]));
	}

	return new_platoon;
}

Platoon Platoon::remove_a_vehicle(int platoon_position_idx)
{
	if (platoon_position_idx > vehicles.size())
	{
		std::clog << "[Removing vehicle from platoon]" 
			<< " Position index greater than platoon length." << std::endl;
		return Platoon(0);
	}

	Platoon new_platoon = split_platoon(platoon_position_idx);
	remove_last_vehicle();
	return new_platoon;
}

void Platoon::merge_into_leading_platoon(std::shared_ptr<Platoon> other_platoon)
{
	for (int i = leader_idx; i >= last_veh_idx; i--)
	{
		other_platoon->add_last_vehicle(std::move(vehicles[i]));
	}
}