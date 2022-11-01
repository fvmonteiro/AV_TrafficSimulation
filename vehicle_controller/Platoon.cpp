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
	return is_empty() ? nullptr : vehicles.at(leader_idx);
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
		<< new_vehicle->get_id() << std::endl;
	leader_idx++;
	add_vehicle(leader_idx, new_vehicle);
}

void Platoon::add_last_vehicle(
	std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	std::clog << "Platoon " << id << ": adding last veh with id "
		<< new_vehicle->get_id() << std::endl;;
	last_veh_idx--;
	add_vehicle(last_veh_idx, new_vehicle);
}

//void Platoon::remove_leader()
//{
//	remove_vehicle_by_position(leader_idx, get_leader_id());
//	//while (vehicles.find(--leader_idx) == vehicles.end())
//	//{
//	//	/* Just being sure we don't fall in an infinite loop */
//	//	if (leader_idx < last_veh_idx) return; // platoon empty!
//	//}
//}

//void Platoon::remove_last_vehicle()
//{
//	remove_vehicle_by_position(last_veh_idx, get_last_veh_id());
//	//while (vehicles.find(++last_veh_idx) == vehicles.end())
//	//{
//	//	/* Just being sure we don't fall in an infinite loop */
//	//	if (last_veh_idx > leader_idx) return; // platoon empty!
//	//}
//}

void Platoon::remove_vehicle_by_id(long veh_id)
{
	if (vehicle_id_to_position.find(veh_id) !=
		vehicle_id_to_position.end())
	{
		std::clog << "Platoon " << get_id() << ", removing veh "
			<< veh_id << ", which is in position " 
			<< vehicle_id_to_position.at(veh_id) << std::endl;
		
		long position_to_remove = vehicle_id_to_position.at(veh_id);
		remove_vehicle_by_position(position_to_remove, veh_id);
	}
	else
	{
		std::clog << "Platoon " << get_id()
			<< ". Trying to erase veh " << veh_id
			<< " which is not in this platoon.\n";
	}
}

void Platoon::set_vehicle_lane_change_gaps_safe(long veh_id, bool is_safe)
{
	vehicles_lane_change_gap_status[veh_id] = is_safe;
}

bool Platoon::can_vehicle_start_lane_change(long veh_id)
{
	switch (lane_change_strategy)
	{
	case Platoon::LaneChangeStrategy::none:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	case Platoon::LaneChangeStrategy::synchronous:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	case Platoon::LaneChangeStrategy::leader_first:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	case Platoon::LaneChangeStrategy::last_vehicle_first:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	case Platoon::LaneChangeStrategy::leader_first_and_invert:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	default:
		return vehicles_lane_change_gap_status[veh_id];
		break;
	}
	return false;
}

std::ostream& operator<< (std::ostream& out, const Platoon& platoon)
{
	out << "Platoon " << platoon.get_id()
		<< ". Vehs by positions (" << platoon.vehicles.size() << ") : ";
	for (int i = platoon.last_veh_idx; i <= platoon.leader_idx; i++)
	{
		if (platoon.vehicles.find(i) != platoon.vehicles.end())
		{
			out << "(" << i << ", " << platoon.vehicles.at(i)->get_id()
				<< "), ";
		}
	}
	out << "leader idx: " << platoon.leader_idx 
		<< ", last veh idx: " << platoon.last_veh_idx;
	return out; // return std::ostream so we can chain calls to operator<<
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon, long veh_id)
{
	std::clog << "Removing by position."
		<< " Erasing vehicle.";
	if (vehicles.erase(idx_in_platoon) == 0)
		std::clog << "Vehicle not found in vehicles map\n";
	std::clog << " Erasing vehicle_id_to_position.";
	if (vehicle_id_to_position.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicle_id_to_position\n";
	std::clog << " Erasing vehicle_lane_change_gap_status.";
	if (vehicles_lane_change_gap_status.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicles_lane_change_gap_status\n";
	/* In case the removed vehicle was the leader or the last vehicle,
	we update the respective indices */
	while (vehicles.find(leader_idx) == vehicles.end())
	{
		leader_idx--;
		if (leader_idx < last_veh_idx) return; // platoon empty!
	}
	while (vehicles.find(last_veh_idx) == vehicles.end())
	{
		last_veh_idx++;
		if (last_veh_idx > leader_idx) return; // platoon empty!
	}
	std::clog << "\tAll erased and indices updated!\n"
		<< "\t" << *this << std::endl;
}

void Platoon::add_vehicle(int idx_in_platoon,
	std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	vehicles.insert({ idx_in_platoon, new_vehicle });
	vehicle_id_to_position.insert({ new_vehicle->get_id(), idx_in_platoon });
	vehicles_lane_change_gap_status.insert({ new_vehicle->get_id(), false });
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