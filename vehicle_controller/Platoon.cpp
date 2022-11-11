#include <iostream>

#include "Platoon.h"
#include "PlatoonVehicle.h"

Platoon::Platoon(long id, std::shared_ptr<PlatoonVehicle> leader,
	bool verbose):
	id {id}, verbose{verbose}
{
	add_leader(leader);
}

Platoon::~Platoon()
{
	if (verbose)
	{
		std::clog << "Inside platoon " << id << " destructor\n"
			<< "Vehs in platoon: " << vehicles.size()
			<< std::endl;
	}
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

long Platoon::get_preceding_vehicle_id(long veh_id) const
{
	int veh_position = vehicle_id_to_position.at(veh_id);
	while (vehicles.find(++veh_position) == vehicles.end())
	{
		if (veh_position > leader_idx) return 0;
	}
	return vehicles.at(veh_position)->get_id();
}

bool Platoon::is_empty() const
{
	return vehicles.size() == 0;
}

void Platoon::add_leader(
	 std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	if (verbose)
	{
		std::clog << "Platoon " << id << ": adding leader with id "
			<< new_vehicle->get_id() << std::endl;
	}
	leader_idx++;
	add_vehicle(leader_idx, new_vehicle);
}

void Platoon::add_last_vehicle(
	std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	if (verbose)
	{
		std::clog << "Platoon " << id << ": adding last veh with id "
			<< new_vehicle->get_id() << std::endl;
	}
	last_veh_idx--;
	add_vehicle(last_veh_idx, new_vehicle);
}

void Platoon::remove_vehicle_by_id(long veh_id)
{
	if (vehicle_id_to_position.find(veh_id) !=
		vehicle_id_to_position.end())
	{
		if (verbose)
		{
			std::clog << "Platoon " << get_id() << ", removing veh "
				<< veh_id << ", which is in position "
				<< vehicle_id_to_position.at(veh_id) << std::endl;
		}
		
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
	case Platoon::LaneChangeStrategy::synchronous:
		return can_start_synchronous_lane_change();
	case Platoon::LaneChangeStrategy::leader_first:
		break;
	case Platoon::LaneChangeStrategy::last_vehicle_first:
		break;
	case Platoon::LaneChangeStrategy::leader_first_and_invert:
		break;
	default:
		break;
	}
	return vehicles_lane_change_gap_status[veh_id];
}

bool Platoon::can_start_synchronous_lane_change()
{
	for (auto const& entry : vehicles_lane_change_gap_status)
	{
		if (!entry.second) return false;
	}
	return true;
}

bool Platoon::can_vehicle_leave_platoon(long veh_id)
{
	/* The platoon leader always stays in its platoon,
	which might be a single vehicle platoon */
	if (veh_id == get_leader_id()) return false;

	switch (lane_change_strategy)
	{
	case Platoon::LaneChangeStrategy::none:
		return true;
	case Platoon::LaneChangeStrategy::synchronous:
		return false;
	case Platoon::LaneChangeStrategy::leader_first:
		break;
	case Platoon::LaneChangeStrategy::last_vehicle_first:
		break;
	case Platoon::LaneChangeStrategy::leader_first_and_invert:
		break;
	default:
		break;
	}

	return true;
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon, long veh_id)
{
	if (vehicles.erase(idx_in_platoon) == 0)
		std::clog << "Vehicle not found in vehicles map\n";
	if (vehicle_id_to_position.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicle_id_to_position\n";
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
	
	if (verbose)
	{
		std::clog << "\tVehicle removed: " << *this << std::endl;
	}
}

void Platoon::add_vehicle(int idx_in_platoon,
	std::shared_ptr <PlatoonVehicle> new_vehicle)
{
	vehicles.insert({ idx_in_platoon, new_vehicle });
	vehicle_id_to_position.insert({ new_vehicle->get_id(), idx_in_platoon });
	vehicles_lane_change_gap_status.insert({ new_vehicle->get_id(), false });
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
