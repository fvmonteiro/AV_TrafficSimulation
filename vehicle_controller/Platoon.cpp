#include <iostream>
#include <map>

#include "Platoon.h"
#include "PlatoonVehicle.h"

Platoon::Platoon(long id, int platoon_lc_strategy, PlatoonVehicle* leader,
	bool verbose):
	id {id}, verbose{verbose}
{
	set_strategy(platoon_lc_strategy);
	add_leader(leader);
	desired_velocity = leader->get_desired_velocity();
}

Platoon::~Platoon()
{
	if (verbose)
	{
		std::clog << "Inside platoon " << id << " destructor\n"
			<< "Vehs in platoon: " << vehicles_by_position.size()
			<< std::endl;
	}
}

const PlatoonVehicle* Platoon::get_platoon_leader() const
{
	return is_empty() ? nullptr : vehicles_by_position.at(leader_idx);
}

const PlatoonVehicle* Platoon::get_last_vehicle() const
{
	return is_empty() ? nullptr : vehicles_by_position.at(last_veh_idx);
}

long Platoon::get_leader_id() const
{
	return is_empty() ? 0 : get_platoon_leader()->get_id();
}

bool Platoon::is_empty() const
{
	return vehicles_by_position.size() == 0;
}

bool Platoon::is_vehicle_in_platoon(long veh_id) const
{
	return (vehicle_id_to_position.find(veh_id) 
		!= vehicle_id_to_position.end());
}

long Platoon::get_last_veh_id() const
{
	return is_empty() ? 0 : get_last_vehicle()->get_id();
}

long Platoon::get_preceding_vehicle_id(long veh_id) const
{
	const PlatoonVehicle* preceding_vehicle =
		get_preceding_vehicle(veh_id);
	return preceding_vehicle != nullptr ? preceding_vehicle->get_id() : 0;
}

long Platoon::get_following_vehicle_id(long veh_id) const
{
	const PlatoonVehicle* following_vehicle =
		get_following_vehicle(veh_id);
	return following_vehicle != nullptr ? following_vehicle->get_id() : 0;
}

//std::shared_ptr<PlatoonVehicle> Platoon::get_preceding_vehicle(
//	const PlatoonVehicle& platoon_vehicle) const
//{
//	return get_preceding_vehicle(platoon_vehicle.get_id());
//}

//PlatoonLaneChangeStrategy::LaneChangeState Platoon::get_lane_change_state(
//	long veh_id) const
//{
//	return vehicles_lane_change_states.at(veh_id);
//}

void Platoon::set_strategy(int strategy_number)
{
	std::unique_ptr<PlatoonLaneChangeStrategy> old_strategy =
		std::move(lane_change_strategy);

	switch (Strategy(strategy_number))
	{
	case Platoon::no_strategy:
		lane_change_strategy = std::make_unique<NoStrategy>();
		break;
	case Platoon::synchronous_strategy:
		lane_change_strategy = std::make_unique<SynchronousStrategy>();
		break;
	case Platoon::leader_first_strategy:
		lane_change_strategy = std::make_unique<LeaderFirstStrategy>();
		break;
	case Platoon::last_vehicle_first_strategy:
		lane_change_strategy = std::make_unique<LastVehicleFirstStrategy>();
		break;
	case Platoon::leader_first_invert_strategy:
		lane_change_strategy = 
			std::make_unique<LeaderFirstAndInvertStrategy>();
		break;
	default:
		std::clog << "ERROR: Platoon lane change strategy not coded\n";
		lane_change_strategy = std::make_unique<NoStrategy>();
		break;
	}

	if (verbose)
	{
		std::clog << "Platoon " << *this << "\n";
		std::clog << "LC Strategy change from ";
		if (old_strategy == nullptr) std::clog << "none";
		else std::clog << *old_strategy;
		std::clog << " to " << *lane_change_strategy << std::endl;
	}

	lane_change_strategy->set_platoon(this);
	lane_change_strategy->reset_state_of_all_vehicles();

}

void Platoon::add_leader(PlatoonVehicle* new_vehicle)
{
	if (verbose)
	{
		std::clog << "Platoon " << id << ": adding leader with id "
			<< new_vehicle->get_id() << std::endl;
	}
	leader_idx++;
	add_vehicle(leader_idx, new_vehicle);
}

void Platoon::add_last_vehicle(PlatoonVehicle* new_vehicle)
{
	if (verbose)
	{
		std::clog << "Platoon " << id << ": adding last veh with id "
			<< new_vehicle->get_id() << std::endl;
	}
	last_veh_idx--;
	add_vehicle(last_veh_idx, new_vehicle);
}

void Platoon::remove_vehicle_by_id(long veh_id, bool is_out_of_simulation)
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
		remove_vehicle_by_position(position_to_remove, veh_id,
			is_out_of_simulation);

		if (verbose)
		{
			std::clog << "Platoon after removal " << *this << std::endl;
		}
	}
	else
	{
		std::clog << "Platoon " << get_id()
			<< ". Trying to erase veh " << veh_id
			<< " which is not in this platoon.\n";
	}
}

const PlatoonVehicle* Platoon::get_vehicle_by_id(long veh_id) const
{
	return is_vehicle_in_platoon(veh_id) ?
		vehicles_by_position.at(vehicle_id_to_position.at(veh_id)) 
		: nullptr;
}

long Platoon::get_destination_lane_vehicle_behind_the_leader() const
{
	/* We're looking for the vehicle at the destination lane that's 
	closest to the platoon leader, and that does not belong to the platoon 
	This vehicle might be:
	- The platoon leader's dest lane follower
	- Some other platoon vehicle's dest lane leader 
	- Some other platoon vehicle's dest lane follower */

	long dest_lane_follower_id = 
		get_platoon_leader()->get_dest_lane_follower_id();
	long platoon_leader_dest_lane_leader = 
		get_platoon_leader()->get_dest_lane_leader_id();
	int i = leader_idx - 1;
	while (dest_lane_follower_id == 0 && i >= last_veh_idx)
	{
		if (vehicles_by_position.find(i) != vehicles_by_position.end())
		{
			/* Let k be the destination lane leader of some platoon vehicle.
			We assume that k exists, is not a platoon vehicle and is behind
			the platoon leader. If any of those assumptions doesn't hold, 
			we move on to the platoon vehicle's dest lane follower. 
			If the dest lane follower doesn't exist (id=0), the loop 
			continues. */
			const auto& veh = vehicles_by_position.at(i);
			dest_lane_follower_id = veh->get_dest_lane_leader_id();
			if (dest_lane_follower_id == 0
				|| is_vehicle_in_platoon(dest_lane_follower_id)
				|| dest_lane_follower_id == platoon_leader_dest_lane_leader)
			{
				dest_lane_follower_id = veh->get_dest_lane_follower_id();
			}
		}
		i--;
	}
	return dest_lane_follower_id;
}

long Platoon::get_destination_lane_vehicle_behind_last_vehicle() const
{
	return get_last_vehicle()->get_dest_lane_follower_id();
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon, long veh_id,
	bool is_out_of_simulation)
{
	try
	{
		/*if (!is_out_of_simulation)
		{
			vehicles_by_position.at(idx_in_platoon)->set_state(
				std::make_unique<SingleVehicleLaneKeepingState>());
		}*/
		vehicles_by_position.erase(idx_in_platoon);
	}
	catch (const std::out_of_range&)
	{
		std::clog << "Vehicle not found in vehicles map\n";
	}

	if (vehicle_id_to_position.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicle_id_to_position\n";
	
	/* In case the removed vehicle was the leader or the last vehicle,
	we update the respective indices */
	while (vehicles_by_position.find(leader_idx) 
		== vehicles_by_position.end())
	{
		leader_idx--;
		if (leader_idx < last_veh_idx) return; // platoon empty!
	}
	while (vehicles_by_position.find(last_veh_idx) 
		== vehicles_by_position.end())
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
	PlatoonVehicle* new_vehicle)
{
	long veh_id = new_vehicle->get_id();
	vehicles_by_position.insert({ idx_in_platoon, new_vehicle });
	vehicle_id_to_position.insert({veh_id , idx_in_platoon });
	if (lane_change_strategy != nullptr)
	{
		/* Tricky part of the code - we are forcing the vehicle out 
		of its current state. This might interrupt some maneuver. */
		new_vehicle->reset_state(
			lane_change_strategy->get_new_lane_keeping_state());
	}
}

const PlatoonVehicle* Platoon::get_preceding_vehicle(
	long veh_id) const
{
	int veh_position = vehicle_id_to_position.at(veh_id);
	while (vehicles_by_position.find(++veh_position) 
		== vehicles_by_position.end())
	{
		if (veh_position > leader_idx) return nullptr;
	}
	return vehicles_by_position.at(veh_position);
}

const PlatoonVehicle* Platoon::get_following_vehicle(
	long veh_id) const
{
	int veh_position = vehicle_id_to_position.at(veh_id);
	while (vehicles_by_position.find(--veh_position) 
		== vehicles_by_position.end())
	{
		if (veh_position < last_veh_idx) return nullptr;
	}
	return vehicles_by_position.at(veh_position);
}

//long Platoon::get_assisted_vehicle_id(long veh_id) const
//{
//	return lane_change_strategy->get_assisted_vehicle_id(
//		*get_vehicle_by_id(veh_id));
//}

//bool Platoon::can_vehicle_start_adjustment_to_virtual_leader(
//	long veh_id) const
//{
//	return lane_change_strategy->can_adjust_to_virtual_leader(
//		*get_vehicle_by_id(veh_id));
//}

bool Platoon::can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	//std::clog << "\t[Platoon] can vehicle leave platoon method\n";
	/* The platoon leader always stays in its platoon,
	which might be a single vehicle platoon */
	return !platoon_vehicle.is_platoon_leader() 
		&& lane_change_strategy->can_vehicle_leave_platoon(platoon_vehicle);
}

bool Platoon::is_stuck() const
{
	if (is_empty()) return false;
	const PlatoonVehicle* leader = get_platoon_leader();
	return (leader->get_velocity() < 5 / 3.6
		&& !leader->has_leader());
}

void Platoon::reorder_vehicles()
{
	/* Get vehicles in increasing order of position (assumes all
	platoon vehicles started at the same place) */
	std::map<double, std::shared_ptr<PlatoonVehicle>>
		vehicles_by_position;
	for (auto& item : vehicles_by_position)
	{
		vehicles_by_position[item.second->get_distance_traveled()]
			= item.second;
	}	
	/* Fill the vectors in the proper order */
	int i = 0;
	for (auto& item : vehicles_by_position)
	{
		vehicles_by_position[last_veh_idx + i] = item.second;
		vehicle_id_to_position[item.second->get_id()] = last_veh_idx + i;
		i++;
	}
	//std::clog << "Platoon vehicles reordered\n";
}

long Platoon::create_lane_change_request_for_vehicle(
	const PlatoonVehicle& platoon_vehicle) const
{
	return lane_change_strategy
		->create_platoon_lane_change_request(platoon_vehicle);
}

std::shared_ptr<NearbyVehicle> Platoon
::define_virtual_leader(const PlatoonVehicle& platoon_vehicle)
const
{
	return lane_change_strategy->define_virtual_leader(platoon_vehicle);
}

std::ostream& operator<< (std::ostream& out, const Platoon& platoon)
{
	out << "Platoon " << platoon.get_id()
		//<< ", v_r=" << platoon.get_desired_velocity()
		<< ", # vehs " << platoon.vehicles_by_position.size() 
		<< ". (pos, veh id) : " ;
	for (int i = platoon.last_veh_idx; i <= platoon.leader_idx; i++)
	{
		if (platoon.vehicles_by_position.find(i) 
			!= platoon.vehicles_by_position.end())
		{
			out << "(" << i << ", " 
				<< platoon.vehicles_by_position.at(i)->get_id()
				<< "), ";
		}
		else
		{
			out << "( " << i << ", x), ";
		}
	}
	out << "leader idx: " << platoon.leader_idx
		<< ", last veh idx: " << platoon.last_veh_idx;
	return out; // return std::ostream so we can chain calls to operator<<
}