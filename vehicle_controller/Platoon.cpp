#include <iostream>

#include "Platoon.h"
#include "PlatoonVehicle.h"

Platoon::Platoon(long id, std::shared_ptr<PlatoonVehicle> leader,
	bool verbose):
	id {id}, verbose{verbose}
{
	add_leader(leader);
	desired_velocity = leader->get_desired_velocity();
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

std::shared_ptr<PlatoonVehicle> Platoon::get_platoon_leader() const
{
	return is_empty() ? nullptr : vehicles.at(leader_idx);
}

std::shared_ptr<PlatoonVehicle> Platoon::get_last_vehicle() const
{
	return is_empty() ? nullptr : vehicles.at(last_veh_idx);
}

long Platoon::get_leader_id() const
{
	return is_empty() ? 0 : get_platoon_leader()->get_id();
}

bool Platoon::is_empty() const
{
	return vehicles.size() == 0;
}

long Platoon::get_last_veh_id() const
{
	return is_empty() ? 0 : get_last_vehicle()->get_id();
}

long Platoon::get_preceding_vehicle_id(long veh_id) const
{
	std::shared_ptr<PlatoonVehicle> preceding_vehicle =
		get_preceding_vehicle(veh_id);
	return preceding_vehicle != nullptr ? preceding_vehicle->get_id() : 0;
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
	/*case Platoon::leader_first_invert_strategy:
		break;*/
	default:
		std::clog << "ERROR: Platoon lane change strategy not coded\n";
		lane_change_strategy = std::make_unique<NoStrategy>();
		break;
	}

	/*if (verbose)
	{*/
		std::clog << "Platoon " << get_id() << "\n";
		std::clog << "LC Strategy change from ";
		if (old_strategy == nullptr) std::clog << "none";
		else std::clog << *old_strategy;
		std::clog << " to " << *lane_change_strategy << std::endl;
	//}

	lane_change_strategy->set_platoon(this);
	lane_change_strategy->set_state_of_all_vehicles();
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

//bool Platoon::check_all_vehicles_lane_change_gaps()
//{
//	for (auto& v : vehicles)
//	{
//		if (!v.second->get_lane_change_gaps_safety().is_lane_change_safe()) 
//		{
//			std::clog << "veh " << v.second->get_id() << " gaps not safe\n";
//			return false;
//		}
//	}
//	return true;
//}

//void Platoon::set_vehicle_lane_change_state(PlatoonVehicle& platoon_vehicle, 
//	bool should_change_lane)
//{
//	lane_change_strategy->update_vehicle_lane_change_state(
//		platoon_vehicle, should_change_lane, vehicles_lane_change_states);
//}

//void Platoon::set_vehicle_lane_change_gap_status(long veh_id, bool is_safe)
//{
//	vehicles_lane_change_gap_status[veh_id] = is_safe;
//}

//bool Platoon::can_vehicle_start_lane_change(
//	const PlatoonVehicle& platoon_vehicle)
//{
//	return lane_change_strategy->can_vehicle_start_lane_change(
//		platoon_vehicle, vehicles_lane_change_gap_status);
//}

std::shared_ptr<PlatoonVehicle> Platoon::get_vehicle_by_id(long veh_id) const
{
	if (vehicle_id_to_position.find(veh_id) == vehicle_id_to_position.end())
	{
		std::clog << "Platoon: " << *this << "\nTrying to get veh "
			<< veh_id << std::endl;
		return nullptr;
	}
	return vehicles.at(vehicle_id_to_position.at(veh_id));
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon, long veh_id)
{
	try
	{
		/* [Nov 18, 22] Should the new state assignment be managed by 
		the vehicle instead? And should it be based on the current state? */
		vehicles.at(idx_in_platoon)->set_state(
			std::make_unique<SingleVehicleLaneKeepingState>());
		vehicles.erase(idx_in_platoon);
	}
	catch (const std::out_of_range&)
	{
		std::clog << "Vehicle not found in vehicles map\n";
	}

	if (vehicle_id_to_position.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicle_id_to_position\n";
	/*if (vehicles_lane_change_gap_status.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicles_lane_change_gap_status\n";*/
	//if (vehicles_lane_change_states.erase(veh_id) == 0)
	//	std::clog << "Veh id not found in vehicles_lane_change_states\n";

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
	long veh_id = new_vehicle->get_id();
	vehicles.insert({ idx_in_platoon, new_vehicle });
	vehicle_id_to_position.insert({veh_id , idx_in_platoon });
	//vehicles_lane_change_gap_status.insert({ veh_id, false });
	if (lane_change_strategy != nullptr)
	{
		new_vehicle->set_state(
			lane_change_strategy->get_new_lane_keeping_state());
	}
	/*vehicles_lane_change_states.insert({ veh_id, 
		PlatoonLaneChangeStrategy::LaneChangeState::lane_keeping });*/
}

std::shared_ptr<PlatoonVehicle> Platoon::get_preceding_vehicle(
	long veh_id) const
{
	int veh_position = vehicle_id_to_position.at(veh_id);
	while (vehicles.find(++veh_position) == vehicles.end())
	{
		if (veh_position > leader_idx) return nullptr;
	}
	return vehicles.at(veh_position);
}

std::shared_ptr<PlatoonVehicle> Platoon::get_following_vehicle(
	long veh_id) const
{
	int veh_position = vehicle_id_to_position.at(veh_id);
	while (vehicles.find(--veh_position) == vehicles.end())
	{
		if (veh_position < last_veh_idx) return nullptr;
	}
	return vehicles.at(veh_position);
}

long Platoon::get_assisted_vehicle_id(long veh_id) const
{
	return lane_change_strategy->get_assisted_vehicle_id(
		*get_vehicle_by_id(veh_id));
}

bool Platoon::can_vehicle_start_adjustment_to_dest_lane_leader(
	long veh_id) const
{
	return lane_change_strategy->can_adjust_to_dest_lane_leader(
		*get_vehicle_by_id(veh_id));
}

bool Platoon::can_vehicle_leave_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	/* The platoon leader always stays in its platoon,
	which might be a single vehicle platoon */
	return !platoon_vehicle.is_platoon_leader() 
		&& lane_change_strategy->can_vehicle_leave_platoon(platoon_vehicle);
}

//bool Platoon::can_vehicle_start_longitudinal_adjustment(long veh_id) const
//{
//	return vehicles_lane_change_states.at(veh_id) 
//		== PlatoonLaneChangeStrategy::LaneChangeState::long_adjustment;
//}

bool Platoon::has_a_vehicle_cut_in_the_platoon(
	const PlatoonVehicle& platoon_vehicle) const
{
	if (platoon_vehicle.is_platoon_leader()
		|| !platoon_vehicle.has_leader())
	{
		return false;
	}

	std::shared_ptr<NearbyVehicle> leader = platoon_vehicle.get_leader();
	std::shared_ptr<PlatoonVehicle> precending_platoon_vehicle =
		platoon_vehicle.get_preceding_vehicle_in_platoon();
	bool are_vehs_lane_keeping =
		(precending_platoon_vehicle->get_state()->get_phase_number() == 0)
		&& (platoon_vehicle.get_state()->get_phase_number() == 0);

	return are_vehs_lane_keeping
		&& *leader != *precending_platoon_vehicle;
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