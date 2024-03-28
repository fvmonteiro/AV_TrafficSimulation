#include <iostream>
#include <map>

#include "NearbyVehicle.h"
#include "Platoon.h"
#include "PlatoonVehicle.h"
#include "PlatoonVehicleState.h"

Platoon::Platoon(long id, int platoon_lc_strategy, PlatoonVehicle* leader,
	bool verbose):
	id {id}, verbose{verbose}
{
	set_strategy(platoon_lc_strategy);
	add_last_vehicle(leader);
	desired_velocity = leader->get_desired_velocity();
}

Platoon::~Platoon()
{
	if (verbose)
	{
		std::cout << "Inside platoon " << id << " destructor\n"
			<< "Vehs in platoon: " << vehicles_by_position.size()
			<< std::endl;
	}
}

long Platoon::get_leader_id() const
{
	return is_empty() ? 0 : get_platoon_leader()->get_id();
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

PlatoonVehicle* Platoon::get_a_vehicle_by_position(int veh_pos) const
{
	try
	{
		return vehicles_by_position.at(veh_pos);
	}
	catch (std::out_of_range&)
	{
		std::stringstream message;
		message << "[Platoon] Trying to access veh at pos " << veh_pos
			<< ", but platoon length is " << get_size() << "\n";
		std::cout << message.str();
		std::clog << message.str();
		return nullptr;
	}
}

const PlatoonVehicle* Platoon::get_vehicle_by_id(long veh_id) const
{
	return is_vehicle_id_in_platoon(veh_id) ?
		vehicles_by_position.at(vehicle_id_to_position.at(veh_id))
		: nullptr;
}

const PlatoonVehicle* Platoon::get_preceding_vehicle(
	long veh_id) const
{
	int pos = vehicle_id_to_position.at(veh_id) - 1;
	return pos >= 0 ? vehicles_by_position.at(pos) : nullptr;
	//int veh_position = vehicle_id_to_position.at(veh_id);
	//while (vehicles_by_position.find(++veh_position)
	//	== vehicles_by_position.end())
	//{
	//	if (veh_position > leader_idx) return nullptr;
	//}
	//return vehicles_by_position.at(veh_position);
}

const PlatoonVehicle* Platoon::get_following_vehicle(
	long veh_id) const
{
	int pos = vehicle_id_to_position.at(veh_id) + 1;
	return pos < vehicles_by_position.size() ?
		vehicles_by_position.at(pos) : nullptr;
	//int veh_position = vehicle_id_to_position.at(veh_id);
	//while (vehicles_by_position.find(--veh_position)
	//	== vehicles_by_position.end())
	//{
	//	if (veh_position < last_veh_idx) return nullptr;
	//}
	//return vehicles_by_position.at(veh_position);
}

const PlatoonVehicle* Platoon::get_platoon_leader() const
{
	return is_empty() ? nullptr : vehicles_by_position.at(0);
}

const PlatoonVehicle* Platoon::get_last_vehicle() const
{
	return is_empty() ? nullptr 
		: vehicles_by_position.at(get_size() - 1);
}

int Platoon::get_vehicle_position_in_platoon(long veh_id) const
{
	try
	{
		return vehicle_id_to_position.at(veh_id);
	}
	catch (std::out_of_range&)
	{
		// just to make it easier to track the error
		std::stringstream message;
		message << "[Platoon::get_vehicle_position_in_platoon]"
			<< " veh id " << veh_id << " not in platoon with vehs "
			<< map_to_string(vehicle_id_to_position) << "\n";
		/* To be sure the message is seen in the persistent log file */
		std::cout << message.str();
		std::clog << message.str();
		throw;
	}
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
		get_platoon_leader()->get_destination_lane_follower_id();
	long platoon_leader_dest_lane_leader =
		get_platoon_leader()->get_destination_lane_leader_id();
	int i = 0;
	while (dest_lane_follower_id == 0 && i < vehicles_by_position.size())
	{
		//if (vehicles_by_position.find(i) != vehicles_by_position.end())
		//{
		/* Let k be the destination lane leader of some platoon vehicle.
		We assume that k exists, is not a platoon vehicle and is behind
		the platoon leader. If any of those assumptions doesn't hold,
		we move on to the platoon vehicle's dest lane follower.
		If the dest lane follower doesn't exist (id=0), the loop
		continues. */
		const auto& veh = vehicles_by_position.at(i);
		dest_lane_follower_id = veh->get_destination_lane_leader_id();
		if (dest_lane_follower_id == 0
			|| is_vehicle_id_in_platoon(dest_lane_follower_id)
			|| dest_lane_follower_id == platoon_leader_dest_lane_leader)
		{
			dest_lane_follower_id = veh->get_destination_lane_follower_id();
		}
		//}
		i++;
	}
	return dest_lane_follower_id;
}

long Platoon::get_destination_lane_vehicle_behind_last_vehicle() const
{
	return get_last_vehicle()->get_destination_lane_follower_id();
}

const NearbyVehicle* Platoon::get_destination_lane_leader() const
{
	long veh_id = 
		lane_change_approach->get_platoon_destination_lane_leader_id();
	
	if (veh_id == 0) return nullptr;
	for (auto const& vehicle : vehicles_by_position)
	{
		if (vehicle->is_vehicle_in_sight(veh_id))
		{
			return vehicle->get_nearby_vehicle_by_id(veh_id).get();
		}
	}
	return nullptr;

	/* We would like to find the non-platoon vehicle behind which
	the platoon is merging. This involves figuring out the front-most
	platoon vehicle that can change lanes, and reading its dest lane
	leader or orig lane leader based on whether it has already finished
	the maneuver.
	The current implementation approximates that by finding the current
	virtual leader of the lane changing vehicle(s) */

	//for (const auto& item : vehicles_by_position)
	//{
	//	if (item.second->has_virtual_leader())
	//	{
	//		return item.second->get_virtual_leader().get();
	//	}
	//}
	//return nullptr;
}

long Platoon::get_assisted_vehicle_id(long ego_id) const
{
	int ego_position = vehicle_id_to_position.at(ego_id);
	return lane_change_approach->get_assisted_vehicle_id(ego_position);
}

long Platoon::get_cooperating_vehicle_id() const
{
	return lane_change_approach->get_cooperating_vehicle_id();
}

std::vector<ContinuousStateVector> Platoon::get_vehicles_states() const
{
	/* By definition, we center around the leader */
	const PlatoonVehicle* platoon_leader = get_platoon_leader();
	std::vector<ContinuousStateVector> state_matrix{ 
		ContinuousStateVector(0., 0., 0., platoon_leader->get_velocity()) };
	for (int i = 1; i < get_size(); i++)
	{
		state_matrix.push_back(
			platoon_leader->get_nearby_vehicle_relative_states(
				vehicles_by_position.at(i)->get_id())
		);
	}

	return state_matrix;
}

std::vector<ContinuousStateVector> Platoon
::get_traffic_states_around_a_vehicle(int lane_changing_vehicle_pos) const
{
	share_vehicle_info_with_platoon_leader();
	std::vector<ContinuousStateVector> system_state_matrix;

	const PlatoonVehicle* platoon_leader = get_platoon_leader();
	ContinuousStateVector lo_states =
		platoon_leader->get_nearby_vehicle_relative_states(
			platoon_leader->get_leader_id(), platoon_leader->get_lane());
	system_state_matrix.push_back(lo_states);

	system_state_matrix.push_back(
		ContinuousStateVector(0., 0., 0., platoon_leader->get_velocity()));
	for (int i = 1; i < get_size(); i++)
	{
		system_state_matrix.push_back(
			platoon_leader->get_nearby_vehicle_relative_states(
				vehicles_by_position.at(i)->get_id())
		);
	}

	long ld_id = get_a_vehicle_by_position(lane_changing_vehicle_pos)
		->get_destination_lane_leader_id();
	ContinuousStateVector ld_states =
		platoon_leader->get_nearby_vehicle_relative_states(
			ld_id,
			platoon_leader->get_desired_lane_change_direction()
			.to_int()
		);
	system_state_matrix.push_back(ld_states);

	return system_state_matrix;
}

void Platoon::set_strategy(int strategy_number)
{
	/* TODO [Jan 24, 24] we're phasing out the current strategy
	class in favor of the approach class */
	/*std::unique_ptr<PlatoonLaneChangeStrategy> old_strategy =
		std::move(lane_change_strategy);*/
	std::unique_ptr<PlatoonLaneChangeApproach> old_approach =
		std::move(lane_change_approach);

	switch (Strategy(strategy_number))
	{
	/*case Platoon::no_strategy:
		lane_change_strategy = std::make_unique<NoStrategy>();
		break;*/
	case Platoon::synchronous_strategy:
		lane_change_approach =
			std::make_unique<SynchoronousApproach>(verbose);
		break;
	case Platoon::leader_first_strategy:
		lane_change_approach =
			std::make_unique<LeaderFirstApproach>(verbose);
		break;
	case Platoon::last_vehicle_first_strategy:
		lane_change_approach =
			std::make_unique<LastVehicleFirstApproach>(verbose);
		break;
	case Platoon::leader_first_invert_strategy:
		lane_change_approach =
			std::make_unique<LeaderFirstReverseApproach>(verbose);
		break;
	case Platoon::graph_strategy_min_time:
		lane_change_approach =
			std::make_unique<GraphApproachMinTime>(verbose);
		break;
	case Platoon::graph_strategy_min_accel:
		lane_change_approach =
			std::make_unique<GraphApproachMinAccel>(verbose);
		break;
	default:
		std::cout << "ERROR: Platoon lane change strategy not coded\n";
		//lane_change_strategy = std::make_unique<NoStrategy>();
		lane_change_approach =
			std::make_unique<SynchoronousApproach>(verbose);
		break;
	}

	if (verbose)
	{
		/*std::cout << "Platoon " << *this << "\n";
		std::cout << "LC Strategy change from ";
		if (old_strategy == nullptr) std::cout << "none";
		else std::cout << *old_strategy;
		std::cout << " to " << *lane_change_strategy << std::endl;*/

		std::cout << "Platoon " << *this << "\n";
		std::cout << "LC Approach change from ";
		if (old_approach == nullptr) std::cout << "none";
		else std::cout << old_approach->get_name();
		std::cout << " to " << lane_change_approach->get_name() << std::endl;
	}

	//lane_change_strategy->set_platoon(this);
	// Phasing out strategy
	//lane_change_strategy->reset_state_of_all_vehicles();

	lane_change_approach->set_platoon(this);
}

bool Platoon::is_empty() const
{
	return vehicles_by_position.size() == 0;
}

bool Platoon::is_vehicle_id_in_platoon(long veh_id) const
{
	return (vehicle_id_to_position.find(veh_id) 
		!= vehicle_id_to_position.end());
}

bool Platoon::has_lane_change_started() const
{
	return !std::isinf(lane_change_start_time);
}

void Platoon::add_last_vehicle(PlatoonVehicle* new_vehicle)
{
	if (verbose)
	{
		std::cout << "Platoon " << id << ": adding veh at the end with id "
			<< new_vehicle->get_id() << std::endl;
	}
	long veh_id = new_vehicle->get_id();
	vehicle_id_to_position.insert({ veh_id , 
		static_cast<int>(vehicles_by_position.size()) });
	vehicles_by_position.push_back(new_vehicle);
	new_vehicle->reset_state(
		std::make_unique<PlatoonVehicleLaneKeepingState>());
}

void Platoon::remove_vehicle_by_id(long veh_id, bool is_out_of_simulation)
{
	if (vehicle_id_to_position.find(veh_id) ==
		vehicle_id_to_position.end())
	{
		std::stringstream message;
		message << "[Platoon] Platoon id: " << get_id()
			<< ". Trying to erase veh " << veh_id
			<< " which is not in this platoon.\n";
		std::cout << message.str();
		std::clog << message.str();
		return;
	}
	long position_to_remove = vehicle_id_to_position.at(veh_id);
	// Double checking:
	if (vehicles_by_position[position_to_remove]->get_id() != veh_id)
	{
		std::stringstream message;
		message << "[Platoon] Mismatch between vehicles array "
			<< "and id_to_pos map " << *this << "\n";
		std::cout << message.str();
		std::clog << message.str();
		return;
	}

	if (verbose)
	{
		std::cout << "Platoon " << get_id() << ", removing veh "
			<< veh_id << ", which is in position "
			<< position_to_remove << std::endl;
	}

	remove_vehicle_by_position(position_to_remove);
	
}

bool Platoon::can_vehicle_leave_platoon(long ego_id) const
{
	int ego_position = vehicle_id_to_position.at(ego_id);
	return lane_change_approach->can_vehicle_leave_platoon(ego_position);
}

bool Platoon::can_vehicle_start_lane_change(long ego_id) const
{
	if (verbose) std::cout << "[Platoon] can_vehicle_start_lane_change\n";

	int veh_position = vehicle_id_to_position.at(ego_id);
	bool all_vehicles_have_or_had_intention =
		all_have_lane_change_intention() || has_lane_change_started();
	//try
	//{
	return all_vehicles_have_or_had_intention
		&& lane_change_approach->can_vehicle_start_lane_change(
			veh_position);
	//}
	//catch (const StateNotFoundException& e)
	//{
	//	std::cout << "[Platoon] " << e.what() << "\n";
	//	for (PlatoonVehicle* vehicle : vehicles_by_position)
	//	{
	//		vehicle->give_up_lane_change();
	//	}
	//	return false;
	//}
}

bool Platoon::all_have_lane_change_intention() const 
{
	return lane_change_intention_counter == get_size();
}

bool Platoon::any_has_lane_change_intention() const
{
	return lane_change_intention_counter > 0;
}

bool Platoon::is_lane_change_done() const 
{
	/* TODO: I'd rather code this as a function of maneuver_step 
	in the platoon approach, but I have to figure out how to increment
	the maneuver_step counter after the last vehicle starts lane changing
	*/
	return has_lane_change_started() 
		&& lane_change_intention_counter == 0;
}

void Platoon::receive_lane_change_intention_signal()
{
	lane_change_intention_counter++;
	if (verbose) std::cout << "[Platoon] lc intention counter "
		<< lane_change_intention_counter << "\n";
}

void Platoon::receive_lane_keeping_signal()
{
	lane_change_intention_counter--;
	if (verbose) std::cout << "[Platoon] lc intention counter "
		<< lane_change_intention_counter << "\n";
}

bool Platoon::is_stuck() const
{
	if (is_empty()) return false;
	const PlatoonVehicle* leader = get_platoon_leader();
	return (leader->get_velocity() < 5 / 3.6
		&& !leader->has_leader());
}

void Platoon::sort_vehicles_by_distance_traveled()
{
	/* Get vehicles in increasing order of position (assumes all
	platoon vehicles started at the same place) */
	std::map<double, PlatoonVehicle*>
		ordered_vehicles;
	for (PlatoonVehicle* vehicle: vehicles_by_position)
	{
		ordered_vehicles[vehicle->get_distance_traveled()]
			= vehicle;
	}	
	/* Traverse in reverse order and fill the class maps */
	int i = 0;
	for (auto iter = ordered_vehicles.rbegin(); 
		 iter != ordered_vehicles.rend(); ++iter) 
	{
		vehicles_by_position[i] = iter->second;
		vehicle_id_to_position[iter->second->get_id()] = i;
		i++;
	}
	/*for (auto& item : ordered_vehicles)
	{
		vehicles_by_position[last_veh_idx + i] = item.second;
		vehicle_id_to_position[item.second->get_id()] = last_veh_idx + i;
		i++;
	}*/
	//std::cout << "Platoon vehicles reordered\n";
}

long Platoon::create_lane_change_request_for_vehicle(
	long ego_id) const
{
	return lane_change_approach->create_platoon_lane_change_request(
			vehicle_id_to_position.at(ego_id));
}

long Platoon::define_desired_destination_lane_leader_id(long ego_id) const
{
	int pos_in_platoon = vehicle_id_to_position.at(ego_id);
	long virtual_leader_id = 
		lane_change_approach->get_desired_destination_lane_leader(
			pos_in_platoon);
	return virtual_leader_id;
}

void Platoon::share_vehicle_info_with_platoon_leader() const
{
	//if (verbose) std::cout << "[Platooon] sharing info with leader\n";
	PlatoonVehicle* platoon_leader = vehicles_by_position.at(0);
	for (int i = 1; i < get_size() - 1; i++)
	{
		PlatoonVehicle* platoon_veh_i = vehicles_by_position.at(i);
		long new_nv_id = vehicles_by_position.at(i + 1)->get_id();
		if (!platoon_leader->is_vehicle_in_sight(new_nv_id))
		{
			platoon_leader->add_nearby_vehicle_from_another(*platoon_veh_i,
				new_nv_id);
		}
	}
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon)
{
	vehicles_by_position.erase(vehicles_by_position.begin() 
		+ idx_in_platoon);
	vehicle_id_to_position.clear();
	for (int i = 0; i < vehicles_by_position.size(); i++)
	{
		vehicle_id_to_position[vehicles_by_position[i]->get_id()] = i;
	}
	if (verbose)
	{
		std::cout << "\tVehicle removed. Platoon now is " 
			<< *this << std::endl;
	}

	/* The lane change strategies get messed up when a vehicle
	leaves the platoon because they're based on the initial
	number of vehicles. */
	for (PlatoonVehicle* vehicle : get_vehicles_by_position())
	{
		vehicle->give_up_lane_change();
	}
	if (!is_lane_change_done())
	{
		lane_change_approach->clear();
	}
}

std::ostream& operator<< (std::ostream& out, const Platoon& platoon)
{
	out << "Platoon " << platoon.get_id()
		<< "; # vehs " << platoon.vehicles_by_position.size() 
		<< "; vehicle array: ( " ;
	for (const auto& vehicle : platoon.vehicles_by_position)
	{
		out << vehicle->get_id() << " ";
	}
	out << "); position to id map: " 
		<<	map_to_string(platoon.vehicle_id_to_position);
	return out;
}