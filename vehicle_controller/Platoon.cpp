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
	if (vehicles_by_position.find(veh_pos) != vehicles_by_position.end())
	{
		return vehicles_by_position.at(veh_pos);
	}
	std::string message = "No veh at pos " + std::to_string(veh_pos)
		+ ". Available platoon positions are: ";
	for (const auto& item : vehicles_by_position)
	{
		message += std::to_string(item.first);
	}
	std::clog << message << "\n";
	return nullptr;
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
		if (vehicles_by_position.find(i) != vehicles_by_position.end())
		{
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
		}
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
	for (auto const& item : vehicles_by_position)
	{
		if (item.second->is_vehicle_in_sight(veh_id))
		{
			return item.second->get_nearby_vehicle_by_id(veh_id).get();
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
	std::vector<ContinuousStateVector> system_state_matrix{ 
		ContinuousStateVector(0., 0., 0., platoon_leader->get_velocity()) };
	for (int i = 1; i < get_size(); i++)
	{
		system_state_matrix.push_back(
			platoon_leader->get_nearby_vehicle_relative_states(
				vehicles_by_position.at(i)->get_id())
		);
	}

	//ContinuousStateVector leader_states(0., 0., 0., 
	//	platoon_leader->get_velocity());
	//double leader_x = platoon_leader->get_state_vector().get_x();
	//double leader_y = platoon_leader->get_state_vector().get_y();
	//double delta_x{ 0.0 };
	//double delta_y{ 0.0 };
	//for (int i = 1; i < vehicles_by_position.size(); i++)
	//{
	//	const PlatoonVehicle* veh = vehicles_by_position.at(i);
	//	/* Relative position values given by VISSIM are more reliable
	//	than the ones obtained by internal computations. However, during a 
	//	lane change the vehicle's leader may not be its platoon preceding 
	//	vehicle. So we address both cases. */
	//	if (veh->get_leader_id() == vehicles_by_position.at(i - 1)->get_id())
	//	{
	//		const NearbyVehicle* leader = veh->get_leader().get();
	//		delta_y -= leader->get_relative_lateral_position();
	//		delta_x -= veh->compute_gap_to_a_leader(leader);
	//	}
	//	else
	//	{
	//		const PlatoonVehicle* leader = vehicles_by_position.at(i - 1);
	//		delta_y += (veh->get_lateral_position()
	//				    - leader->get_lateral_position());
	//		/* Since the platoon vehicles start the simulation at different
	//		positions, the different in distance travelled does not equal
	//		the distance between them. */
	//		delta_x += (veh->get_distance_traveled()
	//					- leader->get_distance_traveled()
	//					- veh->get_free_flow_intra_platoon_gap());
	//	}
	//	system_state_matrix.push_back(
	//		ContinuousStateVector(delta_x, delta_y,
	//			veh->get_orientation_angle(), veh->get_velocity()));
	//	delta_x += veh->get_length();
	//}

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
		//lane_change_strategy = std::make_unique<SynchronousStrategy>();
		lane_change_approach =
			std::make_unique<SynchoronousApproach>(verbose);
		break;
	case Platoon::leader_first_strategy:
		//lane_change_strategy = std::make_unique<LeaderFirstStrategy>();
		lane_change_approach =
			std::make_unique<LeaderFirstApproach>(verbose);
		break;
	case Platoon::last_vehicle_first_strategy:
		//lane_change_strategy = std::make_unique<LastVehicleFirstStrategy>();
		lane_change_approach =
			std::make_unique<LastVehicleFirstApproach>(verbose);
		break;
	case Platoon::leader_first_invert_strategy:
		//lane_change_strategy =
		//	std::make_unique<LeaderFirstAndInvertStrategy>();
		lane_change_approach =
			std::make_unique<LeaderFirstReverseApproach>(verbose);
		break;
	case Platoon::graph_strategy:
		lane_change_approach =
			std::make_unique<GraphApproach>(verbose);
		break;
	default:
		std::clog << "ERROR: Platoon lane change strategy not coded\n";
		//lane_change_strategy = std::make_unique<NoStrategy>();
		lane_change_approach =
			std::make_unique<SynchoronousApproach>(verbose);
		break;
	}

	if (verbose)
	{
		/*std::clog << "Platoon " << *this << "\n";
		std::clog << "LC Strategy change from ";
		if (old_strategy == nullptr) std::clog << "none";
		else std::clog << *old_strategy;
		std::clog << " to " << *lane_change_strategy << std::endl;*/

		std::clog << "Platoon " << *this << "\n";
		std::clog << "LC Approach change from ";
		if (old_approach == nullptr) std::clog << "none";
		else std::clog << old_approach->get_name();
		std::clog << " to " << lane_change_approach->get_name() << std::endl;
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
		std::clog << "Platoon " << id << ": adding last veh with id "
			<< new_vehicle->get_id() << std::endl;
	}
	//last_veh_idx++;
	add_vehicle(get_size(), new_vehicle);
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

bool Platoon::can_vehicle_leave_platoon(long ego_id) const
{
	int ego_position = vehicle_id_to_position.at(ego_id);
	return lane_change_approach->can_vehicle_leave_platoon(ego_position);
}

bool Platoon::can_vehicle_start_lane_change(long ego_id) const
{
	if (verbose) std::clog << "[Platoon] can_vehicle_start_lane_change\n";

	int veh_position = vehicle_id_to_position.at(ego_id);
	bool all_vehicles_have_or_had_intention =
		has_lane_change_intention() || has_lane_change_started();
	try
	{
		return all_vehicles_have_or_had_intention
			&& lane_change_approach->can_vehicle_start_lane_change(
				veh_position);
	}
	catch (const StateNotFoundException& e)
	{
		std::clog << "[Platoon] " << e.what() << "\n";
		for (std::pair<int, PlatoonVehicle*> pos_veh : vehicles_by_position)
		{
			pos_veh.second->give_up_lane_change();
		}
		return false;
	}
}

bool Platoon::has_lane_change_intention() const {
	return lane_change_intention_counter == get_size();
}

void Platoon::receive_lane_change_intention_signal()
{
	lane_change_intention_counter++;
}

void Platoon::receive_lane_keeping_signal()
{
	lane_change_intention_counter--;
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
	for (auto& item : vehicles_by_position)
	{
		ordered_vehicles[item.second->get_distance_traveled()]
			= item.second;
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
	//std::clog << "Platoon vehicles reordered\n";
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
		
		///* The platoon leader's immediate follower should already be in 
		//its nearby vehicles map. The other ones are added in this loop */
		//NearbyVehicle* source_nv = 
		//	platoon_leader->get_nearby_vehicle_by_id(
		//		platoon_veh_i->get_id()).get();
		///* The platoon vehicle behind platoon vehicle i */
		//NearbyVehicle* follower_of_platoon_veh_i =
		//	platoon_veh_i->get_nearby_vehicle_by_id(
		//		vehicles_by_position.at(i + 1)->get_id()).get();
		//if (source_nv == nullptr 
		//	|| follower_of_platoon_veh_i == nullptr)
		//{
		//	std::clog << "Error at "
		//		<< "[Platoon::share_vehicle_info_with_platoon_leader] "
		//		<< (source_nv == nullptr ?
		//			"'nv_already_in_set'" : "'follower_of_platoon_veh_i'") 
		//		<< " not in map.\n";
		//}
		//else if (!platoon_leader->is_vehicle_in_sight(
		//	follower_of_platoon_veh_i->get_id()))
		//{
		//	platoon_leader->add_nearby_vehicle_from_another(*platoon_veh_i,
		//		follower_of_platoon_veh_i->get_id());
		//	NearbyVehicle new_nv(*follower_of_platoon_veh_i);
		//	new_nv.offset_from_another(*source_nv);
		//	platoon_leader->add_nearby_vehicle(new_nv);
		//}
	}
}

void Platoon::add_leader(PlatoonVehicle* new_vehicle)
{
	if (verbose)
	{
		std::clog << "Platoon " << id << ": adding leader with id "
			<< new_vehicle->get_id() << std::endl;
	}
	add_vehicle(0, new_vehicle);
}

void Platoon::add_vehicle(int idx_in_platoon,
	PlatoonVehicle* new_vehicle)
{
	long veh_id = new_vehicle->get_id();
	vehicles_by_position.insert({ idx_in_platoon, new_vehicle });
	vehicle_id_to_position.insert({ veh_id , idx_in_platoon });

	// Phasing out strategy
	//if (lane_change_strategy != nullptr)
	//{
	//	/* Tricky part of the code - we are forcing the vehicle out 
	//	of its current state. This might interrupt some maneuver. */
	//	new_vehicle->reset_state(
	//		lane_change_strategy->get_new_lane_keeping_state());
	//}

	new_vehicle->reset_state(
		std::make_unique<PlatoonVehicleLaneKeepingState>());
}

void Platoon::remove_vehicle_by_position(int idx_in_platoon, long veh_id,
	bool is_out_of_simulation)
{
	/*try
	{
		vehicles_by_position.erase(idx_in_platoon);
	}
	catch (const std::out_of_range&)
	{
		std::clog << "Vehicle not found in vehicles map\n";
	}

	if (vehicle_id_to_position.erase(veh_id) == 0)
		std::clog << "Veh id not found in vehicle_id_to_position\n";*/

	/* In case the removed vehicle was the leader or the last vehicle,
	we update the respective indices */
	//while (vehicles_by_position.find(leader_idx)
	//	== vehicles_by_position.end())
	//{
	//	leader_idx--;
	//	if (leader_idx < last_veh_idx) return; // platoon empty!
	//}
	//while (vehicles_by_position.find(last_veh_idx)
	//	== vehicles_by_position.end())
	//{
	//	last_veh_idx++;
	//	if (last_veh_idx > leader_idx) return; // platoon empty!
	//}
	/* "Move" all vehicles ahead one position which overwrites
	the vehicle being removed */

	for (int i = idx_in_platoon; i < vehicles_by_position.size() - 1; i++)
	{
		vehicles_by_position[idx_in_platoon] =
			vehicles_by_position[idx_in_platoon + 1];
		vehicle_id_to_position[idx_in_platoon] =
			vehicle_id_to_position[idx_in_platoon + 1];
	}

	/* Remove the last vehicle (which has already been copied to the
	last-1 position) */
	vehicles_by_position.erase(get_size() - 1);
	vehicle_id_to_position.erase(get_size() - 1);
	if (verbose)
	{
		std::clog << "\tVehicle removed: " << *this << std::endl;
	}
}

void Platoon::update_position_maps()
{

}

std::ostream& operator<< (std::ostream& out, const Platoon& platoon)
{
	out << "Platoon " << platoon.get_id()
		//<< ", v_r=" << platoon.get_desired_velocity()
		<< ", # vehs " << platoon.vehicles_by_position.size() 
		<< ". (pos, veh id) : " ;
	for (const auto& item : platoon.vehicles_by_position)
	{
		out << "(" << item.first << ", " << item.second->get_id() << "), ";
	}

	//for (int i = platoon.leader_idx; i <= platoon.last_veh_idx; i++)
	//{
	//	if (platoon.vehicles_by_position.find(i) 
	//		!= platoon.vehicles_by_position.end())
	//	{
	//		out << "(" << i << ", " 
	//			<< platoon.vehicles_by_position.at(i)->get_id()
	//			<< "), ";
	//	}
	//	else
	//	{
	//		out << "( " << i << ", x), ";
	//	}
	//}
	//out << "leader idx: " << platoon.leader_idx
	//	<< ", last veh idx: " << platoon.last_veh_idx;

	return out; // return std::ostream so we can chain calls to operator<<
}