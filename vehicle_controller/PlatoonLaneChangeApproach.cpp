#include <algorithm>
#include <numeric>

#include "Platoon.h"
#include "PlatoonLaneChangeApproach.h"
#include "PlatoonVehicle.h"

PlatoonLaneChangeApproach::PlatoonLaneChangeApproach(int id, std::string name,
	bool verbose) 
	: id(id), name(name), verbose(verbose) {}

long PlatoonLaneChangeApproach::get_desired_destination_lane_leader(
	int ego_position)
{
	PlatoonVehicle* ego_vehicle = 
		platoon->get_a_vehicle_by_position(ego_position);

	if (!ego_vehicle->has_lane_change_intention() || !is_initialized
		|| !is_vehicle_turn_to_lane_change(ego_position)) return 0;

	int virtual_leader_id{ 0 };
	
	if (maneuver_step == 0)
	{
		/* The first vehicles to simultaneously change lanes do so behind the
		destination lane leader of the front-most vehicle (similar to
		single vehicle lane change) */
		int first_lc_pos = *std::max_element(
			platoon_lane_change_order.lc_order[0].begin(),
			platoon_lane_change_order.lc_order[0].end());
		const PlatoonVehicle* front_most_veh =
			platoon->get_a_vehicle_by_position(first_lc_pos);
		virtual_leader_id = 
			front_most_veh->get_suitable_destination_lane_leader_id();
		if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
		{
			ego_vehicle->add_nearby_vehicle_from_another(*front_most_veh, 
				virtual_leader_id);
		}
	}
	else
	{
		int coop_pos = get_current_coop_vehicle_position();
		if (coop_pos == -1)
		{
			const PlatoonVehicle* veh = platoon->get_a_vehicle_by_position(
				last_dest_lane_vehicle_pos);
			virtual_leader_id = veh->get_id();
			if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
			{
				ego_vehicle->add_another_as_nearby_vehicle(*veh);
			}
		}
		else
		{
			const PlatoonVehicle* coop_veh =
				platoon->get_a_vehicle_by_position(coop_pos);
			virtual_leader_id = coop_veh->get_leader_id();
			if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
			{
				ego_vehicle->add_nearby_vehicle_from_another(*coop_veh,
					virtual_leader_id);
			}
		}
	}

	return virtual_leader_id;
}

long PlatoonLaneChangeApproach::get_assisted_vehicle_id(int ego_position)
{
	const PlatoonVehicle* ego_vehicle =
		platoon->get_a_vehicle_by_position(ego_position);

	if (ego_vehicle->has_lane_change_intention() || !is_initialized
		|| ego_position != get_current_coop_vehicle_position()) return 0;
	int rear_most_pos = get_rearmost_lane_changing_vehicle_position();
	return platoon->get_a_vehicle_by_position(rear_most_pos)->get_id();
}

bool PlatoonLaneChangeApproach::can_vehicle_start_lane_change(
	int veh_position)
{
	
	if (!is_initialized) decide_lane_change_order();
	if (!is_initialized) return false;

	if (maneuver_step >= platoon_lane_change_order.number_of_steps())
	{
		std::clog << "[WARNING] maneuver step counter greater than "
			"total maneuver steps\n";
		return false;
	}

	std::unordered_set<int> next_to_move = get_current_lc_vehicle_positions();

	if (verbose)
	{
		std::string message = "Next to move: ";
		for (int i : next_to_move) message += std::to_string(i) + ", ";
		message += "my pos. = " + std::to_string(veh_position);
		std::clog << message << "\n";
	}

	/* TODO: change where this check happens.The current lc vehicles should
	call a new method when they finish their maneuvers. In this new
	method we advance the idx */
	// Check if next_in_line has finished its lane change
	bool all_are_done = true;
	for (int i : next_to_move)
	{
		const PlatoonVehicle* vehicle = 
			platoon->get_a_vehicle_by_position(i);
		if (!vehicle->get_has_completed_lane_change())
		{
			all_are_done = false;
			break;
		}
	}
	if (all_are_done)
	{
		if (get_current_coop_vehicle_position() == -1)
		{
			/* If the vehicle completed a maneuver behind all others (no
			coop), it is now the last vehicle */
			last_dest_lane_vehicle_pos =
				get_rearmost_lane_changing_vehicle_position();
		}
		maneuver_step++;
	}


	if (is_vehicle_turn_to_lane_change(veh_position))
	{
		for (int i : next_to_move)
		{
			const PlatoonVehicle* vehicle = 
				platoon->get_a_vehicle_by_position(i);

			if (verbose)
			{

			}

			if ((vehicle->get_virtual_leader_id()
				!= vehicle->get_destination_lane_leader_id()))
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

void PlatoonLaneChangeApproach::set_maneuver_initial_state(int ego_position, StateVector lo_states,
	std::vector<StateVector> platoon_states, StateVector ld_states,
	StateVector fd_states) 
{

}  

void PlatoonLaneChangeApproach::set_empty_maneuver_initial_state(
	int ego_position) 
{

}

void PlatoonLaneChangeApproach::set_platoon_lane_change_order(
	LCOrder lc_order, CoopOrder coop_order)
{
	maneuver_step = 0;
	platoon_lane_change_order = PlatoonLaneChangeOrder(lc_order, coop_order);

	if (verbose) std::clog << "Order set to: "
		<< platoon_lane_change_order << "\n";

	last_dest_lane_vehicle_pos =
		get_rearmost_lane_changing_vehicle_position();
	is_initialized = true;
}

void PlatoonLaneChangeApproach::set_platoon_lane_change_order(
	PlatoonLaneChangeOrder plco)
{
	maneuver_step = 0;
	platoon_lane_change_order = plco;
	last_dest_lane_vehicle_pos =
		get_rearmost_lane_changing_vehicle_position();
	is_initialized = true;
}

bool PlatoonLaneChangeApproach::is_vehicle_turn_to_lane_change(
	int veh_position)
{
	std::unordered_set<int> current_movers =
		get_current_lc_vehicle_positions();
	return current_movers.find(veh_position) != current_movers.end();
}

int PlatoonLaneChangeApproach
::get_rearmost_lane_changing_vehicle_position() const
{
	if (verbose) std::clog << "Looking for rearmost lc veh.\n";

	double min_x = INFINITY;
	int rear_most_pos = 0;
	for (int pos : get_current_lc_vehicle_positions())
	{
		const PlatoonVehicle* vehicle = 
			platoon->get_a_vehicle_by_position(pos);
		if (vehicle->get_distance_traveled() < min_x)
		{
			min_x = vehicle->get_distance_traveled();
			rear_most_pos = pos;
		}
	}

	if (verbose) std::clog << "Found at pos." << rear_most_pos << "\n";
	return rear_most_pos;
}

/* ------------------------------------------------------------------------ */
/* Concrete Approaches ---------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void SynchoronousApproach::decide_lane_change_order()
{
	/*std::vector<int> l1(platoon->get_size());
	std::iota(l1.begin(), l1.end(), 0);*/
	std::unordered_set<int> lc_set_1;
	CoopOrder coop_order{ {-1} };
	for (int i = 0; i < platoon->get_size(); i++)
	{
		lc_set_1.insert(i);
	}
	LCOrder lc_order = { lc_set_1 };
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LeaderFirstApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	CoopOrder coop_order;
	for (int i = 0; i < platoon->get_size(); i++)
	{
		std::unordered_set<int> lc_set;
		lc_set.insert(i);
		lc_order.push_back(lc_set);
		coop_order.push_back(-1);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LastVehicleFirstApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	CoopOrder coop_order;
	int n = platoon->get_size();
	for (int i = 0; i < n; i++)
	{
		if (lc_order.empty()) coop_order.push_back(-1);
		else coop_order.push_back(*lc_order.back().begin());

		std::unordered_set<int> lc_set;
		lc_set.insert(n - i - 1);
		lc_order.push_back(lc_set);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LeaderFirstReverseApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	CoopOrder coop_order;
	int n = platoon->get_size();
	for (int i = 0; i < n; i++)
	{
		if (lc_order.empty()) coop_order.push_back(-1);
		else coop_order.push_back(*lc_order.back().begin());

		std::unordered_set<int> lc_set;
		lc_set.insert(i);
		lc_order.push_back(lc_set);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}