#include <string>
#include "PlatoonLaneChangeOrder.h"

PlatoonLaneChangeOrder::PlatoonLaneChangeOrder(
	std::vector<std::vector<int>>lc_order, 
	std::vector<int> coop_order, float cost)
{
	for (auto& inner_vector : lc_order)
	{
		std::unordered_set<int> s(inner_vector.begin(),
			inner_vector.end());
		this->lc_order.push_back(s);
	}
	this->coop_order = coop_order;
	this->cost = cost;
	for (const auto& id_set : lc_order)
	{
		this->last_dest_lane_vehicle_pos += id_set.size();
	}
}

bool PlatoonLaneChangeOrder::can_vehicle_start_lane_change(int veh_position,
	std::unordered_map<int, PlatoonVehicle*> platoon_vehicles)
{
	if (lane_change_step >= lc_order.size())
	{
		std::clog << "[WARNING] maneuver step counter above "
			"total maneuver steps\n";
		return false;
	}

	std::unordered_set<int> next_to_move = lc_order[lane_change_step];
	
	/* TODO: change where this check happens.The current lc vehicles should
	call a new method when they finish their maneuvers.In this new
	method we advance the idx */
	// Check if next_in_line has finished its lane change
	bool all_are_done = true;
	for (int i : next_to_move)
	{
		PlatoonVehicle* vehicle = platoon_vehicles[i];
		if (vehicle->has_lane_change_intention()) 
		{
			all_are_done = false;
			break;
		}
	}
	if (all_are_done)
	{
		if (coop_order[lane_change_step] == -1)
		{
			/* If the vehicle completed a maneuver behind all others (no
			coop), it is now the last vehicle */
			last_dest_lane_vehicle_pos =
				get_rearmost_lane_changing_vehicle_position(platoon_vehicles);
		}
		lane_change_step++;
	}

	bool is_vehicles_turn =
		next_to_move.find(veh_position) != next_to_move.end();
	if (is_vehicles_turn)
	{
		for (int i : next_to_move)
		{
			PlatoonVehicle* vehicle = platoon_vehicles[i];
			if ((vehicle->get_virtual_leader_id()
				!= vehicle->get_destination_lane_leader_id())
				|| !vehicle->are_all_lane_change_gaps_safe())
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

int PlatoonLaneChangeOrder::get_rearmost_lane_changing_vehicle_position(
	std::unordered_map<int, PlatoonVehicle*> platoon_vehicles) const
{
	double min_x = INFINITY;
	int rear_most_pos = 0;
	for (int pos : lc_order[lane_change_step])
	{
		PlatoonVehicle* vehicle = platoon_vehicles[pos];
		if (vehicle->get_distance_traveled() < min_x)
		{
			min_x = vehicle->get_distance_traveled();
			rear_most_pos = pos;
		}
	}
	return rear_most_pos;
}

std::string PlatoonLaneChangeOrder::to_string() const
{
	std::string text = "lc order : [";
	for (std::unordered_set<int> veh_set : lc_order)
	{
		text += "[";
		for (int veh_id : veh_set)
		{
			text += std::to_string(veh_id) + ", ";
		}
		text.erase(text.size() - 2);
		text += "], ";
	}
	text.erase(text.size() - 2);
	text += "]; coop order: [";
	for (int veh_id : coop_order)
	{
		text += std::to_string(veh_id) + ", ";
	}
	text.erase(text.size() - 2);
	text += "]; cost=" + std::to_string(cost);
	return text;
}

std::ostream& operator<<(std::ostream& out,
	PlatoonLaneChangeOrder const& plcs)
{
	out << plcs.to_string();
	return out;
}