#pragma once

#include <iostream>
#include <unordered_set>
#include <vector>

#include "PlatoonVehicle.h"

using LCOrder = std::vector<std::unordered_set<int>>;
using CoopOrder = std::vector<int>;

class PlatoonLaneChangeOrder
{
public:
	PlatoonLaneChangeOrder() = default;
	PlatoonLaneChangeOrder(std::vector<std::vector<int>>lc_order,
		std::vector<int> coop_order, float cost);

	const LCOrder get_lc_order() const { return lc_order; };
	const CoopOrder get_coop_order() const { return coop_order; };
	float get_cost() const { return cost; };
	std::string to_string() const;

	bool can_vehicle_start_lane_change(int veh_position, 
		std::unordered_map<int, PlatoonVehicle*> platoon_vehicles);

	friend std::ostream& operator<<(std::ostream& out,
		PlatoonLaneChangeOrder const& plcs);

private:
	LCOrder lc_order{};
	CoopOrder coop_order{};
	float cost{ 0.0 };
	int lane_change_step{ 0 };
	long last_dest_lane_vehicle_pos{ 0 };

	 int get_rearmost_lane_changing_vehicle_position(
			std::unordered_map<int, PlatoonVehicle*> platoon_vehicles) const;
};
