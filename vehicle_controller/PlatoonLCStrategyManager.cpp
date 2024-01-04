#include <fstream>
#include <string>
#include "json.hpp"

#include "Constants.h"
#include "PlatoonLCStrategyManager.h"

using json = nlohmann::json;

PlatoonLCStrategyManager::PlatoonLCStrategyManager(int n_platoon,
	std::string cost_name)
{
	load_a_strategy_map(n_platoon, cost_name);
}

void PlatoonLCStrategyManager::load_a_strategy_map(
	int n_platoon, std::string cost_name)
{
	std::string file_name = "min_" + cost_name + "_strategies_for_"
		+ std::to_string(n_platoon) + "_vehicles.json";
	std::ifstream new_file(STRATEGY_MAPS_FOLDER + file_name);
	json json_data = json::parse(new_file);

	OuterMap a_strategy_map;
	for (auto& datum : json_data)
	{
		OuterKey root_node = datum["root"];
		InnerKey first_mover_set(datum["first_mover_set"].begin(),
			datum["first_mover_set"].end());
		PlatoonLaneChangeOrder plcs = PlatoonLaneChangeOrder(
			datum["lc_order"], datum["coop_order"], datum[cost_name]);
		if (a_strategy_map.find(root_node) == a_strategy_map.end())
		{
			a_strategy_map[root_node] = {};
		}
		a_strategy_map[root_node][first_mover_set] = plcs;
	}
	strategy_map_per_size[n_platoon] = a_strategy_map;
}

std::ostream& operator<<(std::ostream& out,
	PlatoonLCStrategyManager const& strategy_manager)
{
	for (const std::pair<int, OuterMap>& outer_item 
		: strategy_manager.strategy_map_per_size)
	{
		out << "=== Strategies for N=" << outer_item.first << "===\n";
		for (const std::pair<const OuterKey, InnerMap>& item
			: outer_item.second)
		{
			std::string text = "root: [";
			for (int i : item.first)
			{
				text += std::to_string(i) + ", ";
			}
			text.erase(text.size() - 2);
			text += "]:\n";
			for (const std::pair<InnerKey, PlatoonLaneChangeOrder>
				inner_item : item.second)
			{
				text += "\tfirst mover: [";
				for (int i : inner_item.first)
				{
					text += std::to_string(i) + ", ";
				}
				text.erase(text.size() - 2);
				text += "]: ";
				text += inner_item.second.to_string() + "\n";
			}
			out << text;
		}
	}
	return out;
}