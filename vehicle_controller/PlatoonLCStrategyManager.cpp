#include <fstream>
#include <string>
#include "json.hpp"

#include "Constants.h"
#include "PlatoonLCStrategyManager.h"

using json = nlohmann::json;

PlatoonLCStrategyManager::PlatoonLCStrategyManager(std::string cost_name,
	bool verbose) 
	: cost_name(cost_name), verbose(verbose)
{}

void PlatoonLCStrategyManager::initialize(int n_platoon)
{
	if (verbose) std::clog << "[PlatoonLCStrategyManager] initializing...";
	
	this->n_platoon = n_platoon;
	load_a_strategy_map(n_platoon, cost_name);
	//load_quantizer_data(n_platoon);

	if (verbose) std::clog << " done.\n";
}

void PlatoonLCStrategyManager::set_maneuver_initial_state(
	int ego_position, std::vector<ContinuousStateVector> system_state_matrix)
{	
	std::vector<QuantizedStateVector> quantized_state_matrix = 
		state_quantizer.quantize_states(system_state_matrix);

	std::vector<int> quantized_state_vector = flatten_state_matrix<int>(
		quantized_state_matrix);

	if (verbose) std::clog << "\tqx="
		<< vector_to_string(quantized_state_vector) << "\n";

	if (strategy_map.find(quantized_state_vector) == strategy_map.end())
	{
		throw StateNotFoundException(quantized_state_vector);
	}
	initial_state_per_vehicle[ego_position] = quantized_state_vector;
}

void PlatoonLCStrategyManager::set_empty_maneuver_initial_state(
	int ego_position)
{
	initial_state_per_vehicle[ego_position] = empty_state;
}

PlatoonLaneChangeOrder PlatoonLCStrategyManager
::find_minimum_cost_order_given_first_mover(
	std::set<int>& first_mover_positions) const
{
	std::vector<int> initial_state;
	for (int p : first_mover_positions)
	{
		if (initial_state.size() == 0) 
		{
			initial_state = initial_state_per_vehicle.at(p);
		}
		else if (initial_state != initial_state_per_vehicle.at(p))
		{
			std::clog << " ==== ERROR: ====\n"
				<< "[PlatoonLCStrategyManager] "
				<< "More than one possible initial state "
				<< "for a given set of initial movers\n";
		}
	}
	return strategy_map.at(initial_state).at(first_mover_positions);
}

void PlatoonLCStrategyManager::load_a_strategy_map(
	int n_platoon, std::string cost_name)
{
	std::string file_name = "min_" + cost_name + "_strategies_for_"
		+ std::to_string(n_platoon) + "_vehicles.json";
	std::ifstream new_file(STRATEGY_MAPS_FOLDER + file_name);
	if (new_file)
	{
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
		strategy_map = a_strategy_map;
	}
	//strategy_map_per_size[n_platoon] = a_strategy_map;
}

std::unordered_map<std::string, double> PlatoonLCStrategyManager
::load_quantizer_data(int n_platoon)
{
	std::string file_name = std::to_string(n_platoon) + "_vehicles.json";
	std::ifstream new_file(QUANTIZATION_PARAMS_FOLDER + file_name);
	json json_data = json::parse(new_file);
	std::unordered_map<std::string, double> params;

	for (auto& datum : json_data.items())
	{
		params[datum.key()] = datum.value();
	}
	return params;
}

template<typename T>
std::vector<T> PlatoonLCStrategyManager::flatten_state_matrix(
	std::vector<StateVector<T>>& state_matrix)
{
	std::vector<T> system_state_vector;
	//int n_vehs = static_cast<int>(state_matrix.size());
	//int n_states = state_matrix.front().get().size();
	//system_state_vector.reserve(n_states * n_vehs);
	for (StateVector<T>& v : state_matrix)
	{
		for (auto i : v.get())
		{
			system_state_vector.push_back(i);
		}
		//system_state_vector.insert(system_state_vector.end(),
		//	v.get().begin(), v.get().end());
	}
	return system_state_vector;
}

std::ostream& operator<<(std::ostream& out,
	PlatoonLCStrategyManager const& strategy_manager)
{
	for (const std::pair<OuterKey, InnerMap>& item 
		: strategy_manager.strategy_map)
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
	}
		
	//for (const std::pair<int, OuterMap>& outer_item 
	//	: strategy_manager.strategy_map_per_size)
	//{
	//	out << "=== Strategies for N=" << outer_item.first << "===\n";
	//	for (const std::pair<const OuterKey, InnerMap>& item
	//		: outer_item.second)
	//	{
	//		std::string text = "root: [";
	//		for (int i : item.first)
	//		{
	//			text += std::to_string(i) + ", ";
	//		}
	//		text.erase(text.size() - 2);
	//		text += "]:\n";
	//		for (const std::pair<InnerKey, PlatoonLaneChangeOrder>
	//			inner_item : item.second)
	//		{
	//			text += "\tfirst mover: [";
	//			for (int i : inner_item.first)
	//			{
	//				text += std::to_string(i) + ", ";
	//			}
	//			text.erase(text.size() - 2);
	//			text += "]: ";
	//			text += inner_item.second.to_string() + "\n";
	//		}
	//		out << text;
	//	}
	//}
	return out;
}