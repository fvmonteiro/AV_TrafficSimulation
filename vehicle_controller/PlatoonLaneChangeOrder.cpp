#include <iostream>
#include <string>

#include "Constants.h"
#include "PlatoonLaneChangeOrder.h"

PlatoonLaneChangeOrder::PlatoonLaneChangeOrder(LCOrder lc_order,
	CoopOrder coop_order) : lc_order(lc_order), coop_order(coop_order) {}

PlatoonLaneChangeOrder::PlatoonLaneChangeOrder(
	std::vector<std::vector<int>> lc_order, CoopOrder coop_order,
	float cost) : coop_order(coop_order), cost(cost)
{
	for (auto& inner_vector : lc_order)
	{
		std::unordered_set<int> s(inner_vector.begin(),
			inner_vector.end());
		this->lc_order.push_back(s);
	}
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
	text += "]; coop order: " + vector_to_string(coop_order) 
		+ "; cost=" + std::to_string(cost);
	/*text += "]; coop order: [";
	for (int veh_id : coop_order)
	{
		text += std::to_string(veh_id) + ", ";
	}
	text.erase(text.size() - 2);
	text += "]; cost=" + std::to_string(cost); */
	return text;
}

std::ostream& operator<<(std::ostream& out,
	PlatoonLaneChangeOrder const& plcs)
{
	out << plcs.to_string();
	return out;
}