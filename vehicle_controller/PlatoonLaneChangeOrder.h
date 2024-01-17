#pragma once

#include <unordered_set>
#include <vector>

// TODO [jan 16, 2024] put together with PlatoonLCStrategyManager code?

using LCOrder = std::vector<std::unordered_set<int>>;
using CoopOrder = std::vector<int>;

struct PlatoonLaneChangeOrder
{
	LCOrder lc_order{};
	CoopOrder coop_order{};
	float cost{ 0.0 };
	PlatoonLaneChangeOrder() = default;
	PlatoonLaneChangeOrder(std::vector<std::vector<int>>lc_order,
		std::vector<int> coop_order, float cost);
	long number_of_steps() { return coop_order.size(); };

	std::string to_string() const;
};

std::ostream& operator<<(std::ostream& out,
	PlatoonLaneChangeOrder const& plcs);