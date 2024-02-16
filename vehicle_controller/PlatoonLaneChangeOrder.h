#pragma once

#include <string>
#include <unordered_set>
#include <vector>

// TODO [jan 16, 2024] put together with PlatoonLCStrategyManager code?

using LCOrder = std::vector<std::unordered_set<int>>;
//using CoopOrder = std::vector<int>;

struct PlatoonLaneChangeOrder
{
	LCOrder lc_order{};
	std::vector<int> coop_order{};
	float cost{ -1.0 };
	PlatoonLaneChangeOrder() = default;
	PlatoonLaneChangeOrder(LCOrder lc_order, std::vector<int> coop_order);
	PlatoonLaneChangeOrder(std::vector<std::vector<int>> lc_order,
		std::vector<int> coop_order, float cost);
	int number_of_steps() { return static_cast<int>(coop_order.size()); };

	std::string to_string() const;
};

std::ostream& operator<<(std::ostream& out,
	PlatoonLaneChangeOrder const& plcs);