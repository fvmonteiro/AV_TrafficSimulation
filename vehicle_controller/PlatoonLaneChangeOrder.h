#pragma once

#include <iostream>
#include <unordered_set>
#include <vector>

using LCOrder = std::vector<std::unordered_set<int>>;

class PlatoonLaneChangeOrder
{
public:
	PlatoonLaneChangeOrder() = default;
	PlatoonLaneChangeOrder(std::vector<std::vector<int>>lc_order,
		std::vector<int> coop_order, float cost);

	std::string to_string() const;

	friend std::ostream& operator<<(std::ostream& out,
		PlatoonLaneChangeOrder const& plcs);

private:
	LCOrder lc_order{};
	std::vector<int> coop_order{};
	float cost{ 0.0 };
};
