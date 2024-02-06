#pragma once

#include <iostream>
#include <set>
#include <unordered_map>
#include <vector>

#include "PlatoonLaneChangeOrder.h"
#include "StateQuantizer.h"
#include "StateVector.h"

/* Hash function taken from:
https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933
and
https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key?rq=3
*/
template <typename Container>
struct container_hash {
    std::size_t operator()(Container const& container) const {
        std::size_t seed = container.size();
        for (auto x : container) {
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = ((x >> 16) ^ x) * 0x45d9f3b;
            x = (x >> 16) ^ x;
            seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

using InnerKey = std::set<int>; /* unordered sets could lead to 
 different hash values */
using OuterKey = std::vector<int>;
using InnerMap = std::unordered_map<InnerKey,
    PlatoonLaneChangeOrder, container_hash<InnerKey>>;
using OuterMap = std::unordered_map<OuterKey, InnerMap,
    container_hash<OuterKey>>;

class PlatoonLCStrategyManager
{
public:
    PlatoonLCStrategyManager() = default;
    PlatoonLCStrategyManager(std::string cost_name);

    void initialize(int n_platoon);

    void set_maneuver_initial_state(int ego_position, 
        std::vector<ContinuousStateVector> system_state_matrix);
    void set_empty_maneuver_initial_state(int ego_position);
    PlatoonLaneChangeOrder find_minimum_cost_order_given_first_mover(
        std::set<int>& first_mover_positions);

    friend std::ostream& operator<<(std::ostream& out,
        PlatoonLCStrategyManager const& strategy_manager);

private:
    std::string cost_name;
    //std::unordered_map<int, OuterMap> strategy_map_per_size{};
    OuterMap strategy_map{};
    StateQuantizer state_quantizer{ StateQuantizer() };
    std::unordered_map<int, std::vector<int>> initial_state_per_vehicle{};
    std::vector<int> empty_state{ 0 };

    void load_a_strategy_map(int n_platoon, std::string cost_name);
    std::unordered_map<std::string, double> load_quantizer_data(
        int n_platoon);

    /* Transforms a vector of vectors into a single long vector */
    template<typename T>
    std::vector<T> flatten_state_matrix(
        std::vector<StateVector<T>>& state_matrix);
};

