#pragma once

#include <memory>
#include <unordered_map>

#include "PlatoonLaneChangeStrategy.h"

class PlatoonVehicle;

class Platoon
{
public:

	enum class LaneChangeStrategy
	{
		none,
		synchronous,
		leader_first,
		last_vehicle_first,
		leader_first_and_invert
	};

	Platoon() = default;
	/*Platoon(long id) : id{ id } {};*/
	Platoon(long id, std::shared_ptr<PlatoonVehicle> leader) :
		Platoon(id, leader, false) {};
	Platoon(long id, std::shared_ptr<PlatoonVehicle> leader,
		bool verbose);
	~Platoon();

	int get_id() const { return id; };
	size_t get_size() const { return vehicles.size(); };
	/*LaneChangeStrategy get_lane_change_strategy() const {
		return lane_change_strategy;
	};*/

	std::shared_ptr<PlatoonVehicle> get_platoon_leader() const;
	long get_leader_id() const;
	long get_last_veh_id() const;
	long get_preceding_vehicle_id(long veh_id) const;
	PlatoonLaneChangeStrategy::LaneChangeState get_lane_change_state(
		long veh_id) const;
	void set_verbose(bool verbose) { this->verbose = verbose; };
	void set_strategy(int strategy_number);

	bool is_empty() const;
	void add_leader(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void add_last_vehicle(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void remove_vehicle_by_id(long veh_id);

	void set_vehicle_lane_change_state(PlatoonVehicle& platoon_vehicle, 
		bool should_change_lane);
	void set_vehicle_lane_change_gap_status(long veh_id, bool is_safe);
	bool can_vehicle_start_lane_change(const PlatoonVehicle& platoon_vehicle);
	bool can_vehicle_leave_platoon(long veh_id) const;
	bool can_vehicle_start_longitudinal_adjustment(long veh_id) const;
	/* Returns true if merge successful and this platoon is now
	empty */
	//bool merge_into_leading_platoon(Platoon& other_platoon);

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out,
		const Platoon& platoon);

private:
	enum Strategy
	{
		no_strategy,
		synchronous_strategy,
		leader_first_strategy,
		last_vehicle_first_strategy,
		leader_first_invert_strategy
	};

	long id{ 0 };
	int leader_idx{ -1 };
	int last_veh_idx{ 0 };
	bool verbose{ false };
	std::unique_ptr<PlatoonLaneChangeStrategy> lane_change_strategy{ 
		nullptr };
	/* Keys are the vehicle position in the platoon */
	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles;
	std::unordered_map<long, int> vehicle_id_to_position;
	/* Keys are the vehicle id */
	std::unordered_map<long, PlatoonLaneChangeStrategy::LaneChangeState>
		vehicles_lane_change_states;
	/* Keys are the vehicle id */
	// TODO [Nov. 16]: should be removed after the lc states are done
	std::unordered_map<long, bool> vehicles_lane_change_gap_status;
	/*std::unordered_map<LaneChangeStrategy,
		std::unique_ptr<PlatoonLaneChangeStrategy>> lane_change_strategies;*/

	std::shared_ptr<PlatoonVehicle> get_vehicle_by_id(long veh_id) const;
	void remove_vehicle_by_position(int idx_in_platoon, long veh_id);
	void add_vehicle(int idx_in_platoon, 
		std::shared_ptr <PlatoonVehicle> new_vehicle);
	std::shared_ptr<PlatoonVehicle> get_preceding_vehicle(long veh_id) const;

	bool has_a_vehicle_cut_in_the_platoon(
		const PlatoonVehicle& platoon_vehicle) const;

};

