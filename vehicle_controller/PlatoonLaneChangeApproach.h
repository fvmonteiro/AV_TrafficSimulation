#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "PlatoonLaneChangeOrder.h"
#include "PlatoonLCStrategyManager.h"
#include "StateVector.h"

class Platoon;
class PlatoonVehicle;

class PlatoonLaneChangeApproach
{
public:
	PlatoonLaneChangeApproach() = default;
	
	int get_id() const { return id; };
	std::string get_name() const { return name; };
	double get_decision_time() const { return decision_time; };
	const PlatoonLaneChangeOrder* get_platoon_lane_change_order() const {
		return &platoon_lane_change_order;
	};
	void set_platoon(Platoon* platoon) { this->platoon = platoon; };

	// [Jan 16, 2024] Let's see if this is needed to read the strategies map
	//void set_strategies_map();	

	long get_desired_destination_lane_leader(int ego_position) ;
	long get_assisted_vehicle_id(int ego_position) const;
	long get_cooperating_vehicle_id() const;
	/* Returns the non-platoon vehicle behind which the entire platoon
	is trying to move. */
	long get_platoon_destination_lane_leader_id() const;

	bool can_vehicle_start_lane_change(int ego_position);
	/* Placeholder. For now always returns false */
	bool can_vehicle_leave_platoon(int ego_position) const;
	/* Placeholder. For now always returns 0*/
	long create_platoon_lane_change_request(int ego_position) const;

protected:
	bool verbose{ false };
	const Platoon* platoon{ nullptr };

	PlatoonLaneChangeApproach(int id, std::string name, bool verbose);

	void set_platoon_lane_change_order(LCOrder lc_order, CoopOrder coop_order);
	void set_platoon_lane_change_order(PlatoonLaneChangeOrder plco);

private:
	int id;
	std::string name;
	double decision_time{ 0.0 };
	bool is_initialized{ false };
	PlatoonLaneChangeOrder platoon_lane_change_order;
	int maneuver_step{ 0 };
	int last_dest_lane_vehicle_pos{ -1 };
	/* Stores the non-platoon vehicle behind which the entire platoon
	is trying to move. */
	long platoon_destination_lane_leader_id{ 0 };

	virtual void decide_lane_change_order() = 0;
	virtual bool implement_can_vehicle_leave_platoon(int veh_position) const;

	std::unordered_set<int> get_current_lc_vehicle_positions() const { 
		return platoon_lane_change_order.lc_order[maneuver_step]; };
	int get_current_coop_vehicle_position() const { 
		return platoon_lane_change_order.coop_order[maneuver_step]; };
	
	bool is_vehicle_turn_to_lane_change(int ego_position) const;
	void check_maneuver_step_done(
		const std::unordered_set<int>& lane_changing_veh_ids);
	int get_rearmost_lane_changing_vehicle_position() const;
};

class SynchoronousApproach: public PlatoonLaneChangeApproach
{
public:
	SynchoronousApproach() 
		: SynchoronousApproach(false) {};
	SynchoronousApproach(bool verbose) 
		: PlatoonLaneChangeApproach(0, "synchronous", verbose) {};

private:
	void decide_lane_change_order() override;
};

class LeaderFirstApproach : public PlatoonLaneChangeApproach
{
public:
	LeaderFirstApproach()
		: LeaderFirstApproach(false) {} ;
	LeaderFirstApproach(bool verbose) 
		: PlatoonLaneChangeApproach(1, "leader first", verbose) {};
private:
	void decide_lane_change_order() override;
};

class LastVehicleFirstApproach : public PlatoonLaneChangeApproach
{
public:
	LastVehicleFirstApproach()
		: LastVehicleFirstApproach(false) {};
	LastVehicleFirstApproach(bool verbose) 
		: PlatoonLaneChangeApproach(2, "last first", verbose) {};
private:
	void decide_lane_change_order() override;
};

class LeaderFirstReverseApproach : public PlatoonLaneChangeApproach
{
public:
	LeaderFirstReverseApproach()
		: LeaderFirstReverseApproach(false) {};
	LeaderFirstReverseApproach(bool verbose)
		: PlatoonLaneChangeApproach(3, "leader first reverse", verbose) {};
private:
	void decide_lane_change_order() override;
};

class GraphApproach : public PlatoonLaneChangeApproach
{
public:
	GraphApproach(): GraphApproach(false) {};
	GraphApproach(bool verbose)
		: PlatoonLaneChangeApproach(4, "graph", verbose) {};

private:
	bool is_data_loaded{ false };
	PlatoonLCStrategyManager strategy_manager;

	void decide_lane_change_order() override;
	void set_maneuver_initial_state_for_all_vehicles();
	PlatoonLaneChangeOrder find_best_order_in_map();
	void save_not_found_state_to_file(std::vector<int> state_vector,
		double free_flow_speed_dest);
};