#pragma once

#include <unordered_map>
#include <vector>

#include "PlatoonLaneChangeOrder.h"
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
	const PlatoonLaneChangeOrder* get_platoon_lane_change_order() {
		return &platoon_lane_change_order;
	};
	void set_platoon(Platoon* platoon) { this->platoon = platoon; };

	// [Jan 16, 2024] Let's see if this is needed to read the strategies map
	//void set_strategies_map();	

	long get_desired_destination_lane_leader_id(int ego_position);
	long get_assisted_vehicle_id(int ego_position);
	// TODO [Jan 17, 2024] waiting to see which version is necessary
	void set_platoon_lane_change_order(LCOrder lc_order, CoopOrder coop_order);
	void set_platoon_lane_change_order(PlatoonLaneChangeOrder plco);

	bool can_vehicle_start_lane_change(int veh_position);

	// [Jan 16, 2024] These two methods only necessary with the graph approach
	void set_maneuver_initial_state(int ego_position, StateVector lo_states,
		std::vector<StateVector> platoon_states, StateVector ld_states,
		StateVector fd_states) {};  // TODO [major]
	void set_empty_maneuver_initial_state(int ego_position) {}; // TODO [major]

protected:
	const Platoon* platoon{ nullptr };

	PlatoonLaneChangeApproach(int id, std::string name);

private:
	int id;
	std::string name;
	double decision_time{ 0.0 };
	bool is_initialized{ false };
	PlatoonLaneChangeOrder platoon_lane_change_order;
	int maneuver_step{ 0 };
	int last_dest_lane_vehicle_pos{ -1 };

	virtual void decide_lane_change_order() = 0;
	std::unordered_set<int> get_current_lc_vehicle_positions() const { 
		return platoon_lane_change_order.lc_order[maneuver_step]; };
	int get_current_coop_vehicle_position() const { 
		return platoon_lane_change_order.coop_order[maneuver_step]; };
	bool is_vehicle_turn_to_lane_change(int ego_position);
	int get_rearmost_lane_changing_vehicle_position() const;
};

class SynchoronousApproach: public PlatoonLaneChangeApproach
{
public:
	SynchoronousApproach() :
		PlatoonLaneChangeApproach(0, "synchronous") {};

private:
	void decide_lane_change_order() override;
};

class LeaderFirstApproach : public PlatoonLaneChangeApproach
{
public:
	LeaderFirstApproach() :
		PlatoonLaneChangeApproach(1, "leader first") {};
private:
	void decide_lane_change_order() override;
};

class LastVehicleFirstApproach : public PlatoonLaneChangeApproach
{
public:
	LastVehicleFirstApproach() :
		PlatoonLaneChangeApproach(2, "last first") {};
private:
	void decide_lane_change_order() override;
};

class LeaderFirstReverseApproach : public PlatoonLaneChangeApproach
{
public:
	LeaderFirstReverseApproach() :
		PlatoonLaneChangeApproach(3, "leader first reverse") {};
private:
	void decide_lane_change_order() override;
};