#pragma once

#include <memory>
#include <unordered_map>

//#include "PlatoonLaneChangeStrategy.h"
#include "PlatoonLaneChangeApproach.h"
//#include "StateVector.h"

class NearbyVehicle;
class PlatoonVehicle;

class Platoon
{
public:

	Platoon() = default;
	/*Platoon(long id) : id{ id } {};*/
	Platoon(long id, int platoon_lc_strategy, PlatoonVehicle* leader) :
		Platoon(id, platoon_lc_strategy, leader, false) {};
	Platoon(long id, int platoon_lc_strategy, PlatoonVehicle* leader,
		bool verbose);
	~Platoon();

	/* Simple getters and setters */

	int get_id() const { return id; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_velocity_at_lane_change_start() const {
		return velocity_at_lane_change_start; };
	int get_size() const { 
		return static_cast<int>(vehicles_by_position.size()); };
	const std::unordered_map<int, PlatoonVehicle*>&
		get_vehicles_by_position() const { return vehicles_by_position; };
	void set_verbose(bool verbose) { this->verbose = verbose; };
	void set_velocity_at_lane_change_start(double velocity) {
		velocity_at_lane_change_start = velocity;
	};
	void set_lane_change_start_time(double time) {
		lane_change_start_time = 
			lane_change_start_time < time ? lane_change_start_time : time;
	};

	/* Other getters and setters */

	long get_leader_id() const;
	long get_last_veh_id() const;
	long get_preceding_vehicle_id(long veh_id) const;
	long get_following_vehicle_id(long veh_id) const;
	/* Returns a nullptr if vehicle not found in platoon */
	PlatoonVehicle* get_a_vehicle_by_position(int veh_pos) const;
	/* Returns a nullptr if vehicle not found in platoon */
	const PlatoonVehicle* get_vehicle_by_id(long veh_id) const;
	const PlatoonVehicle* get_preceding_vehicle(
		long veh_id) const;
	const PlatoonVehicle* get_following_vehicle(
		long veh_id) const;
	const PlatoonVehicle* get_platoon_leader() const;
	const PlatoonVehicle* get_last_vehicle() const;
	long get_destination_lane_vehicle_behind_the_leader() const;
	long get_destination_lane_vehicle_behind_last_vehicle() const;
	/* The vehicle behind which the platoon wants to move */
	const NearbyVehicle* get_destination_lane_leader() const;
	long get_assisted_vehicle_id(long ego_id) const;
	long get_cooperating_vehicle_id() const;
	std::vector<ContinuousStateVector> get_vehicles_states() const;

	void set_strategy(int strategy_number);

	/* Remaining public methods */

	bool is_empty() const;
	bool is_vehicle_id_in_platoon(long veh_id) const;
	void add_last_vehicle(PlatoonVehicle* new_vehicle);
	void remove_vehicle_by_id(long veh_id, bool is_out_of_simulation);
	bool has_lane_change_intention() const;
	void receive_lane_change_intention_signal();
	void receive_lane_keeping_signal();
	/* Returns true if the platoon is stopped right before the mandatory
	lane change point, i.e., stuck waiting to start lane change.*/
	bool is_stuck() const;
	/* True if at least one vehicle started a lane change */
	bool has_lane_change_started() const;
	bool can_vehicle_leave_platoon(long ego_id) const;
	bool can_vehicle_start_lane_change(long ego_id) const;
	void sort_vehicles_by_distance_traveled();
	long create_lane_change_request_for_vehicle(long ego_id) const;
	long define_desired_destination_lane_leader_id(long ego_id) const;

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
		leader_first_invert_strategy,
		graph_strategy,
	};

	long id{ 0 };
	bool verbose{ false };
	
	double desired_velocity{ 0.0 };
	double velocity_at_lane_change_start{ 0.0 };
	double lane_change_start_time{ INFINITY };
	int lane_change_intention_counter{ 0 };
	
	/* TODO [Jan 24, 24] these maps should be vectors */
	/* Keys are the vehicle position in the platoon */
	std::unordered_map<int, PlatoonVehicle*>
		vehicles_by_position;
	std::unordered_map<long, int> vehicle_id_to_position;
	//std::unique_ptr<PlatoonLaneChangeStrategy> lane_change_strategy{ 
	//	std::make_unique<NoStrategy>() };
	std::unique_ptr<PlatoonLaneChangeApproach> lane_change_approach{ 
		nullptr };

	void add_leader(PlatoonVehicle* new_vehicle);
	void add_vehicle(int idx_in_platoon, PlatoonVehicle* new_vehicle);
	void remove_vehicle_by_position(int idx_in_platoon, long veh_id,
		bool is_out_of_simulation);
	void update_position_maps();
};
