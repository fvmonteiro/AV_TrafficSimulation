#pragma once

#include <memory>
#include <unordered_map>

#include "PlatoonLaneChangeStrategy.h"

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

	int get_id() const { return id; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_velocity_at_lane_change_start() const {
		return velocity_at_lane_change_start; };
	size_t get_size() const { return vehicles_by_position.size(); };
	const std::unordered_map<int, PlatoonVehicle*>&
		get_vehicles_by_position() const { return vehicles_by_position; };

	long get_leader_id() const;
	long get_last_veh_id() const;
	long get_preceding_vehicle_id(long veh_id) const;
	const PlatoonVehicle* get_preceding_vehicle(
		long veh_id) const;
	const PlatoonVehicle* get_following_vehicle(
		long veh_id) const;
	long get_assisted_vehicle_id(long veh_id) const;
	const PlatoonVehicle* get_platoon_leader() const;
	const PlatoonVehicle* get_last_vehicle() const;
	const PlatoonVehicle* get_vehicle_by_id(long veh_id) const;

	void set_verbose(bool verbose) { this->verbose = verbose; };
	void set_velocity_at_lane_change_start(double velocity) {
		velocity_at_lane_change_start = velocity;
	};
	void set_strategy(int strategy_number);

	bool is_empty() const;
	void add_leader(PlatoonVehicle* new_vehicle);
	void add_last_vehicle(PlatoonVehicle* new_vehicle);
	void remove_vehicle_by_id(long veh_id, bool is_out_of_simulation);
	bool can_vehicle_start_adjustment_to_dest_lane_leader(long veh_id) const;
	//bool check_all_vehicles_lane_change_gaps();
	bool can_vehicle_leave_platoon(
		const PlatoonVehicle& platoon_vehicle) const;
	void reorder_vehicles();
	long create_lane_change_request_for_vehicle(long veh_id) const;
	/* Checks whether a non platoon vehicle inserted itself 
	in the platoon. Does not checks during lane changes */
	/*bool has_a_vehicle_cut_in_the_platoon(
		const PlatoonVehicle& platoon_vehicle) const;*/
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
	double desired_velocity{ 0.0 };
	double velocity_at_lane_change_start{ 0.0 };
	bool verbose{ false };
	std::unique_ptr<PlatoonLaneChangeStrategy> lane_change_strategy{ 
		//nullptr };
		std::make_unique<NoStrategy>() };
	/* Keys are the vehicle position in the platoon */
	std::unordered_map<int, PlatoonVehicle*> 
		vehicles_by_position;
	std::unordered_map<long, int> vehicle_id_to_position;

	void remove_vehicle_by_position(int idx_in_platoon, long veh_id,
		bool is_out_of_simulation);
	void add_vehicle(int idx_in_platoon, 
		PlatoonVehicle* new_vehicle);
};

