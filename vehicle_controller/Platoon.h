#pragma once

#include <memory>
#include <unordered_map>

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
	std::shared_ptr<PlatoonVehicle> get_platoon_leader() const;
	long get_leader_id() const;
	long get_last_veh_id() const;
	long get_preceding_vehicle_id(long veh_id) const;
	size_t get_size() const { return vehicles.size(); };
	LaneChangeStrategy get_lane_change_strategy() const {
		return lane_change_strategy;
	};

	void set_verbose(bool verbose) { this->verbose = verbose; };

	bool is_empty() const;
	void add_leader(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void add_last_vehicle(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void remove_vehicle_by_id(long veh_id);

	void set_vehicle_lane_change_gaps_safe(long veh_id, bool is_safe);
	bool can_vehicle_start_lane_change(long veh_id);
	bool can_vehicle_leave_platoon(long veh_id);


	/* Returns true if merge successful and this platoon is now
	empty */
	//bool merge_into_leading_platoon(Platoon& other_platoon);

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out,
		const Platoon& platoon);

private:
	long id{ 0 };
	int leader_idx{ -1 };
	int last_veh_idx{ 0 };
	bool verbose{ false };
	LaneChangeStrategy lane_change_strategy{ 
		LaneChangeStrategy::synchronous };
	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles;
	std::unordered_map<long, int> vehicle_id_to_position;
	std::unordered_map<long, bool> vehicles_lane_change_gap_status;

	void remove_vehicle_by_position(int idx_in_platoon, long veh_id);
	void add_vehicle(int idx_in_platoon, 
		std::shared_ptr <PlatoonVehicle> new_vehicle);

	bool can_start_synchronous_lane_change();
};

