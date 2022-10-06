#pragma once

#include <memory>
#include <unordered_map>

class PlatoonVehicle;

class Platoon
{
public:
	Platoon() = default;
	/*Platoon(long id) : id{ id } {};*/
	Platoon(long id, std::shared_ptr<PlatoonVehicle> leader);
	~Platoon();

	int get_id() const { return id; };
	std::shared_ptr<PlatoonVehicle> get_platoon_leader() const;
	long get_leader_id() const;
	long get_last_veh_id() const;
	size_t get_size() const { return vehicles.size(); };
	
	void set_id(long platoon_id);

	bool is_empty() const;
	void add_leader(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void add_last_vehicle(std::shared_ptr<PlatoonVehicle> new_vehicle);
	void remove_leader();
	void remove_last_vehicle();
	void remove_vehicle_by_position(int idx_in_platoon, long veh_id);
	void remove_vehicle_by_id(long veh_id);
	/* Vehicles till platoon_position stay in this platoon and function 
	returns the platoon behind */
	//Platoon split_platoon(int platoon_position);
	/* The parameter indicates the vehicle position in the platoon.
	The leader has idx 1 and the last vehicle has idx N. Returns
	the platton formed by all the vehicles behing the position idx*/
	//Platoon remove_a_vehicle(int platoon_position_idx);

	/* Returns true if merge successful and this platoon is now
	empty */
	//bool merge_into_leading_platoon(Platoon& other_platoon);

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out,
		const Platoon& platoon);

private:
	long id{ 0 };
	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles;
	std::unordered_map<long, int> vehicle_id_to_position;

	int leader_idx{ -1 };
	int last_veh_idx{ 0 };
	//Platoon create_platoon_from_vehicles(
	//	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles,
	//	int new_leader_idx, int new_last_vehicle_idx);
};

