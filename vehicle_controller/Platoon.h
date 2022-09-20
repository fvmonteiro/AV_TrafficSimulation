#pragma once

#include <memory>
#include <unordered_map>

class PlatoonVehicle;

class Platoon
{
public:
	Platoon() = default;
	Platoon(std::shared_ptr<PlatoonVehicle> vehicle);
	~Platoon();

	int get_leader_id();
	bool is_empty();
	void add_leader(const std::shared_ptr<PlatoonVehicle> new_vehicle);
	void add_last_vehicle(const std::shared_ptr<PlatoonVehicle> new_vehicle);
	void remove_leader();
	void remove_last_vehicle();
	/* Vehicles till platoon_position stay in this platoon and function 
	returns the platoon behind */
	Platoon split_platoon(int platoon_position);
	/* The parameter indicates the vehicle position in the platoon.
	The leader has idx 1 and the last vehicle has idx N. Returns
	the platton formed by all the vehicles behing the position idx*/
	Platoon remove_a_vehicle(int platoon_position_idx);

	void merge_into_leading_platoon(std::shared_ptr<Platoon> other_platoon);

private:
	int id{ 0 };  /* TODO: during tests we're setting it equal to the id  
				  of the vehicle that creates this platoon */
	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles;
	int leader_idx{ 0 };
	int last_veh_idx{ 0 };

	//Platoon create_platoon_from_vehicles(
	//	std::unordered_map<int, std::shared_ptr<PlatoonVehicle>> vehicles,
	//	int new_leader_idx, int new_last_vehicle_idx);
};

