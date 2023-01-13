#pragma once

#include "ConnectedAutonomousVehicle.h"

class PlatoonVehicle : public ConnectedAutonomousVehicle
{
public:

	PlatoonVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);
	~PlatoonVehicle();

	/* Returns true if vehicle is the platoon leader 
	or if it is not part of a platoon */
	bool is_platoon_leader() const;
	/* Returns true if vehicle is the last vehicle in 
	the platoon or if it is not part of a platoon */
	bool is_last_platoon_vehicle() const;

	std::shared_ptr<PlatoonVehicle> get_preceding_vehicle_in_platoon() const;
	std::shared_ptr<PlatoonVehicle> get_following_vehicle_in_platoon() const;

protected:

	void implement_analyze_nearby_vehicles() override;
	
	/* Returns true if the vehicle creates a platoon for itself. */
	bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		std::shared_ptr<EgoVehicle> pointer_to_me, 
		long new_platoon_id) override;
private:
	std::shared_ptr<Platoon> platoon{ nullptr };
	// desired velocity when not a part of the platoon
	double alone_desired_velocity{ 0.0 };
	/* Time the vehicle has been without a platoon.
	As soon as the vehicle enters simulation, it may create its own platoon */
	double alone_time{ 1.0 };
	double max_time_looking_for_platoon{ 1.0 };
	double lambda_0_platoon{ 0.0 };
	double lambda_1_platoon{ 0.0 };
	double lambda_1_lane_change_platoon{ 0.0 };

	/* Some constants for the platoon vehicles */
	double in_platoon_comf_accel{ 0.5 };
	double in_platoon_rho{ 0.05 };

	double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	/* SCENARIO SPECIFIC: the platoon vehicles always start at the in ramp, 
	and they try to change lanes as soon as they enter the highway */
	void set_desired_lane_change_direction() override;
	//bool implement_check_lane_change_gaps() override;
	std::shared_ptr<Platoon> implement_get_platoon() const override;
	void pass_this_to_state() override;

	// Check if vehicles in our platoon need gap generation
	void find_cooperation_request_from_platoon();

	void create_platoon(long platoon_id,
		std::shared_ptr<PlatoonVehicle> pointer_to_me);
	void add_myself_to_leader_platoon(
		std::shared_ptr<Platoon> leader_platoon,
		std::shared_ptr<PlatoonVehicle> pointer_to_me);

	void compute_platoon_safe_gap_parameters();
};