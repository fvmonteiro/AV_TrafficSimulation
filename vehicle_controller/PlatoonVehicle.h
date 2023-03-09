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

	bool has_finished_adjusting_time_headway() const;
	bool has_finished_increasing_gap() const;
	bool has_finished_closing_gap() const;
	//bool can_start_adjustment_to_virtual_leader() const;

	long get_preceding_vehicle_id() const;
	long get_following_vehicle_id() const;
	const VehicleState* get_preceding_vehicle_state() const;
	const VehicleState* get_following_vehicle_state() const;
	const PlatoonVehicle* get_preceding_vehicle_in_platoon() const;
	const PlatoonVehicle* get_following_vehicle_in_platoon() const;
	/* Suggests the virtual leader as if the vehicle was not part of 
	a platoon */
	std::shared_ptr<NearbyVehicle> define_virtual_leader_when_alone() const;
	bool is_vehicle_in_sight(long nearby_vehicle_id) const;

protected:

	/* Returns a nullptr if no virtual leader 
	[Jan 24, 2023] TODO: most likely can be private */
	std::shared_ptr<NearbyVehicle> define_virtual_leader() const override;

private:
	//double min_overtaking_rel_vel{ 50.0 / 3.6 };
	std::shared_ptr<Platoon> platoon{ nullptr };
	// desired velocity when not a part of the platoon
	//double alone_desired_velocity{ 0.0 };
	/* Time the vehicle has been without a platoon.
	As soon as the vehicle enters simulation, it may create its own platoon */
	double alone_time{ 1.0 };
	double max_time_looking_for_platoon{ 1.0 };
	double lambda_0_platoon{ 0.0 };
	double lambda_1_platoon{ 0.0 };
	double lambda_0_lane_change_platoon{ 0.0 };
	double lambda_1_lane_change_platoon{ 0.0 };

	/* Some constants for the platoon vehicles */
	double in_platoon_comf_accel{ 0.5 };
	double in_platoon_rho{ 0.1 };

	void implement_create_controller() override {
		this->controller = std::make_unique<ControlManager>(*this,
			is_verbose());
	};
	double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	void implement_analyze_nearby_vehicles() override;
	/* Returns true if the vehicle creates a platoon for itself. */
	bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy) override;
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
	/* Chooses the desired destination lane follower */
	void create_lane_change_request() override;
	bool was_my_cooperation_request_accepted() const override;

	// Check if vehicles in our platoon need gap generation
	void find_cooperation_request_from_platoon();
	void create_platoon(long platoon_id, int platoon_lc_strategy);
	void add_myself_to_leader_platoon(
		std::shared_ptr<Platoon> leader_platoon//,
		/*std::shared_ptr<PlatoonVehicle> pointer_to_me*/);

	void compute_platoon_safe_gap_parameters();
};