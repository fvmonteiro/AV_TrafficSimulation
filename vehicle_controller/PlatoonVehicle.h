#pragma once

#include "ConnectedAutonomousVehicle.h"
#include "PlatoonVehicleController.h"

class PlatoonVehicle : public ConnectedAutonomousVehicle
{
public:

	PlatoonVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);
	~PlatoonVehicle();

	/* Value added to the safe headway to determine the reference
	headway. */
	double get_time_headway_margin() const { return TIME_HEADWAY_MARGIN; };
	/* True if the space between dest lane leader and follower
	is large enough for a lane change after longitudinal adjustments */
	double get_is_space_suitable_for_lane_change() const {
		return is_space_suitable_for_lane_change;
	}
	/* Chooses between in-platoon or non-platoon time headway */
	double decide_safe_time_headway(const NearbyVehicle& nearby_vehicle
	) const;
	/* Returns true if vehicle is the platoon leader 
	or if it is not part of a platoon */
	bool is_platoon_leader() const;
	/* Returns true if vehicle is the last vehicle in 
	the platoon or if it is not part of a platoon */
	bool is_last_platoon_vehicle() const;
	
	long get_suitable_destination_lane_leader_id() const;
	double get_desired_velocity_from_platoon() const;
	bool can_start_lane_change();
	/* True if the dest lane leader and follower correspond to what
	the lane change strategy expects. */
	//bool is_at_right_lane_change_gap() const;

	/* Adds a platoon vehicle that's outside the sensory range to the 
	nearby vehicles list. */
	void add_another_as_nearby_vehicle(
		const PlatoonVehicle& platoon_vehicle);

	void give_up_lane_change();

	/* Returns h.v_ff + c. Attention: h and c are hard coded */
	double get_free_flow_intra_platoon_gap() const;

	/* Methods used by deprecated framework (Jan 24) ---------------------- */
	bool has_finished_adjusting_time_headway() const;
	bool has_finished_increasing_gap() const;
	bool has_finished_closing_gap() const;
	long get_preceding_vehicle_id() const;
	long get_following_vehicle_id() const;
	const VehicleState* get_preceding_vehicle_state() const;
	const VehicleState* get_following_vehicle_state() const;
	const PlatoonVehicle* get_preceding_vehicle_in_platoon() const;
	const PlatoonVehicle* get_following_vehicle_in_platoon() const;
	/* Suggests the virtual leader as if the vehicle was not part of
	a platoon. */
	std::shared_ptr<NearbyVehicle> define_virtual_leader_when_alone() const;
	/* -------------------------------------------------------------------- */

protected:

	/* Pass-to-base constructor */
	PlatoonVehicle(long id, VehicleType type,
		double desired_velocity, double brake_delay,
		bool is_lane_change_autonomous, bool is_connected,
		double simulation_time_step, double creation_time, bool verbose);

	/* Returns a nullptr if no virtual leader */
	std::shared_ptr<NearbyVehicle> choose_behind_whom_to_move() 
		const override;
	void set_controller(
		std::shared_ptr<PlatoonVehicleController> a_controller);
	void compute_platoon_safe_gap_parameters();

private:
	std::shared_ptr<Platoon> platoon{ nullptr };
	std::shared_ptr<PlatoonVehicleController> platoon_vehicle_controller{ 
		nullptr };
	bool has_lane_change_failed{ false };
	bool is_space_suitable_for_lane_change{ false };

	/* Time the vehicle has been without a platoon.
	As soon as the vehicle enters simulation, it may create its own platoon */
	double alone_time{ 1.0 }; // not in use
	double max_time_looking_for_platoon{ 1.0 }; // not in use

	/* Safe and reference gaps' parameters.
	Not being used. It is easier to debug and analyze with 
	fixed values of h */

	double lambda_0_platoon{ 0.0 };
	double lambda_1_platoon{ 0.0 };
	double lambda_0_lane_change_platoon{ 0.0 };
	double lambda_1_lane_change_platoon{ 0.0 };
	double in_platoon_comf_accel{ 0.5 };
	double in_platoon_rho{ 0.1 };

	/* Safe and reference time headway values */

	const double SAFE_TIME_HEADWAY{ 2.0 }; // towards non-platoon vehicles
	const double SAFE_PLATOON_TIME_HEADWAY{ 1.0 }; // towards platoon vehicles
	const double TIME_HEADWAY_MARGIN{ 0.1 };  // h_ref = h_safe + h_margin

	void implement_create_controller() override;
	/*double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights
	) override;*/
	void implement_analyze_nearby_vehicles() override;
	/* Returns true if the vehicle creates a platoon for itself. */
	bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy, double max_computation_time
	) override;
	double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle, double lane_change_speed
	) const override;
	/* SCENARIO SPECIFIC: the platoon vehicles always start at the in ramp, 
	and they try to change lanes as soon as they enter the highway */
	void set_desired_lane_change_direction() override;
	double apply_low_level_dynamics(double unfiltered_acceleration) override;
	const Platoon* implement_get_platoon() const override;
	std::shared_ptr<Platoon> implement_share_platoon() const override;
	void pass_this_to_state() override;
	/* Chooses the desired destination lane follower */
	void create_lane_change_request() override;
	bool was_my_cooperation_request_accepted() const override;
	void implement_prepare_to_start_long_adjustments() override;
	void implement_prepare_to_restart_lane_keeping(
		bool was_lane_change_successful) override;

	double compute_reference_vehicle_following_gap(
		const NearbyVehicle* nearby_vehicle) const;
	/* Checks if the total space on the destination lane is enough
	to fit the vehicle after longitudinal adjustments */
	void check_adjacent_space_suitability();
	// Check if vehicles in our platoon need gap generation
	void find_cooperation_request_from_platoon();
	void create_platoon(long platoon_id, int platoon_lc_strategy,
		double max_computation_time);
	void add_myself_to_leader_platoon(
		std::shared_ptr<Platoon> leader_platoon);

};