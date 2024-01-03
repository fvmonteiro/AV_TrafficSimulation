#pragma once

#include "AVController.h"
#include "LaneChangeGapsSafety.h"
#include "LongitudinallyAutonomousVehicle.h"

/* Vehicle with autonomous longitudinal control during lane keeping and
during adjustments for lane changing. The lane change intention still
comes from VISSIM, but the vehicle decides when it is safe enough to start */
class AutonomousVehicle : public LongitudinallyAutonomousVehicle
{
public:
	AutonomousVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false);
	
	/* Vehicle behind which we want to merge (not necessarily the same as
	the destination lane leader) */
	std::shared_ptr<const NearbyVehicle> get_virtual_leader() const {
		return virtual_leader;
	}

	bool has_destination_lane_leader_leader() const;
	long get_destination_lane_leader_leader_id() const;
	bool has_virtual_leader() const;
	//bool merge_behind_virtual_leader() const;
	bool are_all_lane_change_gaps_safe() const;
	LaneChangeGapsSafety get_lane_change_gaps_safety() const;

	/* State-machine related methods ----------------------------------------- */

	/* Sets the new state (must be a lane keeping state) and resets the
	desired lane change direction and the longitudinal controllers. */
	void reset_state(std::unique_ptr<VehicleState> new_lane_keeping_state);

protected:
	LaneChangeGapsSafety lane_change_gaps_safety;
	/* Necessary when computing lane change gaps with risk */
	double dest_lane_follower_lambda_0{ 0.0 };
	/* Necessary when computing lane change gaps with risk */
	double dest_lane_follower_lambda_1{ 0.0 };

	AutonomousVehicle(long id, VehicleType type, double desired_velocity,
		bool is_connected, double simulation_time_step, double creation_time,
		bool verbose = false);

	double get_lambda_1_lane_change() const { return lambda_1_lane_change; };
	double get_accepted_risk_to_leaders() const {
		return accepted_lane_change_risk_to_leaders;
	};
	double get_accepted_risk_to_follower() const {
		return accepted_lane_change_risk_to_follower;
	};
	std::shared_ptr<NearbyVehicle> get_modifiable_dest_lane_follower() const
	{
		return implement_get_destination_lane_follower();
	};
	std::shared_ptr<NearbyVehicle> get_modifiable_dest_lane_leader() const
	{
		return implement_get_destination_lane_leader();
	};
	std::shared_ptr<NearbyVehicle> get_destination_lane_leader_leader() const
	{
		return destination_lane_leader_leader;
	};

	void find_destination_lane_vehicles();
	bool try_to_overtake_destination_lane_leader() const;
	bool try_to_overtake_destination_lane_leader(double min_rel_vel) const;
	/*[Feb 7, 2023] The four methods below can probably become private */
	void set_virtual_leader(
		std::shared_ptr<NearbyVehicle> new_virtual_leader);
	/*void set_destination_lane_follower_by_id(
		long new_follower_id);*/
	bool is_destination_lane_follower(
		const NearbyVehicle& nearby_vehicle);
	bool is_destination_lane_leader(const NearbyVehicle& nearby_vehicle);
	bool is_leader_of_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);

	/* ----------------------------------------------------- */
	
	bool is_lane_change_gap_safe(
		const NearbyVehicle* nearby_vehicle) const;
	bool has_lane_change_conflict() const;
	/* Non-linear gap based on ego and nearby vehicles states
	and parameters */
	double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle, double current_lambda_1) const;

private:
	double min_overtaking_rel_vel{ 10.0 / 3.6 };
	double max_lane_change_waiting_time{ 60.0 }; // [s]

	AVController av_controller;

	/* Relevant members for lane changing ------------------------------------ */

	/* Vehicle immediately ahead at the destination lane */
	std::shared_ptr<NearbyVehicle> destination_lane_leader{ nullptr };
	/* Vehicle immediately behind at the destination lane */
	std::shared_ptr<NearbyVehicle> destination_lane_follower{ nullptr };
	/* Vehicle in front of the destination lane leader */
	std::shared_ptr<NearbyVehicle> destination_lane_leader_leader{ nullptr };
	/* Vehicle behind which we want to merge (not necessarily the same as 
	the destination lane leader) */
	std::shared_ptr<NearbyVehicle> virtual_leader{ nullptr };
	/* Emergency braking parameter during lane change */
	double lambda_1_lane_change{ 0.0 }; // [m/s]

	/* Risk related variables --------------------------------------------- */
	/*The risk is an estimation of the relative velocity at collision
	time under worst case scenario. */

	/* Defines if the lane change acceptance decision is based on the
	exact risk computation or on the risk estimation based on the time
	headway */
	bool use_linear_lane_change_gap{ false };
	/* Stores the time when the vehicle started trying to
	change lanes */
	//double lane_change_timer_start{ 0.0 }; // [s]
	double accepted_lane_change_risk_to_leaders{ 0.0 }; // [m/s]
	double accepted_lane_change_risk_to_follower{ 0.0 }; // [m/s]
	//double initial_risk{ 0.0 }; // [m/s]
	//double constant_risk_period{ 1.0 }; // [s]
	//double delta_risk{ 3.0 }; // [m/s]
	//double max_risk_to_leader{ 0.0 }; // [m/s]
	/* Risk at which the lane change headway becomes equal to the
	vehicle following headway. */
	//double intermediate_risk_to_leader{ 0.0 }; // [m/s]
	//double max_risk_to_follower{ 0.0 }; // [m/s]

	LongAVController* get_long_av_controller() override;
	const LongAVController* get_long_av_controller_const() const override;
	virtual AVController* get_av_controller() {
		return &av_controller;
	};
	virtual const AVController* get_av_controller_const() const {
		return &av_controller;
	};
	/* Finds the current leader and, if the vehicle has lane change
	intention, the destination lane leader and follower */
	//void find_relevant_nearby_vehicles() override;
	double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	/* Finds the current leader and, if the vehicle has lane change
	intention, the destination lane leader and follower */
	void implement_analyze_nearby_vehicles() override;
	bool give_lane_change_control_to_vissim() const override;
	bool implement_check_lane_change_gaps() override;
	long implement_get_lane_change_request() const override { return 0; };
	double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle) const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const override;
	std::shared_ptr<NearbyVehicle> implement_get_assisted_vehicle()
		const override { return nullptr; };
	long implement_get_virtual_leader_id() const override;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;

	void pass_this_to_state() override;

	/* Time-headway based gap (hv + d) minus a term based on
	accepted risk */
	double compute_time_headway_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle) const;
	void compute_lane_change_gap_parameters();
	/* Non-linear gap based on ego and nearby vehicles states
	and parameters */
	virtual double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle) const;
	double compute_gap_variation_during_lane_change(
		const NearbyVehicle& nearby_vehicle) const;

	/* Returns a nullptr if no virtual leader */
	virtual std::shared_ptr<NearbyVehicle> define_virtual_leader() const;
	void update_virtual_leader(std::shared_ptr<const NearbyVehicle> old_leader);
	virtual void update_destination_lane_follower(
		const std::shared_ptr<NearbyVehicle>& old_follower);
	double estimate_nearby_vehicle_time_headway(
		NearbyVehicle& nearby_vehicle);

	/* Risk related methods --------------------------------------------- */

	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override;
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override;
	void implement_set_use_linear_lane_change_gap(
		long value) override;
	/* Resets accepted risks and time */
	void reset_accepted_lane_change_risks(double time);
	/* Updates the accepted risk periodically - NOT IMPLEMENTED */
	bool update_accepted_risk(double time);
	/* Computes the risk at which he lane change headway becomes equal to the
	vehicle following headway. */
	double compute_intermediate_risk_to_leader(double lambda_1,
		double lane_change_lambda_1, double max_brake_no_lane_change,
		double leader_max_brake);
	/* Maximum possible accepted risk during vehicle following that keeps
	the time headway positive.
	This is the same max risk as for gap generation */
	//double compute_max_vehicle_following_risk(double leader_max_brake);
	/* NOT IMPLEMENTED */
	void update_headways_with_risk(const EgoVehicle& ego_vehicle);
};
