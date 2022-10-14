#pragma once

#include "EgoVehicle.h"

/* Vehicle with autonomous longitudinal control during lane keeping and
during adjustments for lane changing. The lane change intention still 
comes from VISSIM, but the vehicle decides when it is safe enough to start */
class AutonomousVehicle : public EgoVehicle
{
public:

	AutonomousVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) : AutonomousVehicle(id,
			VehicleType::autonomous_car, desired_velocity, false,
			simulation_time_step, creation_time, verbose) {} ;

	/*double get_lambda_1_lane_change() const { return lambda_1_lane_change; };*/
	/* Returns a nullptr if there is no leader at the destination lane */
	//std::shared_ptr<NearbyVehicle> get_destination_lane_leader() const
	//{
	//	return destination_lane_leader;
	//};
	///* Returns a nullptr if there is no follower at the destination lane */
	//std::shared_ptr<NearbyVehicle> get_destination_lane_follower() const
	//{
	//	return destination_lane_follower;
	//};

	/*bool has_destination_lane_leader() const;
	bool has_destination_lane_follower() const;*/

	/* Debugging methods */

	/*long get_dest_lane_leader_id() const override
	{
		return has_destination_lane_leader() ?
			destination_lane_leader->get_id() : 0;
	};
	long get_dest_lane_follower_id() const override {
		return has_destination_lane_follower() ?
			destination_lane_follower->get_id() : 0;
	};
	double get_dest_follower_time_headway() const override {
		return controller.get_destination_lane_controller().
			get_follower_time_headway();
	};*/

protected:
	AutonomousVehicle(long id, VehicleType type, double desired_velocity,
		bool is_connected, double simulation_time_step, double creation_time,
		bool verbose = false);

	/* Finds the current leader and, if the vehicle has lane change
	intention, the destination lane leader and follower */
	void implement_analyze_nearby_vehicles() override;
	void set_desired_lane_change_direction() override;
	bool can_start_lane_change() override;

	double get_lambda_1_lane_change() const { return lambda_1_lane_change; }
	double get_accepted_risk_to_leaders() const {
		return accepted_lane_change_risk_to_leaders;
	}
	double get_accepted_risk_to_follower() const {
		return accepted_lane_change_risk_to_follower;
	}
	
	bool is_destination_lane_follower(
		const NearbyVehicle& nearby_vehicle);
	bool is_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);
	bool is_leader_of_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);
	void deal_with_stopped_destination_lane_leader(
		bool dest_lane_leader_has_leader);
	void update_destination_lane_leader(
		const std::shared_ptr<NearbyVehicle>& old_leader);

	/* Non-linear gap based on ego and nearby vehicles states
	and parameters */
	double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle, double current_lambda_1) const;

	/* Necessary when computing lane change gaps with risk */
	double dest_lane_follower_lambda_0{ 0.0 };
	/* Necessary when computing lane change gaps with risk */
	double dest_lane_follower_lambda_1{ 0.0 };

private:
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	//void set_desired_lane_change_direction() override;
	bool give_lane_change_control_to_vissim() const override;
	//bool can_start_lane_change() override;
	long create_lane_change_request() override { return 0; };
	double compute_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle) override;
	/* Time-headway based gap (hv + d) minus a term based on 
	accepted risk */
	double compute_time_headway_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle);
	
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const override;
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const override;
	std::shared_ptr<NearbyVehicle> implement_get_assisted_vehicle()
		const override { return nullptr; };

	void find_destination_lane_vehicles();

	bool has_lane_change_conflict() const;
	bool is_lane_change_gap_safe(
		std::shared_ptr<NearbyVehicle>& nearby_vehicle);
	void compute_lane_change_gap_parameters();
	/* Non-linear gap based on ego and nearby vehicles states
	and parameters */
	virtual double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle) const;
	double compute_gap_variation_during_lane_change(
		const NearbyVehicle& nearby_vehicle) const;

	virtual void update_destination_lane_follower(
		const std::shared_ptr<NearbyVehicle>& old_follower);
	/*double compute_current_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) override;*/
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
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

	double max_lane_change_waiting_time{ 60.0 }; // [s]

	/* Relevant members for lane changing ------------------------------------ */

	std::shared_ptr<NearbyVehicle> destination_lane_leader{ nullptr };
	std::shared_ptr<NearbyVehicle> destination_lane_follower{ nullptr };
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
};
