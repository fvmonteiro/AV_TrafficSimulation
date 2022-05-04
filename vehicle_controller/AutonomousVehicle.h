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
	std::shared_ptr<NearbyVehicle> get_destination_lane_leader() const
	{
		return destination_lane_leader;
	};
	/* Returns a nullptr if there is no follower at the destination lane */
	std::shared_ptr<NearbyVehicle> get_destination_lane_follower() const
	{
		return destination_lane_follower;
	};

	bool has_destination_lane_leader() const;
	bool has_destination_lane_follower() const;

	/* Debugging methods */

	long get_dest_lane_leader_id() const override
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
	};

protected:
	AutonomousVehicle(long id, VehicleType type, double desired_velocity,
		bool is_connected,
		double simulation_time_step, double creation_time,
		bool verbose = false);

	double get_lambda_1_lane_change() const { return lambda_1_lane_change; }
	void find_destination_lane_vehicles();
	bool check_if_is_destination_lane_follower(
		const NearbyVehicle& nearby_vehicle);
	bool check_if_is_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);
	bool is_leader_of_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);
	void deal_with_stopped_destination_lane_leader(
		bool dest_lane_leader_has_leader);
	void update_destination_lane_leader(
		const std::shared_ptr<NearbyVehicle>& old_leader);
	//void save_other_relevant_nearby_vehicle_ids() override;
	//void clear_other_relevant_nearby_vehicles() override;

private:
	/* Finds the current leader and, if the vehicle has lane change 
	intention, the destination lane leader and follower */
	void find_relevant_nearby_vehicles() override;
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	bool give_lane_change_control_to_vissim() const override;
	bool can_start_lane_change() override;
	bool has_lane_change_conflict() const;
	bool is_lane_change_gap_safe(
		std::shared_ptr<NearbyVehicle>& nearby_vehicle);
	void compute_lane_change_gap_parameters();
	virtual void update_destination_lane_follower(
		const std::shared_ptr<NearbyVehicle>& old_follower);
	double compute_current_desired_time_headway(
		const NearbyVehicle& leader) override;
	virtual double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& leader);
	double estimate_nearby_vehicle_time_headway(NearbyVehicle& nearby_vehicle);

	/* possibly not needed anymore */
	//long dest_lane_leader_id{ 0 };
	/* possibly not needed anymore */
	//long dest_lane_follower_id{ 0 };
	std::shared_ptr<NearbyVehicle> destination_lane_leader{ nullptr };
	std::shared_ptr<NearbyVehicle> destination_lane_follower{ nullptr };
	/* Emergency braking parameter during lane change */
	double lambda_1_lane_change{ 0.0 }; // [m/s]
};
