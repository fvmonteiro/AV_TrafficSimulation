#pragma once

#include "AutonomousVehicle.h"

/* Vehicle similar to AutonomousVehicle, but can also request cooperation
or cooperate with others to generate safe lane changing gaps. */
class ConnectedAutonomousVehicle : public AutonomousVehicle
{
public:

	ConnectedAutonomousVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false);

	/* The connected vehicle brake delay also depends on the other vehicle
	type. If the other vehicle is not connected, the ego connected vehicle
	behaves like an autonomous vehicle. This issue is addressed in parts
	of the code after the ego vehicle identifies its
	surrouding vehicles. */

	/* Returns a nullptr if the ego is no vehicle being assisted
	(for gap generation) */
	/*std::shared_ptr<NearbyVehicle> get_assisted_vehicle() const
	{
		return assisted_vehicle;
	}*/

	bool has_assisted_vehicle() const;
	bool is_cooperating_to_generate_gap() const;

	/* Debugging methods */
	long get_assisted_veh_id() const override { 
		return has_assisted_vehicle() ?
			assisted_vehicle->get_id() : 0;
	};

private:
	/* Finds the current leader, the destination lane leader 
	and follower (if the vehicle has lane change intention),
	and if any nearby vehicle requested cooperation	*/
	void find_relevant_nearby_vehicles() override;
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	bool give_lane_change_control_to_vissim() const override
	{
		return false;
	};
	//void save_other_relevant_nearby_vehicle_ids() override;
	//void clear_other_relevant_nearby_vehicles() override;
	std::shared_ptr<NearbyVehicle>
		implement_get_assisted_vehicle() const override;
	double implement_get_time_headway_to_assisted_vehicle() const override;
	//void try_to_set_nearby_vehicle_type(long nv_type) override;
	/*long try_to_get_nearby_vehicle_type(long nv_type) const override
	{
		return nv_type;
	};*/
	/*double compute_current_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) override;*/
	double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	double compute_vehicle_following_time_headway(
		const NearbyVehicle& nearby_vehicle, 
		double nv_max_lane_change_risk) const;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	virtual double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle) const override;
	long create_lane_change_request() override;

	double get_lambda_1(bool is_leader_connected) const;
	double get_lambda_1_lane_change(bool is_leader_connected) const;

	void find_cooperation_requests();
	/* Returns true if a vehicle is asking to merge in front of us and
	[to do] modifies nearby_vehicle in place to point to the proper 
	virtual leader */
	bool check_if_is_asking_for_cooperation(
		const NearbyVehicle& nearby_vehicle);
	void deal_with_close_and_slow_assited_vehicle(); 
	void compute_connected_safe_gap_parameters();
	void update_destination_lane_follower(
		const std::shared_ptr<NearbyVehicle>& old_follower) override;
	void update_assisted_vehicle(
		const std::shared_ptr<NearbyVehicle>& old_assisted_vehicle);

	/* Emergency braking parameter between connected vehicles */
	double lambda_1_connected{ 0.0 }; // [m/s]
	double lambda_0_connected{ 0.0 }; // [m]
	/* Emergency braking parameter between connected vehicles
	during lane change */
	double lambda_1_lane_change_connected{ 0.0 }; // [m/s]
	long assisted_vehicle_id{ 0 };
	/* Vehicle for which the ego vehicle will help generate a safe
	lane change gap */
	std::shared_ptr<NearbyVehicle> assisted_vehicle{ nullptr };
};