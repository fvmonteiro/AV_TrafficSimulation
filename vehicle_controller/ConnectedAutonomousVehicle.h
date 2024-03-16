#pragma once

#include "AutonomousVehicle.h"
#include "CAVController.h"

/* Vehicle similar to AutonomousVehicle, but can also request cooperation
or cooperate with others to generate safe lane changing gaps. */
class ConnectedAutonomousVehicle : public AutonomousVehicle
{
public:

	ConnectedAutonomousVehicle() = default;
	ConnectedAutonomousVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);

	/* The connected vehicle brake delay also depends on the other vehicle
	type. If the other vehicle is not connected, the ego connected vehicle
	behaves like an autonomous vehicle. This issue is addressed in parts
	of the code after the ego vehicle identifies its
	surrouding vehicles. */

	bool is_cooperating_to_generate_gap() const;
	/* Adds a nearby vehicle that's in another CAV's range.
	Does nothing if the CAV is not in our range. */
	void add_nearby_vehicle_from_another(
		const ConnectedAutonomousVehicle& cav, long nv_id);

	double get_time_headway_to_assisted_vehicle() const override;

protected:
	/* id of the vehicle in front of which we want to merge */
	long lane_change_request{ 0 };

	/* Pass-to-base constructor */
	ConnectedAutonomousVehicle(long id, VehicleType type,
		double desired_velocity, double brake_delay,
		bool is_lane_change_autonomous, bool is_connected,
		double simulation_time_step, double creation_time, bool verbose);

	double get_lambda_1(bool is_leader_connected) const;
	double get_lambda_1_lane_change(bool is_leader_connected) const;
	std::pair<double, double> get_lane_changing_safe_gap_parameters(
		bool is_leader_connected) const;

	void set_controller(std::shared_ptr<CAVController> a_controller);
	void find_cooperation_requests();
	void set_assisted_vehicle_by_id(long assisted_vehicle_id);

	/* Returns a nullptr if no virtual leader */
	std::shared_ptr<NearbyVehicle> choose_behind_whom_to_move()
		const override;

private:
	/* Emergency braking parameter between connected vehicles */
	double lambda_1_connected{ 0.0 }; // [m/s]
	double lambda_0_connected{ 0.0 }; // [m]
	/* Emergency braking parameter between connected vehicles
	during lane change */
	double lambda_1_lane_change_connected{ 0.0 }; // [m/s]
	double lambda_0_lane_change_connected{ 0.0 }; // [m]
	long assisted_vehicle_id{ 0 };
	/* Vehicle for which the ego vehicle will help generate a safe
	lane change gap */
	std::shared_ptr<NearbyVehicle> assisted_vehicle{ nullptr };
	double original_desired_velocity{ 0.0 };
	//CAVController controller_exclusive;
	std::shared_ptr<CAVController> cav_controller{ nullptr };


	void implement_create_controller() override;

	/*double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights
		) override;*/
	bool give_lane_change_control_to_vissim() const override
	{
		return false;
	};
	/* Finds the current leader, the destination lane leader
	and follower (if the vehicle has lane change intention),
	and if any nearby vehicle requested cooperation */
	void implement_analyze_nearby_vehicles() override;
	std::shared_ptr<NearbyVehicle> implement_get_assisted_vehicle() 
		const override;
	double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	double compute_vehicle_following_time_headway_with_risk(
		const NearbyVehicle& nearby_vehicle,
		double nv_max_lane_change_risk) const;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override;
	/*double compute_vehicle_following_gap_for_lane_change(
		const NearbyVehicle& nearby_vehicle) const override;*/
	
	long implement_get_lane_change_request() const override;
	double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle, double lane_change_speed
	) const override;
	void update_destination_lane_follower_in_controller(
		const NearbyVehicle* old_follower) override;

	/* Id of the vehicle in front of which we want to merge
	IF we are trying to perform a mandatory lane change */
	virtual void create_lane_change_request();
	virtual bool was_my_cooperation_request_accepted() const;

	void deal_with_close_and_slow_assited_vehicle();
	void compute_connected_safe_gap_parameters();
	void update_assisted_vehicle_in_controller(
		const NearbyVehicle* old_assisted_vehicle);
	
	void set_max_desired_velocity(bool should_increase);
};
