/*==========================================================================*/
/*  ControlManager.h    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <unordered_map>
#include <vector>

#include "LateralController.h"
#include "LongitudinalController.h"
#include "LongitudinalControllerWithTrafficLights.h"
#include "RealLongitudinalController.h"
#include "VirtualLongitudinalController.h"
#include "Vehicle.h"

class EgoVehicle;

class ControlManager {
public:

	enum class ActiveACC {
		origin_lane,
		destination_lane,
		cooperative_gap_generation,
		end_of_lane,
		vissim,
		traffic_light_acc
	};

	ControlManager() = default;
	ControlManager(const VehicleParameters& vehicle_parameters, bool verbose);
	ControlManager(const VehicleParameters& vehicle_parameters);

	//std::vector<State> get_states() { return states; };
	ActiveACC get_active_longitudinal_controller() const {
		return active_longitudinal_controller;
	}

	LongitudinalController::State get_longitudinal_controller_state();
	LongitudinalControllerWithTrafficLights::State 
		get_longitudinal_controller_with_traffic_lights_state();
	//void create_destination_lane_controller(const Vehicle& ego_vehicle);
	
	/* DEBUGGING FUNCTIONS --------------------------------------------------- */
	/* These functions are used to easily read data from internal instances and 
	methods. */

	/* Each controller should never be accessed directly by external
	functions. */
	const RealLongitudinalController& get_origin_lane_controller() const {
		return origin_lane_controller;
	}
	const VirtualLongitudinalController& get_gap_generation_lane_controller() 
		const {
		return gap_generating_controller;
	};
	/* Each controller should never be accessed directly by external
	functions. */
	const VirtualLongitudinalController& get_destination_lane_controller() const { 
		return destination_lane_controller; 
	};
	/* Each controller should never be accessed directly by external
	functions. */
	const LateralController& get_lateral_controller() const {
		return lateral_controller;
	};
	double get_reference_gap(double ego_velocity,
		bool has_lane_change_intention); /* could be const */
	double get_gap_error() const;

	/* ----------------------------------------------------------------------- */

	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	/* Updates the time headway based on the new leader and 
	resets the leader velocity filter if there was no leader before */
	void update_origin_lane_leader(double ego_velocity, bool had_leader,
		const NearbyVehicle& leader);
	/* Updates the (risky) time headway based on the new leader and
	resets the leader velocity filter */
	void update_destination_lane_leader(double ego_velocity, 
		const NearbyVehicle& leader);
	void update_assisted_vehicle(double ego_velocity, 
		const NearbyVehicle& assisted_vehicle);
	void update_follower_time_headway(NearbyVehicle& follower);
	void reset_origin_lane_velocity_controller(double ego_velocity);

	
	/* Active ACC during lane keeping; human (vissim) control if there is
	lane change intention*/
	double get_acc_desired_acceleration(const EgoVehicle& ego_vehicle);
	/* Computes ACC desired acceleration plus the acceleration during lane
	change adjustments and lateral movement. Gives control to human (vissim)
	if the vehicle is waiting for too long to find a gap. */
	double get_av_desired_acceleration(const EgoVehicle& ego_vehicle);
	/* Computes the AV desired acceleration plus the cooperative acceleration
	to help create a gap for an incoming vehicle, and chooses the minimum. */
	double get_cav_desired_acceleration(const EgoVehicle& ego_vehicle);
	/* TODO */
	double get_traffic_light_acc_acceleration(const EgoVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	void print_tfs(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	double use_vissim_desired_acceleration(const EgoVehicle& ego_vehicle);

	/* Gets the acceleration inputs from the origin (and destination) lane
	ACCs, from the necessary value to avoid colision and from VISSIM and decides
	which one should be applied to the vehicle */
	double determine_desired_acceleration(const EgoVehicle& ego_vehicle);

	double determine_low_velocity_reference(double ego_velocity,
		const NearbyVehicle& other_vehicle);
	double compute_safe_lane_change_gap(const EgoVehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, bool will_accelerate = false);
	/* Returns the time headway part of the safe lane change gap. */
	double compute_safe_time_headway_gap(double ego_velocity,
		bool has_lane_change_intention, const NearbyVehicle& other_vehicle);

	/* Resets accepted risk and risk timer */
	void start_longitudinal_adjustment(double time);
	/* Sets the value of the minimum accepted longitudinal adjustment speed
	and the initial value of accepted risk, and starts the a timer. */
	/*void start_longitudinal_adjustment(double time, double ego_velocity,
		double adjustment_speed_factor);*/

	void update_headways_with_risk(const EgoVehicle& ego_vehicle);

	/* Printing ----------------------------------------------------------- */
	static std::string active_ACC_to_string(
		ActiveACC active_longitudinal_controller);

private:
	VehicleParameters ego_parameters;
	double min_overtaking_rel_vel{ 10.0 / 3.6 };

	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	VelocityControllerGains desired_velocity_controller_gains{ 
		0.5, 0.1, 0.03 };
	VelocityControllerGains adjustment_velocity_controller_gains{
		1, 0.1, 0.03 };

	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;
	VirtualLongitudinalController destination_lane_controller;
	VirtualLongitudinalController gap_generating_controller;
	LongitudinalControllerWithTrafficLights
		with_traffic_lights_controller;

	LateralController lateral_controller;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ActiveACC active_longitudinal_controller{ ActiveACC::origin_lane }; 

	double origin_lane_leader_max_brake{ 0.0 };
	double destination_lane_leader_max_brake{ 0.0 };
	double destination_lane_follower_max_brake{ 0.0 }; 
	double assisted_vehicle_max_brake{ 0.0 };
	VehicleType origin_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_follower_type{ VehicleType::undefined };

	bool verbose{ false };

	/* Initializing controllers */

	void create_acc_controllers(const VehicleParameters& vehicle_parameters,
		bool verbose);
	void create_lane_change_adjustment_controller(
		const VehicleParameters& vehicle_parameters,
		bool verbose);
	void create_cooperative_lane_change_controller(
		const VehicleParameters& vehicle_parameters,
		bool verbose);

	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ActiveACC, double>& possible_accelerations);

	/* [Feb 11, 22] Functions for one style of coding --------------------- */

	/* Desired acceleration relative to the current lane.
	Returns true if the computed acceleration was added to the map */
	bool get_origin_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ActiveACC, double>& possible_accelerations);
	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.
	Returns true if the computed acceleration was added to the map */
	bool get_end_of_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ActiveACC, double>& possible_accelerations);
	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ActiveACC, double>& possible_accelerations);
	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ActiveACC, double>& possible_accelerations);
	/* -------------------------------------------------------------------- */

	/* [Feb 11, 22] Functions for one style of coding --------------------- */
	/* Desired acceleration relative to the current lane */
	//double get_origin_lane_desired_acceleration(
	//	const EgoVehicle& ego_vehicle);
	///* Desired acceleration to wait at the end of the lane while
	//looking for an appropriate lane change gap. Without this,
	//vehicles might miss a desired exit. */
	//double get_end_of_lane_desired_acceleration(
	//	const EgoVehicle& ego_vehicle);
	///* Control to adjust to destination lane leader */
	//double get_destination_lane_desired_acceleration(
	//	const EgoVehicle& ego_vehicle, bool end_of_lane_controller_is_active);
	//double ControlManager::get_cooperative_desired_acceleration(
	//	const EgoVehicle& ego_vehicle);
	/* -------------------------------------------------------------------- */
};