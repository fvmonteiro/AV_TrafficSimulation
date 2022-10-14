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
#include "SwitchedLongitudinalController.h"
#include "LongitudinalControllerWithTrafficLights.h"
#include "RealLongitudinalController.h"
#include "TalebpourALC.h"
#include "VirtualLongitudinalController.h"
#include "Vehicle.h"

class EgoVehicle;
class ACCVehicle;
class AutonomousVehicle;
class ConnectedAutonomousVehicle;
class TrafficLightACCVehicle;
class VirdiVehicle;

class ControlManager {
public:

	enum class ACCType {
		origin_lane,
		destination_lane,
		cooperative_gap_generation,
		end_of_lane,
		vissim,
		traffic_light_acc
	};

	ControlManager() = default;
	ControlManager(const EgoVehicle& ego_vehicle, bool verbose);
	ControlManager(const EgoVehicle& ego_vehicle);

	//std::vector<State> get_states() { return states; };
	ACCType get_active_longitudinal_controller() const {
		return active_longitudinal_controller;
	}

	SwitchedLongitudinalController::State 
		get_longitudinal_controller_state() const;
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
	const VirtualLongitudinalController& get_destination_lane_controller()
		const { 
		return destination_lane_controller; 
	};
	/* Each controller should never be accessed directly by external
	functions. */
	const LateralController& get_lateral_controller() const {
		return lateral_controller;
	};
	double get_reference_gap(double ego_velocity); /* could be const */
	double get_gap_error(VehicleType type) const;

	/* ----------------------------------------------------------------------- */

	/* Extracts and stores the static vehicle parameters */
	//void extract_vehicle_static_parameters(const EgoVehicle& ego_vehicle);

	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	void activate_end_of_lane_controller(double time_headway);
	/* Resets the origin lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_origin_lane_controller(double ego_velocity,
		double time_headway, bool is_leader_connected);
	/* Sets a new time headway and the connectivity of the origin lane 
	controller */
	void update_origin_lane_controller(double time_headway,
		bool is_leader_connected);
	void update_destination_lane_follower_time_headway(double time_headway);
	/* Resets the destination lane controller's velocity and time headway filters
	and sets the time headway*/
	void activate_destination_lane_controller(double ego_velocity,
		double time_headway, bool is_leader_connected);
	/* Sets a new time headway and the connectivity of the destination lane
	controller, and resets its velocity filter. */
	void update_destination_lane_controller(double ego_velocity, 
		double time_headway, bool is_leader_connected);
	void update_gap_generation_controller(double ego_velocity,
		double time_headway);

	void reset_origin_lane_velocity_controller(double ego_velocity);

	/* Active ACC during lane keeping; human (vissim) control if there is
	lane change intention*/
	double get_acc_desired_acceleration(const ACCVehicle& ego_vehicle);
	double get_virdi_desired_acceleration(const VirdiVehicle& ego_vehicle);
	/* Computes ACC desired acceleration plus the acceleration during lane
	change adjustments and lateral movement. Gives control to human (vissim)
	if the vehicle is waiting for too long to find a gap. */
	double get_av_desired_acceleration(const AutonomousVehicle& ego_vehicle);
	/* Computes the AV desired acceleration plus the cooperative acceleration
	to help create a gap for an incoming vehicle, and chooses the minimum. */
	double get_cav_desired_acceleration(
		const ConnectedAutonomousVehicle& ego_vehicle);
	/* TODO */
	double get_traffic_light_acc_acceleration(
		const TrafficLightACCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	void print_traffic_lights(const EgoVehicle& ego,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	double use_vissim_desired_acceleration(const EgoVehicle& ego_vehicle);

	double determine_low_velocity_reference(double ego_velocity,
		const NearbyVehicle& nearby_vehicle);
	
	/* Returns the reference gap used by the longitudinal 
	controller. It uses a previously defined time headway, which might
	have been computed assuming some accepted risk. */
	double compute_desired_lane_change_gap(const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle, bool will_accelerate = false);
	
	/* Returns the accepted lane change gap, which might is different from 
	the longitudinal controller's reference gap if the accepted risk is 
	greater than zero. This assumes the longitudinal controller uses a 
	zero-risk time headway. */
	//double compute_accepted_lane_change_gap(
	//	const AutonomousVehicle& ego_vehicle,
	//	const NearbyVehicle& nearby_vehicle, 
	//	bool will_accelerate = false);
	
	/* Returns the time headway part of the desired lane change gap. */
	double get_desired_time_headway_gap(double ego_velocity,
		const NearbyVehicle& nearby_vehicle);
	/* Returns the time headway part of the accepted lane change gap. */
	/*double get_accepted_time_headway_gap(const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle);*/
	/* Returns the expected gap variation during the lane change */
	double get_gap_variation_during_lane_change(
		const AutonomousVehicle& ego_vehicle,
		const NearbyVehicle& nearby_vehicle,
		bool will_accelerate);


	/* Printing ----------------------------------------------------------- */
	static std::string active_ACC_to_string(
		ACCType active_longitudinal_controller);

private:
	/* ------------ Control Parameters ------------ */

	double min_overtaking_rel_vel{ 10.0 / 3.6 };
	double velocity_filter_gain{ 10.0 };
	double time_headway_filter_gain{ 0.3 };
	AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	VelocityControllerGains desired_velocity_controller_gains{
		0.5, 0.1, 0.03 };
	VelocityControllerGains adjustment_velocity_controller_gains{
		1, 0.1, 0.03 };
	/* The time headway used by the end-of-lane controller */
	double time_headway_to_end_of_lane{ 1.0 };
	/* -------------------------------------------- */
	
	RealLongitudinalController origin_lane_controller;
	RealLongitudinalController end_of_lane_controller;
	VirtualLongitudinalController destination_lane_controller;
	VirtualLongitudinalController gap_generating_controller;
	LongitudinalControllerWithTrafficLights
		with_traffic_lights_controller;

	/* TODO: this is messy... but let's just get it working for now */
	/*TalebpourALC origin_lane_talebpourALC;
	TalebpourALC end_of_lane_talebpourALC;
	TalebpourALC destination_lane_talebpourALC;
	TalebpourALC gap_generating_talebpourALC;
	std::unordered_map<ACCType, TalebpourALC> long_controllers;*/
	TalebpourALC talebpour_alc;

	LateralController lateral_controller;
	/* indicates which controller is active. Used for debugging and
	visualization. */
	ACCType active_longitudinal_controller{ ACCType::origin_lane }; 

	double origin_lane_leader_max_brake{ 0.0 };
	double destination_lane_leader_max_brake{ 0.0 };
	double destination_lane_follower_max_brake{ 0.0 }; 
	double assisted_vehicle_max_brake{ 0.0 };
	VehicleType origin_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_leader_type{ VehicleType::undefined };
	VehicleType destination_lane_follower_type{ VehicleType::undefined };

	bool verbose{ false };

	/* Initializing controllers */

	/* Creates autonomous longitudinal controllers for vehicle ahead and 
	for the end of the lane*/
	void create_autonomous_longitudinal_controllers(
		const EgoVehicle& ego_vehicle, bool verbose);
	void create_lane_change_adjustment_controller(
		const EgoVehicle& ego_vehicle,
		bool verbose);
	void create_cooperative_lane_change_controller(
		const EgoVehicle& ego_vehicle,
		bool verbose);
	/* Based on Virdi et al. (2018) */
	void create_virdi_controllers(const EgoVehicle& ego_vehicle,
		bool verbose);

	/* Gets the minimum of the accelerations in the map and sets the
	active longitudinal controller. */
	double choose_minimum_acceleration(
		std::unordered_map<ACCType, double>& possible_accelerations);

	NearbyVehicle create_virtual_stopped_vehicle(
		const EgoVehicle& ego_vehicle);
	/* Desired accelerations --------------------- */

	/* Desired acceleration relative to the current lane.
	Returns true if the computed acceleration was added to the map */
	bool get_origin_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ACCType, double>& possible_accelerations);
	/* Desired acceleration to wait at the end of the lane while
	looking for an appropriate lane change gap. Without this,
	vehicles might miss a desired exit.
	Returns true if the computed acceleration was added to the map */
	bool get_end_of_lane_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		std::unordered_map<ACCType, double>& possible_accelerations);
	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		const AutonomousVehicle& ego_vehicle,
		std::unordered_map<ACCType, double>& possible_accelerations);
	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		const ConnectedAutonomousVehicle& ego_vehicle,
		std::unordered_map<ACCType, double>& possible_accelerations);
};