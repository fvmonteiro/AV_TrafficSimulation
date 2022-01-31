/*==========================================================================*/
/*  ControlManager.h    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <vector>
#include "LateralController.h"
#include "LongitudinalController.h"
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
	};

	ControlManager() = default;
	ControlManager(const VehicleParameters& vehicle_parameters, bool verbose);
	ControlManager(const VehicleParameters& vehicle_parameters);

	//std::vector<State> get_states() { return states; };
	ActiveACC get_active_longitudinal_controller() const {
		return active_longitudinal_controller;
	}
	LongitudinalController::State get_longitudinal_controller_state();
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
};