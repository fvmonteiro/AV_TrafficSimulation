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
#include "OriginLaneLongitudinalController.h"
#include "DestinationLaneLongitudinalController.h"

class Vehicle;

class ControlManager {
public:

	/* TODO: figure out if there's a way to expand from 
	the longitudinal controller states */
	/*enum class State {
		velocity_control,
		vehicle_following,
		emergency_braking,
		intention_to_change_lane,
	};*/

	enum class ActiveLongitudinalController {
		origin_lane,
		destination_lane,
		end_of_lane,
		vissim,
	};

	ControlManager() = default;
	ControlManager(const Vehicle & ego_vehicle, bool verbose);
	ControlManager(const Vehicle & ego_vehicle);

	//std::vector<State> get_states() { return states; };
	ActiveLongitudinalController get_active_longitudinal_controller() {
		return active_longitudinal_controller;
	}
	LongitudinalController::State get_longitudinal_controller_state();
	//void create_destination_lane_controller(const Vehicle& ego_vehicle);
	
	/* DEBUGGING FUNCTIONS --------------------------------------------------- */
	/* These functions are used to easily read data from internal instances and 
	methods. */

	/* Each controller should never be accessed directly by external
	functions. */
	DestinationLaneLongitudinalController get_destination_lane_controller() const { 
		return destination_lane_controller; 
	};
	/* Each controller should never be accessed directly by external
	functions. */
	LateralController get_lateral_controller() const {
		return lateral_controller;
	};
	double get_reference_gap(double ego_velocity) {
		return origin_lane_controller.compute_desired_gap(ego_velocity);
	}
	/* ----------------------------------------------------------------------- */

	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	/*void update_time_headway(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle);*/
	void update_origin_lane_time_headway(double lambda_1, 
		double leader_max_brake);
	void update_destination_lane_time_headway(double lambda_1,
		double leader_max_brake);
	void estimate_follower_time_headway(const Vehicle& ego_vehicle,
		NearbyVehicle& follower);

	/* Gets the acceleration inputs from the origin (and destination) lane
	ACCs, from the necessary value to avoid colision and from VISSIM and decides
	which one should be applied to the vehicle */
	double determine_desired_acceleration(const Vehicle& ego_vehicle);

	double compute_safe_lane_change_gap(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, bool will_accelerate = false);
	/* Returns the time headway part of the safe lane change gap. */
	double compute_time_headway_gap(const Vehicle& ego_vehicle,
		const NearbyVehicle& other_vehicle);

	/* Sets the value of the minimum accepted longitudinal adjustment speed
	and the initial value of accepted risk, and starts the a timer. */
	void start_longitudinal_adjustment(double time, double velocity);
	void update_accepted_risk(const Vehicle& ego_vehicle);

	/* Printing ----------------------------------------------------------- */
	static std::string active_longitudinal_controller_to_string(
		ActiveLongitudinalController active_longitudinal_controller);

private:
	OriginLaneLongitudinalController origin_lane_controller;
	DestinationLaneLongitudinalController destination_lane_controller;
	OriginLaneLongitudinalController end_of_lane_controller;
	LateralController lateral_controller;
	//std::vector<State> states; // TODO: will be deleted
	ActiveLongitudinalController active_longitudinal_controller{ 
		ActiveLongitudinalController::origin_lane }; /* indicates which
	controller is active: origin_lane, destination_lane or end_of_lane.
	Used for debugging and visualization. */
	bool verbose = false;
};