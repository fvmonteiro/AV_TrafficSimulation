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

class Vehicle;

class ControlManager {
public:

	enum class State {
		velocity_control,
		vehicle_following,
		emergency_braking,
		intention_to_change_lane,
	};

	/* Each controller should never be accessed directly by external
	functions. This getter should be used only for simpler debugging.*/
	/*LongitudinalController get_origin_lane_controller() const { 
		return origin_lane_controller; 
	};*/
	/* Each controller should never be accessed directly by external
	functions. This getter should be used only for simpler debugging.*/
	LateralController get_lateral_controller() const {
		return lateral_controller;
	};

	ControlManager() = default;
	ControlManager(const Vehicle& ego_vehicle, bool verbose);
	ControlManager(const Vehicle& ego_vehicle);

	std::vector<State> get_states() { return states; };

	State get_current_state();

	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

	/* Gets the acceleration inputs from the origin (and destination) lane
	ACCs, from the necessary value to avoid colision and from VISSIM and decides
	which one should be applied to the vehicle */
	double determine_desired_acceleration(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);

	double compute_safe_lane_change_gap(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, bool will_accelerate = false);

	State ControlManager::longitudinal_state_to_vehicle_state(
		LongitudinalController::State controller_state);

private:
	bool verbose = false;
	LongitudinalController origin_lane_controller;
	LateralController lateral_controller;
	std::vector<State> states;
};