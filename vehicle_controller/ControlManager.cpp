/*==========================================================================*/
/*  ControlManager.cpp    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include "ControlManager.h"
#include "VelocityFilter.h"
#include "NearbyVehicle.h"
#include "Vehicle.h"

ControlManager::ControlManager(const Vehicle& ego_vehicle, bool verbose) {

	if (verbose) {
		std::clog << "Creating control manager " << std::endl;
	}

	this->verbose = verbose;
	this->origin_lane_controller = LongitudinalController(
		ego_vehicle, verbose);
	this->destination_lane_controller = LongitudinalController(
		ego_vehicle, verbose);
	this->end_of_lane_controller = LongitudinalController(
		ego_vehicle, verbose);
	this->lateral_controller = LateralController(verbose);
}

ControlManager::ControlManager(const Vehicle& ego_vehicle) 
	: ControlManager(ego_vehicle, false) {}

ControlManager::State ControlManager::get_current_state() {
	if (states.empty()) {
		return ControlManager::State::velocity_control;
	}
	return states.back(); 
}

double ControlManager::compute_drac(double relative_velocity, double gap) {
	/* Deceleration (absolute value) to avoid collision assuming constant
	velocities and instantaneous acceleration. DRAC is only defined when the
	ego vehicle is faster than the leader. We some high negative value
	otherwise */
	//if (relative_velocity > 0) { // ego faster than leader
	//	return relative_velocity * relative_velocity / 2 / gap;
	//}
	return -100;
}

double ControlManager::determine_desired_acceleration(const Vehicle& ego_vehicle,
	const NearbyVehicle& leader) {

	double desired_acceleration_origin_lane;
	LongitudinalController::State origin_lane_controller_state;
	double desired_acceleration;
	double drac;
	double ego_velocity = ego_vehicle.get_current_velocity();
	double leader_velocity = ego_velocity
		- leader.get_current_relative_velocity();

	/* Logic: if the vehicle wants to change lanes, we let VISSIM take care
	of it. Otherwise, we apply the ACC. */
	if (ego_vehicle.get_current_preferred_relative_lane() != 0) {
		states.push_back(State::intention_to_change_lane);
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
	}
	else {
		desired_acceleration_origin_lane = origin_lane_controller.
			compute_desired_acceleration(ego_vehicle, leader);
		origin_lane_controller_state = origin_lane_controller.get_state();
		drac = compute_drac(leader.get_current_relative_velocity(),
			ego_vehicle.compute_gap(leader));
		if ((origin_lane_controller_state 
			 == LongitudinalController::State::vehicle_following)
			&& (-desired_acceleration_origin_lane < drac)) {
			desired_acceleration = drac;
			states.push_back(State::emergency_braking);
		}
		else {
			desired_acceleration = desired_acceleration_origin_lane;
			states.push_back(longitudinal_state_to_vehicle_state(
				origin_lane_controller_state));
		}
	}
	
	return desired_acceleration;
}

double ControlManager::compute_safe_lane_change_gap(
	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	bool will_accelerate) {
	return lateral_controller.compute_safe_lane_change_gap(ego_vehicle,
		other_vehicle, will_accelerate);
}

ControlManager::State 
	ControlManager::longitudinal_state_to_vehicle_state(
		LongitudinalController::State controller_state) {

	switch (controller_state)
	{
	case LongitudinalController::State::velocity_control:
		return ControlManager::State::velocity_control;
	case LongitudinalController::State::vehicle_following:
		return ControlManager::State::vehicle_following;
	default:
		std::cerr << "Unknown controller state" << std::endl;
		return ControlManager::State::vehicle_following;
	}
}