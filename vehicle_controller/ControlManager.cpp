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

	//LongitudinalController::State origin_lane_controller_state;
	double desired_acceleration;
	double max_acceleration = ego_vehicle.get_comfortable_acceleration() * 2;
	double desired_acceleration_origin_lane = max_acceleration;
	double desired_acceleration_dest_lane = max_acceleration;
	double desired_acceleration_end_of_lane = max_acceleration;
	//double drac;
	//double ego_velocity = ego_vehicle.get_current_velocity();
	/*double leader_velocity = ego_velocity
		- leader.get_current_relative_velocity();*/

	if (ego_vehicle.get_desired_lane_change_direction() 
		!= RelativeLane::same) {
		if (ego_vehicle.get_use_internal_lane_change_decision()) {
			desired_acceleration_origin_lane =
				origin_lane_controller.compute_desired_acceleration(
					ego_vehicle, leader);

			/* Longitudinal control to wait at lane's end while waiting
			for appropriate lane change gap. Without this, vehicles might 
			miss a desired exit. */
			if ((ego_vehicle.get_current_active_lane_change() == 0)
				&& (ego_vehicle.get_current_lane_end_distance() > 0)) {
				/* For now, we simulate a stopped vehicle at the end of 
				the lane to force the vehicle to stop before the end of
				the lane. */
				NearbyVehicle virtual_vehicle;
				virtual_vehicle.set_id(1);
				virtual_vehicle.set_relative_lane(RelativeLane::same);
				virtual_vehicle.set_relative_position(1);
				virtual_vehicle.set_relative_velocity(
					ego_vehicle.get_current_velocity());
				virtual_vehicle.set_distance(
					ego_vehicle.get_current_lane_end_distance());
				virtual_vehicle.set_length(0.0);
				desired_acceleration_end_of_lane =
					end_of_lane_controller.compute_desired_acceleration(
						ego_vehicle, virtual_vehicle);
			}

			/* Longitudinal control to adjust to destination lane leader */
			NearbyVehicle* dest_lane_leader =
				ego_vehicle.find_destination_lane_leader();
			if (dest_lane_leader != nullptr) {
				desired_acceleration_dest_lane =
					destination_lane_controller.compute_desired_acceleration(
						ego_vehicle, *dest_lane_leader);
			}

			desired_acceleration = std::min(
				std::min(desired_acceleration_origin_lane,
				desired_acceleration_dest_lane),
				desired_acceleration_end_of_lane);
		}
		else {
			desired_acceleration = 
				ego_vehicle.get_current_vissim_acceleration();			
		}

		states.push_back(State::intention_to_change_lane);
	}
	else {
		desired_acceleration_origin_lane = origin_lane_controller.
			compute_desired_acceleration(ego_vehicle, leader);
		//origin_lane_controller_state = origin_lane_controller.get_state();
		desired_acceleration = desired_acceleration_origin_lane;

		states.push_back(longitudinal_state_to_vehicle_state(
			origin_lane_controller.get_state()));
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