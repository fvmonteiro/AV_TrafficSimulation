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

ControlManager::ControlManager(const Vehicle& ego_vehicle, bool verbose) 
	: origin_lane_controller{ OriginLaneLongitudinalController(
		ego_vehicle, verbose) },
	end_of_lane_controller{ OriginLaneLongitudinalController(
		ego_vehicle, verbose) },
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose } {

	if (verbose) {
		std::clog << "Creating control manager " << std::endl;
	}

}

ControlManager::ControlManager(const Vehicle& ego_vehicle) 
	: ControlManager(ego_vehicle, false) {}

LongitudinalController::State 
	ControlManager::get_longitudinal_controller_state() {
	/*if (states.empty()) {
		return State::velocity_control;
	}
	return states.back(); */
	switch (active_longitudinal_controller) {
	case ActiveLongitudinalController::origin_lane:
		return origin_lane_controller.get_state();
	case ActiveLongitudinalController::destination_lane:
		return destination_lane_controller.get_state();
	case ActiveLongitudinalController::end_of_lane:
		return end_of_lane_controller.get_state();
	case ActiveLongitudinalController::vissim:
		return LongitudinalController::State::uninitialized;
	default:
		return LongitudinalController::State::uninitialized;
	}
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

void ControlManager::update_origin_lane_time_headway(
	const Vehicle& ego_vehicle, const NearbyVehicle& leader) {
	origin_lane_controller.update_time_headway(ego_vehicle,
		leader.get_max_brake());
}

void ControlManager::update_destination_lane_time_headway(
	const Vehicle& ego_vehicle, const NearbyVehicle& leader) {
	destination_lane_controller.update_time_headway(ego_vehicle,
		leader.get_max_brake());
}

void ControlManager::estimate_follower_time_headway(
	const Vehicle& ego_vehicle, const NearbyVehicle& follower) {
	double ego_max_brake = ego_vehicle.get_max_brake();
	double ego_desired_velocity = ego_vehicle.get_desired_velocity();
	destination_lane_follower_time_headway = 
		destination_lane_controller.estimate_follower_time_headway(
		follower, ego_max_brake, ego_desired_velocity);
}

double ControlManager::determine_desired_acceleration(const Vehicle& ego_vehicle) {

	double desired_acceleration;
	double max_acceleration = ego_vehicle.get_comfortable_acceleration() * 2;
	double desired_acceleration_dest_lane = max_acceleration;
	double desired_acceleration_end_of_lane = max_acceleration;
	//double drac;
	double desired_acceleration_origin_lane = 
		origin_lane_controller.compute_desired_acceleration(
			ego_vehicle, ego_vehicle.get_leader());

	if (ego_vehicle.has_lane_change_intention()) {
		if (ego_vehicle.get_use_internal_lane_change_decision()) {
			
			std::vector<double> possible_accelerations{ 
				desired_acceleration_origin_lane };
			
			/* Create the destination lane controller when the ego
			vehicle first shows its intention to change lanes. */
			if (ego_vehicle.get_previous_state() 
				!= ego_vehicle.get_current_state()
				/*(states.empty())
				|| states.back() != State::intention_to_change_lane*/) {
				destination_lane_controller = 
					DestinationLaneLongitudinalController(
						ego_vehicle, verbose);
			}

			/* Longitudinal control to adjust to destination lane leader */
			desired_acceleration_dest_lane =
				destination_lane_controller.compute_desired_acceleration(
					ego_vehicle, ego_vehicle.get_destination_lane_leader());
			/* TODO: other option is to check whether dest lane leader exists
			before calling the above method */
			if (destination_lane_controller.is_active()) {
				possible_accelerations.push_back(
					desired_acceleration_dest_lane);
			}

			/* Longitudinal control to wait at lane's end while waiting
			for appropriate lane change gap. Without this, vehicles might 
			miss a desired exit. */
			if ((ego_vehicle.get_current_active_lane_change() == 0)
				&& (ego_vehicle.get_current_lane_end_distance() > 0)) {
				/* For now, we simulate a stopped vehicle at the end of 
				the lane to force the vehicle to stop before the end of
				the lane. */
				NearbyVehicle virtual_vehicle(1, RelativeLane::same, 1);
				virtual_vehicle.set_relative_velocity(
					ego_vehicle.get_current_velocity());
				virtual_vehicle.set_distance(
					ego_vehicle.get_current_lane_end_distance());
				virtual_vehicle.set_length(0.0);
				desired_acceleration_end_of_lane =
					end_of_lane_controller.compute_desired_acceleration(
						ego_vehicle, &virtual_vehicle);
				possible_accelerations.push_back(
					desired_acceleration_end_of_lane);
			}

			/* Define the actual input */
			desired_acceleration = possible_accelerations[0];
			active_longitudinal_controller = 
				ActiveLongitudinalController::origin_lane;
			for (int i = 1; i < possible_accelerations.size(); i++) {
				if (possible_accelerations[i] < desired_acceleration) {
					desired_acceleration = possible_accelerations[i];
					if (i == 1) { 
						active_longitudinal_controller = 
							ActiveLongitudinalController::destination_lane;
					}
					else if (i == 2) {
						active_longitudinal_controller = 
							ActiveLongitudinalController::end_of_lane;
					}
				}
			}
		}
		else {
			desired_acceleration = 
				ego_vehicle.get_current_vissim_acceleration();
			active_longitudinal_controller = 
				ActiveLongitudinalController::vissim;
		}

		//states.push_back(State::intention_to_change_lane);
	}
	else { // ACC
		desired_acceleration = desired_acceleration_origin_lane;
		active_longitudinal_controller = 
			ActiveLongitudinalController::origin_lane;
		/*states.push_back(longitudinal_state_to_vehicle_state(
			origin_lane_controller.get_state()));*/
	}
	
	return desired_acceleration;
}

double ControlManager::compute_safe_lane_change_gap(
	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	bool will_accelerate) {

	double time_headway_gap = compute_time_headway_gap(ego_vehicle, 
		other_vehicle);
	double transient_gap = lateral_controller.compute_transient_gap(
		ego_vehicle, other_vehicle, will_accelerate);

	return time_headway_gap + transient_gap;
}

double ControlManager::compute_time_headway_gap(const Vehicle& ego_vehicle,
	const NearbyVehicle& other_vehicle) {
	double time_headway_gap = 0.0;
	double ego_velocity = ego_vehicle.get_current_velocity();

	if (other_vehicle.is_ahead()) {
		if (other_vehicle.get_relative_lane() == RelativeLane::same) {
			time_headway_gap = origin_lane_controller.compute_desired_gap(
				ego_velocity);
		}
		else {
			time_headway_gap = destination_lane_controller.compute_desired_gap(
				ego_velocity);
		}
	}
	else {
		time_headway_gap =
			destination_lane_controller.compute_time_headway_gap(
				destination_lane_follower_time_headway,
				other_vehicle.compute_velocity(ego_velocity));
	}

	return time_headway_gap;
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