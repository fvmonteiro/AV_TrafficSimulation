/*==========================================================================*/
/*  LateralController.cpp  													*/
/*  Class implementation.											        */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#define _USE_MATH_DEFINES
#include <math.h>

#include "LateralController.h"
#include "LongitudinalController.h"
#include "NearbyVehicle.h"
#include "Vehicle.h"

/* TODO: NOTHING HERE HAS BEEN TESTED!!!*/
LateralController::LateralController(bool verbose) {
	if (verbose) {
		std::clog << "Creating lateral controller" << std::endl;
	}
	this->verbose = verbose;
	estimate_lane_change_kinematics();
}

LateralController::LateralController() : LateralController(false) {}

//double LateralController::compute_gap_to_nearby_vehicle(
//	const NearbyVehicle& other_vehicle) {
//
//	bool is_other_in_front =
//		other_vehicle.get_current_relative_position() > 0;
//	double length;
//	if (is_other_in_front) {
//		length = other_vehicle.get_current_length();
//	}
//	else {
//		length = ego_vehicle->get_length();
//	}
//	return other_vehicle.get_current_distance() - length;
//}

//std::vector<double> LateralController::compute_safe_lane_change_gap_all_info(
//	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
//	bool will_accelerate) {
//
//	double collision_free_gap = compute_collision_free_gap(
//		ego_vehicle, other_vehicle);
//	double transient_gap = compute_transient_gap(ego_vehicle,
//		other_vehicle, will_accelerate);
//	
//	std::vector<double> gaps { 
//		collision_free_gap + transient_gap,
//		collision_free_gap,
//		transient_gap };
//
//	return gaps;
//}

double LateralController::compute_safe_lane_change_gap(
	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	bool will_accelerate) {

	double collision_free_gap = compute_collision_free_gap(
		ego_vehicle, other_vehicle);
	double transient_gap = compute_transient_gap(ego_vehicle,
		other_vehicle, will_accelerate);

	return collision_free_gap + transient_gap;
}

double LateralController::compute_collision_free_gap(
	const Vehicle& ego_vehicle,const NearbyVehicle& other_vehicle) {

	double lambda_0, lambda_1;
	double v_follower, delta_v, v_leader;
	double brake_follower, brake_leader;
	if (other_vehicle.is_ahead()) {
		lambda_0 = ego_vehicle.get_lambda_0();
		lambda_1 = ego_vehicle.get_lambda_1();
		v_follower = ego_vehicle.get_current_velocity();
		delta_v = other_vehicle.get_current_relative_velocity();
		v_leader = v_follower - delta_v;
		brake_follower = ego_vehicle.get_max_brake();
		brake_leader = other_vehicle.get_max_brake();
	}
	else {
		lambda_0 = other_vehicle.get_lambda_0();
		lambda_1 = other_vehicle.get_lambda_1();
		v_leader = ego_vehicle.get_current_velocity();
		delta_v = other_vehicle.get_current_relative_velocity();
		v_follower = v_leader - delta_v;
		brake_follower = other_vehicle.get_max_brake();
		brake_leader = ego_vehicle.get_max_brake();
	}

	double stop_time_follower = v_follower + lambda_1 / brake_follower;
	double stop_time_leader = v_leader / brake_leader;
	double collision_free_gap;
	if (stop_time_follower >= stop_time_leader) {
		collision_free_gap = 
			std::pow(v_follower + lambda_1, 2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader + lambda_0;
	}
	else if (brake_leader < brake_follower) {
		collision_free_gap = 
			std::pow(delta_v - lambda_1, 2) / 2 / (brake_follower - brake_leader)
			+ lambda_0;
	}
	else {
		collision_free_gap = 0;
	}

	return collision_free_gap;
}

double LateralController::compute_transient_gap(const Vehicle& ego_vehicle,
	const NearbyVehicle& other_vehicle, bool will_accelerate) {
	/* We need to solve:
	max_{t \in T} [(v_E(t_0) - v_L(t_0))(t - t_0) + 1/2 a_E (t-t_0)^2]

	If there's no acceleration, we can find the transient gap in constant
	time, based on whether v_E(t_0) greater or lesser than v_L(t_0). */

	double longitudinal_acceleration;
	double relative_velocity = other_vehicle.get_current_relative_velocity();
	if (will_accelerate) {
		double comfortable_acceleration = 
			ego_vehicle.get_comfortable_acceleration();
		longitudinal_acceleration = -relative_velocity / lane_change_duration;
		longitudinal_acceleration = std::min(std::max(longitudinal_acceleration, 
											          -comfortable_acceleration), 
											 comfortable_acceleration);
	}
	else {
		longitudinal_acceleration = 0.0;
	}

	double t_0;
	double t_f;
	if (other_vehicle.is_on_same_lane()) {
		t_0 = 0;
		t_f = compute_lateral_collision_time(ego_vehicle,
			other_vehicle, longitudinal_acceleration);
	}
	else {
		t_0 = compute_lateral_collision_time(ego_vehicle,
			other_vehicle, longitudinal_acceleration);
		t_f = lane_change_duration;
	}

	if (!other_vehicle.is_ahead()) {
		relative_velocity *= -1;
		longitudinal_acceleration *= -1;
	}

	double t_max;
	double transient_gap;
	if ((relative_velocity >= 0) && (longitudinal_acceleration >= 0)) {
		t_max = t_f;
	}
	else if ((relative_velocity <= 0) && (longitudinal_acceleration <= 0)) {
		t_max = t_0;
	}
	else {
		t_max = - relative_velocity / longitudinal_acceleration;
		if (t_max > t_f) t_max = t_f;
	}
	transient_gap = relative_velocity * t_max
		+ longitudinal_acceleration / 2 * t_max * t_max;
	
	return transient_gap;
}

double LateralController::compute_lateral_collision_time(
	const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	double longitudinal_acceleration) {
	/* lane_change_lateral_position is the estimated lateral position 
	of the vehicle's center front. We assume all lane changes are left lane 
	changes to compute the collision time, which is independent of the lane 
	change direction. 
	In what follows, we compute the lateral coordiantes of the vehicle 
	rear left, front left and front right because those are the relevant 
	points related to vehicles future follower, future leader and current
	leader respectively. 
	Derivation of the formula and further explanation is in Jula H. et al, 
	Collision avoidance analysis for lane changing and merging, 2000. */

	double longitudinal_velocity = ego_vehicle.get_current_velocity();
	double length = ego_vehicle.get_length();
	double width = ego_vehicle.get_width();
	double other_width = other_vehicle.get_current_width();
	std::vector<double> y_ego (lane_change_lateral_position.size());

	bool collision_time_found = false;
	int i = 0;
	while ((!collision_time_found) && (i < y_ego.size())) {
		y_ego[i] = lane_change_lateral_acceleration[i];
		if (!other_vehicle.is_ahead()) { // compute rear coordinate
			double sin_theta = lane_change_lateral_velocity[i]
				/ std::sqrt(std::pow(lane_change_lateral_velocity[i], 2) 
							+ std::pow(longitudinal_velocity, 2));
			y_ego[i] -= (length * sin_theta);
		}
		double cos_theta = longitudinal_velocity
				/ std::sqrt(std::pow(lane_change_lateral_velocity[i], 2)
							+ std::pow(longitudinal_velocity, 2));
		if (other_vehicle.is_on_same_lane()) { // compute right coordinate
			y_ego[i] -= width / 2 * cos_theta;
			if (y_ego[i] >= other_width / 2) {
				collision_time_found = true;
			}
		}
		else { // compute left coordinate
			y_ego[i] += width / 2 * cos_theta;
			if (y_ego[i] >= lane_width - other_width / 2) {
				collision_time_found = true;
			}
		}
		longitudinal_velocity += longitudinal_acceleration * sampling_time;
		i++;
	}

	i--;
	return i*sampling_time;
}

void LateralController::estimate_lane_change_kinematics() {
	double time = 0.0;
	lane_change_lateral_acceleration.push_back(0.0);
	lane_change_lateral_velocity.push_back(0.0);
	lane_change_lateral_position.push_back(0.0);

	time += sampling_time;
	while (time < lane_change_duration) {
		lane_change_lateral_acceleration.push_back(
			2 * M_PI * lane_width / (lane_change_duration*lane_change_duration)
			* std::sin(2*M_PI*time/lane_change_duration)
		);
		lane_change_lateral_velocity.push_back(
			lane_change_lateral_velocity.front() 
			+ lane_change_lateral_acceleration.front() * sampling_time
		);
		lane_change_lateral_position.push_back(
			lane_change_lateral_position.front() 
			+ lane_change_lateral_velocity.front() * sampling_time
		);
		time += sampling_time;
	}
}