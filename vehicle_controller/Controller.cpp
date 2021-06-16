/*==========================================================================*/
/*  Controller.cpp    													    */
/*  Autonomous vehicle controllers                                          */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <iostream>
#include "Controller.h"
#include "Vehicle.h"

double Controller::compute_desired_gap(double velocity_ego) { 
	return h * velocity_ego + d;
}

double Controller::compute_gap(double vissim_distance, double leader_length) {
	return vissim_distance - leader_length;
};

double Controller::compute_gap_error(double gap, double reference_gap) {
	return std::min(max_gap_error, gap - reference_gap);
};

double Controller::compute_velocity_error(double velocity_ego, 
	double velocity_leader) { 
	return velocity_leader - velocity_ego;
};

void Controller::determine_vehicle_state(
	Vehicle& ego_vehicle, const NearbyVehicle& leader) {

	if (ego_vehicle.get_current_preferred_relative_lane() != 0) {
		ego_vehicle.set_vehicle_state(
			Vehicle::VehicleStates::intention_to_change_lane);
	}
	else if (leader.get_current_id() <= 0) { // no vehicle ahead
		ego_vehicle.set_vehicle_state(
			Vehicle::VehicleStates::velocity_control);
	}
	else {
		double ego_velocity = ego_vehicle.get_current_velocity();
		double leader_velocity = ego_vehicle.get_current_velocity()
			- leader.get_current_relative_velocity();
		double gap_threshold = compute_gap_threshold(
			ego_vehicle.get_desired_velocity(), compute_velocity_error(
				ego_velocity, leader_velocity));
		if ((leader.get_current_distance() < gap_threshold)
			&& (leader_velocity < ego_vehicle.get_desired_velocity())) {
			ego_vehicle.set_vehicle_state(
				Vehicle::VehicleStates::vehicle_following);
		}
		else {
			ego_vehicle.set_vehicle_state(
				Vehicle::VehicleStates::velocity_control);
		}
	}
}

double Controller::compute_gap_threshold(double desired_velocity, 
	double velocity_error) {
	return h * desired_velocity + d - 1 / kg * (kv * velocity_error);
	//return 250;
}

double Controller::compute_vehicle_following_input(double gap_error,
	double velocity_error) {
	return kg * gap_error + kv * velocity_error;
}

double Controller::compute_velocity_control_input(double velocity_error) {
	return kp * velocity_error;
}

double Controller::compute_desired_acceleration(Vehicle& ego_vehicle,
	const NearbyVehicle& leader) {
	
	double gap;
	double reference_gap;
	double gap_error;
	double velocity_error;
	double desired_acceleration;
	double drac; 
	double ego_velocity = ego_vehicle.get_current_velocity();
	double leader_velocity = ego_velocity
		- leader.get_current_relative_velocity();
	
	determine_vehicle_state(ego_vehicle, leader);
	/* Logic: if the vehicle wants to change lanes, we let VISSIM take care
	of it. Otherwise, we apply the ACC. */
	switch (ego_vehicle.get_current_vehicle_state())
	{
	case Vehicle::VehicleStates::vehicle_following:
		gap = compute_gap(leader.get_current_distance(), 
			leader.get_current_length());
		reference_gap = compute_desired_gap(ego_velocity);
		gap_error = compute_gap_error(gap, reference_gap);
		velocity_error = compute_velocity_error(
			ego_velocity, leader_velocity);
		desired_acceleration = compute_vehicle_following_input(
			gap_error, velocity_error);
		drac = compute_drac(leader.get_current_relative_velocity(), gap);
		if (desired_acceleration > -drac) {
			desired_acceleration = drac;
		}
		break;
	case Vehicle::VehicleStates::velocity_control:
		desired_acceleration = compute_velocity_error(
			ego_velocity, ego_vehicle.get_desired_velocity());
		desired_acceleration = compute_velocity_control_input(velocity_error);
		break;
	case Vehicle::VehicleStates::intention_to_change_lane:
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
		break;
	default:
		std::cerr << "Unknown vehicle state for vehicle:" << std::endl;
		std::cerr << ego_vehicle << std::endl;
		desired_acceleration = ego_vehicle.get_current_vissim_acceleration();
		break;
	}
	return desired_acceleration;

}

double Controller::compute_drac(double relative_velocity, double gap) {
	/* Deceleration (absolute value) to avoid collision assuming constant 
	velocities and instantaneous acceleration. DRAC is only defined when the
	ego vehicle is faster than the leader. We some high negative value 
	otherwise */
	if (relative_velocity > 0) { // ego faster than leader
		return relative_velocity * relative_velocity / 2 / gap;
	}
	return -100;
}