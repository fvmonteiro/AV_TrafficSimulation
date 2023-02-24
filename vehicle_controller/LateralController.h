/*==========================================================================*/
/*  LateralController.h    													*/
/*  Checks neighboring gaps and determines whether they're safe.	        */
/*  Assumes lane change intention comes from higher level algorithm.		*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <vector>

// Forward declaration
class NearbyVehicle;
class EgoVehicle;

/* Given lane change intention existis, the lateral controller defines 
whether the maneuver can can place. TODO: rename*/
class LateralController {
public:

	LateralController(bool verbose);
	LateralController();

	void set_time_headway_to_leader(double value) {
		time_headway_to_leader = value;
	}
	void set_time_headway_to_destination_lane_leader(double value) {
		time_headway_to_destination_lane_leader = value;
	}
	void set_time_headway_to_destination_lane_follower(double value) {
		time_headway_to_destination_lane_follower = value;
	}
	
	double compute_time_headway_gap(double ego_velocity, 
		const NearbyVehicle& nearby_vehicle);
	
	/* Computes the gap variation due to non-zero relative velocities during
	the lane change. */
	double compute_transient_gap(const EgoVehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, bool will_accelerate) const;

private:
	bool verbose;
	/* Lane change simplified kinematics. We assume a sinusoidal
	lateral acceleration profile to compute the transient lane change
	gap.
	TODO: could we compute the trajectory only once for all vehicles? */
	double sampling_time{ 0.01 }; // [s] can be different from VISSIM
	double lane_change_duration{ 5.0 }; // [s]
	double lane_width{ 3.6  }; // [m]
	double time_headway_to_leader{ 0.0 }; // [s]
	double time_headway_to_destination_lane_leader{ 0.0 }; // [s]
	double time_headway_to_destination_lane_follower{ 0.0 }; // [s]
	double standstill_distance{ 1.0 };
	std::vector<double> lane_change_lateral_acceleration;
	std::vector<double> lane_change_lateral_velocity;
	std::vector<double> lane_change_lateral_position;

	void estimate_lane_change_kinematics();
	double compute_lateral_collision_time(const EgoVehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, double longitudinal_acceleration) const;
};