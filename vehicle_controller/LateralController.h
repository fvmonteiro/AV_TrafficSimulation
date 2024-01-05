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
whether the maneuver can take place. TODO: rename*/
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

	void set_destination_lane_follower_parameters(double new_lambda_0,
		double new_lambda_1);
	void set_destination_lane_follower_time_headway(double new_time_headway);

	double compute_time_headway_gap(double ego_velocity, 
		const NearbyVehicle& nearby_vehicle, double accepted_risk) const;
	double compute_vehicle_following_gap_for_lane_change(
		const EgoVehicle& ego_vehicle, const NearbyVehicle& nearby_vehicle,
		std::pair<double, double> ego_safe_lane_changing_params,
		double accepted_risk) const;

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
	double dest_lane_follower_lambda_0{ 0.0 };
	double dest_lane_follower_lambda_1{ 0.0 };

	double standstill_distance{ 1.0 };
	std::vector<double> lane_change_lateral_acceleration;
	std::vector<double> lane_change_lateral_velocity;
	std::vector<double> lane_change_lateral_position;

	void estimate_lane_change_kinematics();
	double compute_lateral_collision_time(const EgoVehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, double longitudinal_acceleration) const;
};