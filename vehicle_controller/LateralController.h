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
class Vehicle;

/* Given lane change intention existis, the lateral controller defines 
whether the maneuver can can place. 
TODO: I would like the controller to know the address of the vehicle using 
it, but this didn't seem to work. The address of each vehicle at construction 
time seems to change later on.*/
class LateralController {
public:

	LateralController(bool verbose);
	LateralController();

	//double compute_gap_to_nearby_vehicle(const NearbyVehicle& other_vehicle);
	/* The safe lane change gap is the sum of the vehicle following 
	collision free gap and the transient gap */
	double compute_safe_lane_change_gap(const Vehicle& ego_vehicle,
		const NearbyVehicle& other_vehicle, bool will_accelerate);
	/* Same as compute_safe_lane_change_gap but returns a vector with 
	the safe gap, the vehicle following gap and the transient gap. 
	Temporary function for debugging. */
	/*std::vector<double> compute_safe_lane_change_gap_all_info(
		const Vehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
		bool will_accelerate);*/

	/* The collision free gap is computed assuming a worst case braking
	scenario */
	double compute_collision_free_gap(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle);
	/* Computes the gap variation due to non-zero relative velocities during
	the lane change. */
	double compute_transient_gap(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, bool will_accelerate);

private:
	bool verbose;
	/* Lane change simplified kinematics. We assume a sinusoidal
	lateral acceleration profile to compute the transient lane change
	gap.
	TODO: could we compute the trajectory only once for all vehicles? */
	double sampling_time = 0.01; // [s] can be different from VISSIM
	double lane_change_duration = 5.0; // [s]
	double lane_width = 3.6; // [m]
	std::vector<double> lane_change_lateral_acceleration;
	std::vector<double> lane_change_lateral_velocity;
	std::vector<double> lane_change_lateral_position;

	void estimate_lane_change_kinematics();
	double compute_lateral_collision_time(const Vehicle& ego_vehicle, 
		const NearbyVehicle& other_vehicle, double longitudinal_acceleration);
};