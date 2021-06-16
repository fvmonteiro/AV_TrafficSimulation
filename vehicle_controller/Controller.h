/*==========================================================================*/
/*  Controller.h    													    */
/*  Autonomous vehicle controllers                                          */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

// Forward declaration
class NearbyVehicle;
class Vehicle;

class Controller {
public:
	double h{ 1 }; /* time headway in seconds */
	double d{ 1 }; /* standstill distance in meters */

	/* Computes the bumper to bumper distance.
	Vissim's "distance" is the distance between both front bumper, so we
	subtract the leader length from that.*/
	double compute_gap(double vissim_distance, double leader_length);
	
	/* Computes constant time headway following distance */
	double compute_desired_gap(double ego_velocity);
	
	double compute_gap_error(double gap, double reference_gap);
	
	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego, double velocity_leader);

	/* Determines and sets the current state of the longitudinal controller */
	void determine_vehicle_state(Vehicle& ego_vehicle,
		const NearbyVehicle& leader);

	/* Computes the gap threshold to decide whether velocity or vehicle 
	following control */
	double compute_gap_threshold(double desired_velocity, double velocity_error);

	double compute_vehicle_following_input(double gap_error,
		double velocity_error);

	double compute_velocity_control_input(double velocity_error);

	double compute_desired_acceleration(Vehicle& ego_vehicle,
		const NearbyVehicle& leader);

	/* Computes the deceleration rate to avoid collision */
	double compute_drac(double relative_velocity, double gap);

private:
	/*All gains should be set at the begining o the simulation instead of
	being defined here. To keep access to their values, we need to create
	User Defined Variables (UDAs) in VISSIM*/
	/* Vehicle following gains */
	double kg{ 0.2 }; // gain relative to gap error.
	double kv{ 0.5 }; // gain relative to velocity error.
	/* Velocity controller gains */
	double ki{ 0.01 };
	double kp{ 0.1 };
	double kd{ 0.1 };
	/* Other controller parameters */
	double max_gap_error{ 3.0 }; // maximum positive gap error in meters
	double max_acceleration{ 3.0 }; // maximum acceleration in m/s^2. Used in
	// the velocity filter
	double min_acceleration{ -8.0 }; // minimum acceleration in m/s^2. Used in
	// the velocity filter
};