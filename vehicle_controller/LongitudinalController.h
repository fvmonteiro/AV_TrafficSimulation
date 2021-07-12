/*==========================================================================*/
/*  LongitudinalController.h    											*/
/*  Adaptive Cruise controller using the constant time headway policy       */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <string>
#include "VelocityFilter.h"

// Forward declaration
class NearbyVehicle;
class Vehicle;

/* This controller computes inputs for vehicle following and velocity 
control. It also determine which of these inputs should be used */
class LongitudinalController {
public:

	enum class State {
		uninitialized,
		velocity_control,
		vehicle_following,
	};

	LongitudinalController() = default;
	LongitudinalController(const Vehicle& ego_vehicle, bool verbose);
	LongitudinalController(const Vehicle& ego_vehicle);

	State get_state() { return state; };

	/* Computes constant time headway following distance */
	double compute_desired_gap(double ego_velocity);
	
	double compute_gap_error(double gap, double reference_gap);
	
	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego, double velocity_leader);

	/* Determines and sets the current state of the longitudinal controller */
	void set_controller_state(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);

	/* Computes the gap threshold to decide whether velocity or vehicle 
	following control */
	double compute_gap_threshold(double desired_velocity, double velocity_error);

	double compute_vehicle_following_input(double gap_error,
		double velocity_error);

	/* PID velocity controller */
	double compute_velocity_control_input(double velocity_error, 
		double ego_acceleration);

	double compute_desired_acceleration(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);

private:
	bool verbose = false;
	double simulation_time_step{ 0.01 };
	/* Desired gap parameters */
	double h{ 1 }; /* time headway [s] */
	double d{ 1 }; /* standstill distance [m] */

	/*All gains should be set at the begining o the simulation instead of
	being defined here. To keep access to their values, we need to create
	User Defined Variables (UDAs) in VISSIM*/
	/* Vehicle following gains */
	double kg{ 0.2 }; // gain relative to gap error.
	double kv{ 1 }; // gain relative to velocity error.
	/* Velocity controller gains 
	TODO: computed in Matlab but should be computed here */
	double ki{ 0.07 };
	double kp{ 0.5 };
	double kd{ 0.1 };
	/* Other controller parameters */
	double max_gap_error{ 3.0 }; // maximum positive gap error in meters
	double velocity_error_integral{ 0.0 };
	double hysteresis_bias{ 5.0 }; // used to avoid state chattering [m]
	
	VelocityFilter velocity_filter;

	/* Event driven logic variable */
	State state{ State::uninitialized };

	/* Sets integral error to zero*/
	void reset_velocity_error_integrator();
	std::string state_to_string(State state);

};