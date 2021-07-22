/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
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
	LongitudinalController(const Vehicle& ego_vehicle, double max_brake,
		double desired_velocity, double filter_brake_limit, bool verbose);
	/*LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change, bool verbose);
	LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change);*/

	State get_state() const { return state; };
	double get_h() const { return h; };
	
	/* TODO: move to destLaneController */
	void set_longitudinal_adjustment_minimum_velocity(double velocity) {
		this->longitudinal_adjustment_minimum_velocity = velocity;
	}

	double compute_time_headway_gap(double time_headway, double velocity);
	void update_time_headway(const Vehicle& ego_vehicle, 
		double leader_max_brake);
	
	/* The collision free gap is computed assuming a worst case braking
	scenario */
	double compute_exact_collision_free_gap(const Vehicle& ego_vehicle,
		const NearbyVehicle& other_vehicle);

	/* Computes constant time headway following distance */
	double compute_desired_gap(double ego_velocity);
	
	double compute_gap_error(double gap, double reference_gap);
	
	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego, double velocity_leader);

	/* Determines and sets the current state of the longitudinal controller 
	TODO: should this class provide a default implementation?*/
	virtual void determine_controller_state(double ego_velocity,
		const NearbyVehicle* leader) = 0;
	/* Determines and sets the current state of the longitudinal controller */
	/*void determine_controller_state(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);*/

	/* Computes the gap threshold to decide whether velocity or vehicle 
	following control */
	double compute_gap_threshold(double desired_velocity, double velocity_error);

	/* Constant time headway based controller */
	double compute_vehicle_following_input(double gap_error,
		double velocity_error);

	/* PID velocity controller */
	double compute_velocity_control_input(double velocity_error, 
		double ego_acceleration);

	double compute_desired_acceleration(const Vehicle& ego_vehicle,
		const NearbyVehicle* leader);
	//double compute_desired_acceleration(const Vehicle& ego_vehicle,
	//	const NearbyVehicle& leader);

protected:
	State state{ State::uninitialized }; // event driven logic variable
	double hysteresis_bias{ 5.0 }; // used to avoid state chattering [m]
	double ego_vehicle_desired_velocity{ 0.0 }; // [m/s]
	double rho{ 0.15 }; /* proportional maximum expected relative speed */
	bool verbose{ false };

	double compute_time_headway(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho);
private:
	/* When set to true, will assume lane changing parameters, such as
	reduced braking capability. */
	//bool is_used_for_lane_change{ false };
	double simulation_time_step{ 0.01 };
	/* Vehicle parameters related to the emergency braking scenario */
	double ego_vehicle_max_brake{ 0.0 }; // absolute value [m/s^2]
	double ego_vehicle_lambda_0{ 0.0 }; // [m]
	double ego_vehicle_lambda_1{ 0.0 }; // [m/s]
	/* Desired gap parameters */
	double h{ 1 }; /* time headway [s] */
	double d{ 1 }; /* standstill distance [m] */

	/*All gains should be set at the begining o the simulation instead of
	being defined here. To keep access to their values, we need to create
	User Defined Variables (UDAs) in VISSIM*/
	/* Vehicle following gains */
	double kg{ 0.2 }; // gain relative to gap error.
	double kv{ 1.0 }; // gain relative to velocity error.
	/* Velocity controller gains 
	TODO: computed in Matlab but should be computed here */
	double ki{ 0.07 };
	double kp{ 0.5 };
	double kd{ 0.1 };
	/* Other controller parameters */
	double max_gap_error{ 3.0 }; // maximum positive gap error in meters
	double velocity_error_integral{ 0.0 };
	double longitudinal_adjustment_minimum_velocity{ 0.0 };
	VelocityFilter velocity_filter;
	

	/* Internal methods */
	void compute_safe_gap_parameters(double max_jerk,
		double comfortable_acceleration, double maximum_braking,
		double brake_delay);
	
	/* Sets integral error to zero*/
	void reset_velocity_error_integrator();

	/* For printing and debugging */
	std::string state_to_string(State state);
};