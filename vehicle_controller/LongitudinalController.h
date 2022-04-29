/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for Adaptive Cruise controller using the constant time        */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <string>

#include "Constants.h"
#include "VariationLimitedFilter.h"

// Forward declaration
class NearbyVehicle;
class EgoVehicle;

/* Vehicle parameters that do not vary with time */
struct VehicleParameters {
	VehicleType type{ VehicleType::undefined };
	double sampling_interval{ 0.0 };
	double max_brake{ 0.0 };
	double comfortable_brake{ 0.0 };
	double comfortable_acceleration{ 0.0 };
	double desired_velocity{ 0.0 };
	double lambda_1{ 0.0 };
	bool is_connected{ false };
	double lane_change_max_brake{ 0.0 };
	/* double lambda_1_lane_change{ 0.0 };
	double lambda_1_connected{ 0.0 };
	double lambda_1_lane_change_connected{ 0.0 };*/
};

struct AutonomousGains {
	double kg{ 0.0 };
	double kv{ 0.0 };
};

struct ConnectedGains {
	double kg{ 0.0 };
	double kv{ 0.0 };
	double kgd{ 0.0 };
	double ka{ 0.0 };
};

struct VelocityControllerGains {
	double kp{ 0.0 };
	double kd{ 0.0 };
	double ki{ 0.0 };
};

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
	LongitudinalController(const VehicleParameters& ego_parameters,
		VelocityControllerGains velocity_controller_gains,
		AutonomousGains autonomous_gains, ConnectedGains connected_gains,
		double filter_brake_limit, bool verbose);
	/*LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change, bool verbose);
	LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change);*/

	State get_state() const { return state; };
	
	void set_connexion(bool is_conneced) {
		this->is_connected = is_conneced;
	};
	/*void set_vehicle_following_gains(AutonomousGains gains);
	void set_vehicle_following_gains(ConnectedGains gains);
	void set_velocity_controller_gains(VelocityControllerGains gains);*/
	void set_desired_time_headway(double time_headway) {
		this->desired_time_headway = time_headway;
	};

	/* TODO: This function will likely change to return the current 
	filtered value of h*/
	//double get_veh_following_time_headway() const;
	/* Returns time headway or lane changing time headway depending
	on the vehicle intentions*/
	//double get_safe_time_headway(bool has_lane_change_intention) const;
	/* Returns the desired (final) time headway. */
	double get_safe_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;
	

	double compute_time_headway_gap(double time_headway, double velocity);
	/* Computes the time headway values with accepted risk and assigns
	this value to members h_vf and h_lc. */
	/*void update_time_headway(double lambda_1, double lambda_1_lc,
		double new_leader_max_brake);*/
	//void update_time_headway_with_new_risk(double lambda_1);
	void reset_leader_velocity_filter(double reset_velocity);
	void reset_desired_velocity_filter(double reset_velocity);
	void reset_time_headway_filter(double time_headway);
	/* Sets integral error to zero*/
	void reset_velocity_error_integrator();
	virtual void reset_accepted_risks();
	void compute_max_risk_to_leader(bool is_lane_changing);

	double compute_safe_time_headway_gap(double ego_velocity,
		bool has_lane_change_intention);

	/* Computes desired gap with a possibly varying time headway */
	double compute_desired_gap(double ego_velocity, bool has_lane_change_intention);

	/* Determines and sets the current state of the longitudinal controller */
	/*void determine_controller_state(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);*/

	double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference);
	//double compute_desired_acceleration(const Vehicle& ego_vehicle,
	//	const NearbyVehicle& leader);

	/* Printing ----------------------------------------------------------- */
	static std::string state_to_string(State state);

protected:
	State state{ State::uninitialized }; // event driven logic variable
	double hysteresis_bias{ 10.0 }; // used to avoid state chattering [m]
	//double ego_reference_velocity{ 0.0 }; // for the velocity controller [m/s]
	/* Parameters related to the emergency braking scenario */
	double ego_max_brake{ 0.0 }; // absolute value [m/s^2]
	double ego_max_brake_lane_change{ 0.0 }; // absolute value [m/s^2]
	double free_flow_velocity{ 0.0 }; // to compute time headway [m/s]
	double rho{ 0.2 }; // proportional maximum expected relative speed
	/* Desired gap parameters */
	//double h_vehicle_following{ 0.0 }; /* time headway [s] */
	//double h_lane_change{ 0.0 }; /* time headway during lane change [s]*/
	double d{ 1.0 }; /* standstill distance [m] */
	 /* time headway [s] TRYING NEW CODE ORGANIZATION WITH
    THIS PARAMETER INSTEAD OF h_xxx */
	double desired_time_headway{ 0.0 };

	VariationLimitedFilter leader_velocity_filter;
	VariationLimitedFilter desired_velocity_filter;
	VariationLimitedFilter time_headway_filter;

	bool verbose{ false };

	/* Accpeting risks:
	The risk is an estimation of the relative velocity at collision
	time under worst case scenario.
	When the ego vehicle has intention to change lanes, it starts looking
	for gaps with, at most, an initial risk value. After some time of
	unsuccessful search for acceptable gaps, the accepted risk increases
	by some value delta. This happens until a maximum accepted risk.
	*/
	double initial_risk{ 0.0 }; // [m/s]
	double constant_risk_period{ 1.0 }; // [s]
	double delta_risk{ 3.0 }; // [m/s]
	double accepted_risk_to_leader{ initial_risk }; // [m/s]
	double max_risk_to_leader{ 0.0 }; // [m/s]
	double timer_start{ 0.0 }; // [s]

	bool is_connected{ false };

	/* Determines and sets the current state of the longitudinal controller
	TODO: should this class provide a default implementation?*/
	virtual void determine_controller_state(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double reference_velocity) = 0;

	/*double compute_safe_time_headway(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho);*/
	/*double compute_time_headway_with_risk(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho, double accepted_risk);*/

	/* Computes gap minus reference gap and upper bounds it
	with max_gap_error. */
	double compute_gap_error(double gap, double reference_gap);

	/* Computes velocity error "typically": vLeader - vEgo*/
	double compute_velocity_error(double velocity_ego, double velocity_leader);

	double estimate_gap_error_derivative(
		double velocity_error, double acceleration, bool has_lane_change_intention);

	double compute_acceleration_error(
		double acceleration_ego, double acceleration_reference);

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	double compute_gap_threshold(double free_flow_velocity, double velocity_error,
		bool has_lane_change_intention);

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	double compute_gap_threshold(double free_flow_velocity, double velocity_error,
		double gap_error_derivative, double acceleration_error,
		bool has_lane_change_intention);

	/* Constant time headway based controller */
	double compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
		const NearbyVehicle& leader);
	/*double compute_vehicle_following_input(double gap_error,
		double velocity_error);
	double compute_vehicle_following_input(double gap_error,
		double velocity_error, double gap_error_derivative,
		double acceleration_error);*/

		/* PID velocity controller */
	double compute_velocity_control_input(const EgoVehicle& ego_vehicle,
		double velocity_reference);
	/*double compute_velocity_control_input(double velocity_error,
		double acceleration_error, double comfortable_acceleration);*/

private:
	/* When set to true, will assume lane changing parameters, such as
	reduced braking capability. */
	//bool is_used_for_lane_change{ false };
	double simulation_time_step{ 0.01 };

	/* Vehicle following and velocity control gains 
	(computed in Matlab. Should be computed here?) */
	AutonomousGains autonomous_gains;
	ConnectedGains connected_gains;
	VelocityControllerGains velocity_controller_gains;
	
	/* Other controller parameters */
	double max_gap_error{ 5.0 }; // maximum positive gap error in meters
	// [Jan 31, 22] from 10 to 5
	double max_gap_error_connected{ 10.0 }; // maximum positive gap error in meters
	double velocity_error_integral{ 0.0 };

	/* Internal methods */

};