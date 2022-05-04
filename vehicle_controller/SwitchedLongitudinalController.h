/*==========================================================================*/
/* SwtichedLongitudinalController.h     									*/
/* Base class for longitudinal controllers that do switching between		*/
/* gap and velocity control													*/
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <string>

#include "Constants.h"
#include "LongitudinalController.h"
#include "VariationLimitedFilter.h"
#include "VelocityController.h"

// Forward declaration
class NearbyVehicle;
class EgoVehicle;

/* This controller computes inputs for vehicle following and velocity
control. It also determine which of these inputs should be used */
class SwitchedLongitudinalController: public LongitudinalController
{
public:

	SwitchedLongitudinalController() = default;
	SwitchedLongitudinalController(
		const VelocityControllerGains& velocity_controller_gains,
		const AutonomousGains& autonomous_gains, 
		const ConnectedGains& connected_gains,
		double velocity_filter_gain, double time_headway_filter_gain,
		double filter_brake_limit, double comfortable_acceleration,
		double simulation_time_step, bool verbose);
	/*LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change, bool verbose);
	LongitudinalController(const Vehicle& ego_vehicle,
		bool is_used_for_lane_change);*/

	
	/* --------------- Methods related to velocity control ---------------- */
	
	void reset_velocity_controller(double reset_velocity);
	/* Sets integral error to zero*/
	//void reset_velocity_error_integrator();

	/* ------------------ Methods related to gap control ------------------ */

	void connect_gap_controller(bool is_connected);
	void reset_leader_velocity_filter(double reset_velocity);
	void reset_time_headway_filter(double time_headway);
	/* Returns the desired (final) time headway. */
	double get_safe_time_headway() const;
	/* Returns the current time headway in use */
	double get_current_time_headway() const;
	double get_time_headway_gap(double time_headway, double velocity);
	double get_safe_time_headway_gap(double ego_velocity,
		bool has_lane_change_intention);
	/* Computes desired gap with a possibly varying time headway */
	//double compute_desired_gap(double ego_velocity);
	double get_desired_gap(double ego_velocity);
	void set_desired_time_headway(double time_headway);

	virtual void reset_accepted_risks();
	void compute_max_risk_to_leader(bool is_lane_changing);

	/* Determines and sets the current state of the longitudinal controller */
	/*void determine_controller_state(const Vehicle& ego_vehicle,
		const NearbyVehicle& leader);*/

	//double compute_desired_acceleration(const Vehicle& ego_vehicle,
	//	const NearbyVehicle& leader);


protected:
	//State state{ State::uninitialized }; // event driven logic variable
	double hysteresis_bias{ 10.0 }; // used to avoid state chattering [m]
	//double ego_reference_velocity{ 0.0 }; // for the velocity controller [m/s]
	/* Parameters related to the emergency braking scenario */
	//double ego_max_brake{ 0.0 }; // absolute value [m/s^2]
	//double ego_max_brake_lane_change{ 0.0 }; // absolute value [m/s^2]
	//double free_flow_velocity{ 0.0 }; // to compute time headway [m/s]
	//double rho{ 0.2 }; // proportional maximum expected relative speed
	/* Desired gap parameters */
	//double h_vehicle_following{ 0.0 }; /* time headway [s] */
	//double h_lane_change{ 0.0 }; /* time headway during lane change [s]*/
	//double standstill_distance{ 1.0 }; /* standstill distance [m] */
	 /* time headway [s] */
	//double desired_time_headway{ 0.0 };

	/* Controllers - Still working on this [April 29, 22] */
	//VariationLimitedFilter leader_velocity_filter;
	//VariationLimitedFilter desired_velocity_filter;
	//VariationLimitedFilter time_headway_filter;

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

	/*double compute_safe_time_headway(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho);*/
	/*double compute_time_headway_with_risk(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho, double accepted_risk);*/

	/* Computes gap minus reference gap and upper bounds it
	with max_gap_error. */
	//double compute_gap_error(double gap, double reference_gap);

	/* Computes velocity error "typically": vLeader - vEgo*/
	//double compute_velocity_error(double velocity_ego, double velocity_leader);

	/*double estimate_gap_error_derivative(
		double velocity_error, double acceleration);*/

	/*double compute_acceleration_error(
		double acceleration_ego, double acceleration_reference);*/

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	double compute_gap_threshold(double gap, 
		double diff_to_velocity_reference, double gap_control_input);

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	/*double compute_gap_threshold(double free_flow_velocity, 
		double velocity_error);*/

	/* Computes the gap threshold to decide whether velocity or vehicle
	following control */
	//double compute_gap_threshold(double free_flow_velocity, double velocity_error,
	//	double gap_error_derivative, double acceleration_error);

	/* Constant time headway based controller */
	//double compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
	//	const NearbyVehicle& leader);
	/*double compute_vehicle_following_input(double gap_error,
		double velocity_error);
	double compute_vehicle_following_input(double gap_error,
		double velocity_error, double gap_error_derivative,
		double acceleration_error);*/


private:
	double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference) override;

	/* Determines and sets the current state of the longitudinal controller
	TODO: should this class provide a default implementation?*/
	virtual void determine_controller_state(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double reference_velocity, double gap_control_input) = 0;

	double simulation_time_step{ 0.01 };

	/* Vehicle following and velocity control gains
	(computed in Matlab. Should be computed here?) */
	AutonomousGains autonomous_gains;
	ConnectedGains connected_gains;

	/* Other controller parameters */
	double max_gap_error{ 5.0 }; // maximum positive gap error in meters
	// [Jan 31, 22] from 10 to 5
	double max_gap_error_connected{ 10.0 }; // maximum positive gap error in meters

};

