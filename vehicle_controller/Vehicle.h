#pragma once

#include <vector>

#include "Constants.h"
#include "RelativeLane.h"

class Vehicle
{
public:
	Vehicle(long id);
	Vehicle(long id, VehicleType type, double brake_delay);

	/* Getters and setters */

	double get_max_brake() const { return max_brake; };
	long get_id() const { return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	double get_lambda_1() const { return lambda_1; };
	double get_lambda_0() const { return lambda_0; };
	VehicleCategory get_category() const { return category; };
	VehicleType get_type() const { return type; };
	RelativeLane get_desired_lane_change_direction() const {
		return desired_lane_change_direction;
	};

	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	/* Also sets the estimated maximum braking of the vehicle. */
	void set_category(long category);
	
	/*bool is_connected() const;*/
	bool has_lane_change_intention() const;

protected:
	virtual ~Vehicle() {};

	/* TODO: should the functions below become part of a namespace? */
	double compute_lambda_0(double max_jerk,
		double comfortable_acceleration, double max_brake,
		double brake_delay) const;
	double compute_lambda_1(double max_jerk,
		double comfortable_acceleration, double max_brake,
		double brake_delay) const;
	double compute_time_headway_with_risk(double free_flow_velocity,
		double follower_max_brake, double leader_max_brake,
		double lambda_1, double rho, double accepted_risk) const;
	/* Maximum possible accepted risk that keeps the time headway positive. */
	double compute_max_risk(double leader_max_brake,
		double follower_max_brake, double desired_velocity, double rho);

	void compute_safe_gap_parameters();
	
	/* The variables below are a way of describing the emergency braking, 
	but they do not correspond to how VISSIM works. VISSIM has varying 
	max brake based on speed (and no jerk). We estimate these parameters
	for safe gap computations */

	double max_brake{ 0.0 }; // [m/s^2]
	double max_jerk{ 0.0 }; // [m/s^3]
	double brake_delay{ 0.0 }; // [s]
	double comfortable_acceleration{ COMFORTABLE_ACCELERATION }; // [m/s^2]

	VehicleCategory category{ VehicleCategory::undefined };
	VehicleType type{ VehicleType::undefined };

	RelativeLane desired_lane_change_direction{ RelativeLane::same };

	/* Desired time headway */
	//double h{ 0.0 };

private:
	virtual bool is_lane_changing() const = 0;

	long id;
	double length{ 0.0 }; // [m]
	double width{ 0.0 }; // [m]
	/* Parameter related to the emergency braking scenario [m] */
	double lambda_0{ 0.0 };
	/* Parameter related to the emergency braking scenario [m/s] */
	double lambda_1{ 0.0 };
};

