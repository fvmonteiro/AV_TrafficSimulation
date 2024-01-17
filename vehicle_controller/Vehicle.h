#pragma once

#include <array>
#include <vector>

#include "Constants.h"
#include "RelativeLane.h"

//using StateVectorType = std::array<double, 4>;

struct StateVector
{
	double x{ 0.0 };  // [m]
	double y{ 0.0 };  // [m]
	double theta{ 0.0 };  // [rad]
	double v{ 0.0 };  // [m/s]
	bool is_empty{ true };

	StateVector() = default;
	StateVector(double x, double y, double theta, double v)
		: x{ x }, y{ y }, theta{ theta }, v{ v }, is_empty{ false } {};

	void offset(double off_x, double off_y)
	{
		x -= off_x;
		y -= off_y;
	};
};


class Vehicle
{
public:
	Vehicle(long id);
	Vehicle(long id, VehicleType type, double brake_delay);

	/* Getters and setters */

	double get_max_brake() const { return max_brake; };
	double get_max_jerk() const { return max_jerk; };
	long get_id() const { return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	/* angle relative to the middle of the lane [rad]
	(positive = turning left) */
	double get_orientation_angle() const{ return orientation_angle; };
	double get_lambda_1() const { return lambda_1; };
	double get_lambda_0() const { return lambda_0; };
	VehicleCategory get_category() const { return category; };
	VehicleType get_type() const { return type; };
	RelativeLane get_desired_lane_change_direction() const {
		return desired_lane_change_direction;
	};

	void set_length(double value) { length = value; };
	void set_width(double value) { width = value; };
	void set_orientation_angle(double value) { orientation_angle = value; };
	/* Also sets the estimated maximum braking of the vehicle. */
	void set_category(long category);
	
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
	double compute_risky_gap(double v_follower,
		double v_leader, double brake_follower, double brake_leader,
		double lambda_0, double lambda_1, double accepted_risk) const;
	/* Maximum possible accepted risk that keeps the time headway positive. */
	double compute_max_risk(double leader_max_brake,
		double follower_max_brake, double desired_velocity, double rho);
	bool is_a_connected_type(VehicleType vehicle_type) const;

	/* ------------------------------------------------------------ */

	void compute_safe_gap_parameters();
	
	/* The variables below are a way of describing the emergency braking, 
	but they do not correspond to how VISSIM works. VISSIM has varying 
	max brake based on speed (and no jerk). We estimate these parameters
	for safe gap computations */

	double max_brake{ 0.0 }; // [m/s^2] Absolute value
	double max_jerk{ 0.0 }; // [m/s^3] Absolute value
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
	double orientation_angle{ 0.0 }; // [rad]
	/* Parameter related to the emergency braking scenario [m] */
	double lambda_0{ 0.0 };
	/* Parameter related to the emergency braking scenario [m/s] */
	double lambda_1{ 0.0 };
};

bool operator== (const Vehicle& vehicle1, const Vehicle& vehicle2);
bool operator!= (const Vehicle& vehicle1, const Vehicle& vehicle2);