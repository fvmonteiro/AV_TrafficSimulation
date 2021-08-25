#pragma once

#include <vector>

#include"Constants.h"

class Vehicle
{
public:

	/* Getters and setters */

	double get_max_brake() const { return max_brake; };
	long get_id() const { return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	VehicleCategory get_category() const { return category; };
	VehicleType get_type() const { return type; }

	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	/* Setting the category also starts the computation and
	initialization of all the values that depend on the category. */
	void set_category(long category) {
		set_category(VehicleCategory(category));
	};
	void set_type(VehicleType type);
	void set_type(long type);
	
	/* Virtual methods */

	/* Setting the category also starts the computation and
	initialization of all the values that depend on the category. */
	virtual void set_category(VehicleCategory category) = 0;
	virtual void compute_safe_gap_parameters() = 0;
	/*TODO: Maybe this method could belong to the parent class*/
	virtual bool is_lane_changing() const = 0;

protected:
	/* Max brake is an abstraction since VISSIM has varying max brake
	based on speed. We estimate this parameter for safe gap
	computations [m/s^2] */
	double max_brake{ CAR_MAX_BRAKE };

	/* Parameters read from VISSIM which are constant for each vehicle */
	long id{ 0 };
	double length{ 0.0 }; // [m]
	double width{ 0.0 }; // [m]
	VehicleCategory category{ VehicleCategory::undefined };
	VehicleType type{ VehicleType::undefined };

	/* Parameter related to the emergency braking scenario [m] */
	double lambda_0{ 0.0 };
	/* Parameter related to the emergency braking scenario [m/s] */
	double lambda_1{ 0.0 };

	double compute_lambda_0(double max_jerk,
		double comfortable_acceleration, double max_brake,
		double brake_delay);

	double compute_lambda_1(double max_jerk,
		double comfortable_acceleration, double max_brake,
		double brake_delay);
};

