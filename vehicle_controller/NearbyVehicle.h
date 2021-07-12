/*==========================================================================*/
/*  NearbyVehicle.h	    												    */
/*  Class to help manage neighboring vehicles								*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <iostream>

#include "Constants.h"

class NearbyVehicle {
public:
	enum class RelativeLane {
		right_right = -2, // second to the right
		right, // next to the right
		same, 
		left, // next to the left
		left_left, // second to the left
	};

	const std::vector<long>& get_id() const { return id; };
	const std::vector<double>& get_length() const { return length; };
	const std::vector<double>& get_width() const { return width; };
	const std::vector<VehicleCategory>& get_category() const { 
		return category;
	};
	const std::vector<RelativeLane>& get_relative_lane() const { 
		return relative_lane;
	};
	const std::vector<long>& get_relative_position() const { 
		return relative_position;
	};
	const std::vector<double>& get_distance() const { return distance; };
	const std::vector<double>& get_relative_velocity() const { 
		return relative_velocity;
	};
	const std::vector<double>& get_acceleration() const { 
		return acceleration;
	};

	double get_max_brake() const { return max_brake; };
	double get_lambda_0() const { return lambda_0; };
	double get_lambda_1() const { return lambda_1; };
	long get_current_id() const { return id.back(); };
	double get_current_length() const { return length.back(); };
	double get_current_width() const { return width.back(); };
	VehicleCategory get_current_category() const { return category.back(); };
	RelativeLane get_current_relative_lane() const { 
		return relative_lane.back(); 
	};
	long get_current_relative_position() const {
		return relative_position.back();
	};
	double get_current_distance() const { return distance.back(); };
	double get_current_relative_velocity() const {
		return relative_velocity.back();
	};
	double get_current_acceleration() const { return acceleration.back(); };

	void set_id(long id); // set_id checks if the nearby vehicle has changed
	void set_length(double length) { this->length.push_back(length); };
	void set_width(double width) { this->width.push_back(width); };
	void set_category(VehicleCategory category) {
		this->category.push_back(category);
	};
	void set_category(long category) { 
		this->category.push_back(VehicleCategory(category));
	};
	void set_relative_lane(RelativeLane relative_lane) {
		this->relative_lane.push_back(relative_lane);
	}
	void set_relative_lane(long relative_lane) { 
		this->relative_lane.push_back(static_cast<RelativeLane>(relative_lane));
	};
	void set_relative_position(long relative_position) {
		this->relative_position.push_back(relative_position);
	};
	void set_distance(double distance) {
		this->distance.push_back(distance);
	};
	void set_relative_velocity(double relative_velocity) {
		this->relative_velocity.push_back(relative_velocity);
	};
	void set_acceleration(double acceleration) {
		this->acceleration.push_back(acceleration);
	};

	bool is_on_same_lane() const;
	bool is_ahead() const;
	void fill_with_dummy_values();
	void copy_current_states(NearbyVehicle& nearby_vehicle);
	void compute_safe_gap_parameters();
	friend std::ostream& operator<< (std::ostream& out, const NearbyVehicle& vehicle);

private:
	/* Estimated parameter used for safe gap computations (no direct equivalent
	in VISSIM's simulation dynamics) */
	/* TODO : parameter below should vary based on vehicle category */
	double max_brake{ 6.5 }; // [m/s^2]
	double lambda_0;
	double lambda_1;
	
	std::vector<long> id;
	std::vector<double> length; // [m]
	std::vector<double> width; // [m]
	std::vector<VehicleCategory> category;
	std::vector<RelativeLane> relative_lane;
	std::vector<long> relative_position; /* 
	positive = downstream (+1 next, +2 second next)
	negative = upstream (-1 next, -2 second next)
	It's possible to get more vehicles if DRIVER_DATA_WANTS_ALL_NVEHS 
	in DriverModel.cpp  is set to 1 */
	std::vector<double> distance; // front end to front end [m]
	std::vector<double> relative_velocity; // ego speed - other speed [m/s]
	std::vector<double> acceleration; // [m/s^2]
};
