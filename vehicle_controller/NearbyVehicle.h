/*==========================================================================*/
/*  NearbyVehicle.h	    												    */
/*  Class to help manage neighboring vehicles								*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

//#include <iostream>

class NearbyVehicle {
public:
	
	const std::vector<long>& get_id() const { return id; };
	const std::vector<double>& get_length() const { return length; };
	const std::vector<double>& get_width() const { return width; };
	const std::vector<long>& get_category() const { return category; };
	const std::vector<long>& get_relative_lane() const { 
		return relative_lane; };
	const std::vector<long>& get_relative_position() const { 
		return relative_position; };
	const std::vector<double>& get_distance() const { return distance; };
	const std::vector<double>& get_relative_velocity() const { 
		return relative_velocity; };
	const std::vector<double>& get_acceleration() const { 
		return acceleration; };

	long get_current_id() const { return id.back(); };
	double get_current_length() const { return length.back(); };
	double get_current_width() const { return width.back(); };
	long get_current_category() const { return category.back(); };
	long get_current_relative_lane() const { return relative_lane.back(); };
	long get_current_relative_position() const {
		return relative_position.back();
	};
	double get_current_distance() const { return distance.back(); };
	double get_current_relative_velocity() const {
		return relative_velocity.back();
	};
	double get_current_acceleration() const { return acceleration.back(); };

	void set_id(long id) { this->id.push_back(id); };
	void set_length(double length) { this->length.push_back(length); };
	void set_width(double width) { this->width.push_back(width); };
	void set_category(long category) { this->category.push_back(category); };
	void set_relative_lane(long relative_lane) { 
		this->relative_lane.push_back(relative_lane); };
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

	void zero_fill();
	void copy_current_states(NearbyVehicle& nearby_vehicle);
	friend std::ostream& operator<< (std::ostream& out, const NearbyVehicle& vehicle);

private:
	std::vector<long> id;
	std::vector<double> length;
	std::vector<double> width;
	std::vector<long> category;
	std::vector<long> relative_lane;
	std::vector<long> relative_position;
	std::vector<double> distance;
	std::vector<double> relative_velocity;
	std::vector<double> acceleration;
};
