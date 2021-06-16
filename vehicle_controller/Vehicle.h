/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once
#include <unordered_map>
#include <vector>
#include "Constants.h"
#include "Controller.h"
#include "NearbyVehicle.h"

/* Class helps to organize all vehicle parameters in one place */
class Vehicle {
public:

	enum class VehicleStates {
		velocity_control,
		vehicle_following,
		intention_to_change_lane,
	};

	/* Getters and setters */

	long get_id() const{ return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	long get_category() const { return category; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	NearbyVehicle& get_leader() { return leader; };
	
	void set_time(double time);
	void set_id(long id) { this->id = id; };
	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	void set_desired_velocity(double desired_velocity) { 
		this->desired_velocity = desired_velocity; };
	void set_category(long category) { this->category = category; };
	void set_lane(long lane) { this->lane.push_back(lane); };
	void set_preferred_relative_lane(long preferred_relative_lane) { 
		this->preferred_relative_lane.push_back(preferred_relative_lane); };
	void set_velocity(double velocity) { 
		this->velocity.push_back(velocity); };
	void set_acceleration(double acceleration) { 
		this->acceleration.push_back(acceleration); };
	void set_color(long color) { this->color = color; };
	void set_desired_acceleration(double desired_acceleration) {
		this->desired_acceleration.push_back(desired_acceleration);
	};
	void set_vissim_acceleration(double vissim_acceleration) {
		this->vissim_acceleration.push_back(vissim_acceleration);
	}
	void set_vehicle_state(VehicleStates vehicle_state) { 
		this->vehicle_state.push_back(vehicle_state); };

	// TODO: should we check if vehicle is initialized before returning values?
	double get_current_time() const;
	long get_current_lane() const;
	long get_current_preferred_relative_lane() const;
	double get_current_velocity() const;
	double get_current_acceleration() const;
	double get_current_desired_acceleration() const;
	double get_current_vissim_acceleration() const;
	VehicleStates get_current_vehicle_state();

	/* Methods to write the entire vehicle history in a log file*/
	/* TODO: find better names for the methods */
	bool get_should_log() { return should_log; };
	void log_vehicle_states() { this->should_log = true; };
	void write_vehicle_log();

	/* Dealing with nearby vehicles*/
	void clear_nearby_vehicles();
	/* Adds a vehicle at the end of the nearby_vehicles container
	TODO: consider whether we should also check whether nearby vehicle is the
	leader. This would avoid having to search for it later on.*/
	void push_nearby_vehicle(NearbyVehicle*);
	/* Return the most recently added nearby vehicle */
	NearbyVehicle* peek_nearby_vehicles() const;
	/* Write data to the NearbyVehicle leader. If there is no leader, writes 
	all zeros. Returns true if a leader is found, false otherwise*/
	bool update_leader();
	/* Finds closest downstream nearby_vehicle which is on the same lane and
	writes its address to parameter leader. Function returns a boolean indicating whether
	the vehicle has a leader */
	//bool find_leader(NearbyVehicle*& leader) const;
	
	/* Returns the color equivalent to the current state as a long */
	long get_color_by_state();
	double compute_desired_acceleration();

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out, const Vehicle& vehicle);

private:
	double tau{ 0.5 }; /* actuator constant. This parameter comes from the
	physical model but there's no equivalent to it in VISSIM.
	TODO: parameter should vary based on vehicle category
	TODO: create function to simulate vehicle dynamics: tau.da/dt + a = u*/
	Controller controller;
	std::vector<VehicleStates> vehicle_state; // = VehicleStates::velocity_control;
	std::vector<NearbyVehicle*> nearby_vehicles;
	VehicleColor velocity_control_color = VehicleColor::red;
	VehicleColor vehicle_following_color = VehicleColor::yellow;
	VehicleColor intention_to_change_lane_color = VehicleColor::green;

	std::vector<double> simulation_time;
	long id;
	double length;
	double width;
	long color;
	long category;
	double desired_velocity;
	std::vector<long> lane;
	std::vector<long> preferred_relative_lane;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> desired_acceleration;
	std::vector<double> vissim_acceleration;
	NearbyVehicle leader;

	bool should_log = false;
};