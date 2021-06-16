/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include "Vehicle.h"

double Vehicle::get_current_time() const { return simulation_time.back(); }
long Vehicle::get_current_lane() const { return lane.back(); }
long Vehicle::get_current_preferred_relative_lane() const {
	return preferred_relative_lane.back();
}
double Vehicle::get_current_velocity() const { return velocity.back(); }
double Vehicle::get_current_acceleration() const { 
	return acceleration.back(); 
}
double Vehicle::get_current_desired_acceleration() const {
	return desired_acceleration.back();
}
double Vehicle::get_current_vissim_acceleration() const {
	return vissim_acceleration.back();
}
Vehicle::VehicleStates Vehicle::get_current_vehicle_state() {
	if (vehicle_state.empty()) return VehicleStates::velocity_control;
	return vehicle_state.back();
}

void Vehicle::set_time(double time) { 
	if (simulation_time.empty() || get_current_time() != time) {
		this->simulation_time.push_back(time);
	}
	//else if (get_current_time() != time) {
	//	this->simulation_time.push_back(time);
	//}
}

void Vehicle::clear_nearby_vehicles() {
	nearby_vehicles.clear();
}

void Vehicle::push_nearby_vehicle(NearbyVehicle* nearby_vehicle) {
	nearby_vehicles.push_back(nearby_vehicle);
}

NearbyVehicle* Vehicle::peek_nearby_vehicles() const {
	if (!nearby_vehicles.empty()) {
		return nearby_vehicles.back();
	}
	std::cerr << "Empty nearby_vehicles container in vehicle  " << this->id <<
		std::endl;
	return nullptr;//NearbyVehicle();
}

bool Vehicle::update_leader() {
	for (NearbyVehicle* nearby_vehicle : nearby_vehicles) {
		if ((nearby_vehicle->get_current_relative_lane() == 0)
			& (nearby_vehicle->get_current_relative_position() == 1)) {
			leader.copy_current_states(*nearby_vehicle);
			return true;
		}
	}
	leader.zero_fill();
	return false;
}

//bool Vehicle::find_leader(NearbyVehicle*& leader) const {
//	for (NearbyVehicle* nearby_vehicle : nearby_vehicles) {
//		if ((nearby_vehicle->get_current_relative_lane() == 0) 
//			& (nearby_vehicle->get_current_relative_position() == 1)) {
//			leader = nearby_vehicle;
//			return true;
//		}
//	}
//	return false;
//}

long Vehicle::get_color_by_state() {

	/* For debugging purposes, we allow some other methods to alter
	the vehicle's color. This makes is easier to identify certain scenarios*/
	if (has_external_color) {
		return external_color;
	}

	switch (get_current_vehicle_state())
	{
	case VehicleStates::velocity_control:
		return static_cast<std::underlying_type<VehicleColor>::type>(
			velocity_control_color);
	case VehicleStates::vehicle_following:
		return static_cast<std::underlying_type<VehicleColor>::type>(
			vehicle_following_color);
	case VehicleStates::intention_to_change_lane:
		return static_cast<std::underlying_type<VehicleColor>::type>(
			intention_to_change_lane_color);
	default:
		return static_cast<std::underlying_type<VehicleColor>::type>(
			VehicleColor::white);
		break;
	}
}

double Vehicle::compute_desired_acceleration() {
	double desired_acceleration = controller.compute_desired_acceleration(
		*this, leader);
	/*if (leader.get_current_id() == 0) {
		double velocity_error = get_desired_velocity() 
			- get_current_velocity();
		desired_acceleration = controller.compute_velocity_control_input(
			velocity_error);
	}
	else {
		desired_acceleration = controller.compute_desired_acceleration(
			*this, leader);
	}*/
	set_desired_acceleration(desired_acceleration);
	return desired_acceleration;
	/*NearbyVehicle* leader;
	if (!find_leader(leader)) {
		double velocity_error = get_desired_velocity() - get_current_velocity();
		return controller.compute_velocity_control_input(velocity_error);
	}
	double reference_gap = controller.compute_desired_gap(get_current_velocity());
	double gap_error = controller.computed_gap_error(
		leader->get_current_distance(), reference_gap);
	double velocity_error = controller.compute_velocity_error(get_current_velocity(),
		get_current_velocity() - leader->get_current_relative_velocity());
	return controller.compute_desired_acceleration(*this, *leader);*/
}

std::string to_string(Vehicle::VehicleStates vehicle_state) {
	switch (vehicle_state)
	{
	case Vehicle::VehicleStates::velocity_control:
		return "vel. control";
	case Vehicle::VehicleStates::vehicle_following:
		return "veh. following";
	default:
		return "ERROR: unknown vehicle state";
	}
}

void Vehicle::write_vehicle_log() {
	NearbyVehicle leader = get_leader();
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
	vehicle_log.open(file_name);
	vehicle_log << "time; id;"
		//<< "category;" 
		<< "lane; desired vel.; vel.;" 
		<< "control state; desired accel; accel.;"
		//<< "length; width; "
		<< "leader id; distance; rel. vel." << std::endl;
	
	for (int i = 0; i < simulation_time.size(); ++i) {
		vehicle_log << simulation_time[i] << ";" << id << ";"
			//<< category << ";" 
			<< lane[i] << ";"
			<< desired_velocity << ";" << velocity[i] << ";" 
			<< to_string(vehicle_state[i]) << ";"
			<< desired_acceleration[i] << ";"<< acceleration[i] << ";"
			//<< length << ";" << width << ";" 
			<< leader.get_id()[i] << ";"
			<< leader.get_distance()[i] << ";"
			<< leader.get_relative_velocity()[i] << std::endl;
	}
	
	vehicle_log.close();
}

std::ostream& operator<< (std::ostream& out, const Vehicle& vehicle)
{
	std::vector<std::pair<std::string, long>> long_members{ 
		{"Vehicle id", vehicle.get_id()}, //{"category", vehicle.get_category()}, 
		{"lane", vehicle.get_current_lane()} 
	};
	std::vector<std::pair<std::string, double>> double_members{
		{ "velocity", vehicle.get_current_velocity()},
		{ "des. velocity", vehicle.get_desired_velocity()},
		{"acceleration", vehicle.get_current_acceleration()},
		{"des. acceleration", vehicle.get_current_desired_acceleration()},
		//{"length", vehicle.get_length()}, {"width", vehicle.get_width()} 
	};

	for (auto name_value_pair : long_members) {
		out << name_value_pair.first << ": " <<
			name_value_pair.second << " | ";
	}
	for (auto name_value_pair : double_members) {
		out << name_value_pair.first << ": " << std::setprecision(4) <<
			name_value_pair.second << " | ";
	}
	out << std::endl;
	/*for (NearbyVehicle* nearby_vehicle : vehicle.nearby_vehicles) {
		out << "\tNearby vehicle: " << *nearby_vehicle << std::endl;
	}*/
	/*NearbyVehicle* leader;
	if (vehicle.find_leader(leader)) {
		out << "\tLeading vehicle: " << *leader;
	}*/

	return out; // return std::ostream so we can chain calls to operator<<
}