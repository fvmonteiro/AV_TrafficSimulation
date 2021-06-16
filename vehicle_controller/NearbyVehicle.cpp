#include <iomanip>
#include <iostream>
#include <vector>
#include "NearbyVehicle.h"

void NearbyVehicle::zero_fill() {
	set_id(0);
	set_length(0);
	set_width(0);
	set_category(0);
	set_relative_lane(0);
	set_relative_position(0);
	set_distance(0);
	set_relative_velocity(0);
	set_acceleration(0);
}

void NearbyVehicle::copy_current_states(NearbyVehicle& nearby_vehicle) {
	set_id(nearby_vehicle.get_current_id());
	set_length(nearby_vehicle.get_current_length());
	set_width(nearby_vehicle.get_current_width());
	set_category(nearby_vehicle.get_current_category());
	set_relative_lane(nearby_vehicle.get_current_relative_lane());
	set_relative_position(nearby_vehicle.get_current_relative_position());
	set_distance(nearby_vehicle.get_current_distance());
	set_relative_velocity(nearby_vehicle.get_current_relative_velocity());
	set_acceleration(nearby_vehicle.get_current_acceleration());
}

std::ostream& operator<<(std::ostream& out, const NearbyVehicle& vehicle)
{
	std::vector<std::pair<std::string, long>> printed_long_members{ 
		{"id", vehicle.get_current_id()}, {"category", vehicle.get_current_category()}, 
		{"rel. lane", vehicle.get_current_relative_lane()}, 
		{"rel. position", vehicle.get_current_relative_position()} 
	};
	std::vector<std::pair<std::string, double>> printed_double_members{
		{"distance", vehicle.get_current_distance()}, 
		{ "rel. velocity", vehicle.get_current_relative_velocity()},
		//{"acceleration", vehicle.acceleration},
		//{"length", vehicle.length}, {"width", vehicle.width} 
	};

	for (auto name_value_pair : printed_long_members) {
		out << name_value_pair.first << ": " <<
			name_value_pair.second << " | ";
	}
	for (auto name_value_pair : printed_double_members) {
		out << name_value_pair.first << ": " << std::setprecision(4) <<
			name_value_pair.second << " | ";
	}

	return out; // return std::ostream so we can chain calls to operator<<
}
