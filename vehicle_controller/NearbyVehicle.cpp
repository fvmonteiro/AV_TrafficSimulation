#include <iomanip>
#include <iostream>
#include <vector>

#include "Constants.h"
#include "NearbyVehicle.h"

void NearbyVehicle::set_category(VehicleCategory category) {
	if (category == VehicleCategory::truck) {
		max_brake = truck_max_brake;
	}
	else { // assume car if any other category
		max_brake = car_max_brake;
	}
	if ((this->category.empty()) || (this->category.back() != category)) {
		compute_safe_gap_parameters();
	}
	this->category.push_back(category);
}

bool NearbyVehicle::is_on_same_lane() const {
	return get_current_relative_lane() == RelativeLane::same;
}

bool NearbyVehicle::is_ahead() const {
	return get_current_relative_position() > 0;
}

void NearbyVehicle::fill_with_dummy_values() {
	set_id(0);
	set_length(0);
	set_width(0);
	set_category(0);
	set_relative_lane(0);
	set_relative_position(0);
	set_distance(MAX_DISTANCE);
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

void NearbyVehicle::compute_safe_gap_parameters() {
	/* TODO: vary estimated parameters based on category and whether 
	communication is avaiable */
	double jE{ 50.0 }; // [m/s^3]
	double aE{ 0.5 }; // [m/s^2]
	double tau_d{ 0.1 }; // [s]
	double bE = get_max_brake();
	double tau_j = (aE + bE) / jE;
	lambda_0 = -(aE + bE)
		* (std::pow(tau_d, 2) + tau_d * tau_j + std::pow(tau_j, 2) / 3);
	lambda_1 = (aE + bE) * (tau_d + tau_j / 2);
}

std::ostream& operator<<(std::ostream& out, const NearbyVehicle& vehicle)
{
	std::vector<std::pair<std::string, long>> printed_long_members{ 
		{"id", vehicle.get_current_id()}, 
		{"category", static_cast<int>(vehicle.get_current_category())},
		{"rel. lane", static_cast<int>(vehicle.get_current_relative_lane())},
		{"rel. position", vehicle.get_current_relative_position()} 
	};
	std::vector<std::pair<std::string, double>> printed_double_members{
		{"distance", vehicle.get_current_distance()}, 
		{"rel. velocity", vehicle.get_current_relative_velocity()},
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
