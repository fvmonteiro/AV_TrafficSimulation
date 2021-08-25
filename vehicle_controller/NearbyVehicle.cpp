#include <iomanip>
#include <iostream>
#include <vector>
#include <sstream>

#include "Constants.h"
#include "NearbyVehicle.h"

NearbyVehicle::NearbyVehicle(long id, 
	RelativeLane relative_lane, long relative_position) 
	: relative_lane{ relative_lane },
	relative_position{ relative_position } {
	this->id = id;
}

NearbyVehicle::NearbyVehicle(long id, long relative_lane,
	long relative_position)
	: NearbyVehicle(id, RelativeLane(relative_lane),
		relative_position) {}

void NearbyVehicle::set_category(VehicleCategory category) {
	if (category == VehicleCategory::truck) {
		max_brake = TRUCK_MAX_BRAKE;
	}
	else { // assume car if any other category
		max_brake = CAR_MAX_BRAKE;
	}
	/*if ((this->category.empty()) || (this->category.back() != category)) {
		compute_safe_gap_parameters();
	}*/
	this->category = category;
}

double NearbyVehicle::compute_velocity(double ego_velocity) const {
	return ego_velocity - relative_velocity;
}

bool NearbyVehicle::is_on_same_lane() const {
	return get_relative_lane() == RelativeLane::same;
}

bool NearbyVehicle::is_ahead() const {
	return get_relative_position() > 0;
}

bool NearbyVehicle::is_lane_changing() const {
	return lane_change_direction != RelativeLane::same;
}

void NearbyVehicle::compute_safe_gap_parameters() {
	/* TODO: vary estimated parameters based on category and whether 
	communication is avaiable */
	double jE{ 50.0 }; // [m/s^3]
	double aE{ 0.5 }; // [m/s^2]
	double tau_d{ 0.3 }; // [s]
	double bE = get_max_brake();
	double tau_j = (aE + bE) / jE;
	lambda_0 = -(aE + bE)
		* (std::pow(tau_d, 2) + tau_d * tau_j + std::pow(tau_j, 2) / 3);
	lambda_1 = (aE + bE) * (tau_d + tau_j / 2);
}

std::string NearbyVehicle::print_members() const {
	std::ostringstream oss;

	std::vector<Member> printed_members = {
		Member::id, Member::relative_lane, Member::relative_position,
		Member::lateral_position, Member::distance, Member::relative_velocity,
		Member::lane_change_direction
	};

	for (Member m : printed_members) {
		oss << member_to_string.at(m) << "=";
		switch (m) {
		case Member::id:
			oss << id;
			break;
		case Member::length:
			oss << length;
			break;
		case Member::width:
			oss << width;
			break;
		case Member::category:
			oss << static_cast<int>(category);
			break;
		case Member::type:
			oss << static_cast<int>(type);
			break;
		case Member::relative_lane:
			oss << static_cast<int>(relative_lane);
			break;
		case Member::relative_position:
			oss << relative_position;
			break;
		case Member::lateral_position:
			oss << lateral_position;
			break;
		case Member::distance:
			oss << distance;
			break;
		case Member::relative_velocity:
			oss << distance;
			break;
		case Member::acceleration:
			oss << acceleration;
			break;
		case Member::lane_change_direction:
			oss << static_cast<int>(lane_change_direction);
			break;
		default:
			oss << "unknown class member";
			break;
		}
		oss << ", ";
	}

	return oss.str();

}

std::ostream& operator<<(std::ostream& out, const NearbyVehicle& vehicle)
{
	std::vector<std::pair<std::string, long>> printed_long_members{ 
		{"id", vehicle.get_id()}, 
		{"category", static_cast<int>(vehicle.get_category())},
		{"rel. lane", static_cast<int>(vehicle.get_relative_lane())},
		{"rel. position", vehicle.get_relative_position()} 
	};
	std::vector<std::pair<std::string, double>> printed_double_members{
		{"distance", vehicle.get_distance()}, 
		{"rel. velocity", vehicle.get_relative_velocity()},
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
