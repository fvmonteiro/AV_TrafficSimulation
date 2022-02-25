#include <iomanip>
#include <iostream>
#include <vector>
#include <sstream>

#include "Constants.h"
#include "NearbyVehicle.h"

NearbyVehicle::NearbyVehicle(long id, RelativeLane relative_lane,
	long relative_position) :
	relative_lane{ relative_lane },
	relative_position{ relative_position } {
	this->id = id;
}

NearbyVehicle::NearbyVehicle(long id, long relative_lane,
	long relative_position) :
	NearbyVehicle(id, RelativeLane::from_long(relative_lane),
		relative_position) {}

void NearbyVehicle::set_type(int type) 
{
	/* This function is only called by connected ego vehicles
	trying to set the type of a nearby vehicle.
	The nearby vehicle type is only set if the nearby vehicle is 
	also connected. */

	/* Note: if we change the way of dealing with nearby vehicles and they are
	no longer erased every time step, this function needs a little twitch 
	to avoid re-setting the type at every time step. */

	VehicleType temp_veh_type = VehicleType(type);
	switch (temp_veh_type)
	{
	case VehicleType::connected_car:
	case VehicleType::traffic_light_cacc_car:
		this->type = temp_veh_type;
		this->brake_delay = CONNECTED_BRAKE_DELAY;
		break;
	default:
		this->type = VehicleType::undefined;
		this->brake_delay = HUMAN_BRAKE_DELAY;
		break;
	}
}

double NearbyVehicle::compute_velocity(double ego_velocity) const {
	return ego_velocity - relative_velocity;
}

bool NearbyVehicle::is_on_same_lane() const {
	return get_relative_lane() == RelativeLane::same;
}

bool NearbyVehicle::is_immediatly_ahead() const {
	return get_relative_position() == 1;
}

bool NearbyVehicle::is_ahead() const {
	return get_relative_position() > 0;
}

bool NearbyVehicle::is_behind() const {
	return !is_ahead();
}

bool NearbyVehicle::is_lane_changing() const {
	return lane_change_direction != RelativeLane::same;
}

bool NearbyVehicle::is_cutting_in() const {
	if (is_ahead()
		&& (is_lane_changing())) {
		/* The nearby vehicle must be changing lanes towards the ego vehicle
		(that's the first part of the condition below)
		The first condition alone could misidentify the case where a vehicle
		two lanes away moves to an adjacent lane as a cut in. Therefore
		we must check whether the lateral position (with respect to the
		lane center) and the lane change direction have the same sign. */
		bool moving_into_my_lane =
			(relative_lane
				== lane_change_direction.get_opposite())
			&& ((get_lateral_position()
				* lane_change_direction.to_int()) > 0);
		if (moving_into_my_lane) return true;
	}
	return false;
}

bool NearbyVehicle::is_requesting_to_merge_ahead() const {
	if (is_connected() && is_immediatly_ahead() 
		&& has_lane_change_intention()
		&& (relative_lane == desired_lane_change_direction.get_opposite())) {
		return true;
	}
	return false;
}

bool NearbyVehicle::is_requesting_to_merge_behind() const {
	if (is_connected() && is_behind() && has_lane_change_intention()
		&& (relative_lane == desired_lane_change_direction.get_opposite()
			|| relative_lane == RelativeLane::same)) {
		return true;
	}
	return false;
}

void NearbyVehicle::compute_safe_gap_parameters() {
	lambda_0 = compute_lambda_0(max_jerk, comfortable_acceleration, 
		max_brake, brake_delay);
	lambda_1 = compute_lambda_1(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
}

void NearbyVehicle::read_lane_change_request(long lane_change_request) {
	/* Getting the sign of lane change request */
	int request_sign = (lane_change_request > 0) 
		- (lane_change_request < 0);
	set_desired_lane_change_direction(request_sign);
	lane_change_request_veh_id = std::abs(lane_change_request);

	if (id == 12) std::clog << "lc request: " << lane_change_request 
		<< ", request sign: "<< request_sign
		<< ", des lc direction: " << desired_lane_change_direction.to_string()
		<< ", lc req veh id (1): " << std::abs(lane_change_request)
		<< ", lc req veh id (2): " << get_lane_change_request_veh_id() << std::endl;
}

std::string NearbyVehicle::to_string() const {
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
			oss << relative_lane.to_string();
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
			oss << relative_velocity;
			break;
		case Member::acceleration:
			oss << acceleration;
			break;
		case Member::lane_change_direction:
			oss << lane_change_direction.to_string();
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
		{"rel. lane", vehicle.get_relative_lane().to_int()},
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
