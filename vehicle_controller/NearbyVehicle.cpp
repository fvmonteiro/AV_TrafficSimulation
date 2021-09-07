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

void NearbyVehicle::set_type(VehicleType type) {
	/* The nearby vehicle is either identified as connected by
	a connected ego vehicle, or it is seen as undefined.
	Note: if we change the way of dealing with nearby vehicles and they are
	no longer erased every time step, this function needs a little twitch 
	to avoid re-setting the type at every time step. */
	this->type = type;

	switch (this->type)
	{
	case VehicleType::undefined:
		this->brake_delay = HUMAN_BRAKE_DELAY;
		break;
	case VehicleType::connected_car:
		this->brake_delay = CONNECTED_BRAKE_DELAY;
		break;
	default:
		std::clog << "nearby vehicle " << id <<" being set with unknown " 
			<< "type value " << static_cast<int>(type) << std::endl;
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

bool NearbyVehicle::is_ahead() const {
	return get_relative_position() > 0;
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
				== get_opposite_relative_lane(lane_change_direction))
			&& ((get_lateral_position()
				* static_cast<int>(lane_change_direction)) > 0);
		if (moving_into_my_lane) return true;
	}
	return false;
}

bool NearbyVehicle::requesting_to_move_in() const {
	if (is_connected() && (relative_position == 1)
		&& (has_lane_change_intention())
		&& (relative_lane 
			== get_opposite_relative_lane(desired_lane_change_direction))) {
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
