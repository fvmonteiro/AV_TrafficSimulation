#include <iomanip>
#include <iostream>
#include <vector>
#include <sstream>

#include "Constants.h"
#include "NearbyVehicle.h"

NearbyVehicle::NearbyVehicle(long id, RelativeLane relative_lane,
	long relative_position) :
	Vehicle(id),
	relative_lane{ relative_lane },
	relative_position{ relative_position } {}

NearbyVehicle::NearbyVehicle(long id, long relative_lane,
	long relative_position) :
	NearbyVehicle(id, RelativeLane::from_long(relative_lane),
		relative_position) {}

void NearbyVehicle::set_type(VehicleType nv_type, VehicleType ego_type)
{
	/* CAVs can recognize other connected vehicles.
	AVs and CAVs can recognize human-driven vehicles versus all others. */
	if (is_a_connected_type(ego_type) && is_a_connected_type(nv_type))
	{
		this->type = nv_type;
		this->brake_delay = CONNECTED_BRAKE_DELAY;
	}
	else
	{
		/* We assume autonomous vehicles are identifiable visually by other
		autonomous vehicles without need for communications. 
		Both CAVs and AVs can differentiate HDVs from all the rest. */
		switch (nv_type)
		{
		case VehicleType::connected_car:
		case VehicleType::traffic_light_calc_car:
		case VehicleType::autonomous_car:
		case VehicleType::traffic_light_alc_car:
		case VehicleType::platoon_car:
			this->type = VehicleType::autonomous_car;
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			break;
		default:
			this->type = VehicleType::undefined;
			this->brake_delay = HUMAN_BRAKE_DELAY;
			break;
		}
	}
}

void NearbyVehicle::set_destination_lane_leader_id(long veh_id) 
{
	// We can only write/read this info from connected nearby vehicles
	this->dest_lane_leader_id = is_connected() ? veh_id : 0;
}

void NearbyVehicle::set_destination_lane_follower_id(long veh_id) 
{
	// We can only write/read this info from connected nearby vehicles
	this->dest_lane_follower_id = is_connected() ? veh_id : 0;
}

void NearbyVehicle::set_assisted_vehicle_id(long veh_id)
{
	// We can only write/read this info from connected nearby vehicles
	this->assisted_vehicle_id = is_connected() ? veh_id : 0;
}

bool NearbyVehicle::is_connected() const 
{
	/* The nearby vehicle type is only set to connected if the ego vehicle
	is also connected. So this function returns false when called by a non
	connected vehicle. */

	return (type == VehicleType::connected_car
		|| type == VehicleType::traffic_light_calc_car
		|| type == VehicleType::platoon_car);
}

double NearbyVehicle::compute_velocity(double ego_velocity) const 
{
	return ego_velocity - relative_velocity;
}


bool NearbyVehicle::is_on_same_lane() const {
	return get_relative_lane() == RelativeLane::same;
}

bool NearbyVehicle::is_immediatly_ahead() const 
{
	return get_relative_position() == 1;
}

bool NearbyVehicle::is_immediatly_behind() const 
{
	return get_relative_position() == -1;
}

bool NearbyVehicle::is_ahead() const 
{
	return get_relative_position() > 0;
}

bool NearbyVehicle::is_behind() const 
{
	return !is_ahead();
}

bool NearbyVehicle::is_lane_changing() const 
{
	return lane_change_direction != RelativeLane::same;
}

bool NearbyVehicle::is_cutting_in() const 
{
	if (is_ahead() && (is_lane_changing())) 
	{
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

//bool NearbyVehicle::is_requesting_to_merge_ahead() const 
//{
//	return is_connected() 
//		&& has_lane_change_intention()
//		&& is_immediatly_ahead()
//		&& (relative_lane == desired_lane_change_direction.get_opposite());
//}
//
//bool NearbyVehicle::is_requesting_to_merge_behind() const 
//{
//	return is_connected() 
//		&& has_lane_change_intention() 
//		&& is_behind()
//		&& (relative_lane == desired_lane_change_direction.get_opposite()
//			|| relative_lane == RelativeLane::same);
//}

void NearbyVehicle::read_lane_change_request(long lane_change_request) 
{
	/* Getting the sign of lane change request */
	//int request_sign = (lane_change_request > 0) 
	//	- (lane_change_request < 0);
	//set_desired_lane_change_direction(request_sign);
	//lane_change_request_veh_id = std::abs(lane_change_request);
	lane_change_request_veh_id = lane_change_request;
}

bool NearbyVehicle::is_in_a_platoon() const
{
	return platoon_id != -1;
}

double NearbyVehicle::estimate_desired_time_headway(double free_flow_velocity,
	double leader_max_brake, double rho, double risk)
{
	compute_safe_gap_parameters();
	return compute_time_headway_with_risk(free_flow_velocity, 
		get_max_brake(), leader_max_brake, get_lambda_1(), rho, risk);
}

double NearbyVehicle::estimate_max_accepted_risk_to_incoming_vehicle(
	double free_flow_velocity, double leader_max_brake, double rho)
{
	return compute_max_risk(leader_max_brake, get_max_brake(), 
		free_flow_velocity, rho);
}

std::string NearbyVehicle::to_string() const {
	std::ostringstream oss;

	std::vector<Member> printed_members = {
		Member::id, Member::relative_lane, Member::relative_position,
		Member::lateral_position, Member::distance, Member::relative_velocity,
		Member::lane_change_direction
	};

	for (Member m : printed_members) 
	{
		oss << member_to_string.at(m) << "=";
		switch (m) {
		case Member::id:
			oss << get_id();
			break;
		case Member::length:
			oss << get_length();
			break;
		case Member::width:
			oss << get_width();
			break;
		case Member::category:
			oss << static_cast<int>(get_category());
			break;
		case Member::type:
			oss << static_cast<int>(get_type());
			break;
		case Member::relative_lane:
			oss << relative_lane;
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
			oss << lane_change_direction;
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

	for (const auto& name_value_pair : printed_long_members) {
		out << name_value_pair.first << ": " <<
			name_value_pair.second << " | ";
	}
	for (const auto& name_value_pair : printed_double_members) {
		out << name_value_pair.first << ": " << std::setprecision(4) <<
			name_value_pair.second << " | ";
	}

	return out; // return std::ostream so we can chain calls to operator<<
}
