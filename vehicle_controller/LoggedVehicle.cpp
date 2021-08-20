#include <fstream>
#include <sstream>

#include "LoggedVehicle.h"

//LoggedVehicle::LoggedVehicle(long id, double simulation_time_step,
//	double creation_time) :
//	EgoVehicle(id, simulation_time_step, creation_time) {
//	this->verbose = true;
//	std::clog << "Creating vehicle " << id
//		<< " at time " << creation_time
//		<< " with simulation time step " << simulation_time_step
//		<< std::endl; 
//}
//
//LoggedVehicle::~LoggedVehicle() {
//	std::vector<Member> members{
//		Member::creation_time,
//		Member::preferred_relative_lane,
//		Member::state,
//		Member::velocity,
//		Member::desired_acceleration,
//		Member::active_lane_change_direction,
//		Member::leader_id,
//		//Member::ttc,
//		//Member::drac,
//		//Member::collision_severity_risk
//	};
//	std::clog << write_header(members, true);
//	std::clog << "Vehicle " << id
//		<< " out of the simulation"
//		<< "at time " << get_time()
//		<< std::endl;
//
//	write_simulation_log(members);
//}

//void LoggedVehicle::write_simulation_log(std::vector<Member> members) {
//
//	bool write_size = true;
//	std::ofstream vehicle_log;
//	std::string file_name = "vehicle" + std::to_string(id) + ".txt";
//	vehicle_log.open(log_path + "\\" + file_name);
//	if (vehicle_log.is_open()) {
//		vehicle_log << write_header(members, write_size);
//		vehicle_log << write_members(members);
//		vehicle_log.close();
//	}
//	else {
//		std::clog << "Unable to open file to write log of "
//			<< "vehicle " << id
//			<< std::endl;
//	}
//}
//
//std::string LoggedVehicle::write_members(
//	std::vector<EgoVehicle::Member> members) {
//
//	std::ostringstream oss;
//
//	/* Sanity check: some non critical code mistakes could
//	create vector members with different sizes. This prevents
//	the log from being written and crashes vissim. Let's avoid
//	this and just write a warning message instead.*/
//	int n_samples = (int)velocity.size(); /* velocity, lane and link members
//	are the least likely to have the wrong size */
//	std::vector<int> deleted_indices;
//	for (int i = 0; i < members.size(); i++) {
//		Member m = members.at(i);
//		if ((get_member_size(m) != 1) // not a scalar
//			&& (get_member_size(m) != n_samples)) {
//			oss << "Error: member " << member_enum_to_string(m)
//				<< " has " << get_member_size(m) << " samples "
//				<< "instead of the expected " << n_samples
//				<< ". It won't be printed."
//				<< std::endl;
//			deleted_indices.push_back(i);
//		}
//	}
//	for (int idx : deleted_indices) {
//		members.erase(std::next(members.begin(), idx));
//	}
//
//	// Write variables over time
//	for (int i = 0; i < n_samples; i++) {
//		for (auto m : members) {
//			switch (m)
//			{
//			case Member::creation_time:
//				oss << creation_time + i * simulation_time_step;
//				break;
//			case Member::id:
//				oss << id;
//				break;
//			case Member::length:
//				oss << length;
//				break;
//			case Member::width:
//				oss << width;
//				break;
//			case Member::color:
//				oss << "color printing code not done";
//				break;
//			case Member::category:
//				oss << static_cast<int>(category);
//				break;
//			case Member::desired_velocity:
//				oss << get_desired_velocity();
//				break;
//			case Member::lane:
//				oss << lane[i];
//				break;
//			case Member::link:
//				oss << link[i];
//				break;
//			case Member::preferred_relative_lane:
//				oss << preferred_relative_lane[i];
//				break;
//			case Member::velocity:
//				oss << velocity[i];
//				break;
//			case Member::acceleration:
//				oss << acceleration[i];
//				break;
//			case Member::desired_acceleration:
//				oss << desired_acceleration[i];
//				break;
//			case Member::vissim_acceleration:
//				oss << vissim_acceleration[i];
//				break;
//			case Member::leader_id:
//				oss << leader_id[i];
//				break;
//			case Member::state:
//				oss << state_to_string(state[i]);
//				break;
//			case Member::active_lane_change_direction:
//				oss << active_lane_change[i];
//				break;
//			case Member::vissim_active_lane_change_direction:
//				oss << vissim_active_lane_change[i];
//				break;
//			case Member::lane_end_distance:
//				oss << lane_end_distance[i];
//				break;
//			case Member::ttc:
//				oss << ttc[i];
//				break;
//			case Member::drac:
//				oss << drac[i];
//				break;
//			case Member::collision_severity_risk:
//				oss << collision_severity_risk[i];
//				break;
//			case Member::type:
//				oss << static_cast<int>(type);
//				break;
//			default:
//				oss << "";
//				break;
//			}
//			oss << ", ";
//		}
//		oss << std::endl;
//	}
//	return oss.str();
//}
//
//std::string LoggedVehicle::write_header(
//	std::vector<EgoVehicle::Member> members, bool write_size) {
//
//	std::ostringstream oss;
//	for (auto m : members) {
//		oss << member_enum_to_string(m);
//		if (write_size) {
//			oss << " (" << get_member_size(m) << ")";
//		}
//		oss << ", ";
//	}
//	oss << std::endl;
//
//	return oss.str();
//}
//
//int LoggedVehicle::get_member_size(Member member) {
//	switch (member)
//	{
//	case Member::creation_time:
//	case Member::id:
//	case Member::length:
//	case Member::width:
//	case Member::color:
//	case Member::category:
//	case Member::desired_velocity:
//	case Member::type:
//		return 1;
//	case Member::lane:
//		return (int)lane.size();
//	case Member::link:
//		return (int)link.size();
//	case Member::preferred_relative_lane:
//		return (int)preferred_relative_lane.size();
//	case Member::velocity:
//		return (int)velocity.size();
//	case Member::acceleration:
//		return (int)acceleration.size();
//	case Member::desired_acceleration:
//		return (int)desired_acceleration.size();
//	case Member::vissim_acceleration:
//		return (int)vissim_acceleration.size();
//	case Member::leader_id:
//		return (int)leader_id.size();
//	case Member::state:
//		return (int)state.size();
//	case Member::active_lane_change_direction:
//		return (int)active_lane_change.size();
//	case Member::vissim_active_lane_change_direction:
//		return (int)vissim_active_lane_change.size();
//	case Member::lane_end_distance:
//		return (int)lane_end_distance.size();
//	case Member::ttc:
//		return (int)ttc.size();
//	case Member::drac:
//		return (int)drac.size();
//	case Member::collision_severity_risk:
//		return (int)collision_severity_risk.size();
//	default:
//		return 0;
//	}
//}
