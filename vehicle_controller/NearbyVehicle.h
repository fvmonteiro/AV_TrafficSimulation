/*==========================================================================*/
/*  NearbyVehicle.h	    												    */
/*  Class to help manage neighboring vehicles								*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <iostream>
#include <unordered_map>

#include "Vehicle.h"

class NearbyVehicle : public Vehicle{
public:
	using Vehicle::compute_safe_gap_parameters;

	NearbyVehicle() = default;
	NearbyVehicle(long id, RelativeLane relative_lane, long relative_position);
	NearbyVehicle(long id, long relative_lane, long relative_position);

	/* Getters and setters */

	RelativeLane get_relative_lane() const { return relative_lane; };
	/* positive = downstream (+1 next, +2 second next)
	   negative = upstream (-1 next, -2 second next) */
	long get_relative_position() const { return relative_position; };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	double get_lateral_position() const { return lateral_position; };
	double get_distance() const { return distance; };
	/* Relative velocity is: ego speed - other speed [m/s] */
	double get_relative_velocity() const { 
		return relative_velocity; 
	};
	double get_acceleration() const { return acceleration; };
	RelativeLane get_lane_change_direction() const { 
		return lane_change_direction; 
	};
	long get_lane_change_request_veh_id() const {
		return lane_change_request_veh_id;
	};
	double get_h_to_incoming_vehicle() const {
		return h_to_incoming_vehicle;
	};
	double get_max_lane_change_risk_to_follower() const {
		return max_lane_change_risk_to_follower;
	}
	long get_platoon_id() const { return platoon_id; };

	void set_lateral_position(double lateral_position) {
		this->lateral_position = lateral_position;
	};
	void set_distance(double distance) {
		this->distance = distance;
	};
	void set_relative_velocity(double relative_velocity) {
		this->relative_velocity = relative_velocity;
	};
	void set_acceleration(double acceleration) {
		this->acceleration = acceleration;
	};
	void set_lane_change_direction(long lane_change_direction) {
		this->lane_change_direction = 
			RelativeLane::from_long(lane_change_direction);
	};
	void set_desired_lane_change_direction(long lane_change_direction) {
		this->desired_lane_change_direction = 
			RelativeLane::from_long(lane_change_direction);
	};
	void set_h_to_incoming_vehicle(double h) {
		this->h_to_incoming_vehicle = h;
	};
	void set_max_lane_change_risk_to_follower(double r) {
		this->max_lane_change_risk_to_follower = r;
	}
	void set_platoon_id(long platoon_id) {
		this->platoon_id = platoon_id;
	}

	void set_type(VehicleType nv_type, VehicleType ego_type) /*override*/;
	/* Special getters and setters */

	//double get_lambda_0() const { return lambda_0; };
	//double get_lambda_1() const { return lambda_1; };

	bool is_connected() const;
	double compute_velocity(double ego_velocity) const;
	bool is_on_same_lane() const;
	bool is_immediatly_ahead() const;
	bool is_immediatly_behind() const;
	bool is_ahead() const;
	bool is_behind() const;
	bool is_lane_changing() const override;
	bool is_cutting_in() const;
	bool is_requesting_to_merge_ahead() const;
	bool is_requesting_to_merge_behind() const;
	/*void fill_with_dummy_values();
	void copy_current_states(NearbyVehicle& nearby_vehicle);*/
	/*void compute_safe_gap_parameters();*/
	void read_lane_change_request(long lane_change_request);
	bool is_in_a_platoon() const;

	double estimate_desired_time_headway(double free_flow_velocity,
		double leader_max_brake, double rho, double risk);
	double estimate_max_accepted_risk_to_incoming_vehicle(
		double free_flow_velocity, double leader_max_brake, double rho);

	std::string to_string() const;
	friend std::ostream& operator<< (std::ostream& out, 
		const NearbyVehicle& vehicle);

private:
	RelativeLane relative_lane{ RelativeLane::same };
	/* Relative position:
	positive = downstream (+1 next, +2 second next)
	negative = upstream (-1 next, -2 second next)
	It's possible to get more vehicles if DRIVER_DATA_WANTS_ALL_NVEHS
	in DriverModel.cpp  is set to 1 */
	long relative_position{ 0 };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	double lateral_position{ 0 };
	double distance{ 0.0 }; // front end to front end [m]
	double relative_velocity{ 0.0 }; // ego speed - other speed [m/s]
	double acceleration{ 0.0 }; // [m/s^2]
	RelativeLane lane_change_direction{ RelativeLane::same };
	/* If the vehicle is not part of a platoon, the request_id is the 
	same as its own id. If the vehicle is in a platoon, the request id
	depends on the platoon's lane changing strategy. */
	long lane_change_request_veh_id{ 0 };
	/* The time headway the nearby vehicle wants to keep from a connected
	ego vehicle that wants to merge in front of it.*/
	double h_to_incoming_vehicle{ 0.0 };
	double max_lane_change_risk_to_follower{ 0.0 };
	long platoon_id{ -1 };

	enum class Member {
		id,
		length,
		width,
		category,
		type,
		relative_lane,
		relative_position,
		lateral_position,
		distance,
		relative_velocity ,
		acceleration,
		lane_change_direction,
	};
	const std::unordered_map<Member, std::string> member_to_string {
		{Member::id, "id"},
		{Member::length, "length"},
		{Member::width, "width"},
		{Member::category, "category"},
		{Member::type, "type"},
		{Member::relative_lane, "relative_lane"},
		{Member::relative_position, "relative_position"},
		{Member::lateral_position, "lateral_position"},
		{Member::distance, "distance"},
		{Member::relative_velocity, "relative_velocity"},
		{Member::acceleration, "acceleration"},
		{Member::lane_change_direction, "lane_change_direction"},
	};
};
