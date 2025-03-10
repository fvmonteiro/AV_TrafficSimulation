#pragma once
#include "EgoVehicle.h"

/* In case we decide that the EgoVehicle class should not contain memory, 
which allows it to create logs after simulation, this is the derived class
that can have logging capabilities. */

class LoggedVehicle :
    public EgoVehicle
{
public:
	//LoggedVehicle(long id, double simulation_time_step, double creation_time);
	//~LoggedVehicle();

	/*long get_lane() const override {
		return lane.back();
	}
	long get_link() const override {
		return link.back();
	}
	double get_lateral_position() const override{
		return lateral_position.back();
	}
	long get_preferred_relative_lane() const override {
		return preferred_relative_lane.back();
	}
	double get_velocity() const override {
		return velocity.back();
	}
	double get_acceleration() const override {
		return acceleration.back();
	}
	double get_desired_acceleration() const override {
		return desired_acceleration.back();
	}
	double get_vissim_acceleration() const override {
		return vissim_acceleration.back();
	}
	long get_active_lane_change() const override {
		return active_lane_change.back();
	}
	long get_vissim_active_lane_change() const override {
		return vissim_active_lane_change.back();
	}
	double get_lane_end_distance() const override {
		return lane_end_distance.back();
	}
	long get_leader_id() const override {
		return leader_id.back();
	}

	double get_ttc() const override {
		return ttc.back();
	}
	double get_drac() const override {
		return drac.back();
	}
	double get_collision_risk() const override {
		return collision_severity_risk.back();
	}

	void set_lane(long lane) override {
		this->lane.push_back(lane);
	}
	void set_link(long link) override {
		this->link.push_back(link);
	}
	void set_lateral_position(double lateral_position) override {
		this->lateral_position.push_back(lateral_position);
	}
	void set_velocity(double velocity) override {
		this->velocity.push_back(velocity);
	}
	void set_acceleration(double acceleration) override {
		this->acceleration.push_back(acceleration);
	}
	void set_vissim_acceleration(double vissim_acceleration) override {
		this->vissim_acceleration.push_back(vissim_acceleration);
	}
	void set_vissim_active_lane_change(int active_lane_change) override {
		this->vissim_active_lane_change.push_back(active_lane_change);
	}*/

private:
	/* Nearby vehicles data */
	/* TODO: I'm no longer sure a vector of pointer is the way to go.
	Maybe we should use a simple vector of objects. Or maybe some smart
	pointer stuff */
	std::vector<std::shared_ptr<NearbyVehicle>> nearby_vehicles;
	std::vector<long> leader_id;
	long dest_lane_leader_id = 0;
	long dest_lane_follower_id = 0;
	
	/* Data obtained from VISSIM or generated by internal computations */
	//std::vector<double> simulation_time;
	std::vector<long> lane;
	std::vector<long> link;
	std::vector<long> preferred_relative_lane;
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	std::vector<double> lateral_position;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> desired_acceleration;
	/* VISSIM suggested acceleration */
	std::vector<double> vissim_acceleration;
	/* +1 = to the left, 0 = none, -1 = to the right */
	std::vector<long> active_lane_change;
	/* VISSIM suggested active lane change */
	std::vector<long> vissim_active_lane_change;
	/* Determines if we use our lane change decision model or VISSIM's */
	bool use_internal_lane_change_decision{ true };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	std::vector<double> lane_end_distance;
	std::vector<State> state;
	RelativeLane desired_lane_change_direction{ RelativeLane::same };
	double desired_lane_angle{ 0.0 };
	long rel_target_lane{ 0 };
	long turning_indicator{ 0 };

	/*Surrogate Safety Measurements (SSMs)*/

	std::vector<double> ttc; // time-to-collision
	std::vector<double> drac; // deceleration rate to avoid collision
	std::vector<double> collision_severity_risk; /* delta vel. at collision under the
											worst case scenario*/

	std::string log_path = "autonomous_vehicle_logs";

	/*void write_simulation_log(std::vector<Member> members);
	std::string write_header(std::vector<Member> members,
		bool write_size = false);
	std::string write_members(std::vector<Member> members);
	int get_member_size(Member member);*/

};

