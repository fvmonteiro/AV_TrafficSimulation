#pragma once

#include "ConnectedAutonomousVehicle.h"

class PlatoonVehicle : public ConnectedAutonomousVehicle
{
public:

	PlatoonVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);
	~PlatoonVehicle();

protected:
	/* If the current leader is a platoon vehicle and 
	merges into the platoon if yes. */
	bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		std::shared_ptr<EgoVehicle> pointer_to_me, 
		long new_platoon_id) override;

private:
	void set_desired_lane_change_direction() override;
	bool can_start_lane_change() override;
	std::shared_ptr<Platoon> implement_get_platoon() const override
	{
		return platoon;
	};
	void create_platoon(long platoon_id,
		std::shared_ptr<PlatoonVehicle> pointer_to_me);
	void add_myself_to_leader_platoon(
		std::shared_ptr<Platoon> leader_platoon,
		std::shared_ptr<PlatoonVehicle> pointer_to_me);
	bool is_platoon_leader();

	std::shared_ptr<Platoon> platoon{ nullptr };
	// desired velocity when not a part of the platoon
	double alone_desired_velocity{ 0.0 };
	/* Time the vehicle has been without a platoon.
	As soon as the vehicle enters simulation, it may create its own platoon */
	double alone_time{ 1.0 };
	double max_time_looking_for_platoon{ 1.0 };

	//double compute_desired_acceleration(
	//	const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	//double compute_lane_changing_desired_time_headway(
	//	const NearbyVehicle& nearby_vehicle) const override {
	//	return -1.0;
	//};

	//long create_lane_change_request() override { return 0; };

	//double compute_accepted_lane_change_gap(
	//	std::shared_ptr<NearbyVehicle> nearby_vehicle) override {
	//	return 0.0;
	//};

	//std::shared_ptr<NearbyVehicle>
	//	implement_get_destination_lane_leader() const override
	//{
	//	return nullptr;
	//};
	//std::shared_ptr<NearbyVehicle>
	//	implement_get_destination_lane_follower() const override
	//{
	//	return nullptr;
	//};
	//std::shared_ptr<NearbyVehicle>
	//	implement_get_assisted_vehicle() const override
	//{
	//	return nullptr;
	//};
	//void implement_set_accepted_lane_change_risk_to_leaders(
	//	double value) override {};
	//void implement_set_accepted_lane_change_risk_to_follower(
	//	double value) override {};
	//void implement_set_use_linear_lane_change_gap(long value) {};
};