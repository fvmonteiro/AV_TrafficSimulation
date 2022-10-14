#pragma once

#include "EgoVehicle.h"

/* Vehicle with autonomous longitudinal control during lane keeping.
Whenever there's lane change intention, the control is handed over
to VISSIM */
class ACCVehicle : public EgoVehicle
{
public:

	ACCVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);

private:
	//void create_controllers() override;
	double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override
	{
		return 0.0;
	};
	bool give_lane_change_control_to_vissim() const override
	{
		return true;
	};

	/* Follows VISSIM's recommendation */
	void set_desired_lane_change_direction() override;
	/* Follows VISSIM's recommendation */
	bool can_start_lane_change() override;

	long create_lane_change_request() override { return 0; };
	double compute_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle) override {
		return 0.0;
	};
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const override
	{
		return nullptr;
	};
	std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const override
	{
		return nullptr;
	};
	std::shared_ptr<NearbyVehicle>
		implement_get_assisted_vehicle() const override
	{
		return nullptr;
	};
	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override {};
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override {};
	void implement_set_use_linear_lane_change_gap(long value) override {};
};