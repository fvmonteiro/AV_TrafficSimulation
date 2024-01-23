#pragma once

#include "ACCVehicleController.h"
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

protected:
	void set_controller(ACCVehicleController* controller);

private:
	std::unique_ptr<ACCVehicleController> controller_exclusive{ nullptr };
	ACCVehicleController* acc_vehicle_controller;

	void implement_create_controller() override;
	double implement_compute_desired_acceleration(
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
	bool implement_check_lane_change_gaps() override;

	long implement_get_lane_change_request() const override { return 0; };
	double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle) const override {
		return 0.0;
	};
	NearbyVehicle* implement_get_destination_lane_leader() const override
	{
		return nullptr;
	};
	NearbyVehicle*
		implement_get_destination_lane_follower() const override
	{
		return nullptr;
	};
	NearbyVehicle*
		implement_get_assisted_vehicle() const override
	{
		return nullptr;
	};
	long implement_get_virtual_leader_id() const override { return 0; };
	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override {};
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override {};
	void implement_set_use_linear_lane_change_gap(long value) override {};

};
