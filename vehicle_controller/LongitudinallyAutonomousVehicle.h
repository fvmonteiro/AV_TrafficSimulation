#pragma once

#include "EgoVehicle.h"
#include "LongAVController.h"

/* Vehicle with autonomous longitudinal control during lane keeping.
Whenever there's lane change intention, the control is handed over
to VISSIM */
class LongitudinallyAutonomousVehicle : public EgoVehicle
{
public:

	LongitudinallyAutonomousVehicle(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose);

protected:
	LongitudinallyAutonomousVehicle(long id, VehicleType type,
		double desired_velocity, bool is_connected,
		double simulation_time_step,
		double creation_time, bool verbose = false);

private:
	LongAVController long_av_controller;

	VehicleController* get_controller() override;
	const VehicleController* get_controller_const() const override;
	double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) override;
	void update_leader(
		std::shared_ptr<const NearbyVehicle>& old_leader) override;
	/* Follows VISSIM's recommendation */
	bool implement_check_lane_change_gaps() override;

	virtual LongAVController* get_long_av_controller() {
		return &long_av_controller;
	};
	virtual const LongAVController* get_long_av_controller_const() 
		const {
		return &long_av_controller;
	};

	double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const override
	{
		return 0.0;
	};
	bool give_lane_change_control_to_vissim() const override
	{
		return true;
	};

	long implement_get_lane_change_request() const override { return 0; };
	double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle) const override {
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
	long implement_get_virtual_leader_id() const override { return 0; };
	void implement_set_accepted_lane_change_risk_to_leaders(
		double value) override {};
	void implement_set_accepted_lane_change_risk_to_follower(
		double value) override {};
	void implement_set_use_linear_lane_change_gap(long value) override {};

};
