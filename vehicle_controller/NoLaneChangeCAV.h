#pragma once

#include "ConnectedAutonomousVehicle.h"

/* Same as a ConnectedAutonomousVehicle, but cannot perform lane 
changes. */
class NoLaneChangeCAV: public ConnectedAutonomousVehicle
{
public:
	NoLaneChangeCAV(long id, double desired_velocity,
		double simulation_time_step, double creation_time,
		bool verbose = false) :
		ConnectedAutonomousVehicle(id, VehicleType::connected_car,
			desired_velocity, simulation_time_step, creation_time,
			verbose) {};
private:
	/* Does nothing: these vehicles never change lanes */
	void set_desired_lane_change_direction() override {};
};