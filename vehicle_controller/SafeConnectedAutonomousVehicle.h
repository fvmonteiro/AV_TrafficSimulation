#pragma once

#include "ConnectedAutonomousVehicle.h"
#include "EgoVehicle.h"

class SafeConnectedAutonomousVehicle :
    public ConnectedAutonomousVehicle
{
public:
    SafeConnectedAutonomousVehicle() = default;
	SafeConnectedAutonomousVehicle(long id, double desired_velocity, 
		double simulation_time_step, double creation_time, 
		bool verbose);

protected:

	SafeConnectedAutonomousVehicle(long id, VehicleType type,
		double desired_velocity, bool is_connected,
		double simulation_time_step,
		double creation_time, bool verbose) :
		ConnectedAutonomousVehicle(id, type, desired_velocity,
			simulation_time_step, creation_time, verbose) {}

private:

	/*[June 2023] Probably could keep this function non-virtual 
	in EgoVehicle class and use polymorphism on ControlManager class */
    //void update_leader(const NearbyVehicle* old_leader) override;
};

