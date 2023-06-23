#include "SafeConnectedAutonomousVehicle.h"

SafeConnectedAutonomousVehicle::SafeConnectedAutonomousVehicle(
	long id, double desired_velocity, 
	double simulation_time_step, double creation_time, bool verbose) 
	: SafeConnectedAutonomousVehicle(id, VehicleType::safe_connected_car, 
		desired_velocity, true, simulation_time_step, creation_time, 
		verbose) 
{
	/* [June 2023] CREATE CONTROLLER */
	if (verbose)
	{
		std::clog << "[SafeCAV] constructor done BUT NO CONTROLLER" << std::endl;
	}
}

//void SafeConnectedAutonomousVehicle::update_leader(
//	const NearbyVehicle* old_leader)
//{
//	update_leader_implementation(old_leader, true);
//}