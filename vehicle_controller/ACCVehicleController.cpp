#include "ACCVehicle.h"
#include "ACCVehicleController.h"

ACCVehicleController::ACCVehicleController(const ACCVehicle& acc_vehicle,
	bool verbose) : VehicleController(verbose)
	//acc_vehicle{&acc_vehicle}
{
	if (verbose)
	{
		std::clog << "Creating ACC control manager\n";
	}
	add_vissim_controller();
	add_origin_lane_controllers(acc_vehicle);
}

double ACCVehicleController::get_desired_acceleration(
	const ACCVehicle& acc_vehicle)
{
	if (acc_vehicle.has_lane_change_intention() ||
		acc_vehicle.is_lane_changing())
	{
		return get_vissim_desired_acceleration(acc_vehicle);
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(acc_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(acc_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}