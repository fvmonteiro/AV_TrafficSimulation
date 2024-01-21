#include "ACCVehicle.h"
#include "ACCVehicleController.h"

ACCVehicleController::ACCVehicleController(const ACCVehicle& acc_vehicle,
	bool verbose) : VehicleController(verbose)
{
	if (verbose)
	{
		std::clog << "Creating ACC control manager\n";
	}
	add_vissim_controller();
	add_origin_lane_controllers(acc_vehicle);
}