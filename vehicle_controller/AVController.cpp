#include "AVController.h"
#include "AutonomousVehicle.h"

AVController::AVController(const AutonomousVehicle& autonomous_vehicle,
	bool verbose) : VehicleController(verbose)
{
	if (verbose)
	{
		std::clog << "Creating AV control manager\n";
	}
	add_vissim_controller();
	add_origin_lane_controllers(autonomous_vehicle);
	add_lane_change_adjustment_controller(autonomous_vehicle);
}