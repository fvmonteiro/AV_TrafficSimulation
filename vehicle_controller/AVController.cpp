#include "AVController.h"
#include "AutonomousVehicle.h"

AVController::AVController(const AutonomousVehicle& av, bool verbose)
	: VehicleController(verbose)
{
	add_vissim_controller();
	add_origin_lane_controllers(av);
	add_lane_change_adjustment_controller(av);
}