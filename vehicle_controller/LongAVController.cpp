#include "LongAVController.h"
#include "LongitudinallyAutonomousVehicle.h"

LongAVController::LongAVController(
	const LongitudinallyAutonomousVehicle& longAV, bool verbose)
	: VehicleController(verbose) 
{
	add_vissim_controller();
	add_origin_lane_controllers(longAV);
}