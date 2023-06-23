#include "CAVController.h"
#include "ConnectedAutonomousVehicle.h"

CAVController::CAVController(const ConnectedAutonomousVehicle& cav,
	bool verbose) : AVController(cav, verbose)
{
	add_cooperative_lane_change_controller(cav);
}
