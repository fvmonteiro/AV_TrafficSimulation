#include "CAVController.h"
#include "ConnectedAutonomousVehicle.h"

CAVController::CAVController(const ConnectedAutonomousVehicle& cav,
	bool verbose) : AVController(verbose)
{
	if (verbose)
	{
		std::clog << "Creating CAV control manager\n";
	}
	add_origin_lane_controllers(cav);
	add_lane_change_adjustment_controller(cav);
	add_cooperative_lane_change_controller(cav);
}