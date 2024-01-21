#include "PlatoonVehicleController.h"
#include "PlatoonVehicle.h"

PlatoonVehicleController::PlatoonVehicleController(
	const PlatoonVehicle& platoon_vehicle,
	bool verbose) : CAVController(platoon_vehicle, verbose)
{
	if (verbose) std::clog << "Creating Platoon Vehicle control manager\n";

	// TODO major: create controllers 
	/*origin_lane_controller = RealLongitudinalController(platoon_vehicle,
		platoon_vehicle_velocity_gains, platoon_vehicle_autonomous_gains,
		platoon_vehicle_connected_gains, platoon_velocity_filter_gain,
		platoon_time_headway_filter_gain, in_platoon_colors,
		long_controllers_verbose);
	real_leader_controller = &origin_lane_controller;

	virtual_leader_controller = RealLongitudinalController(platoon_vehicle,
		platoon_vehicle_velocity_gains, platoon_vehicle_autonomous_gains,
		platoon_vehicle_connected_gains, platoon_velocity_filter_gain,
		platoon_time_headway_filter_gain, in_platoon_colors,
		long_controllers_verbose);


	available_controllers[ALCType::real_leader] = real_leader_controller;
	available_controllers[ALCType::virtual_leader] =
		&virtual_leader_controller;*/
}