#include "TrafficLightLongAVController.h"
#include "TrafficLightALCVehicle.h"

TrafficLightLongAVController::TrafficLightLongAVController(
	const TrafficLightALCVehicle& tl_av, bool verbose)
	: VehicleController(tl_av, verbose), traffic_light_av{ &tl_av }
{
	add_traffic_lights_controller();
}

double TrafficLightLongAVController::implement_compute_desired_acceleration()
{
	if (verbose) std::clog << "Inside get traffic_light_acc_acceleration\n";

	active_longitudinal_controller_type = ALCType::traffic_light_alc;
	if (traffic_light_av->has_next_traffic_light())
	{
		with_traffic_lights_controller.compute_traffic_light_input_parameters(
			*traffic_light_av, *traffic_lights);
	}

	return with_traffic_lights_controller.compute_desired_acceleration(
		*traffic_light_av, nullptr, traffic_light_av->get_desired_velocity());
}

void TrafficLightLongAVController::add_traffic_lights_controller()
{
	with_traffic_lights_controller =
		LongitudinalControllerWithTrafficLights(tl_alc_colors,
			long_controllers_verbose);
	active_longitudinal_controller_type = ALCType::traffic_light_alc;
}