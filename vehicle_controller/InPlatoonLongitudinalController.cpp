#include "InPlatoonLongitudinalController.h"

InPlatoonLongitudinalController::InPlatoonLongitudinalController(
	const VelocityControllerGains& velocity_controller_gains,
	const ConnectedGains& gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	double filter_brake_limit, double comfortable_acceleration,
	double simulation_time_step,
	std::unordered_map<State, color_t> state_to_color_map,
	bool verbose) :
	LongitudinalController(state_to_color_map, verbose)
{
	/*This controller is only active with a connected leader*/
	AutonomousGains all_zeros { 0.4, 1.0 };
	gap_controller = GapController(simulation_time_step, all_zeros,
		gains, velocity_filter_gain, time_headway_filter_gain,
		comfortable_acceleration, filter_brake_limit, verbose);
	gap_controller.set_connexion(true);
	state = State::vehicle_following; /* always following the preceeding
									  vehicle */
}

double InPlatoonLongitudinalController::
implement_compute_desired_acceleration(
	const EgoVehicle& ego_vehicle, const NearbyVehicle* leader,
	double velocity_reference)
{
	return gap_controller.compute_desired_acceleration(
		ego_vehicle, leader);
}