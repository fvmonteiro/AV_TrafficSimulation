/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "OriginLaneLongitudinalController.h"
#include "Vehicle.h"

OriginLaneLongitudinalController::OriginLaneLongitudinalController()
	: LongitudinalController() {}

OriginLaneLongitudinalController::OriginLaneLongitudinalController(
	const Vehicle& ego_vehicle, bool verbose) 
	: LongitudinalController(ego_vehicle, ego_vehicle.get_max_brake(),
		ego_vehicle.get_desired_velocity(), ego_vehicle.get_max_brake(),
		verbose) {
	
	if (verbose) {
		std::clog << "Created origin lane longitudinal controller"
			<< std::endl;
	}
}

OriginLaneLongitudinalController::OriginLaneLongitudinalController(
	const Vehicle& ego_vehicle)
	: OriginLaneLongitudinalController(ego_vehicle, false) {
}

void OriginLaneLongitudinalController::determine_controller_state(
	double ego_velocity, const NearbyVehicle* leader) {

	if (leader == nullptr) { // no vehicle ahead
		state = State::velocity_control;
	}
	else {
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_threshold = compute_gap_threshold(
			ego_vehicle_desired_velocity, compute_velocity_error(
				ego_velocity, leader_velocity));
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}
		if ((leader->get_distance() < gap_threshold)
			&& (leader_velocity < ego_vehicle_desired_velocity)) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}
	}
}