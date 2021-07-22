/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "DestinationLaneLongitudinalController.h"
#include "Vehicle.h"

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController()
	: LongitudinalController() {}

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController(
	const Vehicle& ego_vehicle, bool verbose)
	: LongitudinalController(ego_vehicle, ego_vehicle.get_max_brake() / 2,
		ego_vehicle.get_current_velocity() * ego_vehicle.get_adjustment_speed_factor(),
		ego_vehicle.get_comfortable_brake(), verbose) {

	if (verbose) {
		std::clog << "Created destination lane longitudinal controller"
			<< std::endl;
	}
}

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController(
	const Vehicle& ego_vehicle)
	: DestinationLaneLongitudinalController(ego_vehicle, false) {
}

void DestinationLaneLongitudinalController::determine_controller_state(
	double ego_velocity, const NearbyVehicle* leader) {

	if (leader == nullptr) { // no vehicle ahead
		/*If there's no leader this controller should not be active */
		state = State::uninitialized;
	}
	else {
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_threshold = compute_gap_threshold(
			ego_vehicle_desired_velocity, compute_velocity_error(
				ego_velocity, leader_velocity));
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}
		if (leader->get_distance() > gap_threshold) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}
	}
}

bool DestinationLaneLongitudinalController::is_active() {
	return state != State::uninitialized;
}

double DestinationLaneLongitudinalController::estimate_follower_time_headway(
	const NearbyVehicle& follower, double ego_max_brake,
	double ego_desired_velocity) {
	
	double follower_h;

	if (verbose) {
		std::clog << "old follower h=" << get_h() << std::endl;
	}

	follower_h = compute_time_headway(ego_desired_velocity,
		follower.get_max_brake(), ego_max_brake,
		follower.get_lambda_1(), rho);

	if (verbose) {
		std::clog << "new follower h=" << get_h() << std::endl;
	}

	return follower_h;
}