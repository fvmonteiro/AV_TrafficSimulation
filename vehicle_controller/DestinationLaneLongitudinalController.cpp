/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "DestinationLaneLongitudinalController.h"
#include "EgoVehicle.h"

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController()
	: LongitudinalController() {}

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController(
	const EgoVehicle& ego_vehicle, bool verbose)
	: LongitudinalController(ego_vehicle,
		ego_vehicle.get_lane_change_max_brake(),
		0.0, // the reference vel. is set when there is lane change intention
		ego_vehicle.get_comfortable_brake(), verbose) {

	if (verbose) {
		std::clog << "Created destination lane longitudinal controller"
			<< std::endl;
	}
}

DestinationLaneLongitudinalController::DestinationLaneLongitudinalController(
	const EgoVehicle& ego_vehicle)
	: DestinationLaneLongitudinalController(ego_vehicle, false) {
}

void DestinationLaneLongitudinalController::determine_controller_state(
	double ego_velocity, const NearbyVehicle* leader) {

	if (verbose) {
		std::clog << "\tdest lane controller - deciding state"
			<< std::endl;
	}

	if (leader == nullptr) { // no vehicle ahead
		/*If there's no leader this controller should not be active */
		if (verbose) std::clog << "\tno leader" << std::endl;
		state = State::uninitialized;
	}
	else {
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_threshold = compute_gap_threshold(
			ego_reference_velocity, compute_velocity_error(
				ego_velocity, leader_velocity));
		double gap = leader->get_distance() - leader->get_length();
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}

		if (verbose) {
			std::clog << "\tgap=" << gap
				<< ", gap thresh=" << gap_threshold
				<< std::endl;
		}

		if ((gap > gap_threshold) 
			|| (ego_velocity < ego_reference_velocity)) {
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

void DestinationLaneLongitudinalController::estimate_follower_time_headway(
	const NearbyVehicle& follower, double ego_max_brake,
	double follower_free_flow_velocity) {

	if (verbose) {
		std::clog << "Updating follower headway from "
			<< destination_lane_follower_time_headway;
	}

	destination_lane_follower_time_headway = compute_time_headway_with_risk(
		follower_free_flow_velocity,
		follower.get_max_brake(), ego_max_brake,
		follower.get_lambda_1(), rho, accepted_risk);

	if (verbose) {
		std::clog << " to "
			<< destination_lane_follower_time_headway
			<< std::endl;
	}
}

bool DestinationLaneLongitudinalController::update_accepted_risk(
	double time, const EgoVehicle& ego_vehicle) {
	if (verbose) {
		std::clog << "\tt=" << time
			<< ", timer_start=" << timer_start << std::endl;
	}
	if (((time - timer_start) >= constant_risk_period)
		&& (accepted_risk < max_risk)) {
		timer_start = time;
		accepted_risk += delta_risk;
		if (verbose) {
			std::clog << "\trisk updated to " << accepted_risk
				<< std::endl;
		}
		return true;
	}
	return false;
}