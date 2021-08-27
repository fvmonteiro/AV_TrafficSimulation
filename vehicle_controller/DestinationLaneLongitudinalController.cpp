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

void DestinationLaneLongitudinalController::set_reference_velocity(
	double ego_velocity, double adjustment_speed_factor) {
	double minimum_adjustment_velocity =
		ego_velocity * adjustment_speed_factor;
	if (verbose) {
		std::clog << "new min vel=" << minimum_adjustment_velocity << std::endl;
	}
	reset_desired_velocity_filter(ego_velocity);
	this->ego_reference_velocity = minimum_adjustment_velocity;
}

void DestinationLaneLongitudinalController::determine_controller_state(
	double ego_velocity, const std::shared_ptr<NearbyVehicle> leader) {

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

		if ((gap > gap_threshold) 
			|| (ego_velocity < ego_reference_velocity)) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}
	}
}

bool DestinationLaneLongitudinalController::is_active() const {
	return state != State::uninitialized;
}

bool DestinationLaneLongitudinalController::is_outdated(
	double ego_velocity) const {
	return ego_velocity < desired_velocity_filter.get_current_value();
}

void DestinationLaneLongitudinalController::estimate_follower_time_headway(
	const NearbyVehicle& follower, double ego_max_brake,
	double follower_free_flow_velocity) {

	if (verbose) {
		std::clog << /*"follower_risk=" << accepted_risk_to_follower
			<< ", follower_free_flow_velocity=" << follower_free_flow_velocity
			<< ", follower max brake=" << follower.get_max_brake()
			<< ", follower lambda1 =" << follower.get_lambda_1()
			<< ", ego_max_brake=" << ego_max_brake << ". "
			<<*/ "Updating follower headway from "
			<< destination_lane_follower_time_headway;
	}

	destination_lane_follower_time_headway = compute_time_headway_with_risk(
		follower_free_flow_velocity,
		follower.get_max_brake(), ego_max_brake,
		follower.get_lambda_1(), rho, accepted_risk_to_follower);

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

	bool has_increased = false;

	if (((time - timer_start) >= constant_risk_period)) {
		if ((accepted_risk_to_leader + delta_risk) 
			< max_risk_to_leader) {
			timer_start = time;
			accepted_risk_to_leader += delta_risk;
			has_increased = true;

			if (verbose) {
				std::clog << "\tleader risk updated to " 
					<< accepted_risk_to_leader
					<< std::endl;
			}
		}
		if (accepted_risk_to_leader > intermediate_risk_to_leader
			&& (accepted_risk_to_follower + delta_risk)
			   < max_risk_to_follower) {
			timer_start = time;
			accepted_risk_to_follower += delta_risk;
			has_increased = true; 
			
			if (verbose) {
				std::clog << "\tfollower risk updated to "
					<< accepted_risk_to_follower
					<< std::endl;
			}
		}
	}
	return has_increased;
}

void DestinationLaneLongitudinalController::
compute_intermediate_risk_to_leader(double lambda_1, 
	double lane_change_lambda_1, double max_brake_no_lane_change, 
	double leader_max_brake) {

	double safe_h_no_lane_change = compute_time_headway_with_risk(
		free_flow_velocity, max_brake_no_lane_change, leader_max_brake,
		lambda_1, rho, 0);
	intermediate_risk_to_leader = std::sqrt(2 * (h - safe_h_no_lane_change)
		* ego_max_brake * free_flow_velocity);

	if (verbose) {
		std::clog << "safe no lc h=" << safe_h_no_lane_change
			<< ", h_lc=" << h
			<< ", mid risk to leader="
			<< intermediate_risk_to_leader << std::endl;
	}
}

void DestinationLaneLongitudinalController::reset_accepted_risks() {
	accepted_risk_to_leader = initial_risk;
	accepted_risk_to_follower = initial_risk;
}

void DestinationLaneLongitudinalController::compute_max_risk_to_follower(
	double follower_max_brake) {
	max_risk_to_follower = std::sqrt(
		2 * destination_lane_follower_time_headway 
		* follower_max_brake * free_flow_velocity);
	if (verbose) std::clog << "max risk to follower=" 
		<< max_risk_to_follower << std::endl;
}