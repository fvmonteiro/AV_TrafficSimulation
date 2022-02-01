/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "RealLongitudinalController.h"
#include "EgoVehicle.h"

RealLongitudinalController::RealLongitudinalController() :
	LongitudinalController() {}

RealLongitudinalController::RealLongitudinalController(
	const VehicleParameters& ego_parameters,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains, 
	bool verbose) :
	LongitudinalController(ego_parameters, velocity_controller_gains,
		autonomous_gains, connected_gains,
		ego_parameters.max_brake, verbose) {

	if (verbose) {
		std::clog << "Created real longitudinal controller" << std::endl;
	}
}

RealLongitudinalController::RealLongitudinalController(
	const VehicleParameters& ego_parameters,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains) :
	RealLongitudinalController(ego_parameters, velocity_controller_gains,
		autonomous_gains, connected_gains, false) {
}

//OriginLaneLongitudinalController::OriginLaneLongitudinalController(
//	const EgoVehicle& ego_vehicle, double kg, double kv, bool verbose)
//	: OriginLaneLongitudinalController(ego_vehicle, verbose) {
//	set_vehicle_following_gains(kg, kv);
//}

void RealLongitudinalController::update_leader_velocity_filter(
	double leader_velocity) {
	leader_velocity_filter.apply_filter(leader_velocity);
}

void RealLongitudinalController::determine_controller_state(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double reference_velocity) {

	if (leader == nullptr) { // no vehicle ahead
		state = State::velocity_control;
	}
	else {
		bool has_lane_change_intention = ego_vehicle.has_lane_change_intention();
		double gap = ego_vehicle.compute_gap(leader);
		//double gap = ego_vehicle.compute_gap(leader);
		double ego_velocity = ego_vehicle.get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double velocity_error = compute_velocity_error(
			ego_velocity, leader_velocity);

		double gap_threshold;
		if (is_connected) {
			double ego_acceleration = ego_vehicle.get_acceleration();
			gap_threshold = compute_gap_threshold(
				reference_velocity,
				velocity_error, 
				estimate_gap_error_derivative(velocity_error, 
					ego_acceleration, has_lane_change_intention),
				compute_acceleration_error(ego_acceleration,
					leader->get_acceleration()),
				has_lane_change_intention
			);
		}
		else {
			gap_threshold = compute_gap_threshold(
				reference_velocity, velocity_error, has_lane_change_intention);
		}
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}

		if ((gap < gap_threshold)
			&& (leader_velocity < reference_velocity)) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}

		if (verbose) {
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}

	}
}