/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "OriginLaneLongitudinalController.h"
#include "EgoVehicle.h"

OriginLaneLongitudinalController::OriginLaneLongitudinalController()
	: LongitudinalController() {}

OriginLaneLongitudinalController::OriginLaneLongitudinalController(
	const VehicleParameters& ego_parameters, bool verbose)
	: LongitudinalController(ego_parameters, 
		ego_parameters.max_brake,
		ego_parameters.max_brake, verbose) {
	
	if (verbose) {
		std::clog << "Created origin lane longitudinal controller"
			<< std::endl;
	}
}

OriginLaneLongitudinalController::OriginLaneLongitudinalController(
	const VehicleParameters& ego_parameters)
	: OriginLaneLongitudinalController(ego_parameters, false) {
}

//OriginLaneLongitudinalController::OriginLaneLongitudinalController(
//	const EgoVehicle& ego_vehicle, double kg, double kv, bool verbose)
//	: OriginLaneLongitudinalController(ego_vehicle, verbose) {
//	set_vehicle_following_gains(kg, kv);
//}


void OriginLaneLongitudinalController::determine_controller_state(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double reference_velocity) {

	if (leader == nullptr) { // no vehicle ahead
		state = State::velocity_control;
	}
	else {
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
					ego_acceleration),
				compute_acceleration_error(ego_acceleration,
					leader->get_acceleration())
			);
		}
		else {
			gap_threshold = compute_gap_threshold(
				reference_velocity, velocity_error);
		}
		if (state == State::vehicle_following) {
			gap_threshold += hysteresis_bias;
		}

		if (verbose) {
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< std::endl;
		}

		if ((gap < gap_threshold)
			&& (leader_velocity < reference_velocity)) {
			state = State::vehicle_following;
		}
		else {
			state = State::velocity_control;
		}
	}
}