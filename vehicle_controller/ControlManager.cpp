/*==========================================================================*/
/*  ControlManager.cpp    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <unordered_map>

#include "ControlManager.h"
#include "VelocityFilter.h"
#include "NearbyVehicle.h"
#include "EgoVehicle.h"

ControlManager::ControlManager(const EgoVehicle& ego_vehicle, bool verbose) 
	: origin_lane_controller{ OriginLaneLongitudinalController(
		ego_vehicle, 0.2, 1.0, verbose) },
	destination_lane_controller{ DestinationLaneLongitudinalController(
		ego_vehicle, verbose) },
	end_of_lane_controller{ OriginLaneLongitudinalController(
		ego_vehicle, 0.2, 1.0, verbose) },
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose } {
	// TODO: better organize the gain setting of the controllers above
	if (verbose) {
		std::clog << "Creating control manager " << std::endl;
	}

}

ControlManager::ControlManager(const EgoVehicle& ego_vehicle) 
	: ControlManager(ego_vehicle, false) {}

LongitudinalController::State 
	ControlManager::get_longitudinal_controller_state() {
	/*if (states.empty()) {
		return State::velocity_control;
	}
	return states.back(); */
	switch (active_longitudinal_controller) {
	case ActiveLongitudinalController::origin_lane:
		return origin_lane_controller.get_state();
	case ActiveLongitudinalController::destination_lane:
		return destination_lane_controller.get_state();
	case ActiveLongitudinalController::end_of_lane:
		return end_of_lane_controller.get_state();
	case ActiveLongitudinalController::vissim:
		return LongitudinalController::State::uninitialized;
	default:
		return LongitudinalController::State::uninitialized;
	}
}

//void ControlManager::create_destination_lane_controller(
//	const Vehicle& ego_vehicle) {
//	destination_lane_controller = DestinationLaneLongitudinalController(
//			ego_vehicle, verbose);
//}

double ControlManager::compute_drac(double relative_velocity, double gap) {
	/* Deceleration (absolute value) to avoid collision assuming constant
	velocities and instantaneous acceleration. DRAC is only defined when the
	ego vehicle is faster than the leader. We some high negative value
	otherwise */
	//if (relative_velocity > 0) { // ego faster than leader
	//	return relative_velocity * relative_velocity / 2 / gap;
	//}
	return -100;
}

void ControlManager::update_origin_lane_controller(double lambda_1,
	double leader_max_brake, double ego_velocity, bool had_leader) {
	origin_lane_controller.update_safe_time_headway(lambda_1,
		leader_max_brake);
	if (!had_leader) {
		origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
	}
}

void ControlManager::update_destination_lane_controller(double lambda_1,
	double leader_max_brake, double ego_velocity) {
	destination_lane_controller.update_time_headway_with_risk(lambda_1,
		leader_max_brake);
	destination_lane_controller.reset_leader_velocity_filter(ego_velocity);
}

void ControlManager::estimate_follower_time_headway(
	const EgoVehicle& ego_vehicle, NearbyVehicle& follower) {
	double ego_max_brake = ego_vehicle.get_max_brake();
	double estimated_follower_free_flow_velocity = 
		ego_vehicle.get_desired_velocity();
	follower.compute_safe_gap_parameters();
	destination_lane_controller.estimate_follower_time_headway(
		follower, ego_max_brake, estimated_follower_free_flow_velocity);
}

void ControlManager::reset_origin_lane_velocity_controller(
	double ego_velocity) {
	origin_lane_controller.reset_desired_velocity_filter(ego_velocity);
	origin_lane_controller.reset_velocity_error_integrator();
}

double ControlManager::determine_desired_acceleration(const EgoVehicle& ego_vehicle) {

	double desired_acceleration;
	double max_acceleration = ego_vehicle.get_comfortable_acceleration() * 2;
	double desired_acceleration_dest_lane = max_acceleration;
	double desired_acceleration_end_of_lane = max_acceleration;
	//double drac;

	if (verbose) {
		std::clog << "Origin lane controller" << std::endl;
	}

	double desired_acceleration_origin_lane = 
		origin_lane_controller.compute_desired_acceleration(
			ego_vehicle, ego_vehicle.get_leader());

	if (ego_vehicle.has_lane_change_intention()) {
		if (ego_vehicle.get_use_internal_lane_change_decision()) {
	
			std::unordered_map<ActiveLongitudinalController, double>
				possible_accelerations;
			possible_accelerations[ActiveLongitudinalController::origin_lane] =
				desired_acceleration_origin_lane;

			/* Longitudinal control to adjust to destination lane leader */
			if (ego_vehicle.has_destination_lane_leader()) {
				if (verbose) {
					std::clog << "Dest. lane controller"
						<< std::endl;
				}

				desired_acceleration_dest_lane =
					destination_lane_controller.compute_desired_acceleration(
						ego_vehicle, ego_vehicle.get_destination_lane_leader());
				possible_accelerations[
					ActiveLongitudinalController::destination_lane] =
					desired_acceleration_dest_lane;
			}
			
			/* Longitudinal control to wait at lane's end while waiting
			for appropriate lane change gap. Without this, vehicles might 
			miss a desired exit. */
			if ((ego_vehicle.get_active_lane_change() == 0)
				&& (ego_vehicle.get_lane_end_distance() > 0)) {
				
				if (verbose) {
					std::clog << "End of lane controller"
						<< std::endl;
				}
				
				/* For now, we simulate a stopped vehicle at the end of 
				the lane to force the vehicle to stop before the end of
				the lane. */
				std::shared_ptr<NearbyVehicle> virtual_vehicle =
					std::shared_ptr<NearbyVehicle>(new
						NearbyVehicle(1, RelativeLane::same, 1));
				virtual_vehicle->set_relative_velocity(
					ego_vehicle.get_velocity());
				virtual_vehicle->set_distance(
					ego_vehicle.get_lane_end_distance());
				virtual_vehicle->set_length(0.0);
				desired_acceleration_end_of_lane =
					end_of_lane_controller.compute_desired_acceleration(
						ego_vehicle, virtual_vehicle);
				possible_accelerations[
					ActiveLongitudinalController::end_of_lane] =
					desired_acceleration_end_of_lane;
			}

			/* Define the actual input */
			desired_acceleration = 1000; // any high value
			for (const auto& it : possible_accelerations) {
				if (it.second < desired_acceleration) {
					desired_acceleration = it.second;
					active_longitudinal_controller = it.first;
				}
			}

			/* If the ego vehicle is braking hard due to conditions on
			the current lane, the destination lane controller, which
			uses comfortable constraints, must be updated. */
			if (active_longitudinal_controller
				!= ActiveLongitudinalController::destination_lane) {
				double ego_velocity = ego_vehicle.get_velocity();
				if (destination_lane_controller.is_outdated(ego_velocity)) {
					destination_lane_controller.set_reference_velocity(
						ego_velocity,
						ego_vehicle.get_adjustment_speed_factor());
				}
			}
		}
		else {
			desired_acceleration = 
				ego_vehicle.get_vissim_acceleration();
			active_longitudinal_controller = 
				ActiveLongitudinalController::vissim;
		}

		if (verbose) {
			std::clog << "\t->Chosen: "
				<< active_longitudinal_controller_to_string(active_longitudinal_controller)
				<< ": " << desired_acceleration
				<< std::endl;
		}

	}
	else { // ACC
		desired_acceleration = desired_acceleration_origin_lane;
		active_longitudinal_controller = 
			ActiveLongitudinalController::origin_lane;
	}
	
	return desired_acceleration;
}

double ControlManager::compute_safe_lane_change_gap(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	bool will_accelerate) {

	double time_headway_gap = compute_time_headway_gap(ego_vehicle, 
		other_vehicle);
	double transient_gap = lateral_controller.compute_transient_gap(
		ego_vehicle, other_vehicle, will_accelerate);

	return time_headway_gap + transient_gap;
}

double ControlManager::compute_time_headway_gap(const EgoVehicle& ego_vehicle,
	const NearbyVehicle& other_vehicle) {
	double time_headway_gap = 0.0;
	double ego_velocity = ego_vehicle.get_velocity();

	if (other_vehicle.is_ahead()) {
		if (other_vehicle.get_relative_lane() == RelativeLane::same) {
			time_headway_gap = origin_lane_controller.compute_desired_gap(
				ego_velocity);
		}
		else {
			time_headway_gap = destination_lane_controller.compute_desired_gap(
				ego_velocity);
		}
	}
	else {
		double dest_lane_follower_time_headway =
			destination_lane_controller.get_follower_time_headway();
		time_headway_gap =
			destination_lane_controller.compute_time_headway_gap(
				dest_lane_follower_time_headway,
				other_vehicle.compute_velocity(ego_velocity));

		//if (verbose) {
		//	std::clog << "t=" << ego_vehicle.get_current_time()
		//		<< ", time headway=" << dest_lane_follower_time_headway
		//		<< ", time headway gap=" << time_headway_gap
		//		<< std::endl;
		//}
	}

	return time_headway_gap;
}

void ControlManager::start_longitudinal_adjustment(double time, 
	double ego_velocity, double adjustment_speed_factor) {
	destination_lane_controller.set_timer_start(time);
	destination_lane_controller.set_reference_velocity(ego_velocity,
		adjustment_speed_factor);
}

void ControlManager::update_accepted_risk(const EgoVehicle& ego_vehicle) {

	if (destination_lane_controller.update_accepted_risk(
		ego_vehicle.get_time(), ego_vehicle)) {
		if (ego_vehicle.has_destination_lane_leader()) {
			std::shared_ptr<NearbyVehicle> dest_lane_leader =
				ego_vehicle.get_destination_lane_leader();
			destination_lane_controller.update_time_headway_with_risk(
				ego_vehicle.get_lane_change_lambda_1(),
				dest_lane_leader->get_max_brake());
		}
		if (ego_vehicle.has_destination_lane_follower()) {
			std::shared_ptr<NearbyVehicle> dest_lane_follower =
				ego_vehicle.get_destination_lane_follower();
			estimate_follower_time_headway(ego_vehicle,
				*ego_vehicle.get_destination_lane_follower());
		}
	}
}

std::string ControlManager::active_longitudinal_controller_to_string(
	ActiveLongitudinalController active_longitudinal_controller) {

	switch (active_longitudinal_controller)
	{
	case ActiveLongitudinalController::origin_lane:
		return "origin lane controller";
	case ActiveLongitudinalController::destination_lane:
		return "destination lane controller";
	case ActiveLongitudinalController::end_of_lane:
		return "end-of-lane controller";
	case ActiveLongitudinalController::vissim:
		return "vissim controller";
	default:
		return "unknown active longitudinal controller";
	}
}