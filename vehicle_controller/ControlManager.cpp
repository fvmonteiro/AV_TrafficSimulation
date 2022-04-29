/*==========================================================================*/
/*  ControlManager.cpp    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include "ACCVehicle.h"
#include "AutonomousVehicle.h"
#include "ConnectedAutonomousVehicle.h"
#include "ControlManager.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"
#include "TrafficLightACCVehicle.h"
#include "VariationLimitedFilter.h"

ControlManager::ControlManager(const VehicleParameters& vehicle_parameters,
	bool verbose) :
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose } {

	if (verbose) {
		std::clog << "Creating control manager " << std::endl;
	}

	this->ego_parameters = vehicle_parameters;

	bool is_long_control_verbose = verbose;

	switch (ego_parameters.type)
	{
	case VehicleType::acc_car:
		create_acc_controllers(vehicle_parameters, is_long_control_verbose);
		break;
	case VehicleType::autonomous_car:
		create_acc_controllers(vehicle_parameters, is_long_control_verbose);
		create_lane_change_adjustment_controller(vehicle_parameters, 
			is_long_control_verbose);
		break;
	case VehicleType::connected_car:
		create_acc_controllers(vehicle_parameters, is_long_control_verbose);
		create_lane_change_adjustment_controller(vehicle_parameters,
			is_long_control_verbose);
		create_cooperative_lane_change_controller(vehicle_parameters,
			is_long_control_verbose);
		break;
	case VehicleType::traffic_light_acc_car:
	case VehicleType::traffic_light_cacc_car: // both get the same controller
		with_traffic_lights_controller =
			LongitudinalControllerWithTrafficLights(is_long_control_verbose);
		break;
	default:
		break;
	}
}

ControlManager::ControlManager(const VehicleParameters& vehicle_parameters)
	: ControlManager(vehicle_parameters, false) {}

void ControlManager::create_acc_controllers(
	const VehicleParameters& vehicle_parameters,
	bool verbose)
{
	origin_lane_controller = RealLongitudinalController(
		vehicle_parameters, desired_velocity_controller_gains,
		autonomous_real_following_gains, connected_real_following_gains,
		verbose);
	/* Note: the end of lane controller could have special vel control gains
	but we want to see how the ego vehicle responds to a stopped vehicle. */
	end_of_lane_controller = RealLongitudinalController(
		vehicle_parameters,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		verbose);
	/* The end of the lane is seen as a stopped vehicle. We can pretend
	this stopped vehicle has a lower max brake so that the time headway
	will be small. THIS IS DONE IN THE VEHICLE CONSTRUCTOR NOW */
	/*end_of_lane_controller.update_time_headway(
		ego_parameters.lambda_1, ego_parameters.lambda_1_lane_change,
		ego_parameters.max_brake / 2);*/
}

void ControlManager::create_lane_change_adjustment_controller(
	const VehicleParameters& vehicle_parameters,
	bool verbose)
{
	destination_lane_controller = VirtualLongitudinalController(
		vehicle_parameters,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		verbose);
}

void ControlManager::create_cooperative_lane_change_controller(
	const VehicleParameters& vehicle_parameters,
	bool verbose)
{
	gap_generating_controller = VirtualLongitudinalController(
		vehicle_parameters,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		verbose);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here*/
	gap_generating_controller.set_connexion(true);
}

LongitudinalController::State 
	ControlManager::get_longitudinal_controller_state() {
	switch (active_longitudinal_controller) {
	case ACCType::origin_lane:
		return origin_lane_controller.get_state();
	case ACCType::destination_lane:
		return destination_lane_controller.get_state();
	case ACCType::cooperative_gap_generation:
		return gap_generating_controller.get_state();
	case ACCType::end_of_lane:
		return end_of_lane_controller.get_state();
	case ACCType::vissim:
		return LongitudinalController::State::uninitialized;
	default:
		return LongitudinalController::State::uninitialized;
	}
}

LongitudinalControllerWithTrafficLights::State
	ControlManager::get_longitudinal_controller_with_traffic_lights_state()
{
	return with_traffic_lights_controller.get_state();
}

/* DEBUGGING FUNCTIONS --------------------------------------------------- */

double ControlManager::get_reference_gap(double ego_velocity, 
	bool has_lane_change_intention) {
	return origin_lane_controller.compute_desired_gap(ego_velocity,
		has_lane_change_intention);
};

double ControlManager::get_gap_error() const
{
	if (ego_parameters.type == VehicleType::traffic_light_acc_car)
	{
		return with_traffic_lights_controller.get_h1();
	}
	else /* TODO */
	{
		return 0.0;
	}
}

/* ----------------------------------------------------------------------- */

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

void ControlManager::activate_end_of_lane_controller(double time_headway)
{
	end_of_lane_controller.reset_time_headway_filter(time_headway);
	end_of_lane_controller.set_desired_time_headway(time_headway);
}

void ControlManager::activate_origin_lane_controller(double ego_velocity,
	double time_headway, bool is_leader_connected)
{
	origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	origin_lane_controller.reset_time_headway_filter(time_headway);
	update_origin_lane_controller(time_headway, is_leader_connected);
}

void ControlManager::update_origin_lane_controller(
	double time_headway, bool is_leader_connected)
{
	origin_lane_controller.set_desired_time_headway(time_headway);
	origin_lane_controller.set_connexion(is_leader_connected);
}

void ControlManager::update_destination_lane_follower_time_headway(
	double time_headway)
{
	destination_lane_controller.set_follower_time_headway(time_headway);
	/*destination_lane_controller.compute_max_risk_to_follower(
		follower.get_max_brake());*/
}

void ControlManager::activate_destination_lane_controller(double ego_velocity,
	double time_headway, bool is_leader_connected)
{
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ? 
	And force initial value to be the non-lane-changing one? */
	destination_lane_controller.reset_time_headway_filter(time_headway);
	update_destination_lane_controller(ego_velocity, time_headway,
		is_leader_connected);
}

void ControlManager::update_destination_lane_controller(double ego_velocity,
	double time_headway, bool is_leader_connected)
{
	destination_lane_controller.set_desired_time_headway(time_headway);
	destination_lane_controller.set_connexion(is_leader_connected);
	origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
}

void ControlManager::update_gap_generation_controller(double ego_velocity,
	double time_headway)
{
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ? 
	And force initial value to be the non-lane-changing one? */
	gap_generating_controller.reset_time_headway_filter(time_headway);
	gap_generating_controller.set_desired_time_headway(time_headway);
	gap_generating_controller.reset_leader_velocity_filter(
		ego_velocity);
}

//void ControlManager::update_origin_lane_leader(double ego_velocity,
//	bool had_leader, const NearbyVehicle& leader) {
//
//	double new_max_brake = leader.get_max_brake();
//	VehicleType new_type = leader.get_type();
//
//	/* To avoid repeated computations, we only recompute time headway if
//	the new leader is different from the previous one. */
//	if ((std::abs(new_max_brake - origin_lane_leader_max_brake) > 0.5)
//		|| (origin_lane_leader_type != new_type)) 
//	{
//		origin_lane_leader_max_brake = new_max_brake;
//		origin_lane_leader_type = new_type;
//		/* If both vehicles are connected, we use the connected value
//		of lambda_1. */
//		double appropriate_lambda_1, appropriate_lambda_1_lane_change;
//		if (ego_parameters.is_connected 
//			&& (leader.is_connected())) 
//		{
//			origin_lane_controller.set_connexion(true);
//			appropriate_lambda_1 = ego_parameters.lambda_1_connected;
//			appropriate_lambda_1_lane_change =
//				ego_parameters.lambda_1_lane_change_connected;
//		}
//		else 
//		{
//			origin_lane_controller.set_connexion(false);
//			appropriate_lambda_1 = ego_parameters.lambda_1;
//			appropriate_lambda_1_lane_change =
//				ego_parameters.lambda_1_lane_change;
//		}
//		origin_lane_controller.update_time_headway(appropriate_lambda_1, 
//			appropriate_lambda_1_lane_change, new_max_brake);
//	}
//	if (!had_leader) 
//	{
//		origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
//	}
//}

//void ControlManager::update_destination_lane_leader(
//	double ego_velocity, const NearbyVehicle& leader) {
//	
//	double new_max_brake = leader.get_max_brake();
//	VehicleType new_type = leader.get_type();
//
//	/* To avoid repeated computations, we only recompute time headway if
//	the new leader is different from the previous one. */
//	if ((std::abs(new_max_brake - destination_lane_leader_max_brake) > 0.5)
//		|| destination_lane_leader_type != new_type) {
//
//		destination_lane_leader_max_brake = new_max_brake;
//		destination_lane_leader_type = new_type;
//		
//		/* If both vehicles are connected, we use the connected value
//		of lambda_1. */
//		double appropriate_lambda_1, appropriate_lambda_1_lane_change;
//		if (ego_parameters.is_connected
//			&& (leader.is_connected())) {
//			destination_lane_controller.set_connexion(true);
//			appropriate_lambda_1 = ego_parameters.lambda_1_connected;
//			appropriate_lambda_1_lane_change =
//				ego_parameters.lambda_1_lane_change_connected;
//		}
//		else {
//			destination_lane_controller.set_connexion(false);
//			appropriate_lambda_1 = ego_parameters.lambda_1;
//			appropriate_lambda_1_lane_change =
//				ego_parameters.lambda_1_lane_change;
//		}
//		destination_lane_controller.update_time_headway(
//			appropriate_lambda_1, appropriate_lambda_1_lane_change,
//			new_max_brake);
//		
//		/*destination_lane_controller.compute_intermediate_risk_to_leader(
//			appropriate_lambda_1,
//			ego_parameters.lambda_1_lane_change,
//			ego_parameters.max_brake,
//			new_max_brake);
//		destination_lane_controller.compute_max_risk_to_leader();*/
//	}
//	//if (!had_leader) {} /*try using this condition*/
//	destination_lane_controller.reset_leader_velocity_filter(
//		ego_velocity);
//}

//void ControlManager::update_assisted_vehicle(
//	double ego_velocity, const NearbyVehicle& assisted_vehicle) {
//
//	// Sanity check
//	if (!(assisted_vehicle.is_connected() && ego_parameters.is_connected)) {
//		std::clog << "CODING ERROR: updated_assisted_vehicle called when: "
//			<< std::endl;
//		if (!assisted_vehicle.is_connected()) {
//			std::clog << "nearby veh is not connected" << std::endl;
//		}
//		if (!ego_parameters.is_connected) {
//			std::clog << "ego veh is not connected" << std::endl;
//		}
//	}
//
//	double new_max_brake = assisted_vehicle.get_max_brake();
//
//	/* To avoid repeated computations, we only recompute time headway if
//	the new leader is different from the previous one. */
//	if ((std::abs(new_max_brake - assisted_vehicle_max_brake) > 0.5)) {
//
//		assisted_vehicle_max_brake = new_max_brake;
//
//		gap_generating_controller.update_time_headway(
//			ego_parameters.lambda_1_connected, 
//			ego_parameters.lambda_1_lane_change_connected,
//			new_max_brake);
//
//		// gap_generating_controller.compute_max_risk_to_leader();
//	}
//	//if (!had_leader) {} /*try using this condition*/
//	gap_generating_controller.reset_leader_velocity_filter(
//		ego_velocity);
//}

// TODO: create two functions, one for connected one for regular
//void ControlManager::update_follower_time_headway(
//	NearbyVehicle& follower) {
//
//	if (ego_parameters.is_connected && follower.is_connected()) {
//
//		destination_lane_controller.set_follower_time_headway(
//			follower.get_h_to_incoming_vehicle());
//		destination_lane_controller.compute_max_risk_to_follower(
//			follower.get_max_brake());
//
//		if (verbose) {
//			std::clog << destination_lane_controller.get_follower_time_headway()
//				<< std::endl;
//		}
//	}
//	else if (std::abs(follower.get_max_brake() 
//		- destination_lane_follower_max_brake) > 0.5) {
//		/* (Re)estimate future follower's headway if the follower is
//		sufficiently different from the previous. */
//		double estimated_follower_free_flow_velocity =
//			ego_parameters.desired_velocity;
//		double ego_max_brake = ego_parameters.max_brake;
//
//		follower.compute_safe_gap_parameters();
//		destination_lane_controller.estimate_follower_time_headway(
//			follower, ego_max_brake, 
//			estimated_follower_free_flow_velocity);
//		destination_lane_controller.compute_max_risk_to_follower(
//			follower.get_max_brake());
//	}
//}

void ControlManager::reset_origin_lane_velocity_controller(
	double ego_velocity) {
	origin_lane_controller.reset_desired_velocity_filter(ego_velocity);
	origin_lane_controller.reset_velocity_error_integrator();
}

double ControlManager::use_vissim_desired_acceleration(
	const EgoVehicle& ego_vehicle) {

	/* We need to ensure the velocity filter keeps active
	while VISSIM has control of the car to guarantee a smooth
	movement when taking back control */
	if (ego_vehicle.has_leader()) {
		origin_lane_controller.update_leader_velocity_filter(
			ego_vehicle.get_leader()->compute_velocity(
				ego_vehicle.get_velocity()));
	}

	active_longitudinal_controller = ACCType::vissim;
	return ego_vehicle.get_vissim_acceleration();
}

/* [Feb 11, 22] Functions for one style of coding --------------------- */
//double ControlManager::get_origin_lane_desired_acceleration(
//	const EgoVehicle& ego_vehicle) 
//{
//	if (verbose) std::clog << "Origin lane controller" << std::endl;
//	return origin_lane_controller.compute_desired_acceleration(
//			ego_vehicle, ego_vehicle.get_leader(),
//			ego_vehicle.get_free_flow_velocity());
//}
//
//double ControlManager::get_end_of_lane_desired_acceleration(
//	const EgoVehicle& ego_vehicle) 
//{
//	double desired_acceleration = 100;
//	/* When the lane change direction equals the preferred relative
//	lane, it means the vehicle is moving into the destination lane.
//	At this point, we don't want to use this controller anymore. */
//	if ((ego_vehicle.get_preferred_relative_lane()
//		!= ego_vehicle.get_active_lane_change_direction())
//		&& (ego_vehicle.get_lane_end_distance() > -1)) {
//		// -1 cause end of lane dist can get small negative values
//
//		if (verbose) {
//			std::clog << "End of lane controller"
//				<< std::endl;
//		}
//		/* We simulate a stopped vehicle at the end of
//		the lane to force the vehicle to stop before the end of
//		the lane. */
//		std::shared_ptr<NearbyVehicle> virtual_vehicle =
//			std::shared_ptr<NearbyVehicle>(new
//				NearbyVehicle(1, RelativeLane::same, 1));
//		virtual_vehicle->set_relative_velocity(
//			ego_vehicle.get_velocity());
//		virtual_vehicle->set_distance(
//			ego_vehicle.get_lane_end_distance());
//		virtual_vehicle->set_length(0.0);
//
//		LongitudinalController::State old_state =
//			end_of_lane_controller.get_state();
//		double end_of_lane_desired_acceleration =
//			end_of_lane_controller.compute_desired_acceleration(
//				ego_vehicle, virtual_vehicle,
//				ego_parameters.desired_velocity);
//
//		/* This controller is only active when it's at vehicle
//		following mode */
//		if (end_of_lane_controller.get_state()
//			== LongitudinalController::State::vehicle_following) {
//			/* To avoid sudden high decelerations */
//			if (old_state
//				== LongitudinalController::State::velocity_control) {
//				end_of_lane_controller.reset_leader_velocity_filter(
//					ego_vehicle.get_velocity());
//			}
//			desired_acceleration = end_of_lane_desired_acceleration;
//		}
//	}
//	return desired_acceleration;
//}
//
//double ControlManager::get_destination_lane_desired_acceleration(
//	const EgoVehicle& ego_vehicle, bool end_of_lane_controller_is_active) 
//{
//	double origin_lane_reference_velocity;
//	double ego_velocity = ego_vehicle.get_velocity();
//	double desired_acceleration = 100;
//
//	/* Get the possible max vel at the origin lane */
//	if (origin_lane_controller.get_state()
//		== LongitudinalController::State::vehicle_following) {
//		origin_lane_reference_velocity = ego_vehicle.get_leader()
//			->compute_velocity(ego_velocity);
//	}
//	else {
//		origin_lane_reference_velocity =
//			ego_parameters.desired_velocity;
//	}
//
//	/* We only activate if the vehicle has a destination lane
//	leader AND
//	(the velocity reference at the origin lane is lower than the one at
//	the destination lane OR the end of lane controller is active)*/
//	if (ego_vehicle.has_destination_lane_leader()
//		&& ((ego_vehicle.get_destination_lane_leader()
//			->compute_velocity(ego_velocity)
//				> origin_lane_reference_velocity - min_overtaking_rel_vel)
//			|| end_of_lane_controller_is_active)) {
//
//		if (verbose) std::clog << "Dest. lane controller"
//				<< std::endl;
//
//		std::shared_ptr<NearbyVehicle> dest_lane_leader =
//			ego_vehicle.get_destination_lane_leader();
//		double ego_velocity = ego_vehicle.get_velocity();
//		double reference_velocity = determine_low_velocity_reference(
//			ego_velocity, *dest_lane_leader);
//
//		/* If the ego vehicle is braking hard due to conditions on
//		the current lane, the destination lane controller, which
//		uses comfortable constraints, must be updated. */
//		if ((active_longitudinal_controller
//			!= ActiveACC::destination_lane)
//			&& destination_lane_controller.is_outdated(ego_velocity)) {
//			destination_lane_controller.reset_desired_velocity_filter(
//				ego_velocity);
//		}
//
//		desired_acceleration =
//			destination_lane_controller.compute_desired_acceleration(
//				ego_vehicle, dest_lane_leader, reference_velocity);
//	}
//	return desired_acceleration;
//}
//
//double ControlManager::get_cooperative_desired_acceleration(
//	const EgoVehicle& ego_vehicle) 
//{
//	double desired_acceleration = 100;
//	if (ego_vehicle.is_cooperating_to_generate_gap()) {
//		if (verbose) {
//			std::clog << "Gap generating controller"
//				<< std::endl;
//		}
//		std::shared_ptr<NearbyVehicle> assited_vehicle =
//			ego_vehicle.get_assisted_vehicle();
//		double ego_velocity = ego_vehicle.get_velocity();
//		double reference_velocity = determine_low_velocity_reference(
//			ego_velocity, *assited_vehicle);
//
//		/* If the ego vehicle is braking hard due to conditions on
//		the current lane, the gap generating controller, which
//		uses comfortable constraints, must be updated. */
//		if ((active_longitudinal_controller
//			!= ActiveACC::cooperative_gap_generation)
//			&& gap_generating_controller.is_outdated(ego_velocity)) {
//			gap_generating_controller.reset_desired_velocity_filter(
//				ego_velocity);
//		}
//
//		desired_acceleration =
//			gap_generating_controller.compute_desired_acceleration(
//				ego_vehicle, assited_vehicle, reference_velocity);
//	}
//	return desired_acceleration;
//}
/* -------------------------------------------------------------------- */

double ControlManager::get_acc_desired_acceleration(
	const ACCVehicle& ego_vehicle)
{
	if (ego_vehicle.has_lane_change_intention() || 
		ego_vehicle.is_lane_changing())
	{
		return use_vissim_desired_acceleration(ego_vehicle);
	}

	std::unordered_map<ACCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	bool end_of_lane_controller_is_active =
		get_end_of_lane_desired_acceleration(ego_vehicle,
			possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_av_desired_acceleration(
	const AutonomousVehicle& ego_vehicle)
{
	if (ego_vehicle.is_vissim_controlling_lane_change()
		&& (ego_vehicle.has_lane_change_intention() 
			|| ego_vehicle.is_lane_changing()))
	{
		return use_vissim_desired_acceleration(ego_vehicle);
	}

	std::unordered_map<ACCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	get_destination_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_cav_desired_acceleration(
	const ConnectedAutonomousVehicle& ego_vehicle)
{
	std::unordered_map<ACCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	get_destination_lane_desired_acceleration(ego_vehicle,
		possible_accelerations);
	get_cooperative_desired_acceleration(ego_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_traffic_light_acc_acceleration(
	const TrafficLightACCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	if (verbose) std::clog << "Inside get traffic_light_acc_acceleration\n";

	std::unordered_map<LongitudinalControllerWithTrafficLights::State, double>
		possible_accelerations;

	with_traffic_lights_controller.get_nominal_input(possible_accelerations);
	with_traffic_lights_controller.compute_vehicle_following_input(
		ego_vehicle, possible_accelerations);
	with_traffic_lights_controller.compute_velocity_control_input(
		ego_vehicle, possible_accelerations);
	with_traffic_lights_controller.compute_traffic_light_input(
		ego_vehicle, traffic_lights, possible_accelerations);

	active_longitudinal_controller = ACCType::traffic_light_acc;

	double ret = with_traffic_lights_controller.
		choose_acceleration(ego_vehicle, possible_accelerations);

	return ret;
}

void ControlManager::print_traffic_lights(const EgoVehicle& ego,
	const std::unordered_map<int, TrafficLight>& traffic_lights) {
	std::clog << "veh id=" << ego.get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;

	if (verbose) std::clog << "Inside placeholder function\n"
		<< "Getting nominal input" << std::endl;

	std::unordered_map<LongitudinalControllerWithTrafficLights::State, double>
		possible_accelerations;

	with_traffic_lights_controller.get_nominal_input(possible_accelerations);
}

/* [Feb 11, 22] Functions for one style of coding --------------------- */

bool ControlManager::get_origin_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations) {

	if (verbose) std::clog << "Origin lane controller" << std::endl;
	possible_accelerations[ACCType::origin_lane] = 
		origin_lane_controller.compute_desired_acceleration(
		ego_vehicle, ego_vehicle.get_leader(),
		ego_vehicle.get_free_flow_velocity());

	return true;
}

bool ControlManager::get_end_of_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations) {

	bool is_active = false;

	/* When the lane change direction equals the preferred relative
	lane, it means the vehicle is moving into the destination lane.
	At this point, we don't want to use this controller anymore. */
	if ((ego_vehicle.get_preferred_relative_lane()
		!= ego_vehicle.get_active_lane_change_direction())
		&& (ego_vehicle.get_lane_end_distance() > -1)) {
		// -1 cause end of lane dist can get small negative values

		if (verbose) {
			std::clog << "End of lane controller"
				<< std::endl;
		}
		/* We simulate a stopped vehicle at the end of
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

		LongitudinalController::State old_state =
			end_of_lane_controller.get_state();
		double desired_acceleration =
			end_of_lane_controller.compute_desired_acceleration(
				ego_vehicle, virtual_vehicle,
				ego_parameters.desired_velocity);

		/* This controller is only active when it's at vehicle 
		following mode */
		if (end_of_lane_controller.get_state()
			== LongitudinalController::State::vehicle_following) {
			/* To avoid sudden high decelerations */
			if (old_state
				== LongitudinalController::State::velocity_control) {
				end_of_lane_controller.reset_leader_velocity_filter(
					ego_vehicle.get_velocity());
			}
			possible_accelerations[ACCType::end_of_lane] = 
				desired_acceleration;
			is_active = true;
		}
	}
	return is_active;
}

bool ControlManager::get_destination_lane_desired_acceleration(
	const AutonomousVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations) {

	bool is_active = false;
	double origin_lane_reference_velocity;
	double ego_velocity = ego_vehicle.get_velocity();

	bool end_of_lane_controller_is_active = 
		end_of_lane_controller.get_state()
		== LongitudinalController::State::vehicle_following;
	/* Get the possible max vel at the origin lane */
	if (origin_lane_controller.get_state()
		== LongitudinalController::State::vehicle_following) {
		origin_lane_reference_velocity = ego_vehicle.get_leader()
			->compute_velocity(ego_velocity);
	}
	else {
		origin_lane_reference_velocity =
			ego_parameters.desired_velocity;
	}

	/* We only activate if the vehicle has a destination lane
	leader AND
	(the velocity reference at the origin lane is lower than the one at
	the destination lane OR the end of lane controller is active)*/
	if (ego_vehicle.has_destination_lane_leader()
		&& ((ego_vehicle.get_destination_lane_leader()
			->compute_velocity(ego_velocity)
				> origin_lane_reference_velocity - min_overtaking_rel_vel)
			|| end_of_lane_controller_is_active)) {
		
		if (verbose) {
			std::clog << "Dest. lane controller"
				<< std::endl;
		}
		std::shared_ptr<NearbyVehicle> dest_lane_leader =
			ego_vehicle.get_destination_lane_leader();
		double ego_velocity = ego_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *dest_lane_leader);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller
			!= ACCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity)) {
			destination_lane_controller.reset_desired_velocity_filter(
				ego_velocity);
		}

		possible_accelerations[ACCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				ego_vehicle, dest_lane_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

bool ControlManager::get_cooperative_desired_acceleration(
	const ConnectedAutonomousVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations) {

	bool is_active = false;

	if (ego_vehicle.is_cooperating_to_generate_gap()) {
		if (verbose) {
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		std::shared_ptr<NearbyVehicle> assited_vehicle =
			ego_vehicle.get_assisted_vehicle();
		double ego_velocity = ego_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *assited_vehicle);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the gap generating controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller
			!= ACCType::cooperative_gap_generation)
			&& gap_generating_controller.is_outdated(ego_velocity)) {
			gap_generating_controller.reset_desired_velocity_filter(
				ego_velocity);
		}
			
		possible_accelerations[ACCType::cooperative_gap_generation] =
			gap_generating_controller.compute_desired_acceleration(
				ego_vehicle, assited_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}

/* ------------------------------------------------------------------------ */

double ControlManager::choose_minimum_acceleration(
	std::unordered_map<ACCType, double>& possible_accelerations)
{
	double desired_acceleration = 1000; // any high value
	for (const auto& it : possible_accelerations) {
		if (verbose) std::clog << active_ACC_to_string(it.first)
			<< ", " << it.second << std::endl;

		if (it.second < desired_acceleration) {
			desired_acceleration = it.second;
			active_longitudinal_controller = it.first;
		}
	}

	if (verbose) std::clog << "des accel=" << desired_acceleration << std::endl;

	return desired_acceleration;
}

double ControlManager::determine_low_velocity_reference(double ego_velocity,
	const NearbyVehicle& other_vehicle) {
	double leader_velocity =
		other_vehicle.compute_velocity(ego_velocity);
	/* The fraction of the leader speed has to vary. Otherwise, vehicles 
	take too long to create safe gaps at low speeds*/
	double vel_fraction;
	if (leader_velocity < 5 / 3.6) {
		vel_fraction = 0;
	}
	else if (leader_velocity < 40 / 3.6) {
		vel_fraction = 0.5;
	}
	else  {
		vel_fraction = 0.8;
	}
	double reference_velocity = std::min(
		leader_velocity * vel_fraction,
		ego_velocity);
	return reference_velocity;
}

double ControlManager::compute_safe_lane_change_gap(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	bool will_accelerate) {

	double safe_time_headway_gap = compute_safe_time_headway_gap(
		ego_vehicle.get_velocity(), ego_vehicle.has_lane_change_intention(),
		other_vehicle);
	/* TODO: the function calls do not make much sense here.
	You call this method from the ego vehicle, and then call an ego vehicle 
	method in here.*/
	/*double collision_free_gap = ego_vehicle.compute_exact_collision_free_gap(
		other_vehicle);*/
	double transient_gap = lateral_controller.compute_transient_gap(
		ego_vehicle, other_vehicle, will_accelerate);

	return safe_time_headway_gap /*collision_free_gap*/ + transient_gap;
}

double ControlManager::compute_safe_time_headway_gap(double ego_velocity,
	bool has_lane_change_intention, const NearbyVehicle& other_vehicle) {
	double time_headway_gap = 0.0;
	//double ego_velocity = ego_vehicle.get_velocity();

	if (other_vehicle.is_ahead()) {
		if (other_vehicle.get_relative_lane() == RelativeLane::same) {
			time_headway_gap = 
				origin_lane_controller.compute_safe_time_headway_gap(
				ego_velocity, has_lane_change_intention);
		}
		else {
			time_headway_gap = 
				destination_lane_controller.compute_safe_time_headway_gap(
				ego_velocity, has_lane_change_intention);
		}
	}
	else {
		double dest_lane_follower_time_headway =
			destination_lane_controller.get_follower_time_headway();
		time_headway_gap =
			destination_lane_controller.compute_time_headway_gap(
				dest_lane_follower_time_headway,
				other_vehicle.compute_velocity(ego_velocity));
	}

	return time_headway_gap;
}

void ControlManager::start_longitudinal_adjustment(double time) {

	if (verbose) std::clog << "Start of long adjust." << std::endl;

	destination_lane_controller.reset_accepted_risks();
	destination_lane_controller.set_timer_start(time);
	/*destination_lane_controller.set_reference_velocity(ego_velocity,
		adjustment_speed_factor);*/
}

//void ControlManager::start_longitudinal_adjustment(double time, 
//	double ego_velocity, double adjustment_speed_factor) {
//
//	if (verbose) std::clog << "Start of long adjust." << std::endl;
//
//	destination_lane_controller.reset_accepted_risks();
//	destination_lane_controller.set_timer_start(time);
//	double reference_velocity = ego_velocity * adjustment_speed_factor;
//	destination_lane_controller.set_reference_velocity(
//		reference_velocity, ego_velocity);
//}

void ControlManager::update_headways_with_risk(const EgoVehicle& ego_vehicle) {

	/* This function has to be substantially changed to work. */

	//if (verbose) {
	//	std::clog << "Considering to increase risk" << std::endl;
	//}

	///* TODO: include difference to check whether vehicles are connected */

	//if (destination_lane_controller.update_accepted_risk(
	//	ego_vehicle.get_time(), ego_vehicle)) {

	//	if (ego_vehicle.has_destination_lane_leader()) {
	//		destination_lane_controller.update_time_headway(
	//			ego_parameters.lambda_1, ego_parameters.lambda_1_lane_change,
	//			ego_vehicle.get_destination_lane_leader()->get_max_brake());
	//	}
	//	/* TODO: should only enter this condition if follower_risk 
	//	was changed 
	//	And even so, we need to recompute the follower's lambda 1
	//	because we erase and rebuild the NearbyVehicle object every time.
	//	Maybe we should avoid this? */
	//	if (ego_vehicle.has_destination_lane_follower()) {
	//		std::shared_ptr<NearbyVehicle> dest_lane_follower =
	//			ego_vehicle.get_destination_lane_follower();

	//		if (ego_parameters.is_connected
	//			&& dest_lane_follower->is_connected()) {
	//			std::clog << "TODO: risk for connected vehicles not "
	//				<< "yet fully coded." << std::endl;
	//		}

	//		double estimated_follower_free_flow_velocity =
	//			ego_parameters.desired_velocity;
	//		double ego_max_brake = ego_parameters.max_brake;
	//		dest_lane_follower->compute_safe_gap_parameters();
	//		destination_lane_controller.estimate_follower_time_headway(
	//			*dest_lane_follower, ego_max_brake,
	//			estimated_follower_free_flow_velocity);
	//	}
	//}
}

std::string ControlManager::active_ACC_to_string(
	ACCType active_longitudinal_controller) {

	switch (active_longitudinal_controller)
	{
	case ACCType::origin_lane:
		return "origin lane controller";
	case ACCType::destination_lane:
		return "destination lane controller";
	case ACCType::cooperative_gap_generation:
		return "gap generation controller";
	case ACCType::end_of_lane:
		return "end-of-lane controller";
	case ACCType::vissim:
		return "vissim controller";
	default:
		return "unknown active longitudinal controller";
	}
}