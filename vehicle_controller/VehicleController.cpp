/*==========================================================================*/
/*  VehicleController.cpp    										        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include "VehicleController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"
#include "VariationLimitedFilter.h"

VehicleController::VehicleController(const EgoVehicle& ego_vehicle,
	bool verbose) :
	ego_vehicle{ &ego_vehicle },
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose }
{
	long_controllers_verbose = verbose;
	if (verbose)
	{
		std::clog << "Creating vehicle controller" << std::endl;
	}
}

color_t VehicleController::get_longitudinal_controller_color() const
{
	const LongitudinalController* active_long_controller =
		get_active_long_controller();
	if (active_long_controller != nullptr)
	{
		return active_long_controller->get_state_color();
	}
	else
	{
		return WHITE;
	}
}

double VehicleController::get_safe_time_headway() const
{
	return implement_get_safe_time_headway();
	// TODO: copy to derived classes
	//return origin_lane_controller.get_desired_time_headway();
}

double VehicleController::get_current_desired_time_headway() const
{
	return implement_get_current_desired_time_headway();
	// TODO: copy to derived classes
	//return origin_lane_controller.get_current_time_headway();
}

LongitudinalController::State
VehicleController::get_longitudinal_controller_state() const
{
	const LongitudinalController* active_long_controller =
		get_active_long_controller();
	if (active_long_controller != nullptr)
	{
		return active_long_controller->get_state();
	}
	else
	{
		return LongitudinalController::State::uninitialized;
	}
}

void VehicleController::set_traffic_lights(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	this->traffic_lights = &traffic_lights;
}

const LongitudinalController* VehicleController::get_active_long_controller() const
{
	switch (active_longitudinal_controller_type)
	{
	case ALCType::origin_lane:
		return &origin_lane_controller;
	case ALCType::destination_lane:
		return &destination_lane_controller;
	case ALCType::cooperative_gap_generation:
		return &gap_generating_controller;
	case ALCType::end_of_lane:
		return &end_of_lane_controller;
	case ALCType::traffic_light_alc:
		return &with_traffic_lights_controller;
	case ALCType::vissim:
		return &vissim_controller;
	// TODO: safe_longitudinal_controller
	default:
		return nullptr;
	}
}

/* DEBUGGING FUNCTIONS --------------------------------------------------- */

double VehicleController::get_reference_gap() const
{
	return implement_get_reference_gap();
	// TODO: copy to derived class
	//return origin_lane_controller.get_desired_gap(ego_vehicle->get_velocity());
}

double VehicleController::get_gap_error() const
{
	return get_active_long_controller()->get_gap_error();
}

/* ----------------------------------------------------------------------- */

double VehicleController::compute_drac(double relative_velocity, double gap)
{
	/* Deceleration (absolute value) to avoid collision assuming constant
	velocities and instantaneous acceleration. DRAC is only defined when the
	ego vehicle is faster than the leader. We some high negative value
	otherwise */
	//if (relative_velocity > 0) { // ego faster than leader
	//	return relative_velocity * relative_velocity / 2 / gap;
	//}
	return -100;
}

void VehicleController::activate_end_of_lane_controller(double time_headway)
{
	end_of_lane_controller.reset_time_headway_filter(time_headway);
	end_of_lane_controller.set_desired_time_headway(time_headway);
}

void VehicleController::activate_destination_lane_controller(
	double leader_velocity,
	double time_headway, bool is_leader_connected)
{
	//destination_lane_controller.smooth_start_leader_velocity_filter();
	destination_lane_controller.reset_leader_velocity_filter(leader_velocity);
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	destination_lane_controller.reset_time_headway_filter(time_headway);
	update_destination_lane_controller(time_headway, is_leader_connected);
}

void VehicleController::update_origin_lane_controller(
	double time_headway, bool is_leader_connected)
{
	if (verbose)
	{
		std::clog << "Setting orig lane ctrl h_r = "
			<< time_headway << std::endl;
	}
	origin_lane_controller.set_desired_time_headway(time_headway);
	origin_lane_controller.connect_gap_controller(is_leader_connected);
}

void VehicleController::update_destination_lane_controller(
	double time_headway, bool is_leader_connected)
{
	destination_lane_controller.smooth_start_leader_velocity_filter();
	destination_lane_controller.set_desired_time_headway(time_headway);
	destination_lane_controller.connect_gap_controller(is_leader_connected);
	//destination_lane_controller.reset_leader_velocity_filter(ego_velocity);
}

void VehicleController::update_destination_lane_follower_time_headway(
	double time_headway)
{
	destination_lane_controller.set_follower_time_headway(time_headway);
}

void VehicleController::update_gap_generation_controller(
	double time_headway)
{
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	gap_generating_controller.reset_time_headway_filter(time_headway);
	gap_generating_controller.set_desired_time_headway(time_headway);
	gap_generating_controller.reset_leader_velocity_filter(
		ego_vehicle->get_velocity());
}

void VehicleController::reset_origin_lane_velocity_controller()
{
	//if (verbose)
	//{
	//	std::clog << "Resetting orig lane ctrl vel ctrl." << std::endl;
	//}
	origin_lane_controller.reset_velocity_controller(ego_vehicle->get_velocity());
}

bool VehicleController::is_in_free_flow_at_origin_lane() const
{
	return origin_lane_controller.get_state()
		== SwitchedLongitudinalController::State::velocity_control;
}

double VehicleController::compute_desired_acceleration()
{
	/* [June 2023] Call an abstract method */
	return 0.0;
}

//double VehicleController::get_desired_acceleration(
//	const LongitudinallyAutonomousVehicle& acc_vehicle)
//{
//	if (acc_vehicle.has_lane_change_intention() ||
//		acc_vehicle.is_lane_changing())
//	{
//		return get_vissim_desired_acceleration(acc_vehicle);
//	}
//
//	std::unordered_map<ALCType, double>
//		possible_accelerations;
//	get_origin_lane_desired_acceleration(acc_vehicle,
//		possible_accelerations);
//	get_end_of_lane_desired_acceleration(acc_vehicle,
//		possible_accelerations);
//
//	return choose_minimum_acceleration(possible_accelerations);
//}
//
//double VehicleController::get_desired_acceleration(
//	const AutonomousVehicle& autonomous_vehicle)
//{
//	if (autonomous_vehicle.is_vissim_controlling_lane_change()
//		&& (autonomous_vehicle.has_lane_change_intention()
//			|| autonomous_vehicle.is_lane_changing()))
//	{
//		return get_vissim_desired_acceleration(autonomous_vehicle);
//	}
//
//	std::unordered_map<ALCType, double>
//		possible_accelerations;
//	get_origin_lane_desired_acceleration(autonomous_vehicle,
//		possible_accelerations);
//	get_end_of_lane_desired_acceleration(autonomous_vehicle,
//		possible_accelerations);
//	get_destination_lane_desired_acceleration(autonomous_vehicle,
//		possible_accelerations);
//
//	return choose_minimum_acceleration(possible_accelerations);
//}
//
//double VehicleController::get_desired_acceleration(
//	const ConnectedAutonomousVehicle& cav)
//{
//	std::unordered_map<ALCType, double>
//		possible_accelerations;
//	get_origin_lane_desired_acceleration(cav,
//		possible_accelerations);
//	get_end_of_lane_desired_acceleration(cav,
//		possible_accelerations);
//	get_destination_lane_desired_acceleration(cav,
//		possible_accelerations);
//	get_cooperative_desired_acceleration(cav,
//		possible_accelerations);
//
//	return choose_minimum_acceleration(possible_accelerations);
//}
//
//double VehicleController::get_desired_acceleration(
//	const PlatoonVehicle& platoon_vehicle)
//{
//	std::unordered_map<ALCType, double> possible_accelerations;
//	
//	get_origin_lane_desired_acceleration(platoon_vehicle,
//		possible_accelerations);
//	get_end_of_lane_desired_acceleration(platoon_vehicle,
//		possible_accelerations);
//	if (platoon_vehicle.is_platoon_leader())
//	{
//		get_destination_lane_desired_acceleration(platoon_vehicle,
//			possible_accelerations);
//	}
//	else
//	{
//		get_destination_lane_desired_acceleration_when_in_platoon(
//			platoon_vehicle, possible_accelerations);
//	}
//	get_cooperative_desired_acceleration(platoon_vehicle,
//		possible_accelerations);
//
//	return choose_minimum_acceleration(possible_accelerations);
//}
//
//double VehicleController::get_desired_acceleration(
//	const TrafficLightALCVehicle& tl_alc_vehicle,
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	if (verbose) std::clog << "Inside get traffic_light_acc_acceleration\n";
//
//	active_longitudinal_controller_type = ALCType::traffic_light_alc;
//	if (tl_alc_vehicle.has_next_traffic_light())
//	{
//		with_traffic_lights_controller.compute_traffic_light_input_parameters(
//			tl_alc_vehicle, traffic_lights);
//	}
//
//	return with_traffic_lights_controller.compute_desired_acceleration(
//		tl_alc_vehicle, nullptr, tl_alc_vehicle.get_desired_velocity());
//}

void VehicleController::print_traffic_lights(const EgoVehicle& ego,
	const std::unordered_map<int, TrafficLight>& traffic_lights) const
{
	std::clog << "veh id=" << ego.get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;
}

double VehicleController::choose_minimum_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	double desired_acceleration = 1000; // any high value
	for (const auto& it : possible_accelerations)
	{
		if (verbose) std::clog << ALC_type_to_string(it.first)
			<< ", " << it.second << std::endl;

		if (it.second < desired_acceleration)
		{
			desired_acceleration = it.second;
			active_longitudinal_controller_type = it.first;
		}
	}

	if (verbose) std::clog << "des accel=" << desired_acceleration << std::endl;

	return desired_acceleration;
}

double VehicleController::determine_low_velocity_reference(
	const NearbyVehicle& nearby_vehicle) const
{
	double ego_velocity = ego_vehicle->get_velocity();
	double leader_velocity =
		nearby_vehicle.compute_velocity(ego_velocity);
	/* The fraction of the leader speed has to vary. Otherwise, vehicles
	take too long to create safe gaps at low speeds*/
	double vel_fraction;
	if (leader_velocity < 5 / 3.6)
	{
		vel_fraction = 0;
	}
	else if (leader_velocity < 40 / 3.6)
	{
		vel_fraction = 0.5;
	}
	else
	{
		vel_fraction = 0.8;
	}
	double reference_velocity = std::min(
		leader_velocity * vel_fraction,
		ego_velocity);

	if (verbose)
	{
		std::clog << "\tDetermining vel ref. v_l=" << leader_velocity
			<< ", v_ref=" << reference_velocity << std::endl;
	}

	return reference_velocity;
}

double VehicleController::compute_desired_lane_change_gap(
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) const
{
	double safe_time_headway_gap = get_desired_time_headway_gap(
		nearby_vehicle);
	/* TODO: the function calls do not make much sense here.
	You call this method from the ego vehicle, and then call an ego vehicle
	method in here.*/
	/*double collision_free_gap = ego_vehicle.compute_exact_collision_free_gap(
		nearby_vehicle);*/
	double transient_gap = lateral_controller.compute_transient_gap(
		*ego_vehicle, nearby_vehicle, will_accelerate);

	return safe_time_headway_gap /*collision_free_gap*/ + transient_gap;
}

double VehicleController::get_desired_time_headway_gap(
	const NearbyVehicle& nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	double ego_velocity = ego_vehicle->get_velocity();
	if (nearby_vehicle.is_ahead())
	{
		if (nearby_vehicle.get_relative_lane() == RelativeLane::same)
		{
			time_headway_gap =
				origin_lane_controller.get_desired_time_headway_gap(
				ego_velocity);
		}
		else
		{
			time_headway_gap =
				destination_lane_controller.get_desired_time_headway_gap(
				ego_velocity);
		}
	}
	else
	{
		double dest_lane_follower_time_headway =
			destination_lane_controller.get_follower_time_headway();
		time_headway_gap =
			destination_lane_controller.get_time_headway_gap(
				dest_lane_follower_time_headway,
				nearby_vehicle.compute_velocity(ego_velocity));
	}

	return time_headway_gap;
}

double VehicleController::get_gap_variation_during_lane_change(
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) const
{
	return lateral_controller.compute_transient_gap(
		*ego_vehicle, nearby_vehicle, will_accelerate);
}

std::string VehicleController::ALC_type_to_string(
	ALCType active_longitudinal_controller)
{
	switch (active_longitudinal_controller)
	{
	case ALCType::origin_lane:
		return "origin lane controller";
	case ALCType::destination_lane:
		return "destination lane controller";
	case ALCType::cooperative_gap_generation:
		return "gap generation controller";
	case ALCType::end_of_lane:
		return "end-of-lane controller";
	case ALCType::traffic_light_alc:
		return "traffic light alc";
	case ALCType::vissim:
		return "vissim controller";
	default:
		return "unknown active longitudinal controller";
	}
}
