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

ControlManager::ControlManager(const EgoVehicle& ego_vehicle,
	bool verbose) :
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose } 
{
	if (verbose) 
	{
		std::clog << "Creating control manager " << std::endl;
	}

	//extract_vehicle_static_parameters(ego_vehicle);

	bool is_long_control_verbose = false; // verbose;

	switch (ego_vehicle.get_type())
	{
	case VehicleType::acc_car:
		create_acc_controllers(ego_vehicle, is_long_control_verbose);
		break;
	case VehicleType::autonomous_car:
		create_acc_controllers(ego_vehicle, is_long_control_verbose);
		create_lane_change_adjustment_controller(ego_vehicle,
			is_long_control_verbose);
		break;
	case VehicleType::connected_car:
		create_acc_controllers(ego_vehicle, is_long_control_verbose);
		create_lane_change_adjustment_controller(ego_vehicle,
			is_long_control_verbose);
		create_cooperative_lane_change_controller(ego_vehicle,
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

ControlManager::ControlManager(const EgoVehicle& ego_vehicle)
	: ControlManager(ego_vehicle, false) {}

void ControlManager::create_acc_controllers(
	const EgoVehicle& ego_vehicle, bool verbose)
{
	origin_lane_controller = RealLongitudinalController(
		ego_vehicle, 
		desired_velocity_controller_gains,
		autonomous_real_following_gains, 
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		verbose);
	/* Note: the end of lane controller could have special vel control gains
	but we want to see how the ego vehicle responds to a stopped vehicle. */
	end_of_lane_controller = RealLongitudinalController(
		ego_vehicle,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		verbose);
	/* The end of the lane is seen as a stopped vehicle. We set some 
	time headway to that "vehicle". */
	activate_end_of_lane_controller(time_headway_to_end_of_lane);
}

void ControlManager::create_lane_change_adjustment_controller(
	const EgoVehicle& ego_vehicle, bool verbose)
{
	destination_lane_controller = VirtualLongitudinalController(
		ego_vehicle, 
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		verbose);
}

void ControlManager::create_cooperative_lane_change_controller(
	const EgoVehicle& ego_vehicle, bool verbose)
{
	gap_generating_controller = VirtualLongitudinalController(
		ego_vehicle,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		verbose);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here*/
	gap_generating_controller.connect_gap_controller(true);
}

SwitchedLongitudinalController::State
	ControlManager::get_longitudinal_controller_state() const
{
	switch (active_longitudinal_controller) 
	{
	case ACCType::origin_lane:
		return origin_lane_controller.get_state();
	case ACCType::destination_lane:
		return destination_lane_controller.get_state();
	case ACCType::cooperative_gap_generation:
		return gap_generating_controller.get_state();
	case ACCType::end_of_lane:
		return end_of_lane_controller.get_state();
	case ACCType::vissim:
		return SwitchedLongitudinalController::State::uninitialized;
	default:
		return SwitchedLongitudinalController::State::uninitialized;
	}
}

LongitudinalControllerWithTrafficLights::State
	ControlManager::get_longitudinal_controller_with_traffic_lights_state()
{
	return with_traffic_lights_controller.get_state();
}

/* DEBUGGING FUNCTIONS --------------------------------------------------- */

double ControlManager::get_reference_gap(double ego_velocity)
{
	return origin_lane_controller.get_desired_gap(ego_velocity);
};

double ControlManager::get_gap_error(VehicleType type) const
{
	if (type == VehicleType::traffic_light_acc_car)
	{
		return with_traffic_lights_controller.get_h1();
	}
	else /* TODO */
	{
		return 0.0;
	}
}

/* ----------------------------------------------------------------------- */

double ControlManager::compute_drac(double relative_velocity, double gap) 
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
	if (verbose)
	{
		std::clog << "Setting orig lane ctrl h_r = "
			<< time_headway << std::endl;
	}
	origin_lane_controller.set_desired_time_headway(time_headway);
	origin_lane_controller.connect_gap_controller(is_leader_connected);
}

void ControlManager::update_destination_lane_follower_time_headway(
	double time_headway)
{
	destination_lane_controller.set_follower_time_headway(time_headway);
}

void ControlManager::activate_destination_lane_controller(double ego_velocity,
	double leader_velocity,
	double time_headway, bool is_leader_connected)
{
	//destination_lane_controller.smooth_start_leader_velocity_filter();
	destination_lane_controller.reset_leader_velocity_filter(leader_velocity);
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
	destination_lane_controller.connect_gap_controller(is_leader_connected);
	//destination_lane_controller.reset_leader_velocity_filter(ego_velocity);
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

void ControlManager::reset_origin_lane_velocity_controller(
	double ego_velocity) 
{
	//if (verbose)
	//{
	//	std::clog << "Resetting orig lane ctrl vel ctrl." << std::endl;
	//}
	origin_lane_controller.reset_velocity_controller(ego_velocity);
}

bool ControlManager::is_in_free_flow_at_origin_lane() const
{
	return origin_lane_controller.get_state() 
		== SwitchedLongitudinalController::State::velocity_control;
}

double ControlManager::use_vissim_desired_acceleration(
	const EgoVehicle& ego_vehicle) 
{
	/* We need to ensure the velocity filter keeps active
	while VISSIM has control of the car to guarantee a smooth
	movement when taking back control */
	if (ego_vehicle.has_leader()) 
	{
		origin_lane_controller.update_leader_velocity_filter(
			ego_vehicle.get_leader()->compute_velocity(
				ego_vehicle.get_velocity()));
	}

	active_longitudinal_controller = ACCType::vissim;
	return ego_vehicle.get_vissim_acceleration();
}

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
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
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

bool ControlManager::get_origin_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations)
{
	if (verbose) std::clog << "Origin lane controller" << std::endl;
	possible_accelerations[ACCType::origin_lane] = 
		origin_lane_controller.get_desired_acceleration(
		ego_vehicle, ego_vehicle.get_leader(),
		ego_vehicle.get_free_flow_velocity());

	return true;
}

bool ControlManager::get_end_of_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations)
{
	bool is_active = false;

	/* When the lane change direction equals the preferred relative
	lane, it means the vehicle is moving into the destination lane.
	At this point, we don't want to use this controller anymore. */
	bool is_lane_changing = ego_vehicle.get_preferred_relative_lane()
		== ego_vehicle.get_active_lane_change_direction();
	bool is_about_to_start_lane_change = 
			ego_vehicle.get_preferred_relative_lane()
			== ego_vehicle.get_lane_change_direction();
	if (!is_lane_changing && !is_about_to_start_lane_change
		&& (ego_vehicle.get_lane_end_distance() > -1)) 
		// -1 because end of lane dist can get small negative values
	{
		if (verbose) 
		{
			std::clog << "End of lane controller"
				<< std::endl;
		}
		/* We simulate a stopped vehicle at the end of
		the lane to force the vehicle to stop before the end of
		the lane. */
		NearbyVehicle virtual_vehicle = 
			create_virtual_stopped_vehicle(ego_vehicle);

		SwitchedLongitudinalController::State old_state =
			end_of_lane_controller.get_state();
		/* To avoid sudden high decelerations */
		if (old_state
			== SwitchedLongitudinalController::State::uninitialized)
		{
			end_of_lane_controller.reset_leader_velocity_filter(
				ego_vehicle.get_velocity());
		}
		double desired_acceleration =
			end_of_lane_controller.get_desired_acceleration(
				ego_vehicle, std::make_shared<NearbyVehicle>(virtual_vehicle),
				ego_vehicle.get_desired_velocity());

		/* This controller is only active when it's at vehicle 
		following mode */
		if (end_of_lane_controller.get_state()
			== SwitchedLongitudinalController::State::vehicle_following)
		{
			if (verbose)
			{
				std::clog << "active EOL controller. Old state is " 
					<< LongitudinalController::state_to_string(old_state)
					<< std::endl;
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
	std::unordered_map<ACCType, double>& possible_accelerations) 
{
	bool is_active = false;
	double ego_velocity = ego_vehicle.get_velocity();

	bool end_of_lane_controller_is_active = 
		end_of_lane_controller.get_state()
		== SwitchedLongitudinalController::State::vehicle_following;

	/* We only activate if we want to merge behding ld 
	or if the end of the lane is close */
	if (ego_vehicle.merge_behind_ld()
		|| (ego_vehicle.has_destination_lane_leader()
			&& end_of_lane_controller_is_active)) 
	{	
		if (verbose) 
		{
			std::clog << "Dest. lane controller"
				<< std::endl;
		}
		std::shared_ptr<NearbyVehicle> dest_lane_leader =
			ego_vehicle.get_destination_lane_leader();
		double ego_velocity = ego_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *dest_lane_leader);

		if (verbose)
		{
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller
			!= ACCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity)) 
		{
			destination_lane_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ACCType::destination_lane] =
			destination_lane_controller.get_desired_acceleration(
				ego_vehicle, dest_lane_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

bool ControlManager::get_cooperative_desired_acceleration(
	const ConnectedAutonomousVehicle& ego_vehicle,
	std::unordered_map<ACCType, double>& possible_accelerations) 
{
	bool is_active = false;

	if (ego_vehicle.is_cooperating_to_generate_gap()) 
	{
		if (verbose) 
		{
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		std::shared_ptr<NearbyVehicle> assisted_vehicle =
			ego_vehicle.get_assisted_vehicle();
		double ego_velocity = ego_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *assisted_vehicle);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the gap generating controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller
			!= ACCType::cooperative_gap_generation)
			&& gap_generating_controller.is_outdated(ego_velocity)) 
		{
			gap_generating_controller.reset_velocity_controller(
				ego_velocity);
		}
			
		possible_accelerations[ACCType::cooperative_gap_generation] =
			gap_generating_controller.get_desired_acceleration(
				ego_vehicle, assisted_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}

double ControlManager::choose_minimum_acceleration(
	std::unordered_map<ACCType, double>& possible_accelerations)
{
	double desired_acceleration = 1000; // any high value
	for (const auto& it : possible_accelerations) 
	{
		if (verbose) std::clog << active_ACC_to_string(it.first)
			<< ", " << it.second << std::endl;

		if (it.second < desired_acceleration) 
		{
			desired_acceleration = it.second;
			active_longitudinal_controller = it.first;
		}
	}

	if (verbose) std::clog << "des accel=" << desired_acceleration << std::endl;

	return desired_acceleration;
}

NearbyVehicle ControlManager::create_virtual_stopped_vehicle(
	const EgoVehicle& ego_vehicle)
{
	NearbyVehicle virtual_vehicle = NearbyVehicle(1, RelativeLane::same, 1);
	virtual_vehicle.set_relative_velocity(
		ego_vehicle.get_velocity());
	virtual_vehicle.set_distance(
		ego_vehicle.get_lane_end_distance());
	virtual_vehicle.set_length(0.0);
	return virtual_vehicle;
}

double ControlManager::determine_low_velocity_reference(double ego_velocity,
	const NearbyVehicle& nearby_vehicle) 
{
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
	return reference_velocity;
}

double ControlManager::compute_desired_lane_change_gap(
	const AutonomousVehicle& ego_vehicle, 
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) 
{
	double safe_time_headway_gap = get_desired_time_headway_gap(
		ego_vehicle.get_velocity(), 
		/*ego_vehicle.has_lane_change_intention(),*/
		nearby_vehicle);
	/* TODO: the function calls do not make much sense here.
	You call this method from the ego vehicle, and then call an ego vehicle 
	method in here.*/
	/*double collision_free_gap = ego_vehicle.compute_exact_collision_free_gap(
		nearby_vehicle);*/
	double transient_gap = lateral_controller.compute_transient_gap(
		ego_vehicle, nearby_vehicle, will_accelerate);

	return safe_time_headway_gap /*collision_free_gap*/ + transient_gap;
}

double ControlManager::get_desired_time_headway_gap(double ego_velocity,
	const NearbyVehicle& nearby_vehicle) 
{
	double time_headway_gap = 0.0;
	//double ego_velocity = ego_vehicle.get_velocity();
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

//double ControlManager::get_accepted_time_headway_gap(
//	const AutonomousVehicle& ego_vehicle, const NearbyVehicle& nearby_vehicle)
//{
//	double time_headway_gap = 0.0;
//	double ego_velocity = ego_vehicle.get_velocity();
//	//bool has_lc_intention = ego_vehicle.has_lane_change_intention();
//
//	if (nearby_vehicle.is_ahead()) 
//	{
//		if (nearby_vehicle.get_relative_lane() == RelativeLane::same)
//		{
//			time_headway_gap =
//				origin_lane_controller.get_desired_time_headway_gap(
//					ego_velocity/*, has_lc_intention*/);
//		}
//		else
//		{
//			time_headway_gap =
//				destination_lane_controller.get_desired_time_headway_gap(
//					ego_velocity/*, has_lc_intention*/);
//		}
//		//double gamma = nearby_vehicle.get_max_brake() / ego_vehicle
//	}
//	else
//	{
//		double dest_lane_follower_time_headway =
//			destination_lane_controller.get_follower_time_headway();
//		time_headway_gap =
//			destination_lane_controller.get_time_headway_gap(
//				dest_lane_follower_time_headway,
//				nearby_vehicle.compute_velocity(ego_velocity));
//	}
//
//	return time_headway_gap;
//}

double ControlManager::get_gap_variation_during_lane_change(
	const AutonomousVehicle& ego_vehicle,
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate)
{
	return lateral_controller.compute_transient_gap(
		ego_vehicle, nearby_vehicle, will_accelerate);
}

std::string ControlManager::active_ACC_to_string(
	ACCType active_longitudinal_controller) 
{
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