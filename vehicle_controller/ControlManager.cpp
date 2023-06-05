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
#include "PlatoonVehicle.h"
#include "SafeConnectedAutonomousVehicle.h"
#include "TrafficLightALCVehicle.h"
#include "VariationLimitedFilter.h"

ControlManager::ControlManager(const EgoVehicle& ego_vehicle,
	bool verbose) :
	lateral_controller{ LateralController(verbose) },
	verbose{ verbose }
{
	long_controllers_verbose = verbose;
	if (verbose)
	{
		std::clog << "Creating control manager" << std::endl;
	}
}

color_t ControlManager::get_longitudinal_controller_color() const
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

double ControlManager::get_safe_time_headway() const
{
	return origin_lane_controller.get_desired_time_headway();
}

double ControlManager::get_current_desired_time_headway() const
{
	return origin_lane_controller.get_current_time_headway();
}

LongitudinalController::State
ControlManager::get_longitudinal_controller_state() const
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


void ControlManager::add_vissim_controller()
{
	vissim_controller = VissimLongitudinalController(
		vissim_colors);
}

void ControlManager::add_origin_lane_controllers(
	const EgoVehicle& ego_vehicle)
{
	if (verbose) std::clog << "Creating origin lane controllers."
		<< std::endl;
	origin_lane_controller = RealLongitudinalController(
		ego_vehicle,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		orig_lane_colors, long_controllers_verbose);
	/* Note: the end of lane controller could have special vel control gains
	but we want to see how the ego vehicle responds to a stopped vehicle. */
	end_of_lane_controller = RealLongitudinalController(
		ego_vehicle,
		desired_velocity_controller_gains,
		autonomous_real_following_gains,
		connected_real_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		end_of_lane_colors, long_controllers_verbose);
	/* The end of the lane is seen as a stopped vehicle. We set some
	time headway to that "vehicle". */
	activate_end_of_lane_controller(time_headway_to_end_of_lane);
}

void ControlManager::add_lane_change_adjustment_controller(
	const AutonomousVehicle& autonomous_vehicle)
{
	if (verbose) std::clog << "Creating lane change adjustment controller."
		<< std::endl;
	destination_lane_controller = VirtualLongitudinalController(
		autonomous_vehicle,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		dest_lane_colors, long_controllers_verbose);
}

void ControlManager::add_cooperative_lane_change_controller(
	const ConnectedAutonomousVehicle& cav)
{
	if (verbose) std::clog << "Creating cooperative lane change controller."
		<< std::endl;
	gap_generating_controller = VirtualLongitudinalController(
		cav,
		adjustment_velocity_controller_gains,
		autonomous_virtual_following_gains,
		connected_virtual_following_gains,
		velocity_filter_gain, time_headway_filter_gain,
		gap_generation_colors, long_controllers_verbose);
	/* the gap generating controller is only activated when there are
	two connected vehicles, so we can set its connection here*/
	gap_generating_controller.connect_gap_controller(true);
}

void ControlManager::add_traffic_lights_controller()
{
	with_traffic_lights_controller =
		LongitudinalControllerWithTrafficLights(tl_alc_colors,
			long_controllers_verbose);
	active_longitudinal_controller_type = ALCType::traffic_light_alc;
}

void ControlManager::add_safety_critical_controllers(
	const SafeConnectedAutonomousVehicle& scav)
{
	nominal_controller = VelocityController()
}

const LongitudinalController* ControlManager::get_active_long_controller() const
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

double ControlManager::get_reference_gap(double ego_velocity) const
{
	return origin_lane_controller.get_desired_gap(ego_velocity);
};

double ControlManager::get_gap_error() const
{
	return get_active_long_controller()->get_gap_error();
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

void ControlManager::activate_origin_lane_controller(double ego_velocity,
	double time_headway, bool is_leader_connected)
{
	origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	origin_lane_controller.reset_time_headway_filter(time_headway);
	update_origin_lane_controller(time_headway, is_leader_connected);
}

void ControlManager::activate_end_of_lane_controller(double time_headway)
{
	end_of_lane_controller.reset_time_headway_filter(time_headway);
	end_of_lane_controller.set_desired_time_headway(time_headway);
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

//void ControlManager::activate_in_platoon_controller(double ego_velocity,
//	double time_headway)
//{
//	in_platoon_controller.reset_leader_velocity_filter(ego_velocity);
//	in_platoon_controller.reset_time_headway_filter(time_headway);
//	in_platoon_controller.set_desired_time_headway(time_headway);
//}

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

void ControlManager::update_destination_lane_controller(double ego_velocity,
	double time_headway, bool is_leader_connected)
{
	destination_lane_controller.smooth_start_leader_velocity_filter();
	destination_lane_controller.set_desired_time_headway(time_headway);
	destination_lane_controller.connect_gap_controller(is_leader_connected);
	//destination_lane_controller.reset_leader_velocity_filter(ego_velocity);
}

void ControlManager::update_destination_lane_follower_time_headway(
	double time_headway)
{
	destination_lane_controller.set_follower_time_headway(time_headway);
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

double ControlManager::get_vissim_desired_acceleration(
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

	active_longitudinal_controller_type = ALCType::vissim;
	return vissim_controller.compute_desired_acceleration(
		ego_vehicle, nullptr, 0);
}

double ControlManager::get_desired_acceleration(
	const ACCVehicle& acc_vehicle)
{
	if (acc_vehicle.has_lane_change_intention() ||
		acc_vehicle.is_lane_changing())
	{
		return get_vissim_desired_acceleration(acc_vehicle);
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(acc_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(acc_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_desired_acceleration(
	const AutonomousVehicle& autonomous_vehicle)
{
	if (autonomous_vehicle.is_vissim_controlling_lane_change()
		&& (autonomous_vehicle.has_lane_change_intention()
			|| autonomous_vehicle.is_lane_changing()))
	{
		return get_vissim_desired_acceleration(autonomous_vehicle);
	}

	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(autonomous_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(autonomous_vehicle,
		possible_accelerations);
	get_destination_lane_desired_acceleration(autonomous_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_desired_acceleration(
	const ConnectedAutonomousVehicle& cav)
{
	std::unordered_map<ALCType, double>
		possible_accelerations;
	get_origin_lane_desired_acceleration(cav,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(cav,
		possible_accelerations);
	get_destination_lane_desired_acceleration(cav,
		possible_accelerations);
	get_cooperative_desired_acceleration(cav,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_desired_acceleration(
	const PlatoonVehicle& platoon_vehicle)
{
	std::unordered_map<ALCType, double> possible_accelerations;
	
	get_origin_lane_desired_acceleration(platoon_vehicle,
		possible_accelerations);
	get_end_of_lane_desired_acceleration(platoon_vehicle,
		possible_accelerations);
	if (platoon_vehicle.is_platoon_leader())
	{
		get_destination_lane_desired_acceleration(platoon_vehicle,
			possible_accelerations);
	}
	else
	{
		get_destination_lane_desired_acceleration_when_in_platoon(
			platoon_vehicle, possible_accelerations);
	}
	get_cooperative_desired_acceleration(platoon_vehicle,
		possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}

double ControlManager::get_desired_acceleration(
	const TrafficLightALCVehicle& tl_alc_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	if (verbose) std::clog << "Inside get traffic_light_acc_acceleration\n";

	active_longitudinal_controller_type = ALCType::traffic_light_alc;
	if (tl_alc_vehicle.has_next_traffic_light())
	{
		with_traffic_lights_controller.compute_traffic_light_input_parameters(
			tl_alc_vehicle, traffic_lights);
	}

	return with_traffic_lights_controller.compute_desired_acceleration(
		tl_alc_vehicle, nullptr, tl_alc_vehicle.get_desired_velocity());
}

double ControlManager::get_desired_acceleration(
	const SafeConnectedAutonomousVehicle& scav)
{
	std::unordered_map<ALCType, double> possible_accelerations;


}

void ControlManager::print_traffic_lights(const EgoVehicle& ego,
	const std::unordered_map<int, TrafficLight>& traffic_lights) const
{
	std::clog << "veh id=" << ego.get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;
}

bool ControlManager::get_origin_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	if (verbose) std::clog << "Origin lane controller" << std::endl;
	possible_accelerations[ALCType::origin_lane] =
		origin_lane_controller.compute_desired_acceleration(
		ego_vehicle, ego_vehicle.get_leader(),
		ego_vehicle.get_desired_velocity());

	return true;
}

bool ControlManager::get_end_of_lane_desired_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<ALCType, double>& possible_accelerations)
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
			end_of_lane_controller.compute_desired_acceleration(
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
			possible_accelerations[ALCType::end_of_lane] =
				desired_acceleration;
			is_active = true;
		}
	}
	return is_active;
}

bool ControlManager::get_destination_lane_desired_acceleration(
	const AutonomousVehicle& autonomous_vehicle,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;
	//double origin_lane_reference_velocity;

	bool end_of_lane_controller_is_active =
		end_of_lane_controller.get_state()
		== SwitchedLongitudinalController::State::vehicle_following;

	/* We only activate if we want to merge behding ld
	or if the end of the lane is close */
	/*if (autonomous_vehicle.merge_behind_destination_lane_leader()
		|| (autonomous_vehicle.has_destination_lane_leader()
			&& end_of_lane_controller_is_active))*/
	/* [Jan 23, 2023] TO DO: changes to reflect the addition of 
	virtual leader. However, this piece of code is not clean. We should 
	be able to get the virtual leader directly from autonomous_vehicle */
	std::shared_ptr<const NearbyVehicle> virtual_leader = nullptr;
	if (autonomous_vehicle.has_virtual_leader())
	{
		virtual_leader =
			autonomous_vehicle.get_virtual_leader();
	}
	else if (autonomous_vehicle.has_destination_lane_leader()
		&& end_of_lane_controller_is_active)
	{
		virtual_leader =
			autonomous_vehicle.get_destination_lane_leader();
	}

	if (virtual_leader != nullptr)
	{
		if (verbose)
		{
			std::clog << "Dest. lane controller"
				<< std::endl;
		}
		/*std::shared_ptr<const NearbyVehicle> virtual_leader =
			autonomous_vehicle.get_virtual_leader();*/
		double ego_velocity = autonomous_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *virtual_leader);

		if (verbose)
		{
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity))
		{
			destination_lane_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				autonomous_vehicle, virtual_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

bool ControlManager::get_cooperative_desired_acceleration(
	const ConnectedAutonomousVehicle& cav,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	bool is_active = false;

	if (cav.is_cooperating_to_generate_gap())
	{
		if (verbose)
		{
			std::clog << "Gap generating controller"
				<< std::endl;
		}
		std::shared_ptr<const NearbyVehicle> assisted_vehicle =
			cav.get_assisted_vehicle();
		double ego_velocity = cav.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *assisted_vehicle);

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the gap generating controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::cooperative_gap_generation)
			&& gap_generating_controller.is_outdated(ego_velocity))
		{
			gap_generating_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::cooperative_gap_generation] =
			gap_generating_controller.compute_desired_acceleration(
				cav, assisted_vehicle, reference_velocity);
		is_active = true;
	}
	return is_active;
}

bool ControlManager
::get_destination_lane_desired_acceleration_when_in_platoon(
	const PlatoonVehicle& platoon_vehicle,
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	/* TODO: reorganize
	Almost copy of get_destination_lane_desired_acceleration
	*/
	bool is_active = false;
	if (platoon_vehicle.has_virtual_leader()
		/*&& platoon_vehicle.can_start_adjustment_to_virtual_leader()*/)
	{
		std::shared_ptr<const NearbyVehicle> virtual_leader =
			platoon_vehicle.get_virtual_leader();
		double ego_velocity = platoon_vehicle.get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			ego_velocity, *virtual_leader);

		if (verbose)
		{
			std::clog << "Dest. lane controller" << std::endl;
			std::clog << "low ref vel=" << reference_velocity << std::endl;
		}

		/* If the ego vehicle is braking hard due to conditions on
		the current lane, the destination lane controller, which
		uses comfortable constraints, must be updated. */
		if ((active_longitudinal_controller_type
			!= ALCType::destination_lane)
			&& destination_lane_controller.is_outdated(ego_velocity))
		{
			destination_lane_controller.reset_velocity_controller(
				ego_velocity);
		}

		possible_accelerations[ALCType::destination_lane] =
			destination_lane_controller.compute_desired_acceleration(
				platoon_vehicle, virtual_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

double ControlManager::choose_minimum_acceleration(
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
	const NearbyVehicle& nearby_vehicle) const
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

	if (verbose)
	{
		std::clog << "\tDetermining vel ref. v_l=" << leader_velocity
			<< ", v_ref=" << reference_velocity << std::endl;
	}

	return reference_velocity;
}

double ControlManager::compute_desired_lane_change_gap(
	const AutonomousVehicle& ego_vehicle,
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) const
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
	const NearbyVehicle& nearby_vehicle) const
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

double ControlManager::get_gap_variation_during_lane_change(
	const AutonomousVehicle& ego_vehicle,
	const NearbyVehicle& nearby_vehicle,
	bool will_accelerate) const
{
	return lateral_controller.compute_transient_gap(
		ego_vehicle, nearby_vehicle, will_accelerate);
}

std::string ControlManager::ALC_type_to_string(
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
