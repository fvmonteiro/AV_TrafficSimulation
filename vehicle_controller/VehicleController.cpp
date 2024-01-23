/*==========================================================================*/
/*  ControlManager.cpp    											        */
/*  State-machine that determines the actual vehicle longitudinal and		*/
/*  lateral inputs															*/
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include "VehicleController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"
//#include "TrafficLightALCVehicle.h"
#include "VariationLimitedFilter.h"
//#include "VirdiVehicle.h"

VehicleController::VehicleController(bool verbose)
	: verbose{ verbose }, long_controllers_verbose{ verbose /*false*/}
{
	if (verbose && !long_controllers_verbose)
	{
		std::clog << "Attention! Long controllers are not verbose\n";
	}
}

//VehicleController::VehicleController(const TrafficLightALCVehicle& ego_vehicle,
//	bool verbose) : VehicleController(verbose)
//{
//	if (verbose)
//	{
//		std::clog << "Creating TL-ALC control manager\n";
//	}
//	add_traffic_lights_controller();
//}

//VehicleController::VehicleController(const VirdiVehicle& virdi_vehicle,
//	bool verbose) : VehicleController(verbose)
//{
//	if (verbose)
//	{
//		std::clog << "Creating Virdi control manager\n";
//	}
//	add_van_arem_controllers(virdi_vehicle);
//	
//}

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
		return BLACK;
	}
}

double VehicleController::get_safe_time_headway() const
{
	return origin_lane_controller.get_desired_time_headway();
}

double VehicleController::get_current_desired_time_headway() const
{
	return origin_lane_controller.get_current_time_headway();
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

void VehicleController::set_verbose(bool value)
{
	if (value && !verbose)
	{
		std::clog << "[ControlManager] set to verbose."
			<< " Note that underlying long controllers are NOT verbose\n";
	}
	verbose = value;
}

void VehicleController::add_vissim_controller()
{
	vissim_controller = VissimLongitudinalController(
		vissim_colors);
	available_controllers[ALCType::vissim] = &vissim_controller;
}

void VehicleController::add_origin_lane_controllers(
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

	available_controllers[ALCType::origin_lane] = &origin_lane_controller;
	available_controllers[ALCType::end_of_lane] = &end_of_lane_controller;
}

//void VehicleController::add_traffic_lights_controller()
//{
//	with_traffic_lights_controller =
//		LongitudinalControllerWithTrafficLights(tl_alc_colors,
//			long_controllers_verbose);
//	active_longitudinal_controller_type = ALCType::traffic_light_alc;
//
//	available_controllers[ALCType::traffic_light_alc] =
//		&with_traffic_lights_controller;
//}

//void VehicleController::add_van_arem_controllers(
//	const VirdiVehicle& virdi_vehicle)
//{
//	double max_jerk_per_interval = 
//		virdi_max_jerk * virdi_vehicle.get_simulation_time_step();
//	van_arem_controllers[ALCType::origin_lane] =
//		VanAremLongitudinalController(
//			van_arem_vel_ctrl_gains, van_arem_gap_ctrl_gains,
//			virdi_vehicle.get_max_brake(), INFINITY,
//			orig_lane_colors, verbose);
//	van_arem_controllers[ALCType::end_of_lane] =
//		VanAremLongitudinalController(
//			van_arem_vel_ctrl_gains, van_arem_gap_ctrl_gains,
//			virdi_vehicle.get_max_brake(), INFINITY,
//			end_of_lane_colors, verbose);
//	van_arem_controllers[ALCType::destination_lane] =
//		VanAremLongitudinalController(
//			van_arem_vel_ctrl_gains, van_arem_gap_ctrl_gains,
//			virdi_vehicle.get_max_brake(), max_jerk_per_interval,
//			dest_lane_colors, verbose);
//	van_arem_controllers[ALCType::cooperative_gap_generation] =
//		VanAremLongitudinalController(
//			van_arem_vel_ctrl_gains, van_arem_gap_ctrl_gains,
//			virdi_vehicle.get_max_brake(), max_jerk_per_interval,
//			gap_generation_colors, verbose);
//
//	for (auto const& item : van_arem_controllers)
//	{
//		available_controllers[item.first] = &item.second;
//	}
//}

//void ControlManager::add_in_platoon_controller(
//	const PlatoonVehicle& platoon_vehicle)
//{
//	in_platoon_controller = RealLongitudinalController(
//		platoon_vehicle,
//		desired_velocity_controller_gains,
//		autonomous_real_following_gains,
//		platoon_following_gains,
//		velocity_filter_gain, time_headway_filter_gain,
//		in_platoon_colors, long_controllers_verbose
//	);
//}

const LongitudinalController*
VehicleController::get_active_long_controller() const
{
	if (available_controllers.find(active_longitudinal_controller_type)
		!= available_controllers.end())
	{
		return available_controllers.at(active_longitudinal_controller_type);
	}
	return nullptr;
}

/* DEBUGGING FUNCTIONS --------------------------------------------------- */

double VehicleController::get_reference_gap(double ego_velocity) const
{
	return origin_lane_controller.get_desired_gap(ego_velocity);
};

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

void VehicleController::activate_origin_lane_controller(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& real_leader)
{
	double ego_velocity = ego_vehicle.get_velocity();
	origin_lane_controller.reset_leader_velocity_filter(ego_velocity);
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	//origin_lane_controller.reset_time_headway_filter(time_headway);
	update_origin_lane_controller(ego_vehicle, real_leader);
}

void VehicleController::activate_end_of_lane_controller(double time_headway)
{
	end_of_lane_controller.reset_time_headway_filter(time_headway);
	end_of_lane_controller.set_desired_time_headway(time_headway);
}

void VehicleController::update_origin_lane_controller(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& real_leader)
{
	implement_update_origin_lane_controller(ego_vehicle, real_leader);
}

void VehicleController::implement_update_origin_lane_controller(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& real_leader)
{
	origin_lane_controller_time_headway = 
		origin_lane_controller.get_current_time_headway();
	double safe_h =	ego_vehicle.compute_current_desired_time_headway(
		real_leader);
	double current_h = origin_lane_controller.get_current_time_headway();
	double comf_h = find_comfortable_time_headway(ego_vehicle, 
		real_leader, origin_lane_controller.get_standstill_distance());
	double new_h = std::max(current_h, std::min(comf_h, safe_h));

	if (verbose)
	{
		std::clog << "Resetting orig lane ctrl (real leader) h_r = "
			<< new_h << " and setting desired value to "
			<< safe_h << std::endl;
	}

	origin_lane_controller.reset_time_headway_filter(new_h);
	origin_lane_controller.set_desired_time_headway(safe_h);
	origin_lane_controller.connect_gap_controller(
		real_leader.is_connected());
}

void VehicleController::reset_origin_lane_velocity_controller(
	double ego_velocity)
{
	//if (verbose)
	//{
	//	std::clog << "Resetting orig lane ctrl vel ctrl." << std::endl;
	//}
	origin_lane_controller.reset_velocity_controller(ego_velocity);
}

double VehicleController::find_comfortable_time_headway(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& a_leader,
	double standstill_distance)
{
	if (verbose)
	{
		std::clog << "\tLooking for good h."
			<< " g=" << ego_vehicle.compute_gap_to_a_leader(a_leader)
			<< ", d=" << standstill_distance
			<< ", v=" << ego_vehicle.get_velocity()
			<< ", h=" << ((ego_vehicle.compute_gap_to_a_leader(a_leader)
				- standstill_distance) / ego_vehicle.get_velocity())
			<< std::endl;
	}
	return (ego_vehicle.compute_gap_to_a_leader(a_leader) 
		- standstill_distance) / ego_vehicle.get_velocity();
}

bool VehicleController::is_in_free_flow_at_origin_lane() const
{
	return origin_lane_controller.get_state()
		== SwitchedLongitudinalController::State::velocity_control;
}

double VehicleController::get_vissim_desired_acceleration(
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
//
//double VehicleController::get_desired_acceleration(
//	const VirdiVehicle& virdi_vehicle)
//{
//	std::unordered_map<ALCType, double>
//		possible_accelerations;
//	double vel_reference = virdi_vehicle.get_desired_velocity();
//
//	possible_accelerations[ALCType::origin_lane] =
//		van_arem_controllers.at(ALCType::origin_lane).
//		compute_desired_acceleration(
//			virdi_vehicle, virdi_vehicle.get_leader(), vel_reference);
//	possible_accelerations[ALCType::end_of_lane] =
//		get_end_of_lane_desired_acceleration(virdi_vehicle);
//	possible_accelerations[ALCType::destination_lane] = 
//		van_arem_controllers.at(ALCType::destination_lane).
//		compute_desired_acceleration(
//			virdi_vehicle, virdi_vehicle.get_destination_lane_leader(),
//			vel_reference);
//	possible_accelerations[ALCType::cooperative_gap_generation] = 
//		van_arem_controllers.at(ALCType::cooperative_gap_generation)
//		.compute_desired_acceleration(
//			virdi_vehicle, virdi_vehicle.get_assisted_vehicle(), 
//			vel_reference);
//	
//	double des_accel = choose_minimum_acceleration(possible_accelerations);
//	return des_accel;
//}

void VehicleController::print_traffic_lights(const EgoVehicle& ego,
	const std::unordered_map<int, TrafficLight>& traffic_lights) const
{
	std::clog << "veh id=" << ego.get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;
}

bool VehicleController::get_origin_lane_desired_acceleration(
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

bool VehicleController::get_end_of_lane_desired_acceleration(
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

		LongitudinalController::State old_state =
			end_of_lane_controller.get_state();
		/* To avoid sudden high decelerations */
		if (old_state
			== LongitudinalController::State::uninitialized)
		{
			end_of_lane_controller.reset_leader_velocity_filter(
				ego_vehicle.get_velocity());
		}
		double desired_acceleration =
			end_of_lane_controller.compute_desired_acceleration(
				ego_vehicle, &virtual_vehicle,
				ego_vehicle.get_desired_velocity());

		/* This controller is only active when it's at vehicle
		following mode */
		if (end_of_lane_controller.get_state()
			== LongitudinalController::State::vehicle_following)
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

//double VehicleController::get_end_of_lane_desired_acceleration(
//	const VirdiVehicle& virdi_vehicle)
//{
//	/* When the lane change direction equals the preferred relative
//	lane, it means the vehicle is moving into the destination lane.
//	At this point, we don't want to use this controller anymore. */
//	bool is_lane_changing = virdi_vehicle.get_preferred_relative_lane()
//		== virdi_vehicle.get_active_lane_change_direction();
//	bool is_about_to_start_lane_change =
//		virdi_vehicle.get_preferred_relative_lane()
//		== virdi_vehicle.get_lane_change_direction();
//	double des_accel;
//	if (!is_lane_changing && !is_about_to_start_lane_change
//		&& (virdi_vehicle.get_lane_end_distance() > -1))
//	{
//		/* We simulate a stopped vehicle at the end of
//		the lane to force the vehicle to stop before the end of
//		the lane. */
//		NearbyVehicle virtual_vehicle =
//			create_virtual_stopped_vehicle(virdi_vehicle);
//		des_accel = van_arem_controllers.at(ALCType::end_of_lane).
//			compute_desired_acceleration(
//			virdi_vehicle, std::make_shared<NearbyVehicle>(virtual_vehicle),
//			virdi_vehicle.get_desired_velocity()
//		);
//	}
//	else
//	{
//		des_accel = INFINITY;
//	}
//	return des_accel;
//}

double VehicleController::choose_minimum_acceleration(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	double desired_acceleration = INFINITY;
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

	if (verbose) 
	{
		std::clog << "Chosen ALC: "
			<< ALC_type_to_string(active_longitudinal_controller_type)
			<< ", des accel=" << desired_acceleration << std::endl;
	}

	return desired_acceleration;
}

NearbyVehicle VehicleController::create_virtual_stopped_vehicle(
	const EgoVehicle& ego_vehicle) const
{
	NearbyVehicle virtual_vehicle = NearbyVehicle(1, RelativeLane::same, 1);
	virtual_vehicle.set_relative_velocity(
		ego_vehicle.get_velocity());
	virtual_vehicle.set_distance(
		ego_vehicle.get_lane_end_distance());
	virtual_vehicle.set_length(0.0);

	if (verbose)
	{
		std::clog << "\tEOL distance: " << ego_vehicle.get_lane_end_distance()
			<< "\n";
		std::clog << "\tVirtual stopped leader " << virtual_vehicle << "\n";
	}

	return virtual_vehicle;
}

double VehicleController::get_desired_time_headway_gap(double ego_velocity,
	const NearbyVehicle& nearby_vehicle) const
{
	return implement_get_desired_time_headway_gap(
		ego_velocity, nearby_vehicle);
}

double VehicleController::implement_get_desired_time_headway_gap(
	double ego_velocity, const NearbyVehicle& nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	if (nearby_vehicle.is_ahead() 
		&& nearby_vehicle.get_relative_lane() == RelativeLane::same)
	{
			time_headway_gap =
				get_desired_time_headway_gap_to_leader(ego_velocity);
	}
	else
	{
		time_headway_gap = -1.0;
	}
	return time_headway_gap;
}

double VehicleController::get_desired_time_headway_gap_to_leader(
	double ego_velocity) const
{
	return origin_lane_controller.get_desired_time_headway_gap(
		ego_velocity);
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
	case ALCType::real_leader:
		return "real leader controller";
	case ALCType::virtual_leader:
		return "virtual leader controller";
	default:
		return "unknown active longitudinal controller";
	}
}
