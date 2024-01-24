/*==========================================================================*/
/*  VehicleController.h    											        */
/*                															*/
/*                                                                          */
/*  Version of xxxx-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include "VehicleController.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"
#include "VariationLimitedFilter.h"

VehicleController::VehicleController(const EgoVehicle* ego_vehicle, 
	bool verbose) : ego_vehicle{ ego_vehicle }, verbose {verbose}, 
	long_controllers_verbose{ verbose /*false*/ }
{
	if (verbose && !long_controllers_verbose)
	{
		std::clog << "Attention! Long controllers are not verbose\n";
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
		return BLACK;
	}
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

double VehicleController::get_desired_acceleration()
{
	return implement_get_desired_acceleration();
}

double VehicleController::get_reference_gap() const
{
	return origin_lane_controller->get_desired_gap();
};

double VehicleController::get_gap_error() const
{
	return get_active_long_controller()->get_gap_error();
}

double VehicleController::get_safe_time_headway() const
{
	return origin_lane_controller->get_desired_time_headway();
}

double VehicleController::get_current_desired_time_headway() const
{
	return origin_lane_controller->get_current_time_headway();
}

double VehicleController::get_desired_time_headway_gap(
	const NearbyVehicle& nearby_vehicle) const
{
	return implement_get_desired_time_headway_gap(
		nearby_vehicle);
}

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

void VehicleController::set_verbose(bool value)
{
	if (value && !verbose)
	{
		std::clog << "[ControlManager] set to verbose."
			<< " Note that underlying long controllers are NOT verbose\n";
	}
	verbose = value;
}

void VehicleController::add_internal_controllers()
{
	implement_add_internal_controllers();
}

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

/* ----------------------------------------------------------------------- */

void VehicleController::activate_origin_lane_controller(
	const NearbyVehicle& real_leader)
{
	double ego_velocity = ego_vehicle->get_velocity();
	origin_lane_controller->reset_leader_velocity_filter(ego_velocity);
	/* NOTE: include 'if (time_headway_filter.get_is_initialized())' ?
	And force initial value to be the non-lane-changing one? */
	//origin_lane_controller->reset_time_headway_filter(time_headway);
	update_origin_lane_controller(real_leader);
}

void VehicleController::update_origin_lane_controller(
	const NearbyVehicle& real_leader)
{
	implement_update_origin_lane_controller(real_leader);
}

void VehicleController::reset_origin_lane_velocity_controller(
	)
{
	//if (verbose)
	//{
	//	std::clog << "Resetting orig lane ctrl vel ctrl." << std::endl;
	//}
	origin_lane_controller->reset_velocity_controller(
		ego_vehicle->get_velocity());
}

double VehicleController::find_comfortable_time_headway(
	const NearbyVehicle& a_leader, double standstill_distance)
{
	if (verbose)
	{
		std::clog << "\tLooking for good h."
			<< " g=" << ego_vehicle->compute_gap_to_a_leader(a_leader)
			<< ", d=" << standstill_distance
			<< ", v=" << ego_vehicle->get_velocity()
			<< ", h=" << ((ego_vehicle->compute_gap_to_a_leader(a_leader)
				- standstill_distance) / ego_vehicle->get_velocity())
			<< std::endl;
	}
	return (ego_vehicle->compute_gap_to_a_leader(a_leader) 
		- standstill_distance) / ego_vehicle->get_velocity();
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

void VehicleController::print_traffic_lights(
	const std::unordered_map<int, TrafficLight>& traffic_lights) const
{
	std::clog << "veh id=" << ego_vehicle->get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;
}

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
