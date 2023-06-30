#include "PlatoonVehicleController.h"
#include "PlatoonVehicle.h"

PlatoonVehicleController::PlatoonVehicleController(
	const PlatoonVehicle& platoon_vehicle, bool verbose)
	: CAVController(platoon_vehicle, verbose),
	platoon_vehicle{ &platoon_vehicle } {}

bool PlatoonVehicleController
::get_destination_lane_desired_acceleration_when_in_platoon(
	std::unordered_map<ALCType, double>& possible_accelerations)
{
	/* TODO: reorganize
	Almost copy of get_destination_lane_desired_acceleration
	*/
	bool is_active = false;
	if (platoon_vehicle->has_virtual_leader()
		/*&& platoon_vehicle.can_start_adjustment_to_virtual_leader()*/)
	{
		const NearbyVehicle* virtual_leader =
			platoon_vehicle->get_virtual_leader().get();
		double ego_velocity = platoon_vehicle->get_velocity();
		double reference_velocity = determine_low_velocity_reference(
			*virtual_leader);

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
				*platoon_vehicle, virtual_leader, reference_velocity);
		is_active = true;
	}
	return is_active;
}

double PlatoonVehicleController::implement_compute_desired_acceleration()
{
	std::unordered_map<ALCType, double> possible_accelerations;

	get_origin_lane_desired_acceleration(possible_accelerations);
	get_end_of_lane_desired_acceleration(possible_accelerations);
	if (platoon_vehicle->is_platoon_leader())
	{
		get_destination_lane_desired_acceleration(possible_accelerations);
	}
	else
	{
		get_destination_lane_desired_acceleration_when_in_platoon(
			possible_accelerations);
	}
	get_cooperative_desired_acceleration(possible_accelerations);

	return choose_minimum_acceleration(possible_accelerations);
}