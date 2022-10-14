/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for all longitudinal controllers								*/
/*																			*/
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <unordered_map>

#include "GapController.h"
#include "VelocityController.h"

class EgoVehicle;
class NearbyVehicle;

class LongitudinalController
{
public:
	enum class State {
		uninitialized,
		velocity_control,
		vehicle_following,
		//traffic_light,
		//max_accel,
		//too_close
	};

	State get_state() const { return state; };

	double get_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference);

	/* Printing ----------------------------------------------------------- */
	static std::string state_to_string(State state);

protected:
	/*VelocityController velocity_controller;
	std::shared_ptr<GapController> gap_controller;*/
	State state{ State::uninitialized }; // event driven logic variable
	bool verbose{ false };

private:
	virtual double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference) = 0;

	static const std::unordered_map<State, std::string> state_to_string_map;
};