/*==========================================================================*/
/* LongitudinalController.h     											*/
/* Base class for all longitudinal controllers								*/
/*																			*/
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <string>

#include "Constants.h"
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
		traffic_light,
		comf_accel,
		too_close,
	};

	LongitudinalController() = default;

	State get_state() const { return state; };
	color_t get_state_color() const;

	double compute_desired_acceleration(const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference);

	/* Printing ----------------------------------------------------------- */
	static std::string state_to_string(State state);

protected:
	LongitudinalController(std::unordered_map<State, color_t>
		state_to_color_map);
	VelocityController velocity_controller;
	GapController gap_controller;
	State state{ State::uninitialized }; // event driven logic variable

private:
	virtual double implement_compute_desired_acceleration(
		const EgoVehicle& ego_vehicle,
		const std::shared_ptr<NearbyVehicle> leader,
		double velocity_reference) = 0;
	std::unordered_map<State, color_t> state_to_color_map;
	static const std::unordered_map<State, std::string> state_to_string_map;
};