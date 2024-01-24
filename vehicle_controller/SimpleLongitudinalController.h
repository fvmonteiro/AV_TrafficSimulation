#pragma once
#include "SwitchedLongitudinalController.h"

/* Mostly based on:
Pooladsanj, Milad et al "Vehicle Following on a Ring Road under Safety 
Constraints: Role of Connectivity and Coordination" 
at IEEE Trans. on Intelligent Vehicles */

class SimpleLongitudinalController : public SwitchedLongitudinalController
{
public:

	SimpleLongitudinalController() = default;
	SimpleLongitudinalController(const EgoVehicle* ego_vehicle,
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

	void set_max_brake(double value) { max_brake = value; };

private:
	double max_brake{ CAR_MAX_BRAKE };
	double threshold_parameter{ 2.0 };
	long previous_leader_id{ -1 };

	double get_max_accepted_brake() override;
	/* Determines and sets the current state of the longitudinal controller */
	void determine_controller_state(const NearbyVehicle* leader,
		double reference_velocity, double gap_control_input) override;

};

