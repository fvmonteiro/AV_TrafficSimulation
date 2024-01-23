#pragma once
#include "AVController.h"

class ConnectedAutonomousVehicle;

class CAVController : public AVController
{
public:
	CAVController(const ConnectedAutonomousVehicle& cav, bool verbose);
	
	const VirtualLongitudinalController& get_gap_generation_lane_controller()
		const {
		return gap_generating_controller;
	};

	/* Computes the AV desired acceleration plus the cooperative acceleration
	to help create a gap for an incoming vehicle, and chooses the minimum. */
	double get_desired_acceleration(
		const ConnectedAutonomousVehicle& cav);

	void add_cooperative_lane_change_controller(
		const ConnectedAutonomousVehicle& cav);

	void update_gap_generation_controller(double ego_velocity,
		double time_headway);

protected:
	VirtualLongitudinalController gap_generating_controller;

	CAVController(bool verbose) : AVController(verbose) {};

	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		const ConnectedAutonomousVehicle& cav,
		std::unordered_map<ALCType, double>& possible_accelerations);
private:
	std::unordered_map<LongitudinalController::State, color_t>
		gap_generation_colors =
	{
		{ LongitudinalController::State::velocity_control, YELLOW },
		{ LongitudinalController::State::vehicle_following, DARK_YELLOW },
	};
};
