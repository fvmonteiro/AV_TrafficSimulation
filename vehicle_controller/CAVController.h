#pragma once
#include "AVController.h"

class ConnectedAutonomousVehicle;

class CAVController : public AVController
{
public:

	CAVController() = default;
	CAVController(const ConnectedAutonomousVehicle* cav, bool verbose);
	
	const VirtualLongitudinalController& get_gap_generation_lane_controller()
		const {
		return gap_generating_controller;
	};

	void update_gap_generation_controller(double ego_velocity,
		double time_headway);

protected:
	VirtualLongitudinalController gap_generating_controller;

	void add_cooperative_lane_change_controller();
	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
	const ConnectedAutonomousVehicle* cav{ nullptr };
	std::unordered_map<LongitudinalController::State, color_t>
		gap_generation_colors =
	{
		{ LongitudinalController::State::velocity_control, YELLOW },
		{ LongitudinalController::State::vehicle_following, DARK_YELLOW },
	};

	void implement_add_internal_controllers() override;
	/* Computes the AV desired acceleration plus the cooperative acceleration
	to help create a gap for an incoming vehicle, and chooses the minimum. */
	double implement_get_desired_acceleration() override;
};
