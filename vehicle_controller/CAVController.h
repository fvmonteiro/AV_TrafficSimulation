#pragma once
#include "AVController.h"

class ConnectedAutonomousVehicle;

class CAVController : public AVController
{
public:
	CAVController(const ConnectedAutonomousVehicle& cav, bool verbose);

protected:
	/* Returns true if the computed acceleration was added to the map */
	bool get_cooperative_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
	const ConnectedAutonomousVehicle* connected_av{ nullptr };
	std::unordered_map<LongitudinalController::State, color_t>
		gap_generation_colors =
	{
		{ LongitudinalController::State::velocity_control, YELLOW },
		{ LongitudinalController::State::vehicle_following, DARK_YELLOW },
	};

	double implement_compute_desired_acceleration() override;

	void add_cooperative_lane_change_controller();
};

