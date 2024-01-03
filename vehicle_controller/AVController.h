#pragma once
#include "LongAVController.h"

class AutonomousVehicle;

class AVController : public LongAVController
{
public:
	AVController() = default;
    AVController(const AutonomousVehicle& av, bool verbose);

protected:

	/* ------------ Control Parameters ------------ */
	AutonomousGains autonomous_virtual_following_gains{ 0.4, 1.0 };
	// Original values for CAVs
	/*ConnectedGains connected_real_following_gains{ 0.2, 2.3, 0.13, 1.3 };*/
	// Values focused on platoon vehicles TODO: separate controllers
	// gains will probably have to be tuned
	ConnectedGains connected_virtual_following_gains{ 0.4, 2.3, 0.13, 1.3 };
	VelocityControllerGains adjustment_velocity_controller_gains{
	1, 0.1, 0.03 };

	/* Desired acceleration to adjust to destination lane leader.
	Returns true if the computed acceleration was added to the map */
	bool get_destination_lane_desired_acceleration(
		std::unordered_map<ALCType, double>& possible_accelerations);

private:
    const AutonomousVehicle* autonomous_vehicle{ nullptr };
	std::unordered_map<LongitudinalController::State, color_t>
		dest_lane_colors =
	{
		{ LongitudinalController::State::comf_accel, BLUE_GREEN },
		{ LongitudinalController::State::velocity_control, LIGHT_BLUE },
		{ LongitudinalController::State::vehicle_following, BLUE },
		{ LongitudinalController::State::creating_gap, DARK_BLUE}
	};

    double implement_compute_desired_acceleration() override;

	void add_lane_change_adjustment_controller();
};

