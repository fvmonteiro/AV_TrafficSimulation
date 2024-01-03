#pragma once
#include "VehicleController.h"

class LongitudinallyAutonomousVehicle;

/* Controller for Longitudinally Autonomous Vehicles */
class LongAVController : public VehicleController
{
public:
    LongAVController() = default;
    LongAVController(const LongitudinallyAutonomousVehicle& longitudinal_av,
        bool verbose);

    /* Resets the origin lane controller's velocity and time headway filters
    and sets the time headway*/
    void activate_origin_lane_controller(double time_headway,
        bool is_leader_connected);

protected:
    /* ------------ Control Parameters ------------ */
    double velocity_filter_gain{ 10.0 };
    double time_headway_filter_gain{ 1.0 }; // 0.3 original value for CAVs
    /* The time headway used by the end-of-lane controller */
    double time_headway_to_end_of_lane{ 0.5 };
    AutonomousGains autonomous_real_following_gains{ 0.2, 1.0 };
    ConnectedGains connected_real_following_gains{ 0.5, 2.2, 0.13, 1.3 };
    VelocityControllerGains desired_velocity_controller_gains{
        0.5, 0.1, 0.03 };

    /* VISSIM's suggested acceleration */
    double get_vissim_desired_acceleration(const EgoVehicle& ego_vehicle);

    /* Desired acceleration relative to the current lane.
    Returns true if the computed acceleration was added to the map */
    bool get_origin_lane_desired_acceleration(
        std::unordered_map<ALCType, double>& possible_accelerations);

    /* Desired acceleration to wait at the end of the lane while
    looking for an appropriate lane change gap. Without this,
    vehicles might miss a desired exit.
    Returns true if the computed acceleration was added to the map */
    bool get_end_of_lane_desired_acceleration(
        std::unordered_map<ALCType, double>& possible_accelerations);

private:
    const LongitudinallyAutonomousVehicle* longitudinal_av{ nullptr };

    double implement_compute_desired_acceleration() override;
    
    void add_vissim_controller();
    void add_origin_lane_controllers();

    NearbyVehicle create_virtual_stopped_vehicle();

    /* Colors to make debugging visually easier
    General rule: bright colors represent vel control,
    darker colors represent vehicle following. */
    std::unordered_map<LongitudinalController::State, color_t>
        vissim_colors =
    {
        { LongitudinalController::State::uninitialized, GRAY }
    };
    std::unordered_map<LongitudinalController::State, color_t>
        orig_lane_colors =
    {
        { LongitudinalController::State::comf_accel, BLUE_GREEN },
        { LongitudinalController::State::velocity_control, GREEN },
        { LongitudinalController::State::vehicle_following, DARK_GREEN },
        { LongitudinalController::State::creating_gap, CYAN}
    };
    std::unordered_map<LongitudinalController::State, color_t>
        end_of_lane_colors =
    {
        { LongitudinalController::State::velocity_control, MAGENTA },
        { LongitudinalController::State::vehicle_following, DARK_MAGENTA },
    };
};

