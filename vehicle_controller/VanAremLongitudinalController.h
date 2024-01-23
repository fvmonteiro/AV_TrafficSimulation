/*==========================================================================*/
/* Implementation of the controller described in "The impact of cooperative 
adaptive cruise control on traffic-flow characteristics", Van Arem et al 
2006, and used in Talebpour and Mahmassani 2016 and in Virdi et al 2019     */
/*==========================================================================*/

#pragma once
#include "LongitudinalController.h"
class VanAremLongitudinalController :public LongitudinalController
{
public:
	VanAremLongitudinalController() = default;
	VanAremLongitudinalController(
		const VelocityControllerGains& velocity_controller_gains,
		const ConnectedGains& gap_controller_gains, double max_brake, 
		double max_jerk, 
		std::unordered_map<State, color_t> state_to_color_map,
		bool verbose);

private:
	double min_gap{ 2.0 }; // [m]
	double ego_max_brake{ CAR_MAX_BRAKE }; // [m/s^2] absolute value
	double leader_max_brake{ CAR_MAX_BRAKE }; // [m/s^2] absolute value
	double gap_error{ MAX_DISTANCE }; // [m]
	double max_jerk_per_interval{ INFINITY };
	ConnectedGains gap_controller_gains;
	VelocityControllerGains velocity_controller_gains;

	double implement_get_gap_error() const override;
	double implement_compute_desired_acceleration(
		const EgoVehicle& ego_vehicle, const NearbyVehicle* leader,
		double velocity_reference) override;

	double compute_desired_gap(double ego_velocity, 
		const NearbyVehicle& leader) const;
	/* Also sets the value of gap error */
	double compute_gap_control_acceleration(const EgoVehicle& ego_vehicle,
		const NearbyVehicle* leader);
	double compute_velocity_control_acceleration(
		double velocity_reference, double ego_velocity) const;
	double apply_jerk_limit(double current_accel, double des_accel);
};

