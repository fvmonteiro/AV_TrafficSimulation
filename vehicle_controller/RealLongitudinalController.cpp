/*==========================================================================*/
/* OriginLaneLongitudinalController.h    									*/
/* Origin Lane (usual) Adaptive Cruise controller using the constant time   */
/* headway policy                                                           */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

#include "RealLongitudinalController.h"
#include "EgoVehicle.h"

RealLongitudinalController::RealLongitudinalController() :
	SwitchedLongitudinalController() {}

RealLongitudinalController::RealLongitudinalController(
	const EgoVehicle& ego_vehicle,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	bool verbose) :
	SwitchedLongitudinalController(velocity_controller_gains,
		autonomous_gains, connected_gains, velocity_filter_gain, 
		time_headway_filter_gain, ego_vehicle.get_max_brake(), 
		ego_vehicle.get_comfortable_acceleration(),
		ego_vehicle.get_sampling_interval(), verbose)
{
	if (verbose) 
	{
		std::clog << "Created real longitudinal controller" << std::endl;
	}
}

RealLongitudinalController::RealLongitudinalController(
	const EgoVehicle& ego_vehicle,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain) :
	RealLongitudinalController(ego_vehicle,
		velocity_controller_gains, autonomous_gains, connected_gains,
		velocity_filter_gain, time_headway_filter_gain, false) {}

//OriginLaneLongitudinalController::OriginLaneLongitudinalController(
//	const EgoVehicle& ego_vehicle, double kg, double kv, bool verbose)
//	: OriginLaneLongitudinalController(ego_vehicle, verbose) {
//	set_vehicle_following_gains(kg, kv);
//}

void RealLongitudinalController::update_leader_velocity_filter(
	double leader_velocity) 
{
	gap_controller.update_leader_velocity_filter(leader_velocity);
	//leader_velocity_filter.apply_filter(leader_velocity);
}

void RealLongitudinalController::determine_controller_state(
	const EgoVehicle& ego_vehicle,
	const std::shared_ptr<NearbyVehicle> leader,
	double reference_velocity, double gap_control_input) {

	if (leader == nullptr) // no vehicle ahead
	{ 
		state = State::velocity_control;
		
		if (verbose)
		{
			std::clog << "No leader id"
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}
	}
	else 
	{
		double gap = ego_vehicle.compute_gap(leader);
		double ego_velocity = ego_vehicle.get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double gap_threshold = compute_gap_threshold(gap,
			reference_velocity - ego_velocity, gap_control_input);
		if (state == State::vehicle_following) 
		{
			gap_threshold += hysteresis_bias;
		}

		bool is_gap_small = gap < gap_threshold;
		bool is_leader_too_fast =
			leader_velocity > (reference_velocity + reference_velocity_margin);
		if (is_gap_small && !is_leader_too_fast)
		{
			state = State::vehicle_following;
		}
		else 
		{
			state = State::velocity_control;
		}

		if (verbose) 
		{
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}
	}
}