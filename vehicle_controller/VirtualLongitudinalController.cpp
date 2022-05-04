/*==========================================================================*/
/* DestinationLaneLongitudinalController.h     								*/
/* Controller to perform longitudinal adjustments before lane changes based */
/* on the constant time headway policy                                      */
/*                                                                          */
/* Version of 2021-xx-xx                             Fernando V. Monteiro   */
/*==========================================================================*/

//#include <algorithm>
#include "VirtualLongitudinalController.h"
#include "EgoVehicle.h"

VirtualLongitudinalController::VirtualLongitudinalController() :
	SwitchedLongitudinalController() {}

VirtualLongitudinalController::VirtualLongitudinalController(
	const EgoVehicle& ego_vehicle,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain,
	bool verbose) :
	SwitchedLongitudinalController(velocity_controller_gains,
		autonomous_gains, connected_gains,
		velocity_filter_gain, time_headway_filter_gain,
		ego_vehicle.get_comfortable_brake(), 
		ego_vehicle.get_comfortable_acceleration(),
		ego_vehicle.get_sampling_interval(), verbose) {

	if (verbose) {
		std::clog << "Created virtual longitudinal controller" << std::endl;
	}
}

VirtualLongitudinalController::VirtualLongitudinalController(
	const EgoVehicle& ego_vehicle,
	VelocityControllerGains velocity_controller_gains,
	AutonomousGains autonomous_gains, ConnectedGains connected_gains,
	double velocity_filter_gain, double time_headway_filter_gain) :
	VirtualLongitudinalController(ego_vehicle,
		velocity_controller_gains, autonomous_gains, connected_gains,
		velocity_filter_gain, time_headway_filter_gain, false) {}

//void DestinationLaneLongitudinalController::set_reference_velocity(
//	double reference_velocity, double ego_velocity) {
//	if (verbose) {
//		std::clog << "new min vel=" << reference_velocity << std::endl;
//	}
//	reset_desired_velocity_filter(ego_velocity);
//	this->ego_reference_velocity = reference_velocity;
//}

//void DestinationLaneLongitudinalController::set_reference_velocity(
//	double ego_velocity, double adjustment_speed_factor) {
//	double minimum_adjustment_velocity =
//		ego_velocity * adjustment_speed_factor;
//	if (verbose) {
//		std::clog << "new min vel=" << minimum_adjustment_velocity << std::endl;
//	}
//	reset_desired_velocity_filter(ego_velocity);
//	this->ego_reference_velocity = minimum_adjustment_velocity;
//}

void VirtualLongitudinalController::determine_controller_state(
	const EgoVehicle& ego_vehicle, 
	const std::shared_ptr<NearbyVehicle> leader,
	double reference_velocity, double gap_control_input) {

	if (leader == nullptr) 
	{ // no vehicle ahead
		/*If there's no leader this controller should not be active */
		if (verbose) std::clog << "\tno leader" << std::endl;
		state = State::uninitialized;
	}
	else 
	{
		/*double ego_velocity = ego_vehicle.get_velocity();
		double leader_velocity = leader->compute_velocity(ego_velocity);
		double velocity_error = compute_velocity_error(
			ego_velocity, leader_velocity);
		double gap_threshold = gap_controller.compute_gap_threshold(
			reference_velocity, velocity_error,
			ego_vehicle.get_acceleration(), leader->get_acceleration()
		);*/
		
		double gap = ego_vehicle.compute_gap(leader);
		double ego_velocity = ego_vehicle.get_velocity();
		double gap_threshold = compute_gap_threshold(gap,
			reference_velocity - ego_velocity, gap_control_input);

		if (state == State::vehicle_following) {
			gap_threshold -= hysteresis_bias;
		}

		if ((gap > gap_threshold) 
			|| (ego_velocity < std::min(reference_velocity, 5.0))) 
		{
			state = State::vehicle_following;
		}
		else 
		{
			state = State::velocity_control;
		}

		if (verbose) {
			std::clog << "Gap threshold = "
				<< gap_threshold
				<< ", gap = " << gap
				<< " to leader id " << leader->get_id()
				<< ". State: " << state_to_string(state)
				<< std::endl;
		}

	}
}

bool VirtualLongitudinalController::is_active() const 
{
	return state != State::uninitialized;
}

bool VirtualLongitudinalController::is_outdated(
	double ego_velocity) const 
{
	/*bool ret = ego_velocity < desired_velocity_filter.get_current_value();
	bool new_value = ego_velocity < velocity_controller.get_reference_value();
	if (ret != new_value)
	{
		std::clog << "is_outdated different results" << std::endl;
	}*/
	return ego_velocity < velocity_controller.get_reference_value();
}

bool VirtualLongitudinalController::update_accepted_risk(
	double time, const EgoVehicle& ego_vehicle) {
	if (verbose) {
		std::clog << "\tt=" << time
			<< ", timer_start=" << timer_start << std::endl;
	}

	bool has_increased = false;

	if (((time - timer_start) >= constant_risk_period)) {
		if ((accepted_risk_to_leader + delta_risk) 
			< max_risk_to_leader) {
			timer_start = time;
			accepted_risk_to_leader += delta_risk;
			has_increased = true;

			if (verbose) {
				std::clog << "\tleader risk updated to " 
					<< accepted_risk_to_leader
					<< std::endl;
			}
		}
		if (accepted_risk_to_leader > intermediate_risk_to_leader
			&& (accepted_risk_to_follower + delta_risk)
			   < max_risk_to_follower) {
			timer_start = time;
			accepted_risk_to_follower += delta_risk;
			has_increased = true; 
			
			if (verbose) {
				std::clog << "\tfollower risk updated to "
					<< accepted_risk_to_follower
					<< std::endl;
			}
		}
	}
	return has_increased;
}

//void VirtualLongitudinalController::
//compute_intermediate_risk_to_leader(double lambda_1, 
//	double lane_change_lambda_1, double max_brake_no_lane_change, 
//	double leader_max_brake) {
//
//	double safe_h_no_lane_change = compute_time_headway_with_risk(
//		free_flow_velocity, max_brake_no_lane_change, leader_max_brake,
//		lambda_1, rho, 0);
//	intermediate_risk_to_leader = std::sqrt(2 
//		* (h_lane_change - safe_h_no_lane_change)
//		* ego_max_brake * free_flow_velocity);
//
//	if (verbose) {
//		std::clog << "safe no lc h=" << safe_h_no_lane_change
//			<< ", h_lc=" << h_lane_change
//			<< ", mid risk to leader="
//			<< intermediate_risk_to_leader << std::endl;
//	}
//}

void VirtualLongitudinalController::reset_accepted_risks() 
{
	accepted_risk_to_leader = initial_risk;
	accepted_risk_to_follower = initial_risk;
}

//void VirtualLongitudinalController::compute_max_risk_to_follower(
//	double follower_max_brake) 
//{
//	max_risk_to_follower = std::sqrt(
//		2 * follower_time_headway 
//		* follower_max_brake * free_flow_velocity);
//	if (verbose) std::clog << "max risk to follower=" 
//		<< max_risk_to_follower << std::endl;
//}