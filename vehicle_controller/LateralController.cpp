/*==========================================================================*/
/*  LateralController.cpp  													*/
/*  Class implementation.											        */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#define _USE_MATH_DEFINES
#include <math.h>

#include "LateralController.h"
#include "LongitudinalController.h"
#include "NearbyVehicle.h"
#include "EgoVehicle.h"

LateralController::LateralController(bool verbose) 
	: verbose{ verbose } {
	
	if (verbose) {
		std::clog << "Creating lateral controller" << std::endl;
	}
	estimate_lane_change_kinematics();
}

LateralController::LateralController() : LateralController(false) {}

void LateralController::set_destination_lane_follower_parameters(
	double new_lambda_0, double new_lambda_1)
{
	if (verbose)
	{
		std::clog << "\tSetting dest lane foll params. "
			<< "lambda0=" << new_lambda_0
			<< ", lambda1=" << new_lambda_1 << "\n";
	}
	dest_lane_follower_lambda_0 = new_lambda_0;
	dest_lane_follower_lambda_0 = new_lambda_1;
}

void LateralController::set_destination_lane_follower_time_headway(
	double new_time_headway)
{
	if (verbose)
	{
		std::clog << "\tSetting dest lane foll "
			<< " h=" << new_time_headway << "\n";
	}
	time_headway_to_destination_lane_follower = new_time_headway;
}

double LateralController::compute_time_headway_gap(double ego_velocity,
	const NearbyVehicle& nearby_vehicle, double accepted_risk)
{
	double time_headway, follower_velocity;
	if (nearby_vehicle.is_ahead())
	{
		follower_velocity = ego_velocity;
		if (nearby_vehicle.get_relative_lane() == RelativeLane::same)
		{
			time_headway = time_headway_to_leader;
		}
		else
		{
			time_headway = time_headway_to_destination_lane_leader;
		}
	}
	else
	{
		follower_velocity = nearby_vehicle.compute_velocity(ego_velocity);
		time_headway = time_headway_to_destination_lane_follower;
	}

	/* [Feb 24, 2023] Copied from old method from AutonomousVehicle.
	Not ready for risk taking yet. */
	//double accepted_risk = nearby_vehicle.is_ahead() ?
	//	accepted_lane_change_risk_to_leaders :
	//	accepted_lane_change_risk_to_follower;

	//if (accepted_risk > 0)
	//{
	//	double leader_max_brake, follower_max_brake, follower_lambda_1;
	//	if (nearby_vehicle.is_ahead())
	//	{
	//		leader_max_brake = nearby_vehicle.get_max_brake();
	//		follower_max_brake = get_lane_change_max_brake();
	//		follower_lambda_1 = get_lambda_1_lane_change();
	//	}
	//	else
	//	{
	//		leader_max_brake = get_max_brake();
	//		follower_max_brake = nearby_vehicle.get_max_brake();
	//		follower_lambda_1 = nearby_vehicle.get_lambda_1();
	//	}
	//	double rho = get_rho();
	//	double vf = get_desired_velocity();
	//	double gamma = leader_max_brake / follower_max_brake;
	//	double Gamma = (1 - rho) * vf / (vf + follower_lambda_1);

	//	double denominator = gamma >= Gamma ? 1 : (1 - gamma);
	//	denominator *= 2 * follower_max_brake;
	//	double risk_term = std::pow(accepted_risk, 2) / denominator;
	//	accepted_time_headway_gap -= risk_term;
	//}

	return time_headway * follower_velocity + standstill_distance;
}

double LateralController::compute_vehicle_following_gap_for_lane_change(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& nearby_vehicle, 
	double ego_lambda_1, double accepted_risk) const
{
	double follower_lambda_0, follower_lambda_1;
	double v_follower, v_leader;
	double brake_follower, brake_leader;
	double ego_velocity = ego_vehicle.get_velocity();
	double delta_v = nearby_vehicle.get_relative_velocity();
	if (nearby_vehicle.is_ahead())
	{
		follower_lambda_0 = ego_vehicle.get_lambda_0();
		follower_lambda_1 = ego_lambda_1;
		v_follower = ego_velocity;
		v_leader = nearby_vehicle.compute_velocity(ego_velocity);
		brake_follower = ego_vehicle.get_lane_change_max_brake();
		brake_leader = nearby_vehicle.get_max_brake();
	}
	else
	{
		follower_lambda_0 = dest_lane_follower_lambda_0;
		follower_lambda_1 = dest_lane_follower_lambda_1;
		v_leader = ego_velocity;
		v_follower = nearby_vehicle.compute_velocity(ego_velocity);
		brake_follower = nearby_vehicle.get_max_brake();
		brake_leader = ego_vehicle.get_max_brake();
		delta_v = -delta_v;
	}

	double accepted_risk_2 = std::pow(accepted_risk, 2);
	double stop_time_follower = (v_follower + follower_lambda_1)
		/ brake_follower;
	double stop_time_leader = v_leader / brake_leader;

	double accepted_gap;
	if (stop_time_follower >= stop_time_leader)
	{
		accepted_gap =
			(std::pow(v_follower + follower_lambda_1, 2)
				- accepted_risk_2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader
			+ follower_lambda_0;
	}
	else if (brake_follower > brake_leader)
	{
		accepted_gap =
			(std::pow(delta_v + follower_lambda_1, 2)
				- accepted_risk_2) / 2 / (brake_follower - brake_leader)
			+ follower_lambda_0;
	}
	else
	{
		accepted_gap = 0.0;
	}

	if (verbose)
	{
		std::clog << "\tVeh following gap computation\n\t"
			<< "vf=" << v_follower
			<< ", lambda1=" << follower_lambda_1
			<< ", df=" << brake_follower
			<< ", vl=" << v_leader
			<< ", dl=" << brake_leader
			<< ", lambda 0=" << follower_lambda_0
			<< "\n\tt_f=" << stop_time_follower
			<< ", t_l=" << stop_time_leader
			<< ", g_vf=" << accepted_gap
			<< std::endl;
	}

	return std::max(0.0, accepted_gap);
}

double LateralController::compute_transient_gap(const EgoVehicle& ego_vehicle,
	const NearbyVehicle& other_vehicle, bool will_accelerate) const 
{
	/* We need to solve:
	max_{t \in T} [(v_E(t_0) - v_L(t_0))(t - t_0) + 1/2 a_E (t-t_0)^2]

	If there's no acceleration, we can find the transient gap in constant
	time, based on whether v_E(t_0) greater or lesser than v_L(t_0). */

	double longitudinal_acceleration;
	double relative_velocity = other_vehicle.get_relative_velocity();
	if (will_accelerate) 
	{
		double comfortable_acceleration = 
			ego_vehicle.get_comfortable_acceleration();
		longitudinal_acceleration = -relative_velocity / lane_change_duration;
		longitudinal_acceleration = std::min(std::max(
			longitudinal_acceleration, -comfortable_acceleration),
			comfortable_acceleration);
	}
	else 
	{
		longitudinal_acceleration = 0.0;
	}

	double t_0;
	double t_f;
	if (other_vehicle.is_on_same_lane()) 
	{
		t_0 = 0;
		t_f = compute_lateral_collision_time(ego_vehicle,
			other_vehicle, longitudinal_acceleration);
	}
	else 
	{
		t_0 = compute_lateral_collision_time(ego_vehicle,
			other_vehicle, longitudinal_acceleration);
		t_f = lane_change_duration;
	}

	if (!other_vehicle.is_ahead()) 
	{
		relative_velocity *= -1;
		longitudinal_acceleration *= -1;
	}

	double t_max;
	double transient_gap;
	if ((relative_velocity >= 0) && (longitudinal_acceleration >= 0)) 
	{
		t_max = t_f;
	}
	else if ((relative_velocity <= 0) && (longitudinal_acceleration <= 0)) 
	{
		t_max = t_0;
	}
	else 
	{
		t_max = - relative_velocity / longitudinal_acceleration;
		if (t_max > t_f) t_max = t_f;
	}
	transient_gap = relative_velocity * t_max
		+ longitudinal_acceleration / 2 * t_max * t_max;
	
	return transient_gap;
}

double LateralController::compute_lateral_collision_time(
	const EgoVehicle& ego_vehicle, const NearbyVehicle& other_vehicle,
	double longitudinal_acceleration) const {
	/* lane_change_lateral_position is the estimated lateral position 
	of the vehicle's center front. We assume all lane changes are left lane 
	changes to compute the collision time, which is independent of the lane 
	change direction. 
	In what follows, we compute the lateral coordiantes of the vehicle 
	rear left, front left and front right because those are the relevant 
	points related to vehicles future follower, future leader and current
	leader respectively. 
	Derivation of the formula and further explanation is in Jula H. et al, 
	Collision avoidance analysis for lane changing and merging, 2000. */

	double longitudinal_velocity = ego_vehicle.get_velocity();
	double length = ego_vehicle.get_length();
	double width = ego_vehicle.get_width();
	double other_width = other_vehicle.get_width();
	//std::vector<double> y_ego (lane_change_lateral_position.size());

	bool collision_time_found = false;
	double y_ego;
	int i = 0;
	while ((!collision_time_found) && (i < lane_change_lateral_position.size())) 
	{
		y_ego = lane_change_lateral_position[i];
		if (!other_vehicle.is_ahead()) // compute rear coordinate
		{ 
			double sin_theta = lane_change_lateral_velocity[i]
				/ std::sqrt(std::pow(lane_change_lateral_velocity[i], 2) 
							+ std::pow(longitudinal_velocity, 2));
			y_ego -= (length * sin_theta);
		}
		double cos_theta = longitudinal_velocity
				/ std::sqrt(std::pow(lane_change_lateral_velocity[i], 2)
							+ std::pow(longitudinal_velocity, 2));
		if (other_vehicle.is_on_same_lane()) // compute right coordinate
		{ 
			y_ego -= width / 2 * cos_theta;
			if (y_ego >= other_width / 2) 
			{
				collision_time_found = true;
			}
		}
		else // compute left coordinate
		{ 
			y_ego += width / 2 * cos_theta;
			if (y_ego >= lane_width - other_width / 2)
			{
				collision_time_found = true;
			}
		}
		longitudinal_velocity += longitudinal_acceleration * sampling_time;
		i++;
	}

	i--;
	return i*sampling_time;
}

void LateralController::estimate_lane_change_kinematics() 
{
	double time = 0.0;
	lane_change_lateral_acceleration.push_back(0.0);
	lane_change_lateral_velocity.push_back(0.0);
	lane_change_lateral_position.push_back(0.0);

	time += sampling_time;
	while (time < lane_change_duration) 
	{
		lane_change_lateral_acceleration.push_back(
			2 * M_PI * lane_width / (lane_change_duration*lane_change_duration)
			* std::sin(2*M_PI*time/lane_change_duration)
		);
		lane_change_lateral_velocity.push_back(
			lane_change_lateral_velocity.front() 
			+ lane_change_lateral_acceleration.front() * sampling_time
		);
		lane_change_lateral_position.push_back(
			lane_change_lateral_position.front() 
			+ lane_change_lateral_velocity.front() * sampling_time
		);
		time += sampling_time;
	}
}