#include "ConnectedAutonomousVehicle.h"

ConnectedAutonomousVehicle::ConnectedAutonomousVehicle(
	long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	AutonomousVehicle(id, VehicleType::connected_car, desired_velocity,
		true, simulation_time_step, creation_time, verbose) 
{
	compute_connected_safe_gap_parameters();

	if (verbose)
	{
		std::clog << "lambda1_connected = " << lambda_1_connected
			<< ", lambda1_connected_lc = " << lambda_1_lane_change_connected
			<< std::endl;
	}
}

bool ConnectedAutonomousVehicle::has_assisted_vehicle() const
{
	return assisted_vehicle != nullptr;
}

bool ConnectedAutonomousVehicle::is_cooperating_to_generate_gap() const {
	return has_assisted_vehicle();
	/* Function seems unnecessary, but we might perform other checks here */
}

std::shared_ptr<NearbyVehicle> ConnectedAutonomousVehicle::
implement_get_assisted_vehicle() const
{
	return assisted_vehicle;
}

double ConnectedAutonomousVehicle::
implement_get_time_headway_to_assisted_vehicle() const {
	if (is_cooperating_to_generate_gap())
	{
		return controller.get_gap_generation_lane_controller().
			get_desired_time_headway();
	}
	return 0;
}

//void ConnectedAutonomousVehicle::try_to_set_nearby_vehicle_type(long nv_type)
//{
//	peek_nearby_vehicles()->set_type(nv_type, );
//}

long ConnectedAutonomousVehicle::create_lane_change_request()
{
	return desired_lane_change_direction.to_int() * get_id();
}

void ConnectedAutonomousVehicle::find_relevant_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_requests();
}

void ConnectedAutonomousVehicle::find_cooperation_requests()
{
	try_go_at_max_vel = false;
	std::shared_ptr<NearbyVehicle> old_assisted_vehicle =
		std::move(assisted_vehicle);

	for (auto& nearby_vehicle : nearby_vehicles)
	{
		if (check_if_is_asking_for_cooperation(*nearby_vehicle))
		{
			assisted_vehicle = nearby_vehicle;
		}
		else if (nearby_vehicle->is_requesting_to_merge_behind())
		{
			try_go_at_max_vel = true;
		}
	}
	deal_with_close_and_slow_assited_vehicle();
	update_assisted_vehicle(old_assisted_vehicle);
}

void ConnectedAutonomousVehicle::deal_with_close_and_slow_assited_vehicle()
{
	/* We need acount for case where the ego vehicle is already too
	close to the vehicle asking to move in and the vehicle asking to move in
	is already very slow. The only solution would be for the ego vehicle to
	go backwards, which would lead to a deadlock situation. */
	if (has_assisted_vehicle()
		&& (assisted_vehicle->compute_velocity(get_velocity()) < 1)
		&& compute_gap(assisted_vehicle) < 1)
	{
		assisted_vehicle = nullptr;
	}
}

void ConnectedAutonomousVehicle::update_destination_lane_follower(
	const std::shared_ptr<NearbyVehicle>& old_follower)
{
	if (has_destination_lane_follower())
	{
		std::shared_ptr<NearbyVehicle>& dest_lane_follower = 
			get_destination_lane_follower();
		controller.update_destination_lane_follower_time_headway(
			dest_lane_follower->get_h_to_incoming_vehicle());

		/* We need to compute fd's lambda1 here cause it's used later in
		computing gaps that accept risks. */
		if ((old_follower == nullptr )
			|| (old_follower->get_id() != dest_lane_follower->get_id()))
		{
			dest_lane_follower->compute_safe_gap_parameters();
			dest_lane_follower_lambda_0 =
				get_destination_lane_follower()->get_lambda_0();
			dest_lane_follower_lambda_1 =
				get_destination_lane_follower()->get_lambda_1();
		}
	}
}

void ConnectedAutonomousVehicle::update_assisted_vehicle(
	const std::shared_ptr<NearbyVehicle>& old_assisted_vehicle)
{
	if (has_assisted_vehicle())
	{
		//double new_max_brake = assisted_vehicle->get_max_brake();
		if ((old_assisted_vehicle == nullptr)
			/* || (std::abs(new_max_brake
				- old_assisted_vehicle->get_max_brake()) > 0.5)*/
			|| (old_assisted_vehicle->get_id() != assisted_vehicle->get_id()))
		{
			double h_to_assisted_vehicle = std::max(0.0,
				compute_vehicle_following_time_headway(
					*assisted_vehicle, 0
					/* assisted_vehicle->get_max_lane_change_risk_to_follower()*/
				));
			controller.update_gap_generation_controller(
				get_velocity(), h_to_assisted_vehicle);
		}
	}
}

double ConnectedAutonomousVehicle::get_lambda_1(
	bool is_leader_connected) const
{
	return is_leader_connected ?
		lambda_1_connected : Vehicle::get_lambda_1();
}

double ConnectedAutonomousVehicle::get_lambda_1_lane_change(
	bool is_leader_connected) const
{
	return is_leader_connected ?
		lambda_1_lane_change_connected 
		: AutonomousVehicle::get_lambda_1_lane_change();
}

//double ConnectedAutonomousVehicle::compute_current_desired_time_headway(
//	const NearbyVehicle& nearby_vehicle)
//{
//	/*double current_lambda_1 = get_lambda_1(leader.is_connected());
//	double risk = 0;*/
//	if (has_lane_change_intention())
//	{
//		return compute_lane_changing_desired_time_headway(nearby_vehicle);
//		/*current_lambda_1 = get_lambda_1_lane_change(leader.is_connected());
//		risk = get_accepted_risk_to_leaders();*/
//	}
//	return compute_vehicle_following_desired_time_headway(nearby_vehicle);
//	/*return compute_time_headway_with_risk(get_desired_velocity(),
//		get_current_max_brake(), leader.get_max_brake(),
//		current_lambda_1, get_rho(), risk);*/
//}

double ConnectedAutonomousVehicle::
compute_vehicle_following_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	/*double current_lambda_1 = get_lambda_1(nearby_vehicle.is_connected());
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), 0);*/
	return compute_vehicle_following_time_headway(nearby_vehicle, 0);
}

double ConnectedAutonomousVehicle::
compute_vehicle_following_time_headway(
	const NearbyVehicle& nearby_vehicle,
	double nv_max_lane_change_risk) const
{
	double current_lambda_1 = get_lambda_1(nearby_vehicle.is_connected());
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), nv_max_lane_change_risk);
}

double ConnectedAutonomousVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1 = 
		get_lambda_1_lane_change(nearby_vehicle.is_connected());
	double h_lc = compute_time_headway_with_risk(get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, get_rho(), 0/*get_accepted_risk_to_leaders()*/);
	return h_lc;
	/* We only allow the lane changing headway towards the leaders to be below
	the safe vehicle following headway if a risky headway to the future
	follower is also allowed */
	/*if (get_accepted_risk_to_follower() <= 0)
	{
		return std::max(h_lc,
			compute_vehicle_following_safe_time_headway(nearby_vehicle));
	}
	else
	{
		return h_lc;
	}*/
}

double ConnectedAutonomousVehicle::
compute_vehicle_following_gap_for_lane_change(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1 = 
		get_lambda_1_lane_change(nearby_vehicle.is_connected());
	return AutonomousVehicle::compute_vehicle_following_gap_for_lane_change(
		nearby_vehicle, current_lambda_1);
}

double ConnectedAutonomousVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_cav_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

bool ConnectedAutonomousVehicle::check_if_is_asking_for_cooperation(
	const NearbyVehicle& nearby_vehicle)
{
	if (nearby_vehicle.is_requesting_to_merge_ahead())
	{
		long lane_change_request_veh_id =
			nearby_vehicle.get_lane_change_request_veh_id();
		if (lane_change_request_veh_id == nearby_vehicle.get_id()) 
		{
			/* The nearby veh is requesting a gap for itself */
			// do nothing
		}
		else 
		{
			/* The nearby veh is requesting a gap for someone
			else in its platoon */
		}
		/* Updating the assisted vehicle parameters */
		/*if (lane_change_request_veh_id != assisted_vehicle_id) {
			controller.update_assisted_vehicle(
				get_velocity(), *nearby_vehicle);
		}*/
		return true;
	}
	return false;
}

void ConnectedAutonomousVehicle::compute_connected_safe_gap_parameters()
{
	lambda_0_connected =
		compute_lambda_0(max_jerk, comfortable_acceleration,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_connected =
		compute_lambda_1(max_jerk, comfortable_acceleration,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_lane_change_connected =
		compute_lambda_1(max_jerk, comfortable_acceleration,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
}