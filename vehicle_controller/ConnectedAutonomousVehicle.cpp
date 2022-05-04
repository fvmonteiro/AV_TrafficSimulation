#include "ConnectedAutonomousVehicle.h"

ConnectedAutonomousVehicle::ConnectedAutonomousVehicle(
	long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	AutonomousVehicle(id, VehicleType::connected_car, desired_velocity,
		true, simulation_time_step, creation_time, verbose) 
{

}

bool ConnectedAutonomousVehicle::has_assisted_vehicle() const
{
	return assisted_vehicle != nullptr;
}

bool ConnectedAutonomousVehicle::is_cooperating_to_generate_gap() const {
	return has_assisted_vehicle();
	/* Function seems unnecessary, but we might perform other checks here */
}

double ConnectedAutonomousVehicle::
implement_get_time_headway_to_assisted_vehicle() const {
	if (is_cooperating_to_generate_gap())
	{
		return controller.get_gap_generation_lane_controller().
			get_safe_time_headway();
	}
	return 0;
}

void ConnectedAutonomousVehicle::try_to_set_nearby_vehicle_type(long nv_type)
{
	peek_nearby_vehicles()->set_type(nv_type);
}

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
	//long new_follower_id{ 0 };
	if (has_destination_lane_follower())
	{
		if (old_follower == nullptr
			|| (old_follower->get_type()
				!= get_destination_lane_follower()->get_type()))
		{
			controller.update_destination_lane_follower_time_headway(
				get_destination_lane_follower()->get_h_to_incoming_vehicle());
		}
	}
	//destination_lane_follower = new_follower;
}

void ConnectedAutonomousVehicle::update_assisted_vehicle(
	const std::shared_ptr<NearbyVehicle>& old_assisted_vehicle)
{
	if (has_assisted_vehicle())
	{
		double new_max_brake = assisted_vehicle->get_max_brake();
		if (old_assisted_vehicle == nullptr
			|| (std::abs(new_max_brake
				- old_assisted_vehicle->get_max_brake()) > 0.5))
		{
			controller.update_gap_generation_controller(get_velocity(),
				compute_vehicle_following_desired_time_headway(
					*assisted_vehicle));
		}
	}
	//assisted_vehicle = new_assisted_vehicle;

	/*const long old_id = assisted_vehicle_id;
	if (has_assisted_vehicle() && assisted_vehicle->get_id() != old_id)
	{
		controller.update_assisted_vehicle(get_velocity(), 
			*assisted_vehicle);
		assisted_vehicle_id = assisted_vehicle->get_id();
	}*/
}

double ConnectedAutonomousVehicle::compute_current_desired_time_headway(
	const NearbyVehicle& leader)
{
	double current_lambda_1;
	if (leader.is_connected())
	{
		current_lambda_1 = has_lane_change_intention() ?
			lambda_1_lane_change_connected : lambda_1_connected;
	}
	else
	{
		current_lambda_1 = has_lane_change_intention() ?
			get_lambda_1_lane_change() : get_lambda_1();
	}
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_current_max_brake(), leader.get_max_brake(),
		current_lambda_1, get_rho(), 0);
}

double ConnectedAutonomousVehicle::
compute_vehicle_following_desired_time_headway(const NearbyVehicle& leader)
{
	double current_lambda_1 = leader.is_connected() ?
		lambda_1_connected : get_lambda_1();
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), leader.get_max_brake(),
		current_lambda_1, get_rho(), 0);
}

double ConnectedAutonomousVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& leader)
{
	double current_lambda_1 = leader.is_connected() ?
		lambda_1_lane_change_connected : get_lambda_1_lane_change();
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_lane_change_max_brake(), leader.get_max_brake(),
		current_lambda_1, get_rho(), 0);
}

double ConnectedAutonomousVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_cav_desired_acceleration(*this);
	return consider_vehicle_dynamics(desired_acceleration);
}

//void ConnectedAutonomousVehicle::save_other_relevant_nearby_vehicle_ids()
//{
//	AutonomousVehicle::save_other_relevant_nearby_vehicle_ids();
//	assisted_vehicle_id = has_assisted_vehicle() ?
//		assisted_vehicle->get_id() : 0;
//}

//void ConnectedAutonomousVehicle::clear_other_relevant_nearby_vehicles()
//{
//	//AutonomousVehicle::clear_other_relevant_nearby_vehicles();
//	//assisted_vehicle = nullptr;
//}

bool ConnectedAutonomousVehicle::check_if_is_asking_for_cooperation(
	const NearbyVehicle& nearby_vehicle)
{
	if (nearby_vehicle.is_requesting_to_merge_ahead())
	{
		long lane_change_request_veh_id =
			nearby_vehicle.get_lane_change_request_veh_id();
		if (lane_change_request_veh_id == nearby_vehicle.get_id()) {
			/* The nearby veh is requesting a gap for itself */
			// do nothing
		}
		else {
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