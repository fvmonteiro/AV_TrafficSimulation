#include "PlatoonVehicle.h"

//void PlatoonVehicle::find_relevant_nearby_vehicles()
//{
//	/* WILL CHANGE */
//	find_leader();
//
//	/* We can only figure out if the leader changed or not after the
//	loop explores all nearby vehicles. Thus, we can only decide whether
//	to update the origin lane controller here. */
//	/*if (has_leader()
//		&& (leader->get_id() != old_leader_id))
//	{
//		controller.update_origin_lane_leader(
//			get_velocity(), old_leader_id != 0, *leader);
//	}*/
//
//	//update_nearby_vehicles();
//}

//double PlatoonVehicle::compute_desired_acceleration(
//	const std::unordered_map<int, TrafficLight>& traffic_lights)
//{
//	double desired_acceleration = get_vissim_acceleration();
//		//controller.get_cav_desired_acceleration(*this);
//	return consider_vehicle_dynamics(desired_acceleration);
//}

//bool PlatoonVehicle::can_start_lane_change()
//{
//	return false;
//}