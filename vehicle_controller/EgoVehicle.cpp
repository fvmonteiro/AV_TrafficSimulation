/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <set>  // temporary
#include <string>
#include <sstream>

#include "ControlManager.h"
#include "EgoVehicle.h"

EgoVehicle::EgoVehicle(long id, double simulation_time_step, 
	double creation_time, bool verbose) :	
	simulation_time_step{ simulation_time_step },
	creation_time{ creation_time },
	verbose{ verbose } {
	
	this->id = id;
	this->tau_d = std::exp(-simulation_time_step / tau);

	if (verbose) {
		std::clog << "Creating vehicle " << id 
			<< " at time " << creation_time
			<< " with simulation time step " << simulation_time_step
			<< std::endl;
	}
	/* Note: the controller is only created once the category is set
	because vehicle parameters depend on the category and type. */
}

EgoVehicle::EgoVehicle(long id, double simulation_time_step, 
	double creation_time) :
	EgoVehicle(id, simulation_time_step, creation_time, false) {}

EgoVehicle::~EgoVehicle() {
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::active_lane_change_direction,
		Member::leader_id,
		//Member::ttc,
		//Member::drac,
		//Member::collision_severity_risk
	};
	if (verbose) {
		std::clog << write_header(members, true);
		std::clog << "Vehicle " << id
			<< " out of the simulation"
			<< "at time " << get_time()
			<< std::endl;

		write_simulation_log(members);
	}
}

/* "Current" getters ------------------------------------------------------ */

double EgoVehicle::get_time() const {
	/* At creation time, the vehicle already has a velocity */
	return creation_time + (velocity.size() - 1) * simulation_time_step; 
}
long EgoVehicle::get_lane() const {
	return lane.back(); 
}
long EgoVehicle::get_link() const {
	return link.back();
}
double EgoVehicle::get_lateral_position() const {
	return lateral_position.back();
}
RelativeLane EgoVehicle::get_preferred_relative_lane() const {
	return preferred_relative_lane.back();
}
double EgoVehicle::get_velocity() const {
	if (velocity.empty()) { /* We shouldn't need to check this condition. 
							This is used to avoid crashing during tests.*/
		return 0;
	}
	return velocity.back(); 
}
double EgoVehicle::get_acceleration() const {
	return acceleration.back(); 
}
double EgoVehicle::get_desired_acceleration() const {
	return desired_acceleration.empty()? 0: desired_acceleration.back();
}
double EgoVehicle::get_vissim_acceleration() const {
	return vissim_acceleration.back();
}
RelativeLane EgoVehicle::get_active_lane_change_direction() const {
	return active_lane_change_direction.back();
}
long EgoVehicle::get_vissim_active_lane_change() const {
	return vissim_active_lane_change.back();
}
double EgoVehicle::get_lane_end_distance() const {
	return lane_end_distance.back();
}
long EgoVehicle::get_leader_id() const {
	return leader_id.back();
}
EgoVehicle::State EgoVehicle::get_state() const {
	return state.empty() ? State::lane_keeping : state.back();
}
double EgoVehicle::get_ttc() const {
	return ttc.back();
}
double EgoVehicle::get_drac() const {
	return drac.back();
}
double EgoVehicle::get_collision_risk() const {
	return collision_severity_risk.back();
}
/* ------------------------------------------------------------------------ */

/* Other getters and setters ---------------------------------------------- */
double EgoVehicle::get_current_max_brake() const {
	if (is_lane_changing()) return get_lane_change_max_brake();
	return max_brake;
}

VehicleParameters EgoVehicle::get_static_parameters() const {
	return {
		type,
		simulation_time_step,
		max_brake,
		comfortable_brake,
		get_lane_change_max_brake(),
		comfortable_acceleration,
		desired_velocity,
		lambda_1,
		lambda_1_lane_change,
		lambda_1_connected,
		lambda_1_lane_change_connected,
		is_connected()
	};
}

double EgoVehicle::get_free_flow_velocity() const {
	return try_go_at_max_vel ? MAX_VELOCITY: desired_velocity;
}

double EgoVehicle::get_time_headway_to_assisted_vehicle() {
	if (is_cooperating_to_generate_gap())
	{
		return controller.get_gap_generation_lane_controller().
			get_veh_following_time_headway();
	}
	return 0;
}

double EgoVehicle::get_safe_time_headway() const {
	return controller.get_origin_lane_controller().
		get_safe_time_headway(has_lane_change_intention());
}

double EgoVehicle::get_dest_follower_time_headway() const {
	return controller.get_destination_lane_controller().
		get_follower_time_headway();
}

double EgoVehicle::get_gap_error() const
{
	return controller.get_gap_error();
}

void EgoVehicle::set_lane(long lane) {
	this->lane.push_back(lane);
}
void EgoVehicle::set_link(long link) {
	this->link.push_back(link);
}
void EgoVehicle::set_lateral_position(double lateral_position) {
	this->lateral_position.push_back(lateral_position);
}
void EgoVehicle::set_velocity(double velocity) {
	this->velocity.push_back(velocity);
}
void EgoVehicle::set_acceleration(double acceleration) {
	this->acceleration.push_back(acceleration);
}
void EgoVehicle::set_vissim_acceleration(double vissim_acceleration) {
	this->vissim_acceleration.push_back(vissim_acceleration);
}
void EgoVehicle::set_active_lane_change_direction(long direction) {
	this->active_lane_change_direction.push_back(
		RelativeLane::from_long(direction));
}
void EgoVehicle::set_vissim_active_lane_change(int active_lane_change) {
	this->vissim_active_lane_change.push_back(active_lane_change);
}

void EgoVehicle::set_type(long type) {

	// we only need to set the type once
	if (this->type == VehicleType::undefined) {

		this->type = VehicleType(type);
		switch (this->type)
		{
		case VehicleType::undefined:
		case VehicleType::human_driven_car:
		case VehicleType::truck:
		case VehicleType::bus:
			this->brake_delay = HUMAN_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = false;
			break;
		case VehicleType::acc_car:
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = false;
			break;
		case VehicleType::autonomous_car:
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = true;
			break;
		case VehicleType::connected_car:
			/* The connected vehicle brake delay also depends
			on the other vehicle type. If the other vehicle is
			not connected, the ego connected vehicle behaves like
			an autonomous vehicle. This issue is addressed in parts
			of the code after the ego vehicle identifies its
			surrouding vehicles. */
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = true;
			break;
		case VehicleType::traffic_light_acc_car:
		case VehicleType::traffic_light_cacc_car: 
			/* both types have the same parameters */
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = true;
			break;
		default:
			this->brake_delay = AUTONOMOUS_BRAKE_DELAY;
			this->is_lane_change_decision_autonomous = false;
			break;
		}

		/* Category should have been set before type, but 
		it doens't hurt to double check*/
		if (this->category != VehicleCategory::undefined) {
			compute_safe_gap_parameters();
			this->controller = ControlManager(get_static_parameters(),
				verbose);
		}
		else {
			std::clog << "[set_type] t=" << get_time()
				<< ", id=" << id
				<< ": trying to create controller before category is set. "
				<< "Category will be set based on the type to avoid "
				<< "crashing the simulation." << std::endl;
			set_category(type / 100);
			compute_safe_gap_parameters();
			this->controller = ControlManager(get_static_parameters(),
				verbose);
		}
	}
}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane) {
	this->preferred_relative_lane.push_back(
		RelativeLane::from_long(preferred_relative_lane));
	//set_desired_lane_change_direction(preferred_relative_lane);	
}

void EgoVehicle::set_rel_target_lane(long target_relative_lane) {
	this->relative_target_lane = 
		RelativeLane::from_long(target_relative_lane);
	//set_desired_lane_change_direction(target_relative_lane);
}

void EgoVehicle::set_lane_end_distance(double lane_end_distance,
	long lane_number) {
	if (lane_number == get_lane()) {
		this->lane_end_distance.push_back(lane_end_distance);
	}
}

void EgoVehicle::set_traffic_light_info(int traffic_light_id,
	double distance)
{
	if (has_next_traffic_light() 
		&& (traffic_light_id != next_traffic_light_id))
	{
		time_crossed_last_traffic_light = get_time();
		/*next_next_traffic_light_id = next_traffic_light_id > 0 ?
			next_traffic_light_id + 1 : 0;*/
	}
	next_traffic_light_id = traffic_light_id;
	distance_to_next_traffic_light = distance;
}

//void EgoVehicle::set_traffic_light_distance(const TrafficLight& traffic_light,
//	double distance)
//{
//	distance_to_next_traffic_light = distance;
//	
//	/* We can check to decide whether or not to update the pointer, but 
//	maybe it's cheaper to just set it at every step (as above). */
//	if (next_traffic_light == nullptr)
//	{
//		/* this happens once when the vehicle is created. Should we just put 
//		it in the constructor? Or maybe initialize all traffic lights with
//		id zero? */
//		next_traffic_light = std::make_shared<TrafficLight>(traffic_light);
//		if (verbose) std::clog << "First traffic light: "
//			<< next_traffic_light->get_id() << "\n";
//	}
//	else if (traffic_light.get_id() != next_traffic_light->get_id())
//	{
//		last_traffic_light = next_traffic_light;
//		next_traffic_light = std::make_shared<TrafficLight>(traffic_light);
//		if (verbose) std::clog << "New traffic light: " 
//			<< next_traffic_light->get_id() << "\n"
//			<< "Last traffic light: " << last_traffic_light->get_id() << "\n";
//	}
//}

//void EgoVehicle::set_traffic_light_state(TrafficLight& traffic_light,
//	long state)
//{
//
//}
//
//void EgoVehicle::set_traffic_light_state_start_time(TrafficLight& traffic_light,
//	double start_time)
//{
//
//}

/* Nearby Vehicles methods ------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles() 
{
	nearby_vehicles.clear();
	leader = nullptr;
	follower = nullptr;
	destination_lane_leader = nullptr;
	destination_lane_follower = nullptr;
	assisted_vehicle = nullptr;
}

void EgoVehicle::save_nearby_vehicles_ids() {
	if (has_leader()) {
		leader_id.push_back(leader->get_id());
	}
	else {
		leader_id.push_back(0);
	}
	if (has_destination_lane_leader()) {
		dest_lane_leader_id = destination_lane_leader->get_id();
	}
	else {
		dest_lane_leader_id = 0;
	}
	if (has_destination_lane_follower()) {
		dest_lane_follower_id = destination_lane_follower->get_id();
	}
	else {
		dest_lane_follower_id = 0;
	}
	if (has_assisted_vehicle()) {
		assisted_vehicle_id = assisted_vehicle->get_id();
	}
	else{
		assisted_vehicle_id = 0;
	}
}

void EgoVehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) 
{
	std::shared_ptr<NearbyVehicle> nearby_vehicle = 
		std::shared_ptr<NearbyVehicle>(new NearbyVehicle(id, relative_lane,
		relative_position));
	nearby_vehicles.push_back(std::move(nearby_vehicle));
}

std::shared_ptr<NearbyVehicle> EgoVehicle::peek_nearby_vehicles() const {
	if (!nearby_vehicles.empty()) {
		return nearby_vehicles.back();
	}
	std::clog << "Empty nearby_vehicles container in vehicle  " << id <<
		std::endl;
	return nullptr;
}

void EgoVehicle::set_nearby_vehicle_type(long nv_type) 
{
	/* If the ego vehicle is connected, the ego "tries" to know the 
	other's type. Otherwise, it is set as undefined. */
	if (is_connected()) 
	{
		peek_nearby_vehicles()->set_type(nv_type);
	}
	/*else 
	{
		peek_nearby_vehicles()->set_type(VehicleType::undefined);
	}*/
}

bool EgoVehicle::has_leader() const {
	return leader != nullptr;
}
bool EgoVehicle::has_follower() const {
	return follower != nullptr;
}
bool EgoVehicle::has_destination_lane_leader() const {
	return destination_lane_leader != nullptr;
}
bool EgoVehicle::has_destination_lane_follower() const {
	return destination_lane_follower != nullptr;
}

bool EgoVehicle::has_assisted_vehicle() const {
	return assisted_vehicle != nullptr;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_leader() const {
	return leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_follower() const {
	return follower;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_destination_lane_leader()
const {
	return destination_lane_leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_destination_lane_follower()
const {
	return destination_lane_follower;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_assisted_vehicle()
const {
	return assisted_vehicle;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_nearby_vehicle_by_id(
	long nv_id) const {
	for (std::shared_ptr<NearbyVehicle> nv : nearby_vehicles) {
		if (nv->get_id() == nv_id) return nv;
	}
	
	/* If we don't find the id in the current nearby vehicle list, 
	the vehicle is way behind us. In this case, we just create a far away 
	virtual vehicle to force the ego vehicle into low vel. control mode */
	/*std::shared_ptr<NearbyVehicle> virtual_vehicle =
		std::shared_ptr<NearbyVehicle>(new
			NearbyVehicle(nv_id, RelativeLane::same, -3));
	virtual_vehicle->set_relative_velocity(
		nv_vel);
	virtual_vehicle->set_distance();*/
	return nullptr;
}

/* TEMPORATY FUNCTION TO DOUBLE CHECK NEARBY_VEHICLES VECOTR */
void EgoVehicle::nv_double_check() {
	std::set<int> nv_ids;
	std::clog << "\tNearby vehicles\n\t";
	for (std::shared_ptr<NearbyVehicle> nearby_vehicle : nearby_vehicles) {
		std::clog << nearby_vehicle->get_id() << ", ";
		nv_ids.insert(nearby_vehicle->get_id());
	}
	std::clog << std::endl;

	std::clog << "\tSaved pointers\n\t";
	int id;
	if (has_leader()) {
		id = leader->get_id();
		std::clog << id;
		if (nv_ids.find(id) == nv_ids.end()) {
			std::clog << ", saved veh not in most recent nearby_vehicles vector" 
				<< std::endl;
		}
	}
	if (has_follower()) {
		id = follower->get_id();
		std::clog << ", " << id;
		if (nv_ids.find(id) == nv_ids.end()) {
			std::clog << ", saved veh not in most recent nearby_vehicles vector" 
				<< std::endl;
		}
	}
	if (has_destination_lane_leader()) {
		id = destination_lane_leader->get_id();
		std::clog << ", " << id;
		if (nv_ids.find(id) == nv_ids.end()) {
			std::clog << ", saved veh not in most recent nearby_vehicles vector" 
				<< std::endl;
		}
	}
	if (has_destination_lane_follower()) {
		id = destination_lane_follower->get_id();
		std::clog << ", " << id;
		if (nv_ids.find(id) == nv_ids.end()) {
			std::clog << ", saved veh not in most recent nearby_vehicles vector" 
				<< std::endl;
		}
	}
	if (assisted_vehicle != nullptr) {
		id = assisted_vehicle->get_id();
		std::clog << ", " << id;
		if (nv_ids.find(id) == nv_ids.end()) {
			std::clog << ", saved veh not in most recent nearby_vehicles vector" 
				<< std::endl;
		}
	}
	std::clog << std::endl;
}

void EgoVehicle::analyze_nearby_vehicles() {

	bool dest_lane_leader_has_leader = false;
	const long old_leader_id = leader_id.empty() ? 0 : leader_id.back();

	for (int i = 0; i < nearby_vehicles.size(); i++) {
		std::shared_ptr<NearbyVehicle> nearby_vehicle = nearby_vehicles[i];
		long current_id = nearby_vehicle->get_id();
		RelativeLane nv_relative_lane = 
			nearby_vehicle->get_relative_lane();
		long relative_position =
			nearby_vehicle->get_relative_position();

		/*if (verbose) {
			std::clog << nearby_vehicle->to_string() << std::endl;
		}*/

		// Looking for "real" (same lane) leader and follower
		if (((relative_position == 1) 
			&& (nv_relative_lane == RelativeLane::same))
			|| nearby_vehicle->is_cutting_in()) {

			//if (verbose) std::clog << "possible leader" << std::endl;
			if (!has_leader()
				|| (nearby_vehicle->get_distance() < leader->get_distance())) {
				//if (verbose) std::clog << "chosen as leader" << std::endl;
				leader = nearby_vehicle;
			}
		}
		if ((relative_position == -1) 
			&& (nv_relative_lane == RelativeLane::same)) {
			follower = nearby_vehicle;
		}

		// Looking for destination lane leader and follower
		if (has_lane_change_intention()
			&& nv_relative_lane == desired_lane_change_direction) {

			if (relative_position == 1) {
				// Update controller if new leader
				if (current_id != dest_lane_leader_id) {
					controller.update_destination_lane_leader(
						get_velocity(), *nearby_vehicle);
				}
				destination_lane_leader = nearby_vehicle;
			}
			else if (relative_position == 2) {
				dest_lane_leader_has_leader = true;
			}
			else if (relative_position == -1) {
				// Update parameters if new follower
				if (current_id != dest_lane_follower_id) {
					controller.update_follower_time_headway(*nearby_vehicle);
				}
				destination_lane_follower = nearby_vehicle;
			}
		}

		/* Dealing with cooperation requests */
		if (is_connected() 
			&& nearby_vehicle->is_requesting_to_merge_ahead()) {
			/* Updating the assisted vehicle parameters */
			long lane_change_request_veh_id = 
				nearby_vehicle->get_lane_change_request_veh_id();

			if (lane_change_request_veh_id == nearby_vehicle->get_id()) {
				/* The nearby veh is requesting a gap for itself */
				assisted_vehicle = nearby_vehicle;
			}
			else {
				/* The nearby veh is requesting a gap for someone
				else in its platoon */
			}

			if (lane_change_request_veh_id != assisted_vehicle_id) {
				controller.update_assisted_vehicle(
					get_velocity(), *nearby_vehicle);
			}
			
		}

		if (is_connected() 
			&& nearby_vehicle->is_requesting_to_merge_behind()) {
			try_go_at_max_vel = true;
		}
		else {
			try_go_at_max_vel = false;
		}
	}

	/* We can only figure out if the leader changed or not after the
	loop explores all nearby vehicles. Thus, we can only decide whether
	to update the origin lane controller here. */
	if (has_leader()
		&& (leader->get_id() != old_leader_id)) {
		
		//if (verbose) std::clog << "updating leader info" << std::endl;

		controller.update_origin_lane_leader(
			get_velocity(), old_leader_id != 0, *leader);
	}	

	/* To avoid deadlocks, we overtake the destination lane leader in case
	it is stopped and has no leader. This situation means that the dest
	lane leader is not moving because we are too close to it.*/
	if (has_destination_lane_leader() 
		&& (destination_lane_leader->compute_velocity(get_velocity()) < 0.1)
		&& !dest_lane_leader_has_leader) {
		destination_lane_follower = destination_lane_leader;
		destination_lane_leader = nullptr;
	}

	/* We need to avoid a deadlock in case the ego vehicle is already too
	close to the vehicle asking to move in and the vehicle asking to move in
	is already very slow. The only solution would be for the ego vehicle to
	go backwards, which would lead to a deadlock situation. */
	if (has_assisted_vehicle()
		&& (assisted_vehicle->compute_velocity(get_velocity()) < 1)
		&& compute_gap(assisted_vehicle) < 1) {
		assisted_vehicle = nullptr;
	}

	//if (verbose) nv_double_check();

	save_nearby_vehicles_ids();
}

double EgoVehicle::get_relative_velocity_to_leader() {
	return has_leader() ? leader->get_relative_velocity() : 0.0;
}

double EgoVehicle::compute_gap(const NearbyVehicle& nearby_vehicle) const {
	/* Vissim's given "distance" is the distance between both front bumpers, 
	so we subtract the length from that. We need to check which vehicle is 
	ahead to determine whose length must be subtracted. */
	if (nearby_vehicle.get_id() <= 0) { // "empty" vehicle
		return MAX_DISTANCE;
	}
	if (nearby_vehicle.is_ahead()) {
		return nearby_vehicle.get_distance()
			- nearby_vehicle.get_length();
	}
	else {
		return -nearby_vehicle.get_distance() - length;
	}
}

double EgoVehicle::compute_gap(
	const std::shared_ptr<NearbyVehicle> nearby_vehicle) const {
	if (nearby_vehicle != nullptr) {
		return compute_gap(*nearby_vehicle);
	}
	else {
		return MAX_DISTANCE;
	}
}

long EgoVehicle::create_lane_change_request() {
	if (is_connected()) return desired_lane_change_direction.to_int() * id;
	else return 0;
}

/* Traffic lights --------------------------------------------------------- */
bool EgoVehicle::has_next_traffic_light() const {
	return next_traffic_light_id != 0;
}

/* State-machine related methods ------------------------------------------ */

void EgoVehicle::update_state() {

	set_desired_lane_change_direction();

	State old_state = state.empty() ? State::lane_keeping : state.back();
	if (desired_lane_change_direction == RelativeLane::same) {
		state.push_back(State::lane_keeping);
	}
	else {
		state.push_back(State::intention_to_change_lanes);
	}

	/* State change:
	- Reset timers when the vehicle first shows its intention to
	change lanes.
	- Reset the desired velocity filter when the vehicle
	finished a lane change */
	if (old_state != get_state()) {
		switch (get_state()) {
		case State::intention_to_change_lanes:
			waiting_time = 0.0;
			controller.start_longitudinal_adjustment(get_time()/*, 
				get_velocity(),
				adjustment_speed_factor*/);
			break;
		case State::lane_keeping:
			waiting_time = 0.0;
			controller.reset_origin_lane_velocity_controller(
				get_velocity());
			break;
		default:
			break;
		}
	}
}

bool EgoVehicle::is_lane_changing() const {
	return get_active_lane_change_direction() != RelativeLane::same;
}

//EgoVehicle::State EgoVehicle::get_previous_state() const {
//	if (state.size() > 2) return state[state.size() - 2];
//	else return State::lane_keeping;
//}

long EgoVehicle::get_color_by_controller_state() {
	/* We'll assign color to vehicles based on the current longitudinal
	controller and on whether or not the vehicle is trying to change lanes.*/
	if (state.empty()) {
		return orig_lane_vel_control_color;
	}
	
	/* TODO: Rewrite code to avoid all these swicth statements.
	Possible solution: ControlManager has a map of controllers and
	controllers get assigned colors for their states.
	Then we can just call controllers[active_controller].get_state_color() */

	switch (controller.get_active_longitudinal_controller()) {
	case ControlManager::ActiveACC::origin_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return try_go_at_max_vel? 
				orig_lane_max_vel_control_color : orig_lane_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return orig_lane_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ActiveACC::cooperative_gap_generation:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return gap_generation_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return gap_generation_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ActiveACC::destination_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return dest_lane_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return dest_lane_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ActiveACC::end_of_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return end_of_lane_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return end_of_lane_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ActiveACC::traffic_light_acc:
		switch (controller.get_longitudinal_controller_with_traffic_lights_state())
		{
		case LongitudinalControllerWithTrafficLights::State::max_accel:
			return max_accel_color;
		case LongitudinalControllerWithTrafficLights::State::vehicle_following:
			return veh_foll_color;
		case LongitudinalControllerWithTrafficLights::State::velocity_control:
			return vel_control_color;
		case LongitudinalControllerWithTrafficLights::State::traffic_light:
			return traffic_light_color;
		case LongitudinalControllerWithTrafficLights::State::too_close:
			return too_close_color;
		default:
			return WHITE;
		}
	case ControlManager::ActiveACC::vissim:
		return CYAN;
	default:
		return WHITE;
	}
}

std::string EgoVehicle::print_detailed_state() {
	std::string state_str = 
		state_to_string(get_state()) + ", "
		+ ControlManager::active_ACC_to_string(
			controller.get_active_longitudinal_controller()) + ", "
		+ LongitudinalController::state_to_string(
			controller.get_longitudinal_controller_state());
	return state_str;
}

void EgoVehicle::update_waiting_time() {
	if (get_velocity() < 5.0/3.6) {
		waiting_time += simulation_time_step;
	}
	else {
		waiting_time = 0.0;
	}
}

bool EgoVehicle::give_lane_change_control_to_vissim() const {
	return ((type == VehicleType::autonomous_car)
		&& (waiting_time > max_waiting_time));
}

/* Control related methods ------------------------------------------------ */

double EgoVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights) 
{
	double desired_acceleration = 0.0;
	bool include_low_level_dynamics = true;
	switch (type)
	{
	case VehicleType::undefined:
	case VehicleType::truck:
	case VehicleType::bus:
	case VehicleType::human_driven_car:
		desired_acceleration = controller.
			use_vissim_desired_acceleration(*this);
		break;
	case VehicleType::acc_car:
		desired_acceleration = controller.
			get_acc_desired_acceleration(*this);
		break;
	case VehicleType::autonomous_car:
		desired_acceleration = controller.
			get_av_desired_acceleration(*this);
		break;
	case VehicleType::connected_car:
		desired_acceleration = controller.
			get_cav_desired_acceleration(*this);
		break;
	case VehicleType::traffic_light_acc_car:
	case VehicleType::traffic_light_cacc_car:
		/* both types call the same control method (at least for now) */
		desired_acceleration = controller.
			get_traffic_light_acc_acceleration(*this, traffic_lights);
		include_low_level_dynamics = false;
		break;
	default:
		break;
	}

	this->desired_acceleration.push_back(desired_acceleration);
	if (include_low_level_dynamics)
	{
		return consider_vehicle_dynamics(desired_acceleration);
	}
	else return desired_acceleration;
}

double EgoVehicle::consider_vehicle_dynamics(double desired_acceleration) {
	/* We assume lower level dynamics as:
	a = u / (tau.s + 1) => tau.da/dt + a = u 
	da/dt = 1/tau * (u - a)
	Discrete approximation:
	a(k+1) = a(k) + (1 - alpha)*(u(k+1) - a(k))
	where alpha = exp(-delta/tau), and delta is the sampling interval */
	double current_acceleration = get_acceleration();
	return current_acceleration + (1 - tau_d)
		* (desired_acceleration - current_acceleration);
}

double EgoVehicle::compute_safe_lane_change_gap(
	std::shared_ptr<NearbyVehicle> other_vehicle) {
	double safe_gap = 0.0;
	if (other_vehicle != nullptr) {
		safe_gap = controller.compute_safe_lane_change_gap(*this,
			*other_vehicle);
	}
	return std::max(safe_gap, 1.0);
}

double EgoVehicle::get_reference_gap() {
	return controller.get_reference_gap(velocity.back(), 
		has_lane_change_intention());
}

double EgoVehicle::compute_time_headway_gap(
	std::shared_ptr<NearbyVehicle> other_vehicle) {

	double time_headway_gap = 0.0;
	if (other_vehicle != nullptr) {
		time_headway_gap = controller.compute_safe_time_headway_gap(
			get_velocity(), has_lane_change_intention(), *other_vehicle);
	}
	return time_headway_gap;
}

double EgoVehicle::compute_transient_gap(
	std::shared_ptr<NearbyVehicle> other_vehicle) {
	double transient_gap = 0.0;
	if (other_vehicle != nullptr) {
		transient_gap = controller.get_lateral_controller().
			compute_transient_gap(*this, *other_vehicle, false);
	}
	return transient_gap;
}

long EgoVehicle::decide_lane_change_direction() {
	
	/* Given the asymptotic nature of controllers, vehicles sometimes are 
	very close to, but no exactly at, the safe gap. We give it some margin
	to avoid unecessary long waits.*/
	double margin = 0.1; 
	long lane_change_direction = 0;
	if (is_lane_change_decision_autonomous 
		&& !give_lane_change_control_to_vissim()) {
		if (has_lane_change_intention()) {

			if (verbose) std::clog << "Deciding lane change" << std::endl;

			bool gap_same_lane_is_safe = (!has_leader())
				|| ((compute_gap(leader) + margin)
					>= compute_safe_lane_change_gap(leader));
			bool gap_ahead_is_safe = (!has_destination_lane_leader()) 
				|| ((compute_gap(destination_lane_leader) + margin) 
					>= compute_safe_lane_change_gap(destination_lane_leader));
			/* besides the regular safety conditions, we add the case 
			where the dest lane follower has completely stopped to give room 
			to the lane changing vehicle */
			bool gap_behind_is_safe = (!has_destination_lane_follower())
				|| ((compute_gap(destination_lane_follower) + margin)
					>= compute_safe_lane_change_gap(destination_lane_follower))
				|| ((destination_lane_follower->
					compute_velocity(get_velocity()) <= 1.0)
					&& (destination_lane_follower->get_distance() <= -1.0));
			
			bool no_conflict = !has_lane_change_conflict();
			if (verbose) {
				std::clog << "gap ahead is safe? " << gap_ahead_is_safe
					<< ", gap behind is safe? " << gap_behind_is_safe
					<< ", no conflict? " << no_conflict
					<< std::endl;
			}

			if (gap_same_lane_is_safe && gap_ahead_is_safe 
				&& gap_behind_is_safe && no_conflict) {
				// will start lane change
				lane_change_direction = 
					desired_lane_change_direction.to_int();
			}
			else {
				update_waiting_time();
				//controller.update_headways_with_risk(*this);
			}
		}
		//if (verbose) std::clog << "LC decided" << std::endl;
	}
	else {
		lane_change_direction = get_vissim_active_lane_change();
	}

	return lane_change_direction;
}

bool EgoVehicle::has_lane_change_conflict() const {

	//if (verbose) std::clog << "checking conflicts" << std::endl;

	/* If there's no lane change intention, there's no conflict */
	if (!has_lane_change_intention()) {
		return false;
	}

	for (int i = 0; i < nearby_vehicles.size();  i++) {
		NearbyVehicle& nv = *nearby_vehicles[i];

		if (nv.is_lane_changing()) {
			RelativeLane& nv_lane = nv.get_relative_lane();
			RelativeLane& nv_lc_direction = nv.get_lane_change_direction();

			// Vehicles on the same lane
			if (nv_lane == RelativeLane::same) {
				if (nv_lc_direction
					== desired_lane_change_direction) return true;
			}
			// Vehicles on other lanes
			else {
				bool nv_moving_towards_ego = 
					!nv_lane.on_same_side(nv_lc_direction);
				bool ego_moving_towards_nv = 
					nv_lane.on_same_side(desired_lane_change_direction);

				if (nv_moving_towards_ego && ego_moving_towards_nv) 
					return true;
			}
		}
	}
	return false;
}

bool EgoVehicle::is_cooperating_to_generate_gap() const {
	/* When does the ego vehicle cooperate:
	The ego vehicle is someone's destination lane follower AND:
	- The ego vehicle is using the origin lane controller and 
	the vehicle requesting cooperation is far away enough 
	OR
	- The ego vehicle is also trying to change lanes
	*/
	/*if (assisted_vehicle != nullptr) {
		if (controller.get_active_longitudinal_controller()
			== ControlManager::ActiveACC::origin_lane) {
			double max_braking_distance =
				std::pow(get_velocity(), 2) / max_brake;
			return compute_gap(assisted_vehicle) > max_braking_distance;
		}
		return true;
	}
	return false;*/
	return assisted_vehicle != nullptr;
}

//std::shared_ptr<NearbyVehicle> EgoVehicle::find_nearby_vehicle(
//	RelativeLane relative_lane, int relative_position) const {
//
//	auto it = std::find_if(nearby_vehicles.begin(), nearby_vehicles.end(),
//		[&relative_lane, &relative_position](const NearbyVehicle* nv) {
//			return (nv->get_relative_lane() == relative_lane)
//				&& (nv->get_relative_position() == relative_position); });
//	if (it != nearby_vehicles.end()) {
//		return *it;
//	}
//	return nullptr;
//	/*for (NearbyVehicle* nearby_vehicle : nearby_vehicles) {
//		if ((nearby_vehicle->get_relative_lane() 
//			 == relative_lane)
//			& 
//			(nearby_vehicle->get_relative_position() 
//			 == relative_position)) {
//			return nearby_vehicle;
//		}
//	}
//	return nullptr;*/
//}


/* Computation of surrogate safety measurements --------------------------- */

void EgoVehicle::compute_all_ssms() {
	if (has_leader()) {
		ttc.push_back(compute_ttc(*leader));
		drac.push_back(compute_drac(*leader));
		collision_severity_risk.push_back(compute_collision_severity_risk(
			*leader));
	}
	else {
		ttc.push_back(-1.0);
		drac.push_back(-1.0);
		collision_severity_risk.push_back(-1.0);
	}
}

double EgoVehicle::compute_ttc(const NearbyVehicle& other_vehicle) {
	/* Time-to-collision is:
	(leader x - leader length - ego x) / (ego vel - leader vel), 
		if ego vel > leader vel 
	underfined, 
		if ego vel < leader vel */
	if (other_vehicle.get_relative_velocity() > 0) {
		return compute_gap(other_vehicle) / other_vehicle.get_relative_velocity();
	}
	return -1.0;	
}

double EgoVehicle::compute_drac(const NearbyVehicle& other_vehicle) {
	/* Deceleration rate to avoid collision
	(ego vel - leader vel)^2 / 2*(leader x - leader length - ego x),
		if ego vel > leader vel
	underfined,
		if ego vel < leader vel
	*/
	if (other_vehicle.get_relative_velocity() > 0) {
		return std::pow(other_vehicle.get_relative_velocity(), 2)
			/ 2 / compute_gap(other_vehicle);
	}
	return -1.0;
}

double EgoVehicle::compute_exact_collision_free_gap(
	const NearbyVehicle& other_vehicle) const {

	double follower_lambda_0, follower_lambda_1;
	double v_follower, v_leader;
	double brake_follower, brake_leader;
	double ego_velocity = get_velocity();
	double delta_v = other_vehicle.get_relative_velocity();
	if (other_vehicle.is_ahead()) {
		follower_lambda_0 = lambda_0;
		follower_lambda_1 = lambda_1;
		v_follower = ego_velocity;
		v_leader = v_follower - delta_v;
		brake_follower = get_current_max_brake();
		brake_leader = other_vehicle.get_max_brake();
	}
	else {
		follower_lambda_0 = other_vehicle.get_lambda_0();
		follower_lambda_1 = other_vehicle.get_lambda_1();
		v_leader = ego_velocity;
		v_follower = v_leader - delta_v;
		brake_follower = other_vehicle.get_max_brake();
		brake_leader = max_brake;
	}

	double stop_time_follower = (v_follower + follower_lambda_1) / brake_follower;
	double stop_time_leader = v_leader / brake_leader;
	double collision_free_gap;

	if (stop_time_follower >= stop_time_leader) {
		collision_free_gap =
			std::pow(v_follower + follower_lambda_1, 2) / 2 / brake_follower
			- std::pow(v_leader, 2) / 2 / brake_leader + follower_lambda_0;
	}
	else if (brake_leader < brake_follower) {
		collision_free_gap =
			std::pow(delta_v - follower_lambda_1, 2) / 2 / (brake_follower - brake_leader)
			+ follower_lambda_0;
	}
	else {
		collision_free_gap = 0.0;
	}
	return collision_free_gap;
}

double EgoVehicle::compute_collision_severity_risk(
	const NearbyVehicle& other_vehicle) {
	
	double current_max_brake, current_lambda_1;
	if (is_lane_changing()) {
		current_max_brake = get_lane_change_max_brake();
		current_lambda_1 = lambda_1_lane_change;
	}
	else {
		current_max_brake = max_brake;
		current_lambda_1 = lambda_1;
	}

	double jerk_delay = (comfortable_acceleration + current_max_brake) / max_jerk;
	double ego_vel = get_velocity();

	double leader_max_brake = other_vehicle.get_max_brake();
	double relative_vel = other_vehicle.get_relative_velocity();
	double leader_vel = other_vehicle.compute_velocity(ego_vel);

	double gamma = leader_max_brake / current_max_brake;
	double gamma_threshold = leader_vel / (ego_vel + current_lambda_1);

	std::vector<double> gap_thresholds(4);
	gap_thresholds[0] = brake_delay
		* (brake_delay * (comfortable_acceleration + leader_max_brake) / 2
			+ relative_vel);
	gap_thresholds[1] = (brake_delay + jerk_delay)
		* (current_lambda_1 + relative_vel
			- (brake_delay + jerk_delay)
			* (current_max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[2] = leader_vel / leader_max_brake
		* (current_lambda_1 + relative_vel
			- leader_vel / leader_max_brake
			* (current_max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[3] = compute_exact_collision_free_gap(other_vehicle);

	double gap = compute_gap(other_vehicle);

	if (verbose & (gap < gap_thresholds[3])) {
		std::clog << "Collision prone situation\n"
			<< "\tgamma = " << gamma << ", gamma_t = " << gamma_threshold
			<< "\n\tgap = " << gap << ", thresholds: ";
		for (double g : gap_thresholds) {
			std::clog << g << ", ";
		}
		std::clog << std::endl;
	}

	double result = 0;
	if (gap < gap_thresholds[0]) {
		result = std::pow(relative_vel, 2)
			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
	}
	else if (gap < gap_thresholds[1]) {
		/* The solution for this case requires solving a 3rd degree equation.
		To avoid that, we will approximate it as the mean of the previous
		and following case. We will also record how often this case
		happens to see if it's important to properly code the solution. */
		result = std::pow(relative_vel, 2)
			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
		result += std::pow(relative_vel + current_lambda_1, 2)
			+ 2 * (leader_max_brake - current_max_brake)
			* (gap - lambda_0);
		result /= 2;
		std::clog << "t=" << get_time()
			<< ", id=" << id
			<< ", collision severity complicated case"
			<< std::endl;
	}
	else if (((gamma >= gamma_threshold) && (gap < gap_thresholds[2]))
		|| (gamma < gamma_threshold) && (gap < gap_thresholds[3])) {
		result = std::pow(relative_vel + current_lambda_1, 2)
			+ 2 * (leader_max_brake - current_max_brake)
			* (gap - lambda_0);
	}
	else if ((gamma >= gamma_threshold) && (gap < gap_thresholds[3])) {
		result = std::pow(ego_vel + current_lambda_1, 2)
			- 2 * current_max_brake
			* (std::pow(leader_vel, 2) / 2 / leader_max_brake
				+ gap - lambda_0);
	}
	result = std::sqrt(result);

	if (verbose) {
		std::clog << "\trisk = " << result << std::endl;
	}

	return result;
}

double EgoVehicle::compute_collision_severity_risk_to_leader() {
	if (has_leader()) {
		return compute_collision_severity_risk(*get_leader());
	}
	return 0.0;
}

/* Private methods -------------------------------------------------------- */

void EgoVehicle::set_desired_lane_change_direction(
	/*long preferred_relative_lane*/) {
	/* Both preferred_relative_lane and relative_target_lane indicate
	desire to change lanes. The former indicates preference due to 
	routing, so it takes precedence over the latter. */
	RelativeLane& pref_rel_lane = get_preferred_relative_lane();

	/* TODO: check all lane change possible variables from vissim */
	/*if (verbose) {
		std::clog << 
	}*/

	if (type == VehicleType::traffic_light_acc_car
		|| type == VehicleType::traffic_light_cacc_car)
	{
		desired_lane_change_direction = RelativeLane::same;
	}
	else
	{
		if (pref_rel_lane.is_to_the_left()) {
			desired_lane_change_direction = RelativeLane::left;
		}
		else if (pref_rel_lane.is_to_the_right()) {
			desired_lane_change_direction = RelativeLane::right;
		}
		else if (relative_target_lane.is_to_the_left()) {
			desired_lane_change_direction = RelativeLane::left;
		}
		else if (relative_target_lane.is_to_the_right()) {
			desired_lane_change_direction = RelativeLane::right;
		}
		else {
			desired_lane_change_direction = RelativeLane::same;
		}
	}
}

void EgoVehicle::compute_safe_gap_parameters() {
	lambda_0 = compute_lambda_0(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
	lambda_1 = compute_lambda_1(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
	lambda_1_lane_change = compute_lambda_1(max_jerk, 
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
	
	/* Non-connected vehicles only have one value for lambda_1 and lambda_0.
	Connected vehicles have a "regular value" and a "non-connected" value
	when dealing with non-connected neighbors. */
	if (is_connected()) {
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
}

/* Methods for printing and debugging ------------------------------------- */

void EgoVehicle::write_simulation_log(std::vector<Member> members) {
	
	bool write_size = true;
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(id) + ".txt";
	vehicle_log.open(log_path + "\\" + file_name);
	if (vehicle_log.is_open()) {
		vehicle_log << write_header(members, write_size);
		vehicle_log	<< write_members(members);
		vehicle_log.close();
	}
	else {
		std::clog << "Unable to open file to write log of "
			<< "vehicle " << id
			<< std::endl;
	}
}

std::string EgoVehicle::write_header(
	std::vector<EgoVehicle::Member> members, bool write_size) {
	
	std::ostringstream oss;
	for (auto m : members) {
		oss << member_enum_to_string(m);
		if (write_size) {
			oss << " (" << get_member_size(m) << ")";
		}
		oss << ", ";
	}
	oss << std::endl;

	return oss.str();
}

std::string EgoVehicle::write_members(
	std::vector<EgoVehicle::Member> members) {

	std::ostringstream oss;

	/* Sanity check: some non critical code mistakes could
	create vector members with different sizes. This prevents
	the log from being written and crashes vissim. Let's avoid
	this and just write a warning message instead.*/
	int n_samples = (int)velocity.size(); /* velocity, lane and link members 
	are the least likely to have the wrong size */
	std::vector<int> deleted_indices;
	for (int i = 0; i < members.size(); i++) {
		Member m = members.at(i);
		if ((get_member_size(m) != 1) // not a scalar
			&& (get_member_size(m) != n_samples)) {
			oss << "Error: member " << member_enum_to_string(m)
				<< " has " << get_member_size(m) << " samples "
				<< "instead of the expected " << n_samples
				<< ". It won't be printed."
				<< std::endl;
			deleted_indices.push_back(i);
		}
	}
	for (int idx : deleted_indices) {
		members.erase(std::next(members.begin(), idx));
	}

	// Write variables over time
	for (int i = 0; i < n_samples; i++) {
		for (auto m : members) {
			switch (m)
			{
			case Member::creation_time:
				oss << creation_time + i * simulation_time_step;
				break;
			case Member::id:
				oss << id;
				break;
			case Member::length:
				oss << length;
				break;
			case Member::width:
				oss << width;
				break;
			case Member::color:
				oss << "color printing code not done";
				break;
			case Member::category:
				oss << static_cast<int>(category);
				break;
			case Member::desired_velocity:
				oss << desired_velocity;
				break;
			case Member::lane:
				oss << lane[i];
				break;
			case Member::link:
				oss << link[i];
				break;
			case Member::preferred_relative_lane:
				oss << preferred_relative_lane[i].to_string();
				break;
			case Member::velocity:
				oss << velocity[i];
				break;
			case Member::acceleration:
				oss << acceleration[i];
				break;
			case Member::desired_acceleration:
				oss << desired_acceleration[i];
				break;
			case Member::vissim_acceleration:
				oss << vissim_acceleration[i];
				break;
			case Member::leader_id:
				oss << leader_id[i];
				break;
			case Member::state:
				oss << state_to_string(state[i]);
				break;
			case Member::active_lane_change_direction:
				oss << active_lane_change_direction[i].to_string();
				break;
			case Member::vissim_active_lane_change_direction:
				oss << vissim_active_lane_change[i];
				break;
			case Member::lane_end_distance:
				oss << lane_end_distance[i];
				break;
			case Member::ttc:
				oss << ttc[i];
				break;
			case Member::drac:
				oss << drac[i];
				break;
			case Member::collision_severity_risk:
				oss << collision_severity_risk[i];
				break;
			case Member::type:
				oss << static_cast<int>(type);
				break;
			default:
				oss << "";
				break;
			}
			oss << ", ";
		}
		oss << std::endl;
	}
	return oss.str();
}

int EgoVehicle::get_member_size(Member member) {
	switch (member)
	{
	case Member::creation_time:
	case Member::id:
	case Member::length:
	case Member::width:
	case Member::color:
	case Member::category:
	case Member::desired_velocity:
	case Member::type:
		return 1;
	case Member::lane:
		return (int)lane.size();
	case Member::link:
		return (int)link.size();
	case Member::preferred_relative_lane:
		return (int)preferred_relative_lane.size();
	case Member::velocity:
		return (int)velocity.size();
	case Member::acceleration:
		return (int)acceleration.size();
	case Member::desired_acceleration:
		return (int)desired_acceleration.size();
	case Member::vissim_acceleration:
		return (int)vissim_acceleration.size();
	case Member::leader_id:
		return (int)leader_id.size();
	case Member::state:
		return (int)state.size();
	case Member::active_lane_change_direction:
		return (int)active_lane_change_direction.size();
	case Member::vissim_active_lane_change_direction:
		return (int)vissim_active_lane_change.size();
	case Member::lane_end_distance:
		return (int)lane_end_distance.size();
	case Member::ttc:
		return (int)ttc.size();
	case Member::drac:
		return (int)drac.size();
	case Member::collision_severity_risk:
		return (int)collision_severity_risk.size();
	default:
		return 0;
	}
}

std::string EgoVehicle::member_enum_to_string(Member member) {
	switch (member)
	{
	case Member::creation_time:
		return "creation_time";
	case Member::id:
		return "id";
	case Member::length:
		return "length";
	case Member::width:
		return "width";
	case Member::color:
		return "color";
	case Member::category:
		return "category";
	case Member::desired_velocity:
		return "des. vel.";
	case Member::lane:
		return "lane";
	case Member::link:
		return "link";
	case Member::preferred_relative_lane:
		return "pref. rel. lane";
	case Member::velocity:
		return "vel.";
	case Member::acceleration:
		return "accel.";
	case Member::desired_acceleration:
		return "des. accel.";
	case Member::vissim_acceleration:
		return "vissim accel.";
	case Member::leader_id:
		return "leader id";
	case Member::state:
		return "state";
	case Member::active_lane_change_direction:
		return "lc direction";
	case Member::vissim_active_lane_change_direction:
		return "vissim lc direction";
	case Member::lane_end_distance:
		return "lane end dist";
	case Member::ttc:
		return "ttc";
	case Member::drac:
		return "drac";
	case Member::collision_severity_risk:
		return "collision sev. risk";
	default:
		return "unknown memeber";
	}
}

std::string EgoVehicle::state_to_string(State vehicle_state) {

	switch (vehicle_state)
	{
	case State::lane_keeping:
		return "lane keeping";
	case State::intention_to_change_lanes:
		return "intention to change lane";
	default:
		return "ERROR: unknown vehicle state";
	}
}

std::ostream& operator<< (std::ostream& out, const EgoVehicle& vehicle)
{
	std::vector<std::pair<std::string, long>> long_members{ 
		{"Vehicle id", vehicle.get_id()}, //{"category", vehicle.get_category()}, 
		{"lane", vehicle.get_lane()} 
	};
	std::vector<std::pair<std::string, double>> double_members{
		{ "velocity", vehicle.get_velocity()},
		{ "des. velocity", vehicle.get_desired_velocity()},
		{"acceleration", vehicle.get_acceleration()},
		{"des. acceleration", vehicle.get_desired_acceleration()},
		//{"length", vehicle.get_length()}, {"width", vehicle.get_width()} 
	};

	for (auto name_value_pair : long_members) {
		out << name_value_pair.first << ": " <<
			name_value_pair.second << " | ";
	}
	for (auto name_value_pair : double_members) {
		out << name_value_pair.first << ": " << std::setprecision(4) <<
			name_value_pair.second << " | ";
	}
	out << std::endl;

	return out; // return std::ostream so we can chain calls to operator<<
}
