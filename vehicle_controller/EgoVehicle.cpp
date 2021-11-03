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
		simulation_time_step,
		max_brake,
		comfortable_brake,
		get_lane_change_max_brake(),
		comfortable_acceleration,
		desired_velocity,
		lambda_1,
		lambda_1_lane_change,
		lambda_1_connected,
		is_connected()
	};
}

double EgoVehicle::get_time_headway_to_assisted_vehicle() {
	return controller.get_gap_generation_lane_controller().
		get_desired_time_headway();
}

double EgoVehicle::get_safe_time_headway() const {
	return controller.get_origin_lane_controller().
		get_safe_time_headway(has_lane_change_intention());
}

double EgoVehicle::get_dest_follower_time_headway() const {
	return controller.get_destination_lane_controller().
		get_follower_time_headway();
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

//void EgoVehicle::set_category(VehicleCategory category) {
//	/* We only need to set the category once, but VISSIM passes the
//	category every time step. */
//
//	if (this->category == VehicleCategory::undefined) {
//		this->category = category;
//		switch (category) {
//		case VehicleCategory::truck:
//			this->max_brake = TRUCK_MAX_BRAKE;
//			this->max_jerk = TRUCK_MAX_JERK;
//			break;
//		case VehicleCategory::car:
//			this->max_brake = CAR_MAX_BRAKE;
//			this->max_jerk = CAR_MAX_JERK;
//			break;
//		default: // shouldn't happen but assume car if any other category
//			this->max_brake = CAR_MAX_BRAKE;
//			this->max_jerk = CAR_MAX_JERK;
//			break;
//		}
//	}
//}

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
		case VehicleType::ACC_car:
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

/* ------------------------------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles() {
	nearby_vehicles.clear();
}

void EgoVehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) {
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

void EgoVehicle::set_nearby_vehicle_type(long nv_type) {
	/* If both vehicle are connected, the ego can know the other's type.
	Otherwise, it is set as undefined. */

	if (is_connected() && 
		(VehicleType(nv_type) == VehicleType::connected_car)) {
		peek_nearby_vehicles()->set_type(VehicleType::connected_car);
	}
	else {
		peek_nearby_vehicles()->set_type(VehicleType::undefined);
	}
}

//void EgoVehicle::set_nearby_vehicle_lc_intention(long relative_lane) {
//	std::shared_ptr<NearbyVehicle> nearby_vehicle = peek_nearby_vehicles();
//	if (is_connected() && nearby_vehicle->is_connected()) {
//
//	}
//}

double EgoVehicle::get_relative_velocity_to_leader() {
	return has_leader() ? leader->get_relative_velocity() : 0.0;
	/*if (has_leader()) return leader->get_relative_velocity();
	else return 0.0;*/
}

//long EgoVehicle::get_leader_type() {
//	return has_leader() ? static_cast<long>(leader->get_type()) : 0;
//	/*if (has_leader()) return static_cast<long>(leader->get_type());
//	else return 0;*/
//}

void EgoVehicle::analyze_nearby_vehicles() {
	bool leader_found = false;
	bool follower_found = false;
	bool dest_lane_leader_found = false;
	bool dest_lane_follower_found = false;
	bool assisted_vehicle_found = false;

	const long old_leader_id = has_leader() ? get_leader()->get_id() : 0;
	bool dest_lane_leader_has_leader = false;
	
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
		/* We look both at the leading vehicle on the same lane
		and at possible cutting-in vehicles */
		if (((relative_position == 1) 
			&& (nv_relative_lane == RelativeLane::same))
			|| nearby_vehicle->is_cutting_in()) {

			//if (verbose) std::clog << "possible leader" << std::endl;

			if (!leader_found 
				|| (nearby_vehicle->get_distance() < leader->get_distance())) {
				//if (verbose) std::clog << "chosen as leader" << std::endl;
				leader = nearby_vehicle;
			}

			leader_found = true;
		}
		else if ((relative_position == -1) 
			&& (nv_relative_lane == RelativeLane::same)) {
			follower_found = true;
			follower = nearby_vehicle;
		}

		// Looking for destination lane leader and follower
		if (has_lane_change_intention()
			&& nv_relative_lane == desired_lane_change_direction) {
			if (relative_position == 1) {
				dest_lane_leader_found = true;
				// Update controller if new leader
				if ((!has_destination_lane_leader())
					|| (destination_lane_leader->get_id()
						!= current_id)) {
					controller.update_destination_lane_leader(
						get_velocity(), *nearby_vehicle);
				}
				destination_lane_leader = nearby_vehicle;
			}
			else if (relative_position == 2) {
				dest_lane_leader_has_leader = true;
			}
			else if (relative_position == -1) {
				dest_lane_follower_found = true;

				/* Three possible conditions to update the 
				follower:
				1. There was no previous follower
				2. The follower id changed
				3. The ego and follower are connected but
				the ego still doesn't have the follower's
				headway*/
				if ((!has_destination_lane_follower())
					|| (destination_lane_follower->get_id()
						!= current_id)
					|| (is_connected() 
						&& destination_lane_follower->is_connected()
						&& destination_lane_follower->get_h_to_incoming_vehicle() == 0)) {

					/*if (verbose) std::clog 
						<< "Dest lane foll found" 
						<< std::endl;*/

					controller.update_follower_time_headway(*nearby_vehicle);
				}
				destination_lane_follower = nearby_vehicle;
			}
		}

		if (is_connected() && nearby_vehicle->requesting_to_move_in()) {
			assisted_vehicle_found = true;
			if (!is_cooperating_to_generate_gap()
				|| (assisted_vehicle->get_id()
					!= current_id)) {
				controller.update_assisted_vehicle(
					get_velocity(), *nearby_vehicle);
			}
			assisted_vehicle = nearby_vehicle;
		}
	}

	/* We must ensure to clear the pointers in case the nearby
	vehicle was not found in this time step. */

	/* Simple cases first */
	if (!follower_found) follower = nullptr;
	if (!dest_lane_follower_found) destination_lane_follower = nullptr;

	/* We can only figure out if the leader changed or not after the
	loop explores all nearby vehicles. Thus, we can only decide whether
	to update the origin lane controller here. */
	if (leader_found) {
		long current_leader_id = leader->get_id();
		if (current_leader_id != old_leader_id) {
			bool had_leader_before = old_leader_id != 0;
			controller.update_origin_lane_leader(
				get_velocity(), had_leader_before, *leader);
		}
		leader_id.push_back(current_leader_id);
	}
	else {
		leader = nullptr;
		leader_id.push_back(0);
	}
	/* To avoid deadlocks, we overtake the destination lane leader in case
	it is stopped and has no leader. This situation means that the dest
	lane leader is not moving because we are too close to it.*/
	if (!dest_lane_leader_found) {
		destination_lane_leader = nullptr;
	}
	else if ((destination_lane_leader->compute_velocity(get_velocity()) < 0.1
			  && !dest_lane_leader_has_leader)) {
		destination_lane_follower = destination_lane_leader;
		destination_lane_leader = nullptr;
	}

	/* We need to avoid a deadlock in case the ego vehicle is already too
	close to the vehicle asking to move in and the vehicle asking to move in
	is already very slow. The only solution would be for the ego vehicle to
	go backwards, which would lead to a deadlock situation. */
	if (!assisted_vehicle_found) {
		assisted_vehicle = nullptr;
	}
	else if ((assisted_vehicle->compute_velocity(get_velocity()) < 1)
		&& compute_gap(assisted_vehicle) < 1) {
		assisted_vehicle = nullptr;
	}
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

std::shared_ptr<NearbyVehicle> EgoVehicle::get_leader() const {
	return leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_follower() const {
	return follower;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_destination_lane_leader() 
	const {
	return destination_lane_leader;
	/* [OLDEST IMPLEMENTATION] If there is no intention to change lanes, the
	function returns the leader on the same lane. */
	/*int relative_position = 1;
	NearbyVehicle* destination_lane_leader = find_nearby_vehicle(
			desired_lane_change_direction, relative_position);
	return destination_lane_leader;*/
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_destination_lane_follower() 
	const {
	return destination_lane_follower;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_assisted_vehicle()
	const {
	return assisted_vehicle;
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
	
	switch (controller.get_active_longitudinal_controller()) {
	case ControlManager::ActiveACC::origin_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return orig_lane_vel_control_color;
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
	case ControlManager::ActiveACC::vissim:
		return CYAN;
	default:
		return WHITE;
	}
	
	/*switch (get_state())
	{
	case State::lane_keeping:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return velocity_control_color;
		case LongitudinalController::State::vehicle_following:
			return vehicle_following_color;
		case LongitudinalController::State::uninitialized:
			return WHITE;
		}
	case State::intention_to_change_lanes:
		switch (controller.get_active_longitudinal_controller())
		{
		case ControlManager::ActiveLongitudinalController::vissim:
		case ControlManager::ActiveLongitudinalController::origin_lane:
			return lane_change_adjustment_origin_lane_color;
		case ControlManager::ActiveLongitudinalController::destination_lane:
			return lane_change_adjustment_destination_lane_color;
		case ControlManager::ActiveLongitudinalController::end_of_lane:
			return lane_change_adjustment_end_of_lane_color;
		}
	default:
		return WHITE;
		break;
	}*/
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

double EgoVehicle::compute_desired_acceleration() {
	double desired_acceleration = 
		controller.determine_desired_acceleration(*this);
	this->desired_acceleration.push_back(desired_acceleration);
	double feasible_acceleration = consider_vehicle_dynamics(
		desired_acceleration);
	return feasible_acceleration;
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
		time_headway_gap = controller.compute_time_headway_gap(
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
	
	if (verbose) std::clog << "deciding lane change" << std::endl;
	
	double margin = 0.01; /* Given the asymptotic nature of controllers, 
						  vehicles sometimes are very close to, but no exactly
						  at, the safe gap. We give it some margin to 
						  avoid unecessary long waits.*/
	long lane_change_direction = 0;
	if (is_lane_change_decision_autonomous 
		&& !give_lane_change_control_to_vissim()) {
		if (has_lane_change_intention()) {
			bool gap_same_lane_is_safe = (!has_leader())
				|| ((compute_gap(leader) + 0.01)
					>= compute_safe_lane_change_gap(leader));

			bool gap_ahead_is_safe = (!has_destination_lane_leader()) 
				|| ((compute_gap(destination_lane_leader) + 0.01) 
					>= compute_safe_lane_change_gap(destination_lane_leader));
			
			/* besides the regular safety conditions, we add the case 
			where the dest lane follower has completely stopped to give room 
			to the lane changing vehicle */
			bool gap_behind_is_safe = (!has_destination_lane_follower())
				|| ((compute_gap(destination_lane_follower) + 0.01)
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

			if (gap_same_lane_is_safe 
				&& gap_ahead_is_safe && gap_behind_is_safe
				&& no_conflict) {
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

	//this->active_lane_change_direction.push_back(temp_result);
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
	if (assisted_vehicle != nullptr) {
		if (controller.get_active_longitudinal_controller()
			== ControlManager::ActiveACC::origin_lane) {
			double max_braking_distance =
				std::pow(get_velocity(), 2) / max_brake;
			return compute_gap(assisted_vehicle) > max_braking_distance;
		}
		return true;
	}
	return false;
	//return assisted_vehicle != nullptr;
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
	
	double current_max_brake = get_current_max_brake();
	double jerk_delay = (comfortable_acceleration + current_max_brake) / max_jerk;
	double ego_vel = get_velocity();

	double leader_max_brake = other_vehicle.get_max_brake();
	double relative_vel = other_vehicle.get_relative_velocity();
	double leader_vel = other_vehicle.compute_velocity(ego_vel);

	double gamma = leader_max_brake / current_max_brake;
	double gamma_threshold = leader_vel / (ego_vel + lambda_1);

	std::vector<double> gap_thresholds(4);
	gap_thresholds[0] = brake_delay
		* (brake_delay * (comfortable_acceleration + leader_max_brake) / 2
			+ relative_vel);
	gap_thresholds[1] = (brake_delay + jerk_delay)
		* (lambda_1 + relative_vel
			- (brake_delay + jerk_delay)
			* (current_max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[2] = leader_vel / leader_max_brake
		* (lambda_1 + relative_vel
			- leader_vel / leader_max_brake
			* (current_max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[3] = compute_exact_collision_free_gap(other_vehicle);

	double gap = compute_gap(other_vehicle);
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
		result += std::pow(relative_vel + lambda_1, 2)
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
		result = std::pow(relative_vel + lambda_1, 2)
			+ 2 * (leader_max_brake - current_max_brake)
			* (gap - lambda_0);
	}
	else if ((gamma >= gamma_threshold) && (gap < gap_thresholds[3])) {
		result = std::pow(ego_vel + lambda_1, 2)
			- 2 * current_max_brake
			* (std::pow(leader_vel, 2) / 2 / leader_max_brake
				+ gap - lambda_0);
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

void EgoVehicle::compute_safe_gap_parameters() {
	lambda_0 = compute_lambda_0(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
	lambda_1 = compute_lambda_1(max_jerk, comfortable_acceleration,
		max_brake, brake_delay);
	lambda_1_lane_change = compute_lambda_1(max_jerk, 
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
	
	/* Non-connected vehicles only have one value for lambda_1 and lambda_0.
	Connected vehicles have the "regular value" and a "non-connected" value
	when dealing with non-connected neighbors. */
	if (is_connected()) {
		lambda_0_connected = 
			compute_lambda_0(max_jerk, comfortable_acceleration,
				max_brake, CONNECTED_BRAKE_DELAY);
		lambda_1_connected =
			compute_lambda_1(max_jerk, comfortable_acceleration,
				max_brake, CONNECTED_BRAKE_DELAY);
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
		return "time";
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
