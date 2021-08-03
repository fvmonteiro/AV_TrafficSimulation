/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

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

	if (verbose) {
		std::clog << "Creating vehicle " << id 
			<< " at time " << creation_time
			<< " with simulation time step " << simulation_time_step
			<< std::endl;
	}
	/* Note: the controller is only created once the category is set
	because vehicle parameters depend on the category. */
}

EgoVehicle::EgoVehicle(long id, double simulation_time_step, double creation_time)
	: EgoVehicle(id, simulation_time_step, creation_time, false) {}

EgoVehicle::~EgoVehicle() {
	if (verbose) {
		std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::active_lane_change_direction,
		Member::leader_id,
		Member::ttc,
		Member::drac,
		Member::collision_severity_risk
		};
		std::clog << create_header(members, true);
		std::clog << "Vehicle " << id 
			<< " out of the simulation" << std::endl;
	}
	if (verbose || should_log) write_simulation_log();
}

/* "Current" getters ------------------------------------------------------ */

double EgoVehicle::get_current_time() const {
	/* At creation time, the vehicle already has a velocity */
	return creation_time + (velocity.size() - 1) * simulation_time_step; 
}
long EgoVehicle::get_current_lane() const {
	return lane.back(); 
}
long EgoVehicle::get_current_link() const {
	return link.back();
}
long EgoVehicle::get_current_preferred_relative_lane() const {
	return preferred_relative_lane.back();
}
double EgoVehicle::get_current_velocity() const {
	if (velocity.empty()) { /* We shouldn't need to check this condition. 
							This is used to avoid crashing during tests.*/
		return 0;
	}
	return velocity.back(); 
}
double EgoVehicle::get_current_acceleration() const {
	return acceleration.back(); 
}
double EgoVehicle::get_current_desired_acceleration() const {
	return desired_acceleration.back();
}
double EgoVehicle::get_current_vissim_acceleration() const {
	return vissim_acceleration.back();
}
long EgoVehicle::get_current_active_lane_change() const {
	return active_lane_change.back();
}
long EgoVehicle::get_current_vissim_active_lane_change() const {
	return vissim_active_lane_change.back();
}
double EgoVehicle::get_current_lane_end_distance() const {
	return lane_end_distance.back();
}
long EgoVehicle::get_current_leader_id() const {
	return leader_id.back();
}
EgoVehicle::State EgoVehicle::get_current_state() const {
	return state.back();
}
double EgoVehicle::get_current_ttc() const {
	return ttc.back();
}
double EgoVehicle::get_current_drac() const {
	return drac.back();
}
double EgoVehicle::get_current_collision_risk() const {
	return collision_severity_risk.back();
}
/* ------------------------------------------------------------------------ */

/* Other setters ----------------------------------------- */
void EgoVehicle::set_lane(long lane) {
	this->lane.push_back(lane);
}
void EgoVehicle::set_link(long link) {
	this->link.push_back(link);
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
void EgoVehicle::set_vissim_active_lane_change(int active_lane_change) {
	this->vissim_active_lane_change.push_back(active_lane_change);
}

void EgoVehicle::set_category(VehicleCategory category) {
	/* We only need to set the category once, but VISSIM passes the
	category every time step. */
	if (this->category == VehicleCategory::undefined) {
		if (category == VehicleCategory::truck) {
			max_brake = TRUCK_MAX_BRAKE;
		}
		else { // assume car if any other category
			max_brake = CAR_MAX_BRAKE;
		}
		lane_change_max_brake = max_brake / 2;
		compute_safe_gap_parameters();
		this->controller = ControlManager(*this, verbose);
		//compute_safe_gap_parameters();
		this->category = category;
	}
}

void EgoVehicle::set_type(VehicleType type) {
	// we only need to set the type once
	switch (this->type) {
	case VehicleType::undefined:
		this->type = type;
		break;
	default: // no changes
		break;
	}
}

void EgoVehicle::set_type(long type) { 
	// we only need to set the type once
	switch (this->type) {
	case VehicleType::undefined:
		set_type(VehicleType(type));
		break;
	default: // no changes
		break;
	}
}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane) {
	this->preferred_relative_lane.push_back(preferred_relative_lane);
	set_desired_lane_change_direction(preferred_relative_lane);
	set_state(preferred_relative_lane);
	/* Create the destination lane controller when the
	vehicle first shows its intention to change lanes. */
	if (get_previous_state() != get_current_state()) {
		controller.start_longitudinal_adjustment(
			get_current_time(),
			get_current_velocity() * adjustment_speed_factor);
	}
}

void EgoVehicle::set_lane_end_distance(double lane_end_distance,
	long lane_number) {
	if (lane_number == get_current_lane()) {
		this->lane_end_distance.push_back(lane_end_distance);
	}
}

/* ------------------------------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles() {
	nearby_vehicles.clear();
}

void EgoVehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) {
	/* TODO: this function might be creating memory leaks.
	Should we delete the nearby_vehicles at some point? */
	NearbyVehicle* nearby_vehicle = new NearbyVehicle(id, relative_lane,
		relative_position);
	nearby_vehicles.push_back(nearby_vehicle);
}

NearbyVehicle* EgoVehicle::peek_nearby_vehicles() const {
	if (!nearby_vehicles.empty()) {
		return nearby_vehicles.back();
	}
	std::cerr << "Empty nearby_vehicles container in vehicle  " << this->id <<
		std::endl;
	return nullptr;
}

double EgoVehicle::get_relative_velocity_to_leader() {
	if (has_leader()) return leader->get_relative_velocity();
	else return 0.0;
}

long EgoVehicle::get_leader_category() {
	if (has_leader()) return static_cast<long>(leader->get_category());
	else return 0;
}

void EgoVehicle::analyze_nearby_vehicles() {
	bool leader_found = false;
	bool follower_found = false;
	bool dest_lane_leader_found = false;
	bool dest_lane_follower_found = false;

	for (int i = 0; i < nearby_vehicles.size(); i++) {
		NearbyVehicle* nearby_vehicle = nearby_vehicles[i];
		long current_id = nearby_vehicle->get_id();
		RelativeLane relative_lane = 
			nearby_vehicle->get_relative_lane();
		long relative_position =
			nearby_vehicle->get_relative_position();

		if (relative_lane == RelativeLane::same) {
			if (relative_position == 1) {				
				leader_found = true;
				/*if new leader, recompute time headway*/
				if ((!has_leader()) || (leader->get_id() != current_id)) {
					controller.update_origin_lane_time_headway(
						lambda_1, nearby_vehicle->get_max_brake());
				}
				leader_id.push_back(nearby_vehicle->get_id());
				leader = nearby_vehicle;
			}
			else if (relative_position == -1) {				
				follower_found = true;
				follower = nearby_vehicle;
			}
		}
		else if (relative_lane == desired_lane_change_direction) {
			if (relative_position == 1) {
				dest_lane_leader_found = true;
				if ((!has_destination_lane_leader())
					|| (destination_lane_leader->get_id()
						!= current_id)) {
					controller.update_destination_lane_time_headway(
						lane_change_lambda_1, nearby_vehicle->get_max_brake());
				}
				destination_lane_leader = nearby_vehicle;
			}
			else if (relative_position == -1) {
				dest_lane_follower_found = true;

				if ((!has_destination_lane_follower())
					|| (destination_lane_follower->get_id()
						!= current_id)) {
					controller.estimate_follower_time_headway(*this,
						*nearby_vehicle);
				}
				destination_lane_follower = nearby_vehicle;
			}
		}
	}

	/* We must ensure to clear the pointers in case the nearby
	vehicle was not found in this time step. */
	if (!leader_found) {
		leader = nullptr;
		leader_id.push_back(0);
	}
	if (!follower_found) {
		follower = nullptr;
	}
	if (!dest_lane_leader_found) {
		destination_lane_leader = nullptr;
	}
	if (!dest_lane_follower_found) {
		destination_lane_follower = nullptr;
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

double EgoVehicle::compute_gap(const NearbyVehicle* nearby_vehicle) const {
	if (nearby_vehicle != nullptr) {
		return compute_gap(*nearby_vehicle);
	}
	else {
		return MAX_DISTANCE;
	}
}

//RelativeLane Vehicle::get_lane_change_direction() {
//	RelativeLane relative_lane;
//	long preferred_relative_lane = get_current_preferred_relative_lane();
//	if (preferred_relative_lane > 0) {
//		relative_lane = RelativeLane::left;
//	}
//	else if (preferred_relative_lane < 0) {
//		relative_lane = RelativeLane::right;
//	}
//	else {
//		relative_lane = RelativeLane::same;
//	}
//	return relative_lane;
//}

NearbyVehicle* EgoVehicle::get_leader() const {
	return leader;
}

NearbyVehicle* EgoVehicle::get_follower() const {
	return follower;
}

NearbyVehicle* EgoVehicle::get_destination_lane_leader() const {
	return destination_lane_leader;

	/* [OLDEST IMPLEMENTATION] If there is no intention to change lanes, the
	function returns the leader on the same lane. */
	/*int relative_position = 1;
	NearbyVehicle* destination_lane_leader = find_nearby_vehicle(
			desired_lane_change_direction, relative_position);
	return destination_lane_leader;*/
}

NearbyVehicle* EgoVehicle::get_destination_lane_follower() const {
	return destination_lane_follower;
}

bool EgoVehicle::has_lane_change_intention() const {
	return get_current_preferred_relative_lane() != 0;
}

EgoVehicle::State EgoVehicle::get_previous_state() const {
	if (state.size() > 2) return state[state.size() - 2];
	else return State::lane_keeping;
}

long EgoVehicle::get_color_by_controller_state() {

	if (state.empty()) {
		return velocity_control_color;
	}

	switch (get_current_state())
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
	}
}

std::string EgoVehicle::print_detailed_state() {
	std::string state_str = 
		state_to_string(get_current_state()) + ", "
		+ ControlManager::active_longitudinal_controller_to_string(
			controller.get_active_longitudinal_controller()) + ", "
		+ LongitudinalController::state_to_string(
			controller.get_longitudinal_controller_state());
	return state_str;
}

double EgoVehicle::compute_desired_acceleration() {
	if (verbose) std::clog << "t=" << get_current_time()
		<< ", computing desired accel" << std::endl;
	double desired_acceleration = 
		controller.determine_desired_acceleration(*this);
	this->desired_acceleration.push_back(desired_acceleration);
	double feasible_acceleration = consider_vehicle_dynamics(
		desired_acceleration);
	return desired_acceleration;
}

double EgoVehicle::consider_vehicle_dynamics(double desired_acceleration) {
	/* We assume lower level dynamics as: 
	tau.da/dt + a = u 
	tau.(a(k+1)-a(k))/delta + a(k) = u(k)
	a(k+1) = a(k) + delta.(u(k) - a(k)) / tau*/
	double current_acceleration = get_current_acceleration();
	return current_acceleration + simulation_time_step / tau
		* (desired_acceleration - current_acceleration);
}

double EgoVehicle::compute_safe_lane_change_gap(NearbyVehicle* other_vehicle) {
	double safe_gap = 0.0;
	if (other_vehicle != nullptr) {
		safe_gap = controller.compute_safe_lane_change_gap(*this,
			*other_vehicle);
		// TODO: properly code a min_safe_gap
		if (safe_gap < 1.0) safe_gap = 1.0;
	}
	return safe_gap;
}

//double EgoVehicle::compute_safe_gap_to_destination_lane_leader() {
//	double gap = 0.0;
//	if (has_lane_change_intention()) {
//		if (has_destination_lane_leader()) {
//			gap = controller.compute_safe_lane_change_gap(*this, 
//				*destination_lane_leader);
//		}
//	}
//	return gap;
//}

//double EgoVehicle::compute_safe_gap_to_destination_lane_follower() {
//	double gap = 0.0;
//	if (has_lane_change_intention()) {
//		if (has_destination_lane_follower()) {
//			gap = controller.compute_safe_lane_change_gap(*this,
//				*destination_lane_follower);
//		}
//	}
//	return gap;
//}

double EgoVehicle::get_reference_gap() {
	return controller.get_reference_gap(velocity.back());
}

double EgoVehicle::compute_time_headway_gap(
	NearbyVehicle* other_vehicle) {

	double time_headway_gap = 0.0;
	if (other_vehicle != nullptr) {
		time_headway_gap = controller.compute_time_headway_gap(
			*this, *other_vehicle);
	}
	return time_headway_gap;
}

double EgoVehicle::compute_transient_gap(NearbyVehicle* other_vehicle) {
	double transient_gap = 0.0;
	if (other_vehicle != nullptr) {
		transient_gap = controller.get_lateral_controller().
			compute_transient_gap(*this, *other_vehicle, false);
	}
	return transient_gap;
}

long EgoVehicle::decide_active_lane_change_direction() {
	
	//if (verbose) std::clog << "deciding lane change" << std::endl;
	
	long active_lane_change_direction = 0;
	if (use_internal_lane_change_decision) {
		if (has_lane_change_intention()) {
			bool gap_ahead_is_safe = (!has_destination_lane_leader()) 
				|| (compute_gap(destination_lane_leader) 
					> compute_safe_lane_change_gap(destination_lane_leader));
			
			//if (verbose) std::clog << "gap ahead checked" << std::endl;

			/* besides the regular safety conditions, we add the case 
			where the dest lane follower has completely stopped to give room 
			to the lane changing vehicle */
			bool gap_behind_is_safe = (!has_destination_lane_follower())
				|| (compute_gap(destination_lane_follower)
					> compute_safe_lane_change_gap(destination_lane_follower))
				|| ((destination_lane_follower->
					compute_velocity(get_current_velocity()) <= 0.5)
					&& (destination_lane_follower->get_distance() <= -1.0));
			
			//if (verbose) std::clog << "gap behind checked" << std::endl;

			if (gap_ahead_is_safe && gap_behind_is_safe
				&& !find_lane_change_conflicts()) {
				// will start lane change
				active_lane_change_direction = static_cast<int>(
					desired_lane_change_direction);
			}
			else {
				controller.update_accepted_risk(*this);
			}
		}

		//if (verbose) std::clog << "LC decided" << std::endl;

	}
	else {
		active_lane_change_direction = get_current_vissim_active_lane_change();
	}


	this->active_lane_change.push_back(active_lane_change_direction);
	return active_lane_change_direction;
}

bool EgoVehicle::find_lane_change_conflicts() {

	//if (verbose) std::clog << "checking conflicts" << std::endl;

	/* If there's no lane change intention, there's no conflict */
	if (!has_lane_change_intention()) {
		return false;
	}

	RelativeLane opposite_direction;
	if (desired_lane_change_direction == RelativeLane::right) {
		opposite_direction = RelativeLane::left;
	}
	else {
		opposite_direction = RelativeLane::right;
	}

	if ((has_leader()) && (leader->get_lane_change_direction()
		== desired_lane_change_direction)) return true;
	if ((has_follower()) && (follower->get_lane_change_direction()
		== desired_lane_change_direction)) return true;
	if ((has_destination_lane_leader()) 
		&& (destination_lane_leader->get_lane_change_direction() 
			== opposite_direction)) return true;
	if ((has_destination_lane_follower())
		&& (destination_lane_follower->get_lane_change_direction()
			== opposite_direction)) return true;

	//if (verbose) std::clog << "conflicts checked" << std::endl;

	return false;
}

NearbyVehicle* EgoVehicle::find_nearby_vehicle(
	RelativeLane relative_lane, int relative_position) const {

	for (NearbyVehicle* nearby_vehicle : nearby_vehicles) {
		if ((nearby_vehicle->get_relative_lane() 
			 == relative_lane)
			& 
			(nearby_vehicle->get_relative_position() 
			 == relative_position)) {
			return nearby_vehicle;
		}
	}
	return nullptr;
}


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
	double ego_velocity, const NearbyVehicle& other_vehicle) {

	double follower_lambda_0, follower_lambda_1;
	double v_follower, v_leader;
	double brake_follower, brake_leader;
	double delta_v = other_vehicle.get_relative_velocity();
	if (other_vehicle.is_ahead()) {
		follower_lambda_0 = lambda_0;
		follower_lambda_1 = lambda_1;
		v_follower = ego_velocity;
		v_leader = v_follower - delta_v;
		brake_follower = max_brake;
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
	
	double jerk_delay = (comfortable_acceleration + max_brake) / max_jerk;
	double ego_vel = get_current_velocity();

	double leader_max_brake = other_vehicle.get_max_brake();
	double relative_vel = other_vehicle.get_relative_velocity();
	double leader_vel = other_vehicle.compute_velocity(ego_vel);

	double gamma = leader_max_brake / max_brake;
	double gamma_threshold = leader_vel / (ego_vel + lambda_1);

	std::vector<double> gap_thresholds(4);
	gap_thresholds[0] = brake_delay
		* (brake_delay * (comfortable_acceleration + leader_max_brake) / 2
			+ relative_vel);
	gap_thresholds[1] = (brake_delay + jerk_delay)
		* (lambda_1 + relative_vel
			- (brake_delay + jerk_delay)
			* (max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[2] = leader_vel / leader_max_brake
		* (lambda_1 + relative_vel
			- leader_vel / leader_max_brake
			* (max_brake - leader_max_brake) / 2)
		+ lambda_0;
	gap_thresholds[3] = compute_exact_collision_free_gap(ego_vel, other_vehicle);

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
			+ 2 * (leader_max_brake - max_brake)
			* (gap - lambda_0);
		result /= 2;
		std::clog << "t=" << get_current_time()
			<< ", id=" << id
			<< ", collision severity complicated case"
			<< std::endl;
	}
	else if (((gamma >= gamma_threshold) && (gap < gap_thresholds[2]))
		|| (gamma < gamma_threshold) && (gap < gap_thresholds[3])) {
		result = std::pow(relative_vel + lambda_1, 2)
			+ 2 * (leader_max_brake - max_brake)
			* (gap - lambda_0);
	}
	else if ((gamma >= gamma_threshold) && (gap < gap_thresholds[3])) {
		result = std::pow(ego_vel + lambda_1, 2)
			- 2 * max_brake
			* (std::pow(leader_vel, 2) / 2 / leader_max_brake
				+ gap - lambda_0);
	}

	return result;
}

/* Private methods -------------------------------------------------------- */

void EgoVehicle::set_state(long preferred_relative_lane) {
	if (preferred_relative_lane == 0) {
		state.push_back(State::lane_keeping);
	}
	else {
		state.push_back(State::intention_to_change_lanes);
	}
}

void EgoVehicle::set_desired_lane_change_direction(
	long preferred_relative_lane) {

	if (preferred_relative_lane > 0) {
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (preferred_relative_lane < 0) {
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
	lane_change_lambda_1 = compute_lambda_1(max_jerk, 
		comfortable_acceleration, lane_change_max_brake, brake_delay);
}

/* Methods for printing and debugging ------------------------------------- */

void EgoVehicle::write_simulation_log() {
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::link,
		Member::lane,
		Member::active_lane_change_direction,
		Member::ttc,
		Member::drac,
		Member::collision_severity_risk
	};
	bool write_size = true;
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
	vehicle_log.open(log_path + "\\" + file_name);
	if (vehicle_log.is_open()) {
		vehicle_log << write_members(members, write_size).str();
		vehicle_log.close();
	}
	else {
		std::clog << "Unable to open file to write log of "
			<< "vehicle " << id
			<< std::endl;
	}
}

std::string EgoVehicle::create_header(
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

std::ostringstream EgoVehicle::write_members(
	std::vector<EgoVehicle::Member> members, bool write_size) {

	std::ostringstream oss;
	
	oss << create_header(members, write_size);

	// Write variables over time
	for (int i = 0; i < velocity.size(); i++) {
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
				oss << preferred_relative_lane[i];
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
				oss << active_lane_change[i];
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
			default:
				oss << "";
				break;
			}
			oss << ", ";
		}
		oss << std::endl;
	}
	return oss;
}

int EgoVehicle::get_member_size(Member member) {
	switch (member)
	{
	case Member::creation_time:
		return 1;
	case Member::id:
		return 1;
	case Member::length:
		return 1;
	case Member::width:
		return 1;
	case Member::color:
		return 1;
	case Member::category:
		return 1;
	case Member::desired_velocity:
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
		return (int)active_lane_change.size();
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
		{"lane", vehicle.get_current_lane()} 
	};
	std::vector<std::pair<std::string, double>> double_members{
		{ "velocity", vehicle.get_current_velocity()},
		{ "des. velocity", vehicle.get_desired_velocity()},
		{"acceleration", vehicle.get_current_acceleration()},
		{"des. acceleration", vehicle.get_current_desired_acceleration()},
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
