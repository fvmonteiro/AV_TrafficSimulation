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
#include "Vehicle.h"

Vehicle::Vehicle(long id, double simulation_time_step, 
	double creation_time, bool verbose) :
	verbose{ verbose },
	id{ id },
	simulation_time_step{ simulation_time_step },
	creation_time{ creation_time } {

	if (verbose) {
		std::clog << "Creating vehicle " << id 
			<< " at time " << creation_time
			<< " with simulation time step " << simulation_time_step
			<< std::endl;
	}

	/* Note: the controller is only created once the category is set
	because vehicle parameters depend on the category. */
}

Vehicle::Vehicle(long id, double simulation_time_step, double creation_time)
	: Vehicle(id, simulation_time_step, creation_time, false) {}

Vehicle::~Vehicle() {
	if (verbose) {
		std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::vissim_acceleration,
		Member::active_lane_change_direction,
		Member::vissim_active_lane_change_direction,
		};
		std::clog << create_header(members, true);
		std::clog << "Vehicle " << id 
			<< " out of the simulation" << std::endl;
	}
}

/* "Current" getters ------------------------------------------------------ */

double Vehicle::get_current_time() const {
	/* At creation time, the vehicle already has a velocity */
	return creation_time + (velocity.size() - 1) * simulation_time_step; 
}
long Vehicle::get_current_lane() const {
	return lane.back(); 
}
long Vehicle::get_current_preferred_relative_lane() const {
	return preferred_relative_lane.back();
}
double Vehicle::get_current_velocity() const {
	if (velocity.empty()) { /* We shouldn't need to check this condition. 
							This is used to avoid crashing during tests.*/
		return 0;
	}
	return velocity.back(); 
}
double Vehicle::get_current_acceleration() const {
	return acceleration.back(); 
}
double Vehicle::get_current_desired_acceleration() const {
	return desired_acceleration.back();
}
double Vehicle::get_current_vissim_acceleration() const {
	return vissim_acceleration.back();
}
long Vehicle::get_current_active_lane_change() const {
	return active_lane_change.back();
}
long Vehicle::get_current_vissim_active_lane_change() const {
	return vissim_active_lane_change.back();
}
double Vehicle::get_current_lane_end_distance() const {
	return lane_end_distance.back();
}
long Vehicle::get_current_leader_id() const {
	return leader_id.back();
}
Vehicle::State Vehicle::get_current_state() const {
	return state.back();
	//return controller.get_current_state();
}
/* ------------------------------------------------------------------------ */

/* Setters of vector type members ----------------------------------------- */
void Vehicle::set_lane(long lane) {
	this->lane.push_back(lane);
}
void Vehicle::set_preferred_relative_lane(long preferred_relative_lane) {
	this->preferred_relative_lane.push_back(preferred_relative_lane);
	update_state();
}
void Vehicle::set_velocity(double velocity) {
	this->velocity.push_back(velocity);
}
void Vehicle::set_acceleration(double acceleration) {
	this->acceleration.push_back(acceleration);
}
void Vehicle::set_vissim_acceleration(double vissim_acceleration) {
	this->vissim_acceleration.push_back(vissim_acceleration);
}
void Vehicle::set_vissim_active_lane_change(int active_lane_change) {
	this->vissim_active_lane_change.push_back(active_lane_change);
}
void Vehicle::set_lane_end_distance(double lane_end_distance,
	long lane_number) {
	if (lane_number == get_current_lane()) {
		this->lane_end_distance.push_back(lane_end_distance);
	}
}
/* ------------------------------------------------------------------------ */

/* Other particular getters and setters ----------------------------------- */

void Vehicle::set_category(VehicleCategory category) {
	/* We only need to set the category once, but VISSIM passes the 
	category every time step. */
	if (this->category == VehicleCategory::undefined) {
		if (category == VehicleCategory::truck) {
			max_brake = TRUCK_MAX_BRAKE;
		}
		else { // assume car if any other category
			max_brake = CAR_MAX_BRAKE;
		}
		this->controller = ControlManager(*this, verbose);
		//compute_safe_gap_parameters();
		this->category = category;
	}
}

void Vehicle::set_category(long category) {
	set_category(VehicleCategory(category));
};

/* ------------------------------------------------------------------------ */

//void Vehicle::compute_safe_gap_parameters() {
//	double jE = max_jerk;
//	double aE = comfortable_acceleration;
//	double bE = max_brake;
//	double tau_d = brake_delay;
//	double tau_j = (aE + bE) / jE;
//	lambda_0 = -(aE + bE)
//		* (std::pow(tau_d, 2) + tau_d * tau_j + std::pow(tau_j, 2) / 3);
//	lambda_1 = (aE + bE) * (tau_d + tau_j / 2);
//}

void Vehicle::clear_nearby_vehicles() {
	nearby_vehicles.clear();
}

void Vehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) {
	/* TODO: this function might be creating memory leaks.
	Should we delete the nearby_vehicles at some point? */
	NearbyVehicle* nearby_vehicle = new NearbyVehicle(id, relative_lane,
		relative_position);
	nearby_vehicles.push_back(nearby_vehicle);
}

NearbyVehicle* Vehicle::peek_nearby_vehicles() const {
	if (!nearby_vehicles.empty()) {
		return nearby_vehicles.back();
	}
	std::cerr << "Empty nearby_vehicles container in vehicle  " << this->id <<
		std::endl;
	return nullptr;
}

/*TODO: still deciding between possible implementations.
Ideally should profile execution speed and compare. 
But how? Could save the time at each execution and then average */
void Vehicle::analyze_nearby_vehicles() {
	/*leader_idx = -1;
	follower_idx = -1;
	dest_lane_leader_idx = -1;
	dest_lane_follower_idx = -1;*/

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
				//leader_idx = i;
				
				leader_found = true;
				/*if new leader, recompute time headway*/
				if ((relevant_nearby_vehicles.leader == nullptr) 
					|| (relevant_nearby_vehicles.leader->get_id()
						!= current_id)) {
					controller.update_origin_lane_time_headway(*this,
						*nearby_vehicle);
				}
				leader_id.push_back(nearby_vehicle->get_id());
				relevant_nearby_vehicles.leader = nearby_vehicle;
			}
			else if (relative_position == -1) {
				//follower_idx = i;
				
				follower_found = true;
				relevant_nearby_vehicles.follower = nearby_vehicle;
			}
		}
		else if (relative_lane == determine_desired_lane_change_direction()) {
			if (relative_position == 1) {
				/*dest_lane_leader_idx = i;
				long new_id = nearby_vehicle->get_id();
				if (new_id != dest_lane_leader_id) {
					controller.update_destination_lane_time_headway(*this,
						*nearby_vehicle);
					dest_lane_leader_id = new_id;
				}*/

				dest_lane_leader_found = true;
				if ((relevant_nearby_vehicles.destination_lane_leader
					== nullptr)
					|| (relevant_nearby_vehicles.destination_lane_leader->get_id()
						!= current_id)) {
					controller.update_destination_lane_time_headway(*this,
						*nearby_vehicle);
				}
				relevant_nearby_vehicles.destination_lane_leader =
					nearby_vehicle;
			}
			else if (relative_position == -1) {
			    /*dest_lane_follower_idx = i;
				long new_id = nearby_vehicle->get_id();
				if (new_id != dest_lane_follower_id) {
					nearby_vehicle->compute_safe_gap_parameters();
					controller.estimate_follower_time_headway(*this,
						*nearby_vehicle);
				}*/

				dest_lane_follower_found = true;
				if ((relevant_nearby_vehicles.destination_lane_follower
					== nullptr)
					|| (relevant_nearby_vehicles.destination_lane_follower->get_id()
						!= current_id)) {
					nearby_vehicle->compute_safe_gap_parameters();
					controller.estimate_follower_time_headway(*this,
						*nearby_vehicle);
				}
				relevant_nearby_vehicles.destination_lane_follower =
					nearby_vehicle;
			}
		}
	}

	if (!leader_found) {
		relevant_nearby_vehicles.leader = nullptr;
		leader_id.push_back(0);
	}
	if (!follower_found) {
		relevant_nearby_vehicles.follower = nullptr;
	}
	if (!dest_lane_leader_found) {
		relevant_nearby_vehicles.destination_lane_leader = nullptr;
	}
	if (!dest_lane_follower_found) {
		relevant_nearby_vehicles.destination_lane_follower = nullptr;
	}

	/* Keep track of the leading vehicle's id and recompute time headway
	if there's a new leader (could only when there's a new leader category)*/
	/*if (leader_idx > -1) {
		long current_leader_id = nearby_vehicles[leader_idx]->get_id();
		if ((leader_id.empty()) || (current_leader_id != leader_id.back())) {
			controller.update_origin_lane_time_headway(*this,
				*nearby_vehicles[leader_idx]);
		}
		leader_id.push_back(current_leader_id);
	}
	else {
		leader_id.push_back(0);
	}*/
}

double Vehicle::compute_gap(const NearbyVehicle& nearby_vehicle) const {
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

double Vehicle::compute_gap(const NearbyVehicle* nearby_vehicle) const {
	if (nearby_vehicle != nullptr) {
		return compute_gap(*nearby_vehicle);
	}
	else {
		return MAX_DISTANCE;
	}
}

RelativeLane Vehicle::get_lane_change_direction() {
	RelativeLane relative_lane;
	long preferred_relative_lane = get_current_preferred_relative_lane();
	if (preferred_relative_lane > 0) {
		relative_lane = RelativeLane::left;
	}
	else if (preferred_relative_lane < 0) {
		relative_lane = RelativeLane::right;
	}
	else {
		relative_lane = RelativeLane::same;
	}
	return relative_lane;
}

NearbyVehicle* Vehicle::get_leader() const {
	return relevant_nearby_vehicles.leader;
	/*if (leader_idx > -1) {
		return nearby_vehicles[leader_idx];
	}
	else {
		return nullptr;
	}*/
}

NearbyVehicle* Vehicle::get_follower() const {
	return relevant_nearby_vehicles.follower;
	/*if (follower_idx > -1) {
		return nearby_vehicles[follower_idx];
	}
	else {
		return nullptr;
	}*/
}

NearbyVehicle* Vehicle::get_destination_lane_leader() const {
	return relevant_nearby_vehicles.destination_lane_leader;
	/*if (dest_lane_leader_idx > -1) {
		return nearby_vehicles[dest_lane_leader_idx];
	}
	else {
		return nullptr;
	}*/

	/* [OLDEST IMPLEMENTATION] If there is no intention to change lanes, the
	function returns the leader on the same lane. */
	/*int relative_position = 1;
	NearbyVehicle* destination_lane_leader = find_nearby_vehicle(
			desired_lane_change_direction, relative_position);
	return destination_lane_leader;*/
}

NearbyVehicle* Vehicle::get_destination_lane_follower() const {
	return relevant_nearby_vehicles.destination_lane_follower;
	/*if (dest_lane_follower_idx > -1) {
		return nearby_vehicles[dest_lane_follower_idx];
	}
	else {
		return nullptr;
	}*/
}

bool Vehicle::has_lane_change_intention() const {
	return get_current_preferred_relative_lane() != 0;
}

RelativeLane Vehicle::determine_desired_lane_change_direction() {
	long current_preferred_relative_lane = 
		get_current_preferred_relative_lane();
	if (current_preferred_relative_lane > 0) {
		return RelativeLane::left;
	}
	else if (current_preferred_relative_lane < 0) {
		return RelativeLane::right;
	}
	else {
		std::clog << "Warning"
			<< "\tt=" << get_current_time()
			<< ", id=" << id
			<< " : trying to determine lane change direction "
			<< "when there is no lane change intention."
			<< std::endl;
		return RelativeLane::same;
	}
}

Vehicle::State Vehicle::get_previous_state() const {
	if (state.size() > 2) return state[state.size() - 2];
	else return State::lane_keeping;
}

long Vehicle::get_color_by_controller_state() {

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

double Vehicle::compute_desired_acceleration() {
	double desired_acceleration = 
		controller.determine_desired_acceleration(*this);
	this->desired_acceleration.push_back(desired_acceleration);
	double feasible_acceleration = consider_vehicle_dynamics(
		desired_acceleration);
	return desired_acceleration;
}

double Vehicle::consider_vehicle_dynamics(double desired_acceleration) {
	/* We assume lower level dynamics as: 
	tau.da/dt + a = u 
	tau.(a(k+1)-a(k))/delta + a(k) = u(k)
	a(k+1) = a(k) + delta.(u(k) - a(k)) / tau*/
	double current_acceleration = get_current_acceleration();
	return current_acceleration + simulation_time_step / tau
		* (desired_acceleration - current_acceleration);
}

double Vehicle::compute_safe_lane_change_gap(NearbyVehicle* other_vehicle) {
	double gap = 0.0;
	if (other_vehicle != nullptr) {
		gap = controller.compute_safe_lane_change_gap(*this,
			*other_vehicle);
	}
	return gap;
}

double Vehicle::compute_safe_gap_to_destination_lane_leader() {
	double gap = 0.0;
	if (has_lane_change_intention()) {
		NearbyVehicle* destination_lane_leader = 
			get_destination_lane_leader();
		if (destination_lane_leader != nullptr) {
			gap = controller.compute_safe_lane_change_gap(*this, 
				*destination_lane_leader);
		}
	}
	return gap;
}

double Vehicle::compute_safe_gap_to_destination_lane_follower() {
	double gap = 0.0;
	if (has_lane_change_intention()) {
		NearbyVehicle* destination_lane_follower =
			get_destination_lane_follower();
		if (destination_lane_follower != nullptr) {
			gap = controller.compute_safe_lane_change_gap(*this,
				*destination_lane_follower);
		}
	}
	return gap;
}

double Vehicle::get_reference_gap() {
	return controller.get_reference_gap(velocity.back());
}

double Vehicle::compute_time_headway_gap(
	NearbyVehicle* other_vehicle) {

	double time_headway_gap = 0.0;
	if (other_vehicle != nullptr) {
		time_headway_gap = controller.compute_time_headway_gap(
			*this, *other_vehicle);
	}
	return time_headway_gap;
}

double Vehicle::compute_transient_gap(NearbyVehicle* other_vehicle) {
	double transient_gap = 0.0;
	if (other_vehicle != nullptr) {
		transient_gap = controller.get_lateral_controller().
			compute_transient_gap(*this, *other_vehicle, false);
	}
	return transient_gap;
}

//double Vehicle::compute_time_headway_gap_to_destination_lane_follower() {
//	double vehicle_following_gap = 0.0;
//	if (desired_lane_change_direction != RelativeLane::same) {
//		NearbyVehicle* destination_lane_follower =
//			get_destination_lane_follower();
//		if (destination_lane_follower != nullptr) {
//			vehicle_following_gap = controller.
//				get_destination_lane_controller().compute_time_headway_gap(
//					destination_lane_follower->compute_velocity(velocity.back()));
//		}
//	}
//	return vehicle_following_gap;
//}

//double Vehicle::compute_transient_gap_to_destination_lane_follower() {
//	double transient_gap = 0.0;
//	if (desired_lane_change_direction != RelativeLane::same) {
//		NearbyVehicle* destination_lane_follower =
//			get_destination_lane_follower();
//		if (destination_lane_follower != nullptr) {
//			transient_gap = controller.get_lateral_controller().
//				compute_transient_gap(*this, *destination_lane_follower, false);
//		}
//	}
//	return transient_gap;
//}

long Vehicle::decide_active_lane_change_direction() {
	long active_lane_change_direction = 0;
	if (use_internal_lane_change_decision) {
		//long current_preferred_lane = get_current_preferred_relative_lane();
		if (has_lane_change_intention()) {
			NearbyVehicle* dest_lane_leader = get_destination_lane_leader();
			NearbyVehicle* dest_lane_follower = get_destination_lane_follower();
			if ((compute_gap(dest_lane_leader) 
				 > compute_safe_lane_change_gap(dest_lane_leader)) 
				&& 
				(compute_gap(dest_lane_follower) 
				 > compute_safe_lane_change_gap(dest_lane_follower))
				&& 
				(!find_lane_change_conflicts())) {

				active_lane_change_direction = static_cast<int>(
					determine_desired_lane_change_direction());
			}
		}
	}
	else {
		active_lane_change_direction = get_current_vissim_active_lane_change();
	}

	this->active_lane_change.push_back(active_lane_change_direction);
	return active_lane_change_direction;
}

bool Vehicle::find_lane_change_conflicts() {

	/* If there's no lane change intention, there's no conflict */
	if (has_lane_change_intention()) {
		return false;
	}

	NearbyVehicle* leader = get_leader();
	NearbyVehicle* follower = get_follower();
	NearbyVehicle* dest_lane_leader = get_destination_lane_leader();
	NearbyVehicle* dest_lane_follower = get_destination_lane_follower();
	RelativeLane lane_change_direction = determine_desired_lane_change_direction();
	RelativeLane opposite_direction;
	if (lane_change_direction == RelativeLane::right) {
		opposite_direction = RelativeLane::left;
	}
	else {
		opposite_direction = RelativeLane::right;
	}

	if ((leader != nullptr) && (leader->get_lane_change_direction()
		== lane_change_direction)) return true;
	if ((follower != nullptr) && (follower->get_lane_change_direction()
		== lane_change_direction)) return true;
	if ((dest_lane_leader != nullptr) 
		&& (dest_lane_leader->get_lane_change_direction() 
			== opposite_direction)) return true;
	if ((dest_lane_follower!= nullptr)
		&& (dest_lane_follower->get_lane_change_direction()
			== opposite_direction)) return true;

	return false;
}

NearbyVehicle* Vehicle::find_nearby_vehicle(
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

/* Private methods -------------------------------------------------------- */

void Vehicle::update_state() {
	if (get_current_preferred_relative_lane() == 0) {
		state.push_back(State::lane_keeping);
	}
	else {
		state.push_back(State::intention_to_change_lanes);
	}
}

/* Methods for printing and debugging ------------------------------------- */

void Vehicle::write_vehicle_log() {
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		//Member::desired_acceleration,
		//Member::vissim_acceleration,
		Member::active_lane_change_direction,
		Member::vissim_active_lane_change_direction,
		Member::lane,
		Member::lane_end_distance,
	};
	bool write_size = true;
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
	vehicle_log.open(log_path + "\\" + file_name);
	vehicle_log << write_members(members, write_size).str();
	vehicle_log.close();
}

std::string Vehicle::create_header(
	std::vector<Vehicle::Member> members, bool write_size) {
	
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

std::ostringstream Vehicle::write_members(
	std::vector<Vehicle::Member> members, bool write_size) {

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

int Vehicle::get_member_size(Member member) {
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
		return (int) lane.size();
	case Member::preferred_relative_lane:
		return (int) preferred_relative_lane.size();
	case Member::velocity:
		return (int) velocity.size();
	case Member::acceleration:
		return (int) acceleration.size();
	case Member::desired_acceleration:
		return (int) desired_acceleration.size();
	case Member::vissim_acceleration:
		return (int) vissim_acceleration.size();
	case Member::leader_id:
		return (int) leader_id.size();
	case Member::state:
		return (int) state.size();
	case Member::active_lane_change_direction:
		return (int) active_lane_change.size();
	case Member::vissim_active_lane_change_direction:
		return (int) vissim_active_lane_change.size();
	case Member::lane_end_distance:
		return (int) lane_end_distance.size();
	default:
		return 0;
	}
}

std::string Vehicle::member_enum_to_string(Member member) {
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
	default:
		return "unknown memeber";
	}
}

std::string Vehicle::state_to_string(State vehicle_state) {

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

std::ostream& operator<< (std::ostream& out, const Vehicle& vehicle)
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
	/*for (NearbyVehicle* nearby_vehicle : vehicle.nearby_vehicles) {
		out << "\tNearby vehicle: " << *nearby_vehicle << std::endl;
	}*/
	/*NearbyVehicle* leader;
	if (vehicle.find_leader(leader)) {
		out << "\tLeading vehicle: " << *leader;
	}*/

	return out; // return std::ostream so we can chain calls to operator<<
}
