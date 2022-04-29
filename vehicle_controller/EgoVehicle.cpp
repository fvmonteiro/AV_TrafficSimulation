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

EgoVehicle::EgoVehicle(long id, VehicleType type, double desired_velocity,
	double brake_delay, bool is_lane_change_autonomous, bool is_connected,
	double simulation_time_step, double creation_time, bool verbose) :
	Vehicle(id, type, brake_delay),
	desired_velocity{ desired_velocity },
	is_lane_change_autonomous { is_lane_change_autonomous },
	is_connected { is_connected },
	simulation_time_step{ simulation_time_step },
	creation_time{ creation_time },
	verbose{ verbose },
	tau_d{ std::exp(-simulation_time_step / tau) }
{
	if (verbose) 
	{
		std::clog << "Creating vehicle " << get_id()
			<< " at time " << this->creation_time
			<< " with simulation time step " << this->simulation_time_step
			<< ", category " << static_cast<int>(category)
			<< ", type " << static_cast<int>(get_type())
			<< std::endl;
	}

	compute_safe_gap_parameters();
	this->controller = ControlManager(get_static_parameters(), verbose);
	/* The end of the lane is seen as a stopped vehicle. We can pretend
	this stopped vehicle has a lower max brake so that the time headway
	will be small. */
	controller.activate_end_of_lane_controller(
		compute_vehicle_following_desired_time_headway(max_brake / 2, false));
}

EgoVehicle::~EgoVehicle() {
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::active_lane_change_direction,
		Member::leader_id,
	};
	if (verbose) {
		std::clog << write_header(members, true);
		std::clog << "Vehicle " << get_id()
			<< " out of the simulation at time "
			<< get_time() << std::endl;

		//write_simulation_log(members);
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
//long EgoVehicle::get_vissim_active_lane_change() const {
//	return vissim_active_lane_change.back();
//}
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
double EgoVehicle::get_current_max_brake() const 
{
	return has_lane_change_intention() ? 
		get_lane_change_max_brake() : max_brake;
	/*if (is_lane_changing()) return get_lane_change_max_brake();
	return max_brake;*/
}

double EgoVehicle::get_current_lambda_1(
	bool is_other_connected) const
{
	return lambda_1;
}

VehicleParameters EgoVehicle::get_static_parameters() const 
{
	return {
		get_type(),
		simulation_time_step,
		max_brake,
		comfortable_brake,
		comfortable_acceleration,
		desired_velocity,
		lambda_1,
		is_connected,
		get_lane_change_max_brake(),
		/*lambda_1_lane_change,
		lambda_1_connected,
		lambda_1_lane_change_connected,*/
	};
}

double EgoVehicle::get_free_flow_velocity() const {
	return try_go_at_max_vel ? MAX_VELOCITY: desired_velocity;
}

double EgoVehicle::get_safe_time_headway() const {
	return controller.get_origin_lane_controller().
		get_safe_time_headway();
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
//void EgoVehicle::set_vissim_active_lane_change(int active_lane_change) {
//	this->vissim_active_lane_change.push_back(active_lane_change);
//}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane) {
	this->preferred_relative_lane.push_back(
		RelativeLane::from_long(preferred_relative_lane));
	//set_desired_lane_change_direction(preferred_relative_lane);	
}

void EgoVehicle::set_relative_target_lane(long target_relative_lane) {
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

/* Nearby Vehicles methods ------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles() 
{
	nearby_vehicles.clear();
	//leader = nullptr;
	//clear_other_relevant_nearby_vehicles();
}

//void EgoVehicle::update_nearby_vehicles() 
//{
//	update_leader();
//	update_other_relevant_nearby_vehicles();
//}

void EgoVehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) 
{
	/*if (verbose && get_time() > 68) std::clog << "Emplacing nv id=" << id
		<< std::endl;*/
	std::shared_ptr<NearbyVehicle> nearby_vehicle = 
		std::shared_ptr<NearbyVehicle>(new NearbyVehicle(id, relative_lane,
		relative_position));
	nearby_vehicles.push_back(std::move(nearby_vehicle));
}

std::shared_ptr<NearbyVehicle> EgoVehicle::peek_nearby_vehicles() const {
	if (!nearby_vehicles.empty()) {
		return nearby_vehicles.back();
	}
	std::clog << "Empty nearby_vehicles container in vehicle  " << get_id()
		<< std::endl;
	return nullptr;
}

void EgoVehicle::set_nearby_vehicle_type(long nv_type) 
{
	try_to_set_nearby_vehicle_type(nv_type);
	/*if (is_connected) 
	{
		peek_nearby_vehicles()->set_type(nv_type);
	}*/
}

//long EgoVehicle::try_to_get_nearby_vehicle_type(long nv_type) const
//{
//	return static_cast<int>(VehicleType::undefined);
//}

bool EgoVehicle::has_leader() const 
{
	return leader != nullptr;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_leader() const 
{
	return leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_nearby_vehicle_by_id(
	long nv_id) const 
{
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

double EgoVehicle::get_relative_velocity_to_leader() 
{
	return has_leader() ? leader->get_relative_velocity() : 0.0;
}

double EgoVehicle::compute_gap(const NearbyVehicle& nearby_vehicle) const 
{
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
		return -nearby_vehicle.get_distance() - get_length();
	}
}

double EgoVehicle::compute_gap(
	const std::shared_ptr<NearbyVehicle> nearby_vehicle) const 
{
	if (nearby_vehicle != nullptr) {
		return compute_gap(*nearby_vehicle);
	}
	else {
		return MAX_DISTANCE;
	}
}

long EgoVehicle::get_lane_change_request() 
{
	return create_lane_change_request();
}

void EgoVehicle::find_relevant_nearby_vehicles()
{
	find_leader();
}

void EgoVehicle::find_leader()
{
	std::shared_ptr<NearbyVehicle> old_leader = std::move(leader);
	
	for (auto& nearby_vehicle : nearby_vehicles)
	{
		if (check_if_is_leader(*nearby_vehicle)) leader = nearby_vehicle;
	}
	update_leader(old_leader);

	/*if (verbose)
	{
		if (has_leader()) std::clog << "Leader id=" << leader->get_id();
		else std::clog << "No leader";
		std::clog << std::endl;
	}*/
}

bool EgoVehicle::check_if_is_leader(const NearbyVehicle& nearby_vehicle) const
{
	if ((nearby_vehicle.is_immediatly_ahead()
		&& nearby_vehicle.is_on_same_lane())
		|| nearby_vehicle.is_cutting_in())
	{
		if (!has_leader()
			|| (nearby_vehicle.get_distance() < leader->get_distance()))
		{
			//leader = nearby_vehicle;
			return true;
		}
	}
	return false;
}

void EgoVehicle::update_leader(
	const std::shared_ptr<NearbyVehicle>& old_leader)
{
	//long new_leader_id{ 0 };
	if (has_leader())
	{
		//new_leader_id = leader->get_id();
		leader_id.push_back(leader->get_id());
		double new_leader_max_brake = leader->get_max_brake();
		bool is_new_leader_connected = leader->is_connected();
		if (old_leader==nullptr)
		{
			controller.activate_origin_lane_controller(get_velocity(),
				compute_current_desired_time_headway(new_leader_max_brake, 
					is_new_leader_connected), is_new_leader_connected);
		}
		else if((std::abs(new_leader_max_brake 
			- old_leader->get_max_brake()) > 0.5)
			|| (leader->get_type() != old_leader->get_type()))
		{
			controller.update_origin_lane_controller(
				compute_current_desired_time_headway(new_leader_max_brake,
					is_new_leader_connected), is_new_leader_connected);
		}
	}
	else 
	{
		leader_id.push_back(0);
	}

	//long new_leader_id;
	//const long old_leader_id = leader_id.empty() ? 0 : leader_id.back();
	//if (has_leader() && (leader->get_id() != old_leader_id))
	//{
	//	controller.update_origin_lane_leader(
	//		get_velocity(), old_leader_id != 0, *leader);
	//	new_leader_id = leader->get_id();
	//}
	//else
	//{
	//	new_leader_id = old_leader_id;
	//}
	//leader_id.push_back(new_leader_id);
}

double EgoVehicle::compute_current_desired_time_headway(
	double nearby_vehicle_max_brake, bool is_nearby_vehicle_connected)
{
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_current_max_brake(), nearby_vehicle_max_brake, 
		get_current_lambda_1(is_nearby_vehicle_connected), rho, 0);
}

double EgoVehicle::compute_vehicle_following_desired_time_headway(
	double nearby_vehicle_max_brake, bool is_nearby_vehicle_connected)
{
	/* TODO: function still has a bug. We must ensure that lambda1 is chosen */
	return compute_time_headway_with_risk(get_desired_velocity(),
		max_brake, nearby_vehicle_max_brake,
		get_current_lambda_1(is_nearby_vehicle_connected), rho, 0);
}

double EgoVehicle::compute_lane_changing_desired_time_headway(
	double nearby_vehicle_max_brake, bool is_nearby_vehicle_connected)
{
	/* TODO: function still has a bug. We must ensure that lambda1_lc is chosen */
	return compute_time_headway_with_risk(get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle_max_brake,
		get_current_lambda_1(is_nearby_vehicle_connected), rho, 0);
}

/* State-machine related methods ------------------------------------------ */

void EgoVehicle::update_state() 
{
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
	if (old_state != get_state()) 
	{
		switch (get_state()) {
		case State::intention_to_change_lanes:
			if (has_leader())
			{
				controller.update_origin_lane_controller(
					compute_lane_changing_desired_time_headway(
						leader->get_max_brake(), leader->is_connected()), 
					leader->is_connected());
			}
			lane_change_waiting_time = 0.0;
			controller.start_longitudinal_adjustment(get_time());
			break;
		case State::lane_keeping:
			if (has_leader())
			{
				controller.update_origin_lane_controller(
					compute_vehicle_following_desired_time_headway(
						leader->get_max_brake(), leader->is_connected()),
					leader->is_connected());
			}
			lane_change_waiting_time = 0.0;
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

long EgoVehicle::get_color_by_controller_state() 
{
	/* We'll assign color to vehicles based on the current longitudinal
	controller and on whether or not the vehicle is trying to change lanes.*/
	if (state.empty()) return orig_lane_vel_control_color;
	
	/* TODO: Rewrite code to avoid all these swicth statements.
	Possible solution: ControlManager has a map of controllers and
	controllers get assigned colors for their states.
	Then we can just call controllers[active_controller].get_state_color() */

	switch (controller.get_active_longitudinal_controller()) 
	{
	case ControlManager::ACCType::origin_lane:
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
	case ControlManager::ACCType::cooperative_gap_generation:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return gap_generation_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return gap_generation_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ACCType::destination_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return dest_lane_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return dest_lane_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ACCType::end_of_lane:
		switch (controller.get_longitudinal_controller_state())
		{
		case LongitudinalController::State::velocity_control:
			return end_of_lane_vel_control_color;
		case LongitudinalController::State::vehicle_following:
			return end_of_lane_veh_foll_color;
		default:
			return WHITE;
		}
	case ControlManager::ACCType::traffic_light_acc:
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
	case ControlManager::ACCType::vissim:
		return CYAN;
	default:
		return WHITE;
	}
}

std::string EgoVehicle::print_detailed_state() 
{
	std::string state_str = 
		state_to_string(get_state()) + ", "
		+ ControlManager::active_ACC_to_string(
			controller.get_active_longitudinal_controller()) + ", "
		+ LongitudinalController::state_to_string(
			controller.get_longitudinal_controller_state());
	return state_str;
}

void EgoVehicle::update_waiting_time() 
{
	if (get_velocity() < 5.0/3.6) {
		lane_change_waiting_time += simulation_time_step;
	}
	else {
		lane_change_waiting_time = 0.0;
	}
}

/* Control related methods ------------------------------------------------ */

double EgoVehicle::consider_vehicle_dynamics(double desired_acceleration) 
{
	/* We assume lower level dynamics as:
	a = u / (tau.s + 1) => tau.da/dt + a = u 
	da/dt = 1/tau * (u - a)
	Discrete approximation:
	a(k+1) = a(k) + (1 - alpha)*(u(k+1) - a(k))
	where alpha = exp(-delta/tau), and delta is the sampling interval */

	double current_acceleration = get_acceleration();
	double filtered_acceleration = current_acceleration + (1 - tau_d)
		* (desired_acceleration - current_acceleration);
	if (verbose) std::clog << "[in veh. object] des. accel="
		<< desired_acceleration
		<< ", filtered accel.=" << filtered_acceleration
		<< std::endl;;
	return filtered_acceleration;
}

long EgoVehicle::decide_lane_change_direction()
{	
	if (can_start_lane_change())
	{
		return desired_lane_change_direction.to_int();
	}
	update_waiting_time();
	return 0;
}

double EgoVehicle::compute_safe_lane_change_gap(
	std::shared_ptr<NearbyVehicle> other_vehicle) 
{
	double safe_gap = 0.0;
	if (other_vehicle != nullptr) 
	{
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

/* Computation of surrogate safety measurements --------------------------- */

double EgoVehicle::compute_ttc(const NearbyVehicle& other_vehicle) 
{
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

double EgoVehicle::compute_drac(const NearbyVehicle& other_vehicle) 
{
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
	const NearbyVehicle& other_vehicle) const 
{
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
	const NearbyVehicle& other_vehicle) const
{	
	double current_max_brake = get_current_max_brake();
	double current_lambda_1 = get_current_lambda_1(other_vehicle.is_connected());

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
			<< ", id=" << get_id()
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

double EgoVehicle::compute_collision_severity_risk_to_leader() 
{
	if (has_leader()) {
		return compute_collision_severity_risk(*get_leader());
	}
	return 0.0;
}

/* Private methods -------------------------------------------------------- */

void EgoVehicle::set_desired_lane_change_direction() 
{
	/* Both preferred_relative_lane and relative_target_lane indicate
	desire to change lanes. The former indicates preference due to 
	routing, so it takes precedence over the latter. */
	RelativeLane current_preferred_lane = get_preferred_relative_lane();
	desired_lane_change_direction = RelativeLane::same;
	if (current_preferred_lane.is_to_the_left()) {
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (current_preferred_lane.is_to_the_right()) {
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

/* Methods for printing and debugging ------------------------------------- */

void EgoVehicle::write_simulation_log(std::vector<Member> members)
{
	bool write_size = true;
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
	vehicle_log.open(log_path + "\\" + file_name);
	if (vehicle_log.is_open()) 
	{
		vehicle_log << write_header(members, write_size);
		vehicle_log	<< write_members(members);
		vehicle_log.close();
	}
	else 
	{
		std::clog << "Unable to open file to write log of "
			<< "vehicle " << get_id()
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
				oss << get_id();
				break;
			case Member::length:
				oss << get_length();
				break;
			case Member::width:
				oss << get_width();
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
			/*case Member::vissim_active_lane_change_direction:
				oss << vissim_active_lane_change[i];
				break;*/
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
				oss << static_cast<int>(get_type());
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
	/*case Member::vissim_active_lane_change_direction:
		return (int)vissim_active_lane_change.size();*/
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
	/*case Member::vissim_active_lane_change_direction:
		return "vissim lc direction";*/
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
	out << "t=" << vehicle.get_time()
		<< ", id=" << vehicle.get_id()
		<< ", type=" << static_cast<int>(vehicle.get_type())
		<< ", lane=" << vehicle.get_lane()
		<< ", pref. lane="
		<< vehicle.get_preferred_relative_lane().to_string()
		<< ", use preferred lane="
		<< vehicle.get_vissim_use_preferred_lane()
		<< ", target lane="
		<< vehicle.relative_target_lane.to_string()
		/*<< ", vissim active lc="
		<< RelativeLane::from_long(
			vehicle.get_vissim_active_lane_change()).to_string()*/
		<< ", active lc.="
		<< vehicle.get_active_lane_change_direction().to_string()
		<< ", vel=" << vehicle.get_velocity()
		<< ", accel=" << vehicle.get_acceleration();

	return out; // return std::ostream so we can chain calls to operator<<
}
