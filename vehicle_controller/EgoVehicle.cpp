/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of xxxx-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "EgoVehicle.h"
#include "Platoon.h"

EgoVehicle::EgoVehicle(long id, VehicleType type, double desired_velocity,
	double brake_delay, bool is_lane_change_autonomous, bool is_connected,
	double simulation_time_step, double creation_time, bool verbose) :
	Vehicle(id, type, brake_delay),
	desired_velocity{ desired_velocity },
	is_lane_change_autonomous { is_lane_change_autonomous },
	is_connected { is_connected },
	simulation_time_step{ simulation_time_step },
	creation_time{ creation_time },
	current_time{ creation_time },
	verbose{ verbose },
	tau_d{ std::exp(-simulation_time_step / tau) }
{
	compute_safe_gap_parameters();
	compute_lane_change_gap_parameters();

	set_state(std::make_unique<SingleVehicleLaneKeepingState>());

	if (verbose)
	{
		std::clog << "Creating vehicle " << get_id()
			<< " at time " << this->creation_time
			<< ", category " << static_cast<int>(category)
			<< ", type " << static_cast<int>(get_type())
			<< ", des. vel. = " << desired_velocity
			<< ", lambda 1 = " << get_lambda_1()
			<< ", lambda 0 = " << get_lambda_0()
			<< ", lambda 1 lc = " << get_lambda_1_lane_change()
			<< ", lambda 0 lc = " << get_lambda_0_lane_change()
			<< std::endl;
	}

	//this->controller = ControlManager(*this, verbose_control_manager);
}

EgoVehicle::~EgoVehicle()
{
	if (verbose)
	{
		std::clog << "Vehicle " << get_id()
			<< " out of the simulation at time "
			<< get_current_time() << std::endl;
	}
}

/* Non-trivial Getters and setters ---------------------------------------- */

long EgoVehicle::get_leader_id() const
{
	return has_leader() ? leader->get_id() : 0;
}

long EgoVehicle::get_old_leader_id() const
{
	return old_leader_id;
}

const Platoon* EgoVehicle::get_platoon() const {
	return implement_get_platoon();
};

std::shared_ptr<Platoon> EgoVehicle::share_platoon() const {
	return implement_share_platoon();
};

double EgoVehicle::get_current_max_brake() const
{
	return is_lane_changing() ?
		get_lane_change_max_brake() : max_brake;
}

std::pair<double, double> EgoVehicle::get_current_safe_gap_parameters() const
{
	return is_lane_changing() ?
		get_lane_changing_safe_gap_parameters()
		: std::make_pair(get_lambda_0(), get_lambda_1());
}

std::pair<double, double> EgoVehicle::get_lane_changing_safe_gap_parameters()
const
{
	return std::make_pair(lambda_0_lane_change, lambda_1_lane_change);
}

double EgoVehicle::get_safe_time_headway() const
{
	return controller->get_safe_time_headway();
}

double EgoVehicle::get_gap_error() const
{
	return controller->get_gap_error();
}

double EgoVehicle::get_current_desired_time_headway() const
{
	return controller->get_current_desired_time_headway();
}

double EgoVehicle::get_gap_variation_to(
	const NearbyVehicle* nearby_vehicle) const
{
	if ((nearby_vehicle != nullptr)
		&& (gap_variation_during_lane_change.find(nearby_vehicle->get_id())
			!= gap_variation_during_lane_change.end()))
		return gap_variation_during_lane_change.at(nearby_vehicle->get_id());
	if (verbose && nearby_vehicle != nullptr)
	{
		std::clog << "Couldnt find the delta g for nv id: "
			<< nearby_vehicle->get_id() << "\n"
			<< "Only have it for vehs. ";

		for (auto it = gap_variation_during_lane_change.begin();
			it != gap_variation_during_lane_change.end(); ++it)
		{
			std::cout << it->first << " ,";
		}
		std::clog << std::endl;
	}
	return -1.0;
}

double EgoVehicle::get_collision_free_gap_to(
	const NearbyVehicle* nearby_vehicle) const
{
	if ((nearby_vehicle != nullptr)
		&& (collision_free_gap.find(nearby_vehicle->get_id())
			!= collision_free_gap.end()))
		return collision_free_gap.at(nearby_vehicle->get_id());
	if (verbose && nearby_vehicle != nullptr)
	{
		std::clog << "Couldnt find g* for nv id: "
			<< nearby_vehicle->get_id() << "\n"
			<< "Only have g* for vehs. ";

		for (auto it = collision_free_gap.begin();
			it != collision_free_gap.end(); ++it)
		{
			std::cout << it->first << " ,";
		}
		std::clog << std::endl;
	}
	return -1.0;
}

const VehicleState* EgoVehicle::get_state() const
{
	return state.get();
}

double EgoVehicle::get_road_reference_lateral_position() const
{
	return (get_lane() + 1 / 2)* LANE_WIDTH + get_lateral_position();
}

StateVector EgoVehicle::get_state_vector() const
{
	return StateVector{ get_distance_traveled(), 
		get_road_reference_lateral_position(),
		get_orientation_angle(), get_velocity() };
}

void EgoVehicle::set_active_lane_change_direction(long direction)
{
	this->active_lane_change_direction =
		RelativeLane::from_long(direction);
}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane)
{
	this->preferred_relative_lane =
		RelativeLane::from_long(preferred_relative_lane);
}

//void EgoVehicle::set_relative_target_lane(long target_relative_lane)
//{
//	this->relative_target_lane =
//		RelativeLane::from_long(target_relative_lane);
//}

void EgoVehicle::set_vissim_lane_suggestion(long target_relative_lane)
{
	this->vissim_lane_suggestion =
		RelativeLane::from_long(target_relative_lane);
}

void EgoVehicle::set_lane_end_distance(double lane_end_distance,
	long lane_number)
{
	if (lane_number == get_lane())
	{
		this->lane_end_distance = lane_end_distance;
	}
}

void EgoVehicle::set_verbose(bool value)
{
	if (value && !verbose)
	{
		std::clog << "[EgoVehicle] set to verbose.\n";
	}
	else if (!value && verbose)
	{
		std::clog << "[EgoVehicle] trying to set not verbose failed.\n";
		return;
	}
	verbose = value;
	controller->set_verbose(value);
}

void EgoVehicle::set_controller(VehicleController* controller)
{
	this->controller = controller;
}

void EgoVehicle::set_gap_variation_during_lane_change(int nv_id, double value)
{
	gap_variation_during_lane_change[nv_id] = value;
}

void EgoVehicle::set_collision_free_gap(int nv_id, double value)
{
	collision_free_gap[nv_id] = value;
}

/* Nearby Vehicles methods ------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles()
{
	nearby_vehicles.clear();
	gap_variation_during_lane_change.clear();
	collision_free_gap.clear();
}

void EgoVehicle::emplace_nearby_vehicle(long nv_id, long relative_lane,
	long relative_position)
{
	nearby_vehicles[nv_id] = std::make_shared<NearbyVehicle>(
		nv_id, relative_lane, relative_position); 
}

void EgoVehicle::add_nearby_vehicle(NearbyVehicle nearby_vehicle)
{
	nearby_vehicles[nearby_vehicle.get_id()] = 
		std::make_shared<NearbyVehicle>(nearby_vehicle);
}

void EgoVehicle::set_nearby_vehicle_type(long nv_id, long nv_type)
{
	get_nearby_vehicle_by_id(nv_id)->set_type(VehicleType(nv_type), type);
}

bool EgoVehicle::has_leader() const
{
	return leader != nullptr;
}

const NearbyVehicle* EgoVehicle::get_leader() const
{
	return leader;
}

NearbyVehicle* EgoVehicle::get_nearby_vehicle_by_id(
	long nv_id) const
{
	return nearby_vehicles.find(nv_id) != nearby_vehicles.end() ?
		nearby_vehicles.at(nv_id).get() : nullptr;
}

double EgoVehicle::get_relative_velocity_to_leader()
{
	return has_leader() ? leader->get_relative_velocity() : 0.0;
}

bool EgoVehicle::has_destination_lane_leader() const
{
	return get_destination_lane_leader() != nullptr;
}
bool EgoVehicle::has_destination_lane_follower() const
{
	return get_destination_lane_follower() != nullptr;
}
bool EgoVehicle::has_assisted_vehicle() const
{
	return get_assisted_vehicle() != nullptr;
}

long EgoVehicle::get_destination_lane_leader_id() const
{
	return has_destination_lane_leader() ?
		get_destination_lane_leader()->get_id() : 0;
}
long EgoVehicle::get_destination_lane_follower_id() const
{
	return has_destination_lane_follower() ?
		get_destination_lane_follower()->get_id() : 0;
}
long EgoVehicle::get_assisted_veh_id() const
{
	return has_assisted_vehicle() ?
		get_assisted_vehicle()->get_id() : 0;
}

long EgoVehicle::get_virtual_leader_id() const
{
	return implement_get_virtual_leader_id();
}

bool EgoVehicle::is_in_a_platoon() const
{
	return get_platoon() != nullptr;
}

long EgoVehicle::get_platoon_id() const
{
	return is_in_a_platoon() ? get_platoon()->get_id() : -1;
}

double EgoVehicle::compute_gap_to_a_leader(
	const NearbyVehicle& nearby_vehicle) const
{
	/* Note: Vissim's distance is from front end to front end, and 
	negative values indicate the nearby vehicle is behind */
	if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
	{
		return MAX_DISTANCE;
	}

	return nearby_vehicle.get_distance() - nearby_vehicle.get_length();
}

double EgoVehicle::compute_gap_to_a_leader(
	const NearbyVehicle* nearby_vehicle) const
{
	if (nearby_vehicle != nullptr)
	{
		return compute_gap_to_a_leader(*nearby_vehicle);
	}
	else
	{
		return MAX_DISTANCE;
	}
}

double EgoVehicle::compute_gap_to_a_follower(
	const NearbyVehicle& nearby_vehicle) const
{
	/* Note: Vissim's distance is from front end to front end, and
	negative values indicate the nearby vehicle is behind */
	if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
	{
		return MAX_DISTANCE;
	}
	return -nearby_vehicle.get_distance() - get_length();
}

double EgoVehicle::compute_gap_to_a_follower(
	const NearbyVehicle* nearby_vehicle) const
{
	if (nearby_vehicle != nullptr)
	{
		return compute_gap_to_a_follower(*nearby_vehicle);
	}
	else
	{
		return MAX_DISTANCE;
	}
}

//double EgoVehicle::compute_absolute_gap(const NearbyVehicle& nearby_vehicle) const
//{
//	/* Vissim's given "distance" is the distance between both front bumpers,
//	so we subtract the length from that. We need to check which vehicle is
//	ahead to determine whose length must be subtracted. */
//	return std::abs(compute_gap_to_a_leader(nearby_vehicle));
//	//if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
//	//{
//	//	return MAX_DISTANCE;
//	//}
//	//if (verbose) 
//	//{
//	//	std::clog << "\tveh " << nearby_vehicle.get_id()
//	//		<< " is ahead? " << (nearby_vehicle.is_ahead() ? "yes" : "no")
//	//		<< std::endl;
//	//}
//	//if (nearby_vehicle.is_ahead())
//	//{
//	//	return nearby_vehicle.get_distance()
//	//		- nearby_vehicle.get_length();
//	//}
//	//else
//	//{
//	//	if (verbose)
//	//	{
//	//		std::clog << "\tnv get distance: " << nearby_vehicle.get_distance()
//	//			<< std::endl;
//	//	}
//	//	return -nearby_vehicle.get_distance() - get_length();
//	//}
//}
//
//double EgoVehicle::compute_absolute_gap(
//	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
//{
//	if (nearby_vehicle != nullptr)
//	{
//		return compute_absolute_gap(*nearby_vehicle);
//	}
//	else
//	{
//		return MAX_DISTANCE;
//	}
//}

long EgoVehicle::get_lane_change_request() const
{
	return implement_get_lane_change_request();
}

void EgoVehicle::pass_this_to_state()
{
	state->set_ego_vehicle(this);
}

const Platoon* EgoVehicle::implement_get_platoon() const
{
	return nullptr;
};

std::shared_ptr<Platoon> EgoVehicle::implement_share_platoon() const
{
	return nullptr;
};

void EgoVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
}

const std::unordered_map<long, std::shared_ptr<NearbyVehicle>> EgoVehicle
::get_nearby_vehicles() const
{
	return nearby_vehicles;
}

void EgoVehicle::set_leader_by_id(long new_leader_id)
{
	leader = get_nearby_vehicle_by_id(new_leader_id);
}

void EgoVehicle::find_leader()
{
	old_leader_id = get_leader_id();
	const NearbyVehicle* old_leader = leader;
	leader = nullptr;
	for (auto const& id_veh_pair : nearby_vehicles)
	{
		if (check_if_is_leader(*id_veh_pair.second)) 
		{
			leader = id_veh_pair.second.get();
		}
	}
	update_leader(old_leader);
}

bool EgoVehicle::check_if_is_leader(const NearbyVehicle& nearby_vehicle) const
{
	if ((nearby_vehicle.is_ahead()
		&& nearby_vehicle.is_on_same_lane())
		|| nearby_vehicle.is_cutting_in())
	{
		if (!has_leader()
			|| (nearby_vehicle.get_distance() < leader->get_distance()))
		{
			return true;
		}
	}
	return false;
}

bool EgoVehicle::is_destination_lane_follower(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();
	return nv_relative_lane == desired_lane_change_direction
		&& nearby_vehicle.is_immediatly_behind();
}

bool EgoVehicle::is_destination_lane_leader(
	const NearbyVehicle& nearby_vehicle)
{
	int current_id = nearby_vehicle.get_id();
	RelativeLane nv_relative_lane = nearby_vehicle.get_relative_lane();
	return nv_relative_lane == desired_lane_change_direction
		&& nearby_vehicle.is_immediatly_ahead();
}

bool EgoVehicle::is_leader_of_destination_lane_leader(
	const NearbyVehicle& nearby_vehicle)
{
	RelativeLane nv_relative_lane =
		nearby_vehicle.get_relative_lane();
	long relative_position =
		nearby_vehicle.get_relative_position();
	return (nv_relative_lane == desired_lane_change_direction
		&& relative_position == 2);
}

void EgoVehicle::update_leader(
	const NearbyVehicle* old_leader)
{
	if (has_leader())
	{
		double new_leader_max_brake = leader->get_max_brake();
		bool is_new_leader_connected = leader->is_connected();
		if (old_leader == nullptr)
		{
			controller->activate_origin_lane_controller(*leader);
		}
		else if((std::abs(new_leader_max_brake
			- old_leader->get_max_brake()) > 0.5)
			|| (leader->get_type() != old_leader->get_type()))
		{
			controller->update_origin_lane_controller(*leader);
		}
	}
}

double EgoVehicle::compute_current_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	if (has_lane_change_intention() || is_lane_changing())
	{
		return compute_lane_changing_desired_time_headway(nearby_vehicle);
	}
	return compute_vehicle_following_safe_time_headway(nearby_vehicle);
}

double EgoVehicle::compute_vehicle_following_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	return compute_time_headway_with_risk(get_desired_velocity(),
		max_brake, nearby_vehicle.get_max_brake(), get_lambda_1(), rho, 0);
}

void EgoVehicle::compute_lane_change_gap_parameters()
{
	lambda_0_lane_change = compute_lambda_0(max_jerk,
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
	lambda_1_lane_change = compute_lambda_1(max_jerk,
		comfortable_acceleration, get_lane_change_max_brake(), brake_delay);
}

/* State-machine related methods ------------------------------------------ */

void EgoVehicle::update_state()
{
	current_time += simulation_time_step;
	set_desired_lane_change_direction();

	if (has_lane_change_intention())
	{
		state->handle_lane_change_intention();
	}
	else
	{
		state->handle_lane_keeping_intention();
	}
}

void EgoVehicle::set_state(std::unique_ptr<VehicleState> new_state)
{
	if (verbose)
	{
		std::clog << ">>>> STATE TRANSITION <<<<\n"
		<< "\tat t=" << get_current_time() << " from ";
		if (state == nullptr) std::clog << "none";
		else std::clog << *state;
		std::clog << " to " << *new_state << std::endl;
	}
	state = std::move(new_state);
	pass_this_to_state();
}

void EgoVehicle::reset_state(
	std::unique_ptr<VehicleState> new_lane_keeping_state)
{
	// TODO: poor condition checking: hard coded variables
	if (new_lane_keeping_state->get_state_number() != 1)
	{
		std::clog << "[WARNING] t=" << get_current_time()
			<< ", veh " << get_id() << ": reseting vehicle state to "
			<< *new_lane_keeping_state
			<< ", which is not a lane keeping state." << std::endl;
	}

	if (verbose) std::clog << "[EgoVehicle] Resetting state\n";

	set_lane_change_direction(RelativeLane::same);
	reset_lane_change_waiting_time();
	update_time_headway_to_leader();
	set_state(std::move(new_lane_keeping_state));
}

void EgoVehicle::update_lane_change_waiting_time()
{
	if (get_velocity() < 5.0 / 3.6)
	{
		lane_change_waiting_time += simulation_time_step;
	}
	else
	{
		lane_change_waiting_time = 0.0;
	}
}

void EgoVehicle::reset_lane_change_waiting_time()
{
	lane_change_waiting_time = 0.0;
}

bool EgoVehicle::check_lane_change_gaps()
{
	return implement_check_lane_change_gaps();
}

void EgoVehicle::prepare_to_start_long_adjustments()
{
	implement_prepare_to_start_long_adjustments();
}

void EgoVehicle::prepare_to_restart_lane_keeping(
	bool was_lane_change_successful)
{
	implement_prepare_to_restart_lane_keeping(was_lane_change_successful);
}

bool EgoVehicle::is_lane_changing() const
{
	return get_active_lane_change_direction() != RelativeLane::same;
}

long EgoVehicle::get_color_by_controller_state()
{
	long state_color = controller->get_longitudinal_controller_color();
	if (compute_gap_to_a_leader(leader) < compute_safe_gap_to_leader())
	{
		state_color = RED;
	}
	return state_color;
}


/* Control related methods ------------------------------------------------ */

double EgoVehicle::apply_low_level_dynamics(double unfiltered_acceleration)
{
	/* We assume lower level dynamics as:
	a = u / (tau.s + 1) => tau.da/dt + a = u
	da/dt = 1/tau * (u - a)
	Discrete approximation:
	a(k+1) = a(k) + (1 - alpha)*(u(k+1) - a(k))
	where alpha = exp(-delta/tau), and delta is the sampling interval */

	double current_acceleration = get_acceleration();
	double filtered_acceleration = current_acceleration + (1 - tau_d)
		* (unfiltered_acceleration - current_acceleration);
	if (verbose) std::clog << "[in veh. object] des. accel="
		<< unfiltered_acceleration
		<< ", filtered accel.=" << filtered_acceleration
		<< std::endl;;
	return filtered_acceleration;
}

void EgoVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	//this->desired_acceleration =
	//	implement_compute_desired_acceleration(traffic_lights);
	double accel = controller->get_desired_acceleration();
	this->desired_acceleration = apply_low_level_dynamics(accel);
}

double EgoVehicle::get_accepted_lane_change_gap(
	const NearbyVehicle* nearby_vehicle)
{
	return compute_accepted_lane_change_gap(nearby_vehicle);
}

double EgoVehicle::get_reference_gap() const
{
	return controller->get_reference_gap();
}

double EgoVehicle::compute_time_headway_gap(
	const NearbyVehicle* nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	if (nearby_vehicle != nullptr)
	{
		time_headway_gap = controller->get_desired_time_headway_gap(
			*nearby_vehicle);
	}
	return time_headway_gap;
}

void EgoVehicle::update_time_headway_to_leader()
{
	if (has_leader())
	{
		controller->update_origin_lane_controller(*leader);
	}
}

void EgoVehicle::reset_origin_lane_velocity_controller()
{
	controller->reset_origin_lane_velocity_controller();
}

/* Computation of surrogate safety measurements --------------------------- */
double EgoVehicle::compute_safe_gap_to_leader()
{
	return has_leader() ?
		compute_safe_gap_to_a_leader(*leader)
		: 0.0;
}

double EgoVehicle::compute_safe_gap_to_a_leader(
	const NearbyVehicle& a_leader) const
{
	return compute_risky_gap_to_leader(a_leader, 0.0);
}

double EgoVehicle::compute_risky_gap_to_leader(const NearbyVehicle& a_leader,
	double accepted_risk) const
{
	double v_follower = get_velocity();
	double brake_follower = get_current_max_brake();
	double v_leader = a_leader.compute_velocity(v_follower);
	double brake_leader = a_leader.get_max_brake();
	auto params = get_current_safe_gap_parameters();
	return compute_risky_gap(v_follower, v_leader, brake_follower,
		brake_leader, params.first, params.second, accepted_risk);
}

double EgoVehicle::compute_ttc(const NearbyVehicle& nearby_vehicle)
{
	/* Time-to-collision is:
	(leader x - leader length - ego x) / (ego vel - leader vel),
		if ego vel > leader vel
	undefined,
		if ego vel < leader vel */
	if (nearby_vehicle.get_relative_velocity() > 0)
	{
		return compute_gap_to_a_leader(nearby_vehicle) 
			/ nearby_vehicle.get_relative_velocity();
	}
	return -1.0;
}

double EgoVehicle::compute_drac(const NearbyVehicle& nearby_vehicle)
{
	/* Deceleration rate to avoid collision
	(ego vel - leader vel)^2 / 2*(leader x - leader length - ego x),
		if ego vel > leader vel
	underfined,
		if ego vel < leader vel
	*/
	if (nearby_vehicle.get_relative_velocity() > 0)
	{
		return std::pow(nearby_vehicle.get_relative_velocity(), 2)
			/ 2 / compute_gap_to_a_leader(nearby_vehicle);
	}
	return -1.0;
}

void EgoVehicle::set_desired_lane_change_direction()
{
	/* Both preferred_relative_lane and relative_target_lane indicate
	desire to change lanes. The former indicates preference due to
	routing, so it takes precedence over the latter. */
	RelativeLane current_preferred_lane = get_preferred_relative_lane();
	//desired_lane_change_direction = RelativeLane::same;
	if (current_preferred_lane.is_to_the_left())
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (current_preferred_lane.is_to_the_right())
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else if (vissim_lane_suggestion.is_to_the_left())
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (vissim_lane_suggestion.is_to_the_right())
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else
	{
		desired_lane_change_direction = RelativeLane::same;
	}
}

/* Methods for printing and debugging ------------------------------------- */

//void EgoVehicle::write_simulation_log(std::vector<Member> members)
//{
//	bool write_size = true;
//	std::ofstream vehicle_log;
//	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
//	vehicle_log.open(log_path + "\\" + file_name);
//	if (vehicle_log.is_open())
//	{
//		vehicle_log << write_header(members, write_size);
//		vehicle_log	<< write_members(members);
//		vehicle_log.close();
//	}
//	else
//	{
//		std::clog << "Unable to open file to write log of "
//			<< "vehicle " << get_id()
//			<< std::endl;
//	}
//}

//std::string EgoVehicle::write_header(
//	std::vector<EgoVehicle::Member> members, bool write_size)
//{
//
//	std::ostringstream oss;
//	for (auto m : members)
//	{
//		oss << member_enum_to_string(m);
//		if (write_size)
//		{
//			oss << " (" << get_member_size(m) << ")";
//		}
//		oss << ", ";
//	}
//	oss << std::endl;
//
//	return oss.str();
//}

//std::string EgoVehicle::write_members(
//	std::vector<EgoVehicle::Member> members)
//{
//	std::ostringstream oss;
//
//	/* Sanity check: some non critical code mistakes could
//	create vector members with different sizes. This prevents
//	the log from being written and crashes vissim. Let's avoid
//	this and just write a warning message instead.*/
//	int n_samples = (int)velocity.size(); /* velocity, lane and link members
//	are the least likely to have the wrong size */
//	std::vector<int> deleted_indices;
//	for (int i = 0; i < members.size(); i++)
//	{
//		Member m = members.at(i);
//		if ((get_member_size(m) != 1) // not a scalar
//			&& (get_member_size(m) != n_samples))
//		{
//			oss << "Error: member " << member_enum_to_string(m)
//				<< " has " << get_member_size(m) << " samples "
//				<< "instead of the expected " << n_samples
//				<< ". It won't be printed."
//				<< std::endl;
//			deleted_indices.push_back(i);
//		}
//	}
//	for (int idx : deleted_indices)
//	{
//		members.erase(std::next(members.begin(), idx));
//	}
//
//	// Write variables over time
//	for (int i = 0; i < n_samples; i++)
//	{
//		for (auto m : members)
//		{
//			switch (m)
//			{
//			case Member::creation_time:
//				oss << creation_time + i * simulation_time_step;
//				break;
//			case Member::id:
//				oss << get_id();
//				break;
//			case Member::length:
//				oss << get_length();
//				break;
//			case Member::width:
//				oss << get_width();
//				break;
//			case Member::color:
//				oss << "color printing code not done";
//				break;
//			case Member::category:
//				oss << static_cast<int>(category);
//				break;
//			case Member::desired_velocity:
//				oss << desired_velocity;
//				break;
//			case Member::lane:
//				oss << lane[i];
//				break;
//			case Member::link:
//				oss << link[i];
//				break;
//			case Member::preferred_relative_lane:
//				oss << preferred_relative_lane[i];
//				break;
//			case Member::velocity:
//				oss << velocity[i];
//				break;
//			case Member::acceleration:
//				oss << acceleration[i];
//				break;
//			//case Member::desired_acceleration:
//			//	oss << desired_acceleration[i];
//			//	break;
//			case Member::vissim_acceleration:
//				oss << vissim_acceleration[i];
//				break;
//			case Member::leader_id:
//				oss << leader_id[i];
//				break;
//			/*case Member::state:
//				oss << state_to_string_map.at(state_implementation_v1[i]);
//				break;*/
//			case Member::active_lane_change_direction:
//				oss << active_lane_change_direction[i];
//				break;
//			/*case Member::vissim_active_lane_change_direction:
//				oss << vissim_active_lane_change[i];
//				break;*/
//			case Member::lane_end_distance:
//				oss << lane_end_distance[i];
//				break;
//			case Member::ttc:
//				oss << ttc[i];
//				break;
//			case Member::drac:
//				oss << drac[i];
//				break;
//			case Member::collision_severity_risk:
//				oss << collision_severity_risk[i];
//				break;
//			case Member::type:
//				oss << static_cast<int>(get_type());
//				break;
//			default:
//				oss << "";
//				break;
//			}
//			oss << ", ";
//		}
//		oss << std::endl;
//	}
//	return oss.str();
//}

//int EgoVehicle::get_member_size(Member member)
//{
//	switch (member)
//	{
//	case Member::creation_time:
//	case Member::id:
//	case Member::length:
//	case Member::width:
//	case Member::color:
//	case Member::category:
//	case Member::desired_velocity:
//	case Member::type:
//	case Member::state:
//		return 1;
//	case Member::lane:
//		return (int)lane.size();
//	case Member::link:
//		return (int)link.size();
//	case Member::preferred_relative_lane:
//		return (int)preferred_relative_lane.size();
//	case Member::velocity:
//		return (int)velocity.size();
//	case Member::acceleration:
//		return (int)acceleration.size();
//	case Member::desired_acceleration:
//		return 1; // (int)desired_acceleration.size();
//	case Member::vissim_acceleration:
//		return (int)vissim_acceleration.size();
//	case Member::leader_id:
//		return (int)leader_id.size();
//	case Member::active_lane_change_direction:
//		return (int)active_lane_change_direction.size();
//	/*case Member::vissim_active_lane_change_direction:
//		return (int)vissim_active_lane_change.size();*/
//	case Member::lane_end_distance:
//		return (int)lane_end_distance.size();
//	case Member::ttc:
//		return (int)ttc.size();
//	case Member::drac:
//		return (int)drac.size();
//	case Member::collision_severity_risk:
//		return (int)collision_severity_risk.size();
//	default:
//		return 0;
//	}
//}

std::string EgoVehicle::member_enum_to_string(Member member)
{
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

std::ostream& operator<< (std::ostream& out, const EgoVehicle& ego_vehicle)
{
	out << "t=" << ego_vehicle.get_current_time()
		<< ", id=" << ego_vehicle.get_id()
		<< ", type=" << static_cast<int>(ego_vehicle.get_type())
		<< ", state=" << *(ego_vehicle.state)
		<< ", vel=" << ego_vehicle.get_velocity()
		<< ", des accel=" << ego_vehicle.get_desired_acceleration()
		<< ", accel=" << ego_vehicle.get_acceleration()
		<< ", lane=" << ego_vehicle.get_lane()
		<< ", platoon=" << ego_vehicle.get_platoon_id()
		<< "\n\tLane changing:" 
		<< " pref.=" << ego_vehicle.get_preferred_relative_lane()
		<< ", vissim suggestion=" << ego_vehicle.get_vissim_lane_suggestion()
		<< ", desired=" << ego_vehicle.get_desired_lane_change_direction()
		<< ", decision (our choice)="
		<< ego_vehicle.get_lane_change_direction()
		<< ", active=" << ego_vehicle.get_active_lane_change_direction();

	return out; // return std::ostream so we can chain calls to operator<<
}
