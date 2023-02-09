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
	bool verbose_control_manager = verbose;
	set_state(std::make_unique<SingleVehicleLaneKeepingState>());

	if (verbose)
	{
		std::clog << "Creating vehicle " << get_id()
			<< " at time " << this->creation_time
			<< ", category " << static_cast<int>(category)
			<< ", type " << static_cast<int>(get_type())
			<< ", des. vel. = " << desired_velocity
			<< ", lambda 1 = " << get_lambda_1()
			<< ", lambda 0 = " << get_lambda_0();
			if (!verbose_control_manager)
			{
				std::clog << "\nNOTE: control manager is NOT verbose.";
			}
			std::clog << std::endl;
	}

	this->controller = ControlManager(*this, verbose_control_manager);
}

EgoVehicle::~EgoVehicle()
{
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::active_lane_change_direction,
		Member::leader_id,
	};
	if (verbose)
	{
		std::clog << write_header(members, true);
		std::clog << "Vehicle " << get_id()
			<< " out of the simulation at time "
			<< get_time() << std::endl;

		//write_simulation_log(members);
	}
}

/* "Current" getters ------------------------------------------------------ */

double EgoVehicle::get_time() const
{
	/* At creation time, the vehicle already has a velocity */
	return current_time;
	//creation_time + (velocity.size() - 1) * simulation_time_step;
}
long EgoVehicle::get_lane() const
{
	return lane.back();
}
double EgoVehicle::get_distance_traveled() const
{
	return distance_traveled.back();
}
long EgoVehicle::get_link() const
{
	return link.back();
}
double EgoVehicle::get_lateral_position() const
{
	return lateral_position.back();
}
RelativeLane EgoVehicle::get_preferred_relative_lane() const
{
	return preferred_relative_lane.back();
}
double EgoVehicle::get_velocity() const
{
	if (velocity.empty()) /* We shouldn't need to check this condition.
							This is used to avoid crashing during tests.*/
	{
		return 0;
	}
	return velocity.back();
}
double EgoVehicle::get_acceleration() const
{
	return acceleration.back();
}
double EgoVehicle::get_desired_acceleration() const
{
	return desired_acceleration;
}
double EgoVehicle::get_vissim_acceleration() const
{
	return vissim_acceleration.back();
}
RelativeLane EgoVehicle::get_active_lane_change_direction() const
{
	return active_lane_change_direction.back();
}
double EgoVehicle::get_lane_end_distance() const
{
	return lane_end_distance.back();
}
long EgoVehicle::get_leader_id() const
{
	return leader_id.back();
}
//EgoVehicle::State EgoVehicle::get_state_implementation_v1() const
//{
//	return state_implementation_v1.empty() ?
//		State::lane_keeping : state_implementation_v1.back();
//}
double EgoVehicle::get_ttc() const
{
	return ttc.back();
}
double EgoVehicle::get_drac() const
{
	return drac.back();
}
double EgoVehicle::get_collision_risk() const
{
	return collision_severity_risk.back();
}
/* ------------------------------------------------------------------------ */

/* Other getters and setters ---------------------------------------------- */

double EgoVehicle::get_current_max_brake() const
{
	return has_lane_change_intention() ?
		get_lane_change_max_brake() : max_brake;
}

double EgoVehicle::get_safe_time_headway() const
{
	return controller.get_safe_time_headway();
}

double EgoVehicle::get_gap_error() const
{
	return controller.get_gap_error();
}

double EgoVehicle::get_current_desired_time_headway() const
{
	return controller.get_current_desired_time_headway();
}

double EgoVehicle::get_gap_variation_to(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
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
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
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

void EgoVehicle::set_lane(long lane)
{
	this->lane.push_back(lane);
}
void EgoVehicle::set_distance_traveled(double distance_traveled)
{
	this->distance_traveled.push_back(distance_traveled);
}
void EgoVehicle::set_link(long link)
{
	this->link.push_back(link);
}
void EgoVehicle::set_lateral_position(double lateral_position)
{
	this->lateral_position.push_back(lateral_position);
}
void EgoVehicle::set_velocity(double velocity)
{
	this->velocity.push_back(velocity);
}
void EgoVehicle::set_acceleration(double acceleration)
{
	this->acceleration.push_back(acceleration);
}
void EgoVehicle::set_vissim_acceleration(double vissim_acceleration)
{
	this->vissim_acceleration.push_back(vissim_acceleration);
}
void EgoVehicle::set_active_lane_change_direction(long direction)
{
	this->active_lane_change_direction.push_back(
		RelativeLane::from_long(direction));
}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane)
{
	this->preferred_relative_lane.push_back(
		RelativeLane::from_long(preferred_relative_lane));
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
		this->lane_end_distance.push_back(lane_end_distance);
	}
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
	std::shared_ptr<NearbyVehicle> nearby_vehicle =
		std::make_shared<NearbyVehicle>(NearbyVehicle(nv_id, relative_lane,
		relative_position));
	nearby_vehicles[nv_id] = std::move(nearby_vehicle);
}

//std::shared_ptr<NearbyVehicle> EgoVehicle::peek_nearby_vehicles() const
//{
//	if (!nearby_vehicles.empty())
//	{
//		return nearby_vehicles.back();
//	}
//	std::clog << "Empty nearby_vehicles container in vehicle  " << get_id()
//		<< std::endl;
//	return nullptr;
//}

void EgoVehicle::set_nearby_vehicle_type(long nv_id, long nv_type)
{
	get_nearby_vehicle_by_id(nv_id)->set_type(VehicleType(nv_type), type);
}

bool EgoVehicle::has_leader() const
{
	return leader != nullptr;
}

double EgoVehicle::get_time_headway_to_assisted_vehicle() const
{
	if (has_assisted_vehicle())
	{
		return controller.get_gap_generation_lane_controller().
			get_desired_time_headway();
	}
	/* We return a high value when there's no assisted vehicle because,
	when a vehicle first requests assistance, it takes one simulation
	iteration for the headway to be computed and transferred to the
	assisted vehicle. */
	return 3.0;
}

std::shared_ptr<const NearbyVehicle> EgoVehicle::get_leader() const
{
	return leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_nearby_vehicle_by_id(
	long nv_id) const
{
	return nearby_vehicles.find(nv_id) != nearby_vehicles.end() ?
		nearby_vehicles.at(nv_id) : nullptr;
	/*for (std::shared_ptr<NearbyVehicle> nv : nearby_vehicles)
	{
		if (nv->get_id() == nv_id) return nv;
	}
	return nullptr;*/
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

long EgoVehicle::get_dest_lane_leader_id() const
{
	return has_destination_lane_leader() ?
		get_destination_lane_leader()->get_id() : 0;
}
long EgoVehicle::get_dest_lane_follower_id() const
{
	return has_destination_lane_follower() ?
		get_destination_lane_follower()->get_id() : 0;
}
long EgoVehicle::get_assisted_veh_id() const
{
	return has_assisted_vehicle() ?
		get_assisted_vehicle()->get_id() : 0;
}
double EgoVehicle::get_dest_follower_time_headway() const
{
	return controller.get_destination_lane_controller().
		get_follower_time_headway();
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

double EgoVehicle::compute_gap(const NearbyVehicle& nearby_vehicle) const
{
	/* Vissim's given "distance" is the distance between both front bumpers,
	so we subtract the length from that. We need to check which vehicle is
	ahead to determine whose length must be subtracted. */
	if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
	{
		return MAX_DISTANCE;
	}
	double offset_length;
	if (nearby_vehicle.is_ahead())
	{
		offset_length = -nearby_vehicle.get_length();
	}
	else
	{
		offset_length = get_length();
	}
	return nearby_vehicle.get_distance() + offset_length;
}

double EgoVehicle::compute_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	if (nearby_vehicle != nullptr)
	{
		return compute_gap(*nearby_vehicle);
	}
	else
	{
		return MAX_DISTANCE;
	}
}

double EgoVehicle::compute_absolute_gap(const NearbyVehicle& nearby_vehicle) const
{
	/* Vissim's given "distance" is the distance between both front bumpers,
	so we subtract the length from that. We need to check which vehicle is
	ahead to determine whose length must be subtracted. */
	return std::abs(compute_gap(nearby_vehicle));
	//if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
	//{
	//	return MAX_DISTANCE;
	//}
	//if (verbose) 
	//{
	//	std::clog << "\tveh " << nearby_vehicle.get_id()
	//		<< " is ahead? " << (nearby_vehicle.is_ahead() ? "yes" : "no")
	//		<< std::endl;
	//}
	//if (nearby_vehicle.is_ahead())
	//{
	//	return nearby_vehicle.get_distance()
	//		- nearby_vehicle.get_length();
	//}
	//else
	//{
	//	if (verbose)
	//	{
	//		std::clog << "\tnv get distance: " << nearby_vehicle.get_distance()
	//			<< std::endl;
	//	}
	//	return -nearby_vehicle.get_distance() - get_length();
	//}
}

double EgoVehicle::compute_absolute_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	if (nearby_vehicle != nullptr)
	{
		return compute_absolute_gap(*nearby_vehicle);
	}
	else
	{
		return MAX_DISTANCE;
	}
}

long EgoVehicle::get_lane_change_request() const
{
	return implement_get_lane_change_request();
}

void EgoVehicle::pass_this_to_state()
{
	state->set_ego_vehicle(this);
}

std::shared_ptr<Platoon> EgoVehicle::implement_get_platoon() const
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
	// Note: this makes the leader = nullptr
	std::shared_ptr<const NearbyVehicle> old_leader = std::move(leader);
	leader = get_nearby_vehicle_by_id(new_leader_id);
	update_leader(old_leader);
}

void EgoVehicle::find_leader()
{
	// Note: this makes the leader = nullptr
	std::shared_ptr<const NearbyVehicle> old_leader = std::move(leader);

	for (auto const& id_veh_pair : nearby_vehicles)
	{
		if (check_if_is_leader(*id_veh_pair.second)) 
		{
			leader = id_veh_pair.second;
		}
	}
	update_leader(old_leader);
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
			return true;
		}
	}
	return false;
}

void EgoVehicle::update_leader(
	std::shared_ptr<const NearbyVehicle>& old_leader)
{
	if (has_leader())
	{
		leader_id.push_back(leader->get_id());
		double new_leader_max_brake = leader->get_max_brake();
		bool is_new_leader_connected = leader->is_connected();
		if (old_leader == nullptr)
		{
			controller.activate_origin_lane_controller(get_velocity(),
				compute_current_desired_time_headway(*leader),
				is_new_leader_connected);
		}
		else if((std::abs(new_leader_max_brake
			- old_leader->get_max_brake()) > 0.5)
			|| (leader->get_type() != old_leader->get_type()))
		{
			controller.update_origin_lane_controller(
				compute_current_desired_time_headway(*leader),
				is_new_leader_connected
			);
		}
	}
	else
	{
		leader_id.push_back(0);
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
		std::clog << "t=" << get_time() << ", veh " << get_id() << "\n";
		std::clog << "Transition from ";
		if (state == nullptr) std::clog << "null";
		else std::clog << *state;
		std::clog << " to " << *new_state << std::endl;
	}
	state = std::move(new_state);
	pass_this_to_state();
}

void EgoVehicle::reset_state(
	std::unique_ptr<VehicleState> new_lane_keeping_state)
{
	// TODO: poor condition checking: hard coding variables
	if (new_lane_keeping_state->get_state_number() != 1)
	{
		std::clog << "[WARNING] t=" << get_time()
			<< ", veh " << get_id() << ": reseting vehicle state to "
			<< *new_lane_keeping_state
			<< ", which is not a lane keeping state." << std::endl;
	}

	set_lane_change_direction(RelativeLane::same);
	reset_lane_change_waiting_time();
	update_origin_lane_controller();
	update_origin_lane_controller();
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

bool EgoVehicle::is_lane_changing() const
{
	return get_active_lane_change_direction() != RelativeLane::same;
}

long EgoVehicle::get_color_by_controller_state()
{
	return controller.get_longitudinal_controller_color();
}


/* Control related methods ------------------------------------------------ */

double EgoVehicle::consider_vehicle_dynamics(double unfiltered_acceleration)
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
	this->desired_acceleration =
		implement_compute_desired_acceleration(traffic_lights);
}

//void EgoVehicle::decide_lane_change_direction()
//{
//	if (is_lane_changing())
//	{
//		lane_change_direction = get_active_lane_change_direction();
//	}
//	else if (has_lane_change_intention() && implement_check_lane_change_gaps())
//	{
//		lane_change_direction = desired_lane_change_direction;
//	}
//	else
//	{
//		//update_lane_change_waiting_time();
//		lane_change_direction = RelativeLane::same;
//	}
//}

double EgoVehicle::get_accepted_lane_change_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle)
{
	return compute_accepted_lane_change_gap(nearby_vehicle);
}

double EgoVehicle::get_reference_gap()
{
	return controller.get_reference_gap(get_velocity());
}

double EgoVehicle::compute_time_headway_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle) const
{
	double time_headway_gap = 0.0;
	if (nearby_vehicle != nullptr)
	{
		time_headway_gap = controller.get_desired_time_headway_gap(
			get_velocity(), /*has_lane_change_intention(),*/
			*nearby_vehicle);
	}
	return time_headway_gap;
}

double EgoVehicle::compute_transient_gap(
	std::shared_ptr<const NearbyVehicle> nearby_vehicle)
{
	double transient_gap = 0.0;
	if (nearby_vehicle != nullptr)
	{
		transient_gap = controller.get_lateral_controller().
			compute_transient_gap(*this, *nearby_vehicle, false);
	}
	return transient_gap;
}

void EgoVehicle::update_origin_lane_controller()
{
	if (has_leader())
	{
		controller.update_origin_lane_controller(
			compute_current_desired_time_headway(*get_leader()),
			get_leader()->is_connected()
		);
	}
}

void EgoVehicle::reset_origin_lane_velocity_controller()
{
	controller.reset_origin_lane_velocity_controller(
		get_velocity());
}

/* Computation of surrogate safety measurements --------------------------- */

double EgoVehicle::compute_ttc(const NearbyVehicle& nearby_vehicle)
{
	/* Time-to-collision is:
	(leader x - leader length - ego x) / (ego vel - leader vel),
		if ego vel > leader vel
	underfined,
		if ego vel < leader vel */
	if (nearby_vehicle.get_relative_velocity() > 0)
	{
		return compute_absolute_gap(nearby_vehicle) / nearby_vehicle.get_relative_velocity();
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
			/ 2 / compute_absolute_gap(nearby_vehicle);
	}
	return -1.0;
}

/* TODO: move to autonomous vehicle class */
//double EgoVehicle::compute_collision_severity_risk(
//	const NearbyVehicle& nearby_vehicle) const
//{
//	double current_max_brake = get_current_max_brake();
//	/* TODO: must change to get the appropriate lambda 1 */
//	double current_lambda_1 = get_lambda_1();
//
//	double jerk_delay = (comfortable_acceleration + current_max_brake) / max_jerk;
//	double ego_vel = get_velocity();
//
//	double leader_max_brake = nearby_vehicle.get_max_brake();
//	double relative_vel = nearby_vehicle.get_relative_velocity();
//	double leader_vel = nearby_vehicle.compute_velocity(ego_vel);
//
//	double gamma = leader_max_brake / current_max_brake;
//	double gamma_threshold = leader_vel / (ego_vel + current_lambda_1);
//
//	std::vector<double> gap_thresholds(4);
//	gap_thresholds[0] = brake_delay
//		* (brake_delay * (comfortable_acceleration + leader_max_brake) / 2
//			+ relative_vel);
//	gap_thresholds[1] = (brake_delay + jerk_delay)
//		* (current_lambda_1 + relative_vel
//			- (brake_delay + jerk_delay)
//			* (current_max_brake - leader_max_brake) / 2)
//		+ get_lambda_0();
//	gap_thresholds[2] = leader_vel / leader_max_brake
//		* (current_lambda_1 + relative_vel
//			- leader_vel / leader_max_brake
//			* (current_max_brake - leader_max_brake) / 2)
//		+ get_lambda_0();
//	gap_thresholds[3] = compute_collision_free_gap(nearby_vehicle);
//
//	double gap = compute_gap(nearby_vehicle);
//
//	if (verbose && (gap < gap_thresholds[3])) {
//		std::clog << "Collision prone situation\n"
//			<< "\tgamma = " << gamma << ", gamma_t = " << gamma_threshold
//			<< "\n\tgap = " << gap << ", thresholds: ";
//		for (double g : gap_thresholds) {
//			std::clog << g << ", ";
//		}
//		std::clog << std::endl;
//	}
//
//	double result = 0;
//	if (gap < gap_thresholds[0]) {
//		result = std::pow(relative_vel, 2)
//			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
//	}
//	else if (gap < gap_thresholds[1]) {
//		/* The solution for this case requires solving a 3rd degree equation.
//		To avoid that, we will approximate it as the mean of the previous
//		and following case. We will also record how often this case
//		happens to see if it's important to properly code the solution. */
//		result = std::pow(relative_vel, 2)
//			+ 2 * (comfortable_acceleration + leader_max_brake) * gap;
//		result += std::pow(relative_vel + current_lambda_1, 2)
//			+ 2 * (leader_max_brake - current_max_brake)
//			* (gap - get_lambda_0());
//		result /= 2;
//		std::clog << "t=" << get_time()
//			<< ", id=" << get_id()
//			<< ", collision severity complicated case"
//			<< std::endl;
//	}
//	else if (((gamma >= gamma_threshold) && (gap < gap_thresholds[2]))
//		|| (gamma < gamma_threshold) && (gap < gap_thresholds[3])) {
//		result = std::pow(relative_vel + current_lambda_1, 2)
//			+ 2 * (leader_max_brake - current_max_brake)
//			* (gap - get_lambda_0());
//	}
//	else if ((gamma >= gamma_threshold) && (gap < gap_thresholds[3])) {
//		result = std::pow(ego_vel + current_lambda_1, 2)
//			- 2 * current_max_brake
//			* (std::pow(leader_vel, 2) / 2 / leader_max_brake
//				+ gap - get_lambda_0());
//	}
//	result = std::sqrt(result);
//
//	if (verbose) {
//		std::clog << "\trisk = " << result << std::endl;
//	}
//
//	return result;
//}

//double EgoVehicle::compute_collision_severity_risk_to_leader()
//{
//	if (has_leader()) {
//		return compute_collision_severity_risk(*get_leader());
//	}
//	return 0.0;
//}

void EgoVehicle::set_desired_lane_change_direction()
{
	/* Both preferred_relative_lane and relative_target_lane indicate
	desire to change lanes. The former indicates preference due to
	routing, so it takes precedence over the latter. */
	RelativeLane current_preferred_lane = get_preferred_relative_lane();
	desired_lane_change_direction = RelativeLane::same;
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
	std::vector<EgoVehicle::Member> members, bool write_size)
{

	std::ostringstream oss;
	for (auto m : members)
	{
		oss << member_enum_to_string(m);
		if (write_size)
		{
			oss << " (" << get_member_size(m) << ")";
		}
		oss << ", ";
	}
	oss << std::endl;

	return oss.str();
}

std::string EgoVehicle::write_members(
	std::vector<EgoVehicle::Member> members)
{
	std::ostringstream oss;

	/* Sanity check: some non critical code mistakes could
	create vector members with different sizes. This prevents
	the log from being written and crashes vissim. Let's avoid
	this and just write a warning message instead.*/
	int n_samples = (int)velocity.size(); /* velocity, lane and link members
	are the least likely to have the wrong size */
	std::vector<int> deleted_indices;
	for (int i = 0; i < members.size(); i++)
	{
		Member m = members.at(i);
		if ((get_member_size(m) != 1) // not a scalar
			&& (get_member_size(m) != n_samples))
		{
			oss << "Error: member " << member_enum_to_string(m)
				<< " has " << get_member_size(m) << " samples "
				<< "instead of the expected " << n_samples
				<< ". It won't be printed."
				<< std::endl;
			deleted_indices.push_back(i);
		}
	}
	for (int idx : deleted_indices)
	{
		members.erase(std::next(members.begin(), idx));
	}

	// Write variables over time
	for (int i = 0; i < n_samples; i++)
	{
		for (auto m : members)
		{
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
				oss << preferred_relative_lane[i];
				break;
			case Member::velocity:
				oss << velocity[i];
				break;
			case Member::acceleration:
				oss << acceleration[i];
				break;
			//case Member::desired_acceleration:
			//	oss << desired_acceleration[i];
			//	break;
			case Member::vissim_acceleration:
				oss << vissim_acceleration[i];
				break;
			case Member::leader_id:
				oss << leader_id[i];
				break;
			/*case Member::state:
				oss << state_to_string_map.at(state_implementation_v1[i]);
				break;*/
			case Member::active_lane_change_direction:
				oss << active_lane_change_direction[i];
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

int EgoVehicle::get_member_size(Member member)
{
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
	case Member::state:
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
		return 1; // (int)desired_acceleration.size();
	case Member::vissim_acceleration:
		return (int)vissim_acceleration.size();
	case Member::leader_id:
		return (int)leader_id.size();
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


//const std::unordered_map<EgoVehicle::State, std::string>
//EgoVehicle::state_to_string_map = {
//	{ State::lane_keeping, "lane keeping" },
//	{ State::intention_to_change_lanes, "intention to LC" },
//};

std::ostream& operator<< (std::ostream& out, const EgoVehicle& ego_vehicle)
{
	out << "t=" << ego_vehicle.get_time()
		<< ", id=" << ego_vehicle.get_id()
		<< ", type=" << static_cast<int>(ego_vehicle.get_type())
		<< ", state="
		<< *(ego_vehicle.state)
		<< ", vel=" << ego_vehicle.get_velocity()
		<< ", des accel=" << ego_vehicle.get_desired_acceleration()
		<< ", accel=" << ego_vehicle.get_acceleration()
		<< ", lane=" << ego_vehicle.get_lane()
		<< "\nLane changing:" 
		<< " pref.="
		<< ego_vehicle.get_preferred_relative_lane()
		<< ", vissim suggestion="
		<< ego_vehicle.get_vissim_lane_suggestion()
		<< ", desired=" << ego_vehicle.get_desired_lane_change_direction()
		<< ", decision (our choice)="
		<< ego_vehicle.get_lane_change_direction()
		<< ", active="
		<< ego_vehicle.get_active_lane_change_direction();

	return out; // return std::ostream so we can chain calls to operator<<
}
