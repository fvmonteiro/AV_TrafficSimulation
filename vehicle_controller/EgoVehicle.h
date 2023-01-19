/*==========================================================================*/
/*  EgoVehicle.h	    													*/
/*  TODO																    */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <memory>
#include <vector>

#include "ControlManager.h"
#include "NearbyVehicle.h"
#include "TrafficLight.h"
#include "Vehicle.h"
#include "VehicleState.h"

class Platoon;

class EgoVehicle : public Vehicle
{
public:

	//enum class State {
	//	lane_keeping,
	//	intention_to_change_lanes,
	//};

	/* Constructors ---------------------------------------------------------- */
	EgoVehicle() = default;
	virtual ~EgoVehicle();

	/* Getters and setters */

	double get_sampling_interval() const { return simulation_time_step; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_comfortable_acceleration() const {
		return comfortable_acceleration;
	};
	/* Returns max_brake/2 */
	double get_lane_change_max_brake() const {
		return max_brake / 2;
	};
	double get_comfortable_brake() const { return comfortable_brake; };
	double get_max_jerk() const { return max_jerk; };
	double get_brake_delay() const { return brake_delay; };
	double get_desired_lane_angle() const { return desired_lane_angle; };
	RelativeLane get_vissim_lane_suggestion() const {
		return vissim_lane_suggestion;
	}
	/*int get_relative_target_lane() const {
		return relative_target_lane.to_int();
	};*/
	long get_turning_indicator() const { return turning_indicator; };
	double get_waiting_time() const { return lane_change_waiting_time; };
	long get_vissim_use_preferred_lane() const {
		return vissim_use_preferred_lane;
	};
	bool get_is_connected() const { return is_connected; };
	RelativeLane get_lane_change_direction() const {
		return lane_change_direction;
	}
	/* Our internal lane change decision */
	int get_lane_change_direction_to_int() const {
		return lane_change_direction.to_int(); };
	/* Proportional maximum expected relative speed */
	//double get_rho() const { return rho; };

	void set_desired_velocity(double desired_velocity) {
		this->desired_velocity = desired_velocity;
	};
	void set_number_of_lanes(long number_of_lanes) {
		this->number_of_lanes = number_of_lanes;
	};
	void set_desired_lane_angle(double desired_lane_angle) {
		this->desired_lane_angle = desired_lane_angle;
	};
	void set_turning_indicator(long turning_indicator) {
		this->turning_indicator = turning_indicator;
	};
	void set_vissim_use_preferred_lane(long value) {
		this->vissim_use_preferred_lane = value;
	};
	void set_lane_change_direction(RelativeLane relative_lane)
	{
		this->lane_change_direction = relative_lane;
	}

	/* Getters of most recent values ----------------------------------------- */

	double get_time() const;
	long get_lane() const;
	double get_distance_traveled() const;
	long get_link() const;
	double get_lateral_position() const;
	RelativeLane get_preferred_relative_lane() const;
	double get_velocity() const;
	double get_acceleration() const;
	double get_desired_acceleration() const;
	double get_vissim_acceleration() const;
	/* Returns the active lane change direction given by VISSIM */
	RelativeLane get_active_lane_change_direction() const;
	//long get_vissim_active_lane_change() const;
	double get_lane_end_distance() const;
	long get_leader_id() const;

	//State get_state_implementation_v1() const;
	double get_ttc() const;
	double get_drac() const;
	/* delta vel. at collision under the worst case scenario*/
	double get_collision_risk() const;

	/* Other getters and setters --------------------------------------------- */

	/* Returns the desired velocity or the max road velocity */
	double get_free_flow_velocity() const;

	std::shared_ptr<Platoon> get_platoon() const
	{
		return implement_get_platoon();
	};
	
	double get_safe_time_headway() const;
	double get_gap_error() const;
	double get_current_desired_time_headway() const;
	/* Returns a nullptr if there is no leader at the destination lane */
	std::shared_ptr<NearbyVehicle> get_destination_lane_leader() const {
		return implement_get_destination_lane_leader();
	};
	/* Returns a nullptr if there is no follower at the destination lane */
	std::shared_ptr<NearbyVehicle> get_destination_lane_follower() const {
		return implement_get_destination_lane_follower();
	};
	/* Returns a nullptr if there is no assisted vehicle */
	std::shared_ptr<NearbyVehicle> get_assisted_vehicle() const {
		return implement_get_assisted_vehicle();
	};
	double get_gap_variation_to(
		const std::shared_ptr<NearbyVehicle> nearby_vehicle) const;
	double get_collision_free_gap_to(
		const std::shared_ptr<NearbyVehicle> nearby_vehicle) const;
	const VehicleState* get_state() const;

	void set_lane(long lane);
	void set_distance_traveled(double distance_traveled);
	void set_link(long link);
	void set_lateral_position(double lateral_position);
	void set_velocity(double velocity);
	void set_acceleration(double acceleration);
	void set_vissim_acceleration(double vissim_acceleration);
	/* Sets the active lane change direction given by VISSIM */
	void set_active_lane_change_direction(long direction);
	/*void set_vissim_active_lane_change(int active_lane_change);*/
	/* Mandatory/route related lane changes */
	void set_preferred_relative_lane(long preferred_relative_lane);
	/* Discretionary lane changes */
	//void set_relative_target_lane(long target_relative_lane);
	/* VISSIM lane change suggestion */
	void set_vissim_lane_suggestion(long target_relative_lane);
	void set_lane_end_distance(double lane_end_distance,
		long lane_number);
	void read_traffic_light(int traffic_light_id, double distance)
	{
		set_traffic_light_information(traffic_light_id, distance);
	}
	void set_max_lane_change_risk_to_leaders(double value)
	{
		implement_set_accepted_lane_change_risk_to_leaders(value);
	}
	void set_max_lane_change_risk_to_follower(double value)
	{
		implement_set_accepted_lane_change_risk_to_follower(value);
	}
	void set_use_linear_lane_change_gap(long value) {
		implement_set_use_linear_lane_change_gap(value);
	}

	/* Dealing with nearby vehicles --------------------------------------- */

	/* Clears the vector of pointers and the individually named pointers */
	void clear_nearby_vehicles();
	/* Creates an instance of nearby vehicle and populates it with
	the given data. */
	void emplace_nearby_vehicle(long id, long relative_lane,
		long relative_position);
	/* Returns the most recently added nearby vehicle */
	std::shared_ptr<NearbyVehicle> peek_nearby_vehicles() const;
	/* Sets the nearby vehicle if both the ego and nearby vehicles are connected.
	Otherwise, set the type as unknown. */
	void set_nearby_vehicle_type(long type);
	/* Looks at all nearby vehicles to find the relevant ones, such
	* as the leader. */
	void analyze_nearby_vehicles() { implement_analyze_nearby_vehicles(); };
	/* Returns true if a new platoon was created. */
	bool analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy)
	{
		return implement_analyze_platoons(platoons, new_platoon_id,
			platoon_lc_strategy);
	};
	//bool is_cutting_in(const NearbyVehicle& nearby_vehicle) const;
	bool has_leader() const;
	double get_time_headway_to_assisted_vehicle() const;
	/* Returns a nullptr if there is no leader */
	std::shared_ptr<NearbyVehicle> get_leader() const;
	std::shared_ptr<NearbyVehicle> get_nearby_vehicle_by_id(long nv_id) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is empty. */
	double compute_gap(const NearbyVehicle& nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	* Returns MAX_DISTANCE if nearby_vehicle is a nullptr. */
	double compute_gap(
		const std::shared_ptr<NearbyVehicle> nearby_vehicle) const;
	/* Ego velocity minus leader velocity. Returns zero if there
	* is no leader */
	double get_relative_velocity_to_leader();
	/* The lane change request is an int whose absolute value equals the
	vehicle's id. The signal of the lane change request indicates whether
	it is a right (-1) or left (+1) lane change. Only connected vehicles
	can create a lane change request*/
	long get_lane_change_request() const;

	bool has_destination_lane_leader() const;
	bool has_destination_lane_follower() const;
	bool has_assisted_vehicle() const;

	bool is_in_a_platoon() const;
	long get_platoon_id() const;

	/* Methods to debug nearby vehicles information */

	long get_dest_lane_leader_id() const;
	long get_dest_lane_follower_id() const;
	long get_assisted_veh_id() const;
	double get_dest_follower_time_headway() const;

	bool has_next_traffic_light() const {
		return implement_has_next_traffic_light();
	};

	/* Computation of surrogate safety measurements ----------------------- */

	double compute_ttc(const NearbyVehicle& nearby_vehicle);
	double compute_drac(const NearbyVehicle& nearby_vehicle);
	/* Relative velocity at collision time under the worst case scenario
	TODO: move to autonomous vehicle class */
	/*double compute_collision_severity_risk(
		const NearbyVehicle& nearby_vehicle) const;*/
	//double compute_collision_severity_risk_to_leader();

	/* State-machine related methods ----------------------------------------- */

	void update_state();
	void set_state(std::unique_ptr<VehicleState> new_state);
	//bool has_lane_change_intention() const;
	bool is_lane_changing() const override;
	//State get_previous_state() const;
	/* Returns the color equivalent to the current state as a long */
	long get_color_by_controller_state();
	//std::string print_detailed_state() const;
	/* If the lane change decision is autonomous, but the vehicle takes
	too long to find a suitable gap, we may want to give control to VISSM */
	bool is_vissim_controlling_lane_change() const
	{
		return give_lane_change_control_to_vissim();
	};
	/* Updates the stopped time waiting for lane change */
	void update_lane_change_waiting_time();
	void reset_lane_change_waiting_time();
	bool check_lane_change_gaps();

	/* Control related methods ----------------------------------------------- */

	void compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights);
	//void decide_lane_change_direction();

	double get_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle);

	/*double compute_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle, double accepted_risk);*/
	/* Methods to access internal values. Used for quicker debugging --------- */

	/* Returns the current reference gap to the leader */
	double get_reference_gap();
	/* Computes the desired time headway based on lane change intention
	TODO [Nov 17]: double check if it needs to be public*/
	double compute_current_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const;
	/* Returns the time headway gap from the ego to the other vehicle
	if the ego vehicle is behind and from the other to the ego vehicle
	if the other vehicle is behind. */
	double compute_time_headway_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle);
	/* Returns the transient lane changing gap between ego vehicle
	and other. */
	double compute_transient_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle);
	void update_origin_lane_controller();
	void reset_origin_lane_velocity_controller();

	/* Methods for logging --------------------------------------------------- */
	bool is_verbose() const { return verbose; };

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out,
		const EgoVehicle& vehicle);

protected:
	ControlManager controller;
	/* Keeps track of stopped time waiting for lane change */
	double lane_change_waiting_time{ 0.0 };
	std::vector<std::shared_ptr<NearbyVehicle>> nearby_vehicles;
	std::unique_ptr<VehicleState> state{ nullptr };
	/* Determines whether the vel control ref speed is the vehicle's
	own desired speed or the max legal velocity. */
	bool try_go_at_max_vel{ false };

	bool verbose = false; /* when true, will print results to
						  the default log file and
						  create a specific log file for this
						  vehicle */

	EgoVehicle(long id, VehicleType type, double desired_velocity,
		double brake_delay, bool is_lane_change_autonomous, bool is_connected,
		double simulation_time_step, double creation_time, bool verbose);

	virtual double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const;

	double get_rho() const { return rho; };
	/* Checks whether vehicle is lane changing and returns proper value
	TODO: move to autonomous vehicle class */
	double get_current_max_brake() const;
	void set_gap_variation_during_lane_change(int nv_id, double value);
	void set_collision_free_gap(int nv_id, double value);

	/* Takes the desired acceleration given by the controller and
	returns the feasible acceleration given the approximated low level
	dynamics */
	double consider_vehicle_dynamics(double unfiltered_acceleration);
	void set_leader_by_id(long new_leader_id);
	void find_leader();
	
private:
	/* Computes the longitudinal controller input */
	virtual double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) = 0;
	virtual bool give_lane_change_control_to_vissim() const = 0;
	/* Decides whether the vehicle can start a
	lane change. Returns -1 for right lane changes, +1 for left lane
	changes and 0 for lane keeping. */
	virtual bool implement_check_lane_change_gaps() = 0;
	virtual long create_lane_change_request() const = 0;
	virtual void set_traffic_light_information(int traffic_light_id,
		double distance) {};
	virtual bool implement_has_next_traffic_light() const { return false; };
	//virtual void compute_lane_change_risks() {};
	virtual double compute_accepted_lane_change_gap(
		std::shared_ptr<NearbyVehicle> nearby_vehicle) = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_assisted_vehicle() const = 0;
	virtual void implement_set_accepted_lane_change_risk_to_leaders(
		double value) = 0;
	virtual void implement_set_accepted_lane_change_risk_to_follower(
		double value) = 0;
	virtual void implement_set_use_linear_lane_change_gap(long value) = 0;
	
	virtual void pass_this_to_state();
	// TODO: should this be abstract?
	virtual std::shared_ptr<Platoon> implement_get_platoon() const;

	/* Finds the current leader */
	virtual void implement_analyze_nearby_vehicles();
	virtual bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy) {
		return false;
	};
	virtual void set_desired_lane_change_direction();

	virtual double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const = 0;

	bool check_if_is_leader(const NearbyVehicle& nearby_vehicle) const;
	void update_leader(const std::shared_ptr<NearbyVehicle>& old_leader);

	/* Estimated parameters used for safe gap computations (no direct
	equivalent in VISSIM's simulation dynamics) --------------------------- */

	double tau{ ACTUATOR_CONSTANT }; // actuator constant [s].
	double rho{ 0.2 }; // proportional maximum expected relative speed
	/* constant used in the discrete approximation of
	the vehicle first order actuator dynamics */
	double tau_d{ 0.0 };
	double comfortable_brake{ COMFORTABLE_BRAKE }; // [m/s^2]

	std::shared_ptr<NearbyVehicle> leader{ nullptr };
	std::vector<long> leader_id;

	/* Data obtained from VISSIM or generated by internal computations ---- */
	double creation_time{ 0.0 };
	double current_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	long color{ 0 };
	double desired_velocity{ 0 }; /* from VISSIM's desired
								  velocity distribution */
	std::vector<long> lane;
	std::vector<double> distance_traveled;
	std::vector<long> link;
	long number_of_lanes{ 0 };
	std::vector<RelativeLane> preferred_relative_lane;
	/* 0 = only preferable (e.g. European highway)
	   1 = necessary (e.g. before a connector)     */
	long vissim_use_preferred_lane{ 0 };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	std::vector<double> lateral_position;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	double desired_acceleration{ 0.0 };
	/* VISSIM suggested acceleration */
	std::vector<double> vissim_acceleration;
	/* Value of lane change determined by internal algorithm
	+1 = to the left, 0 = none, -1 = to the right */
	RelativeLane lane_change_direction{ RelativeLane::same };
	/* Value of active lane change in VISSIM:
	+1 = to the left, 0 = none, -1 = to the right */
	std::vector<RelativeLane> active_lane_change_direction;
	/* Determines if we use our lane change decision model or VISSIM's */
	bool is_lane_change_autonomous{ true };
	bool is_connected{ false };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	std::vector<double> lane_end_distance;
	double desired_lane_angle{ 0.0 };
	RelativeLane vissim_lane_suggestion{ RelativeLane::same };
	long turning_indicator{ 0 };

	/* Safe lane change decision parameters ------------------------------- */
	/* Variables are only needed if we want to be able to see values in
	VISSIM. Otherwise there is no need to save these values.
	[May 6, 2022] Not being used. The choice is to keep measuring safety based
	only on the time headway */

	std::unordered_map<int, double> gap_variation_during_lane_change;
	std::unordered_map<int, double> collision_free_gap;

	/*Surrogate Safety Measurements (SSMs) ----------------------------------- */

	std::vector<double> ttc; // time-to-collision
	std::vector<double> drac; // deceleration rate to avoid collision
	std::vector<double> collision_severity_risk; /* delta vel. at collision
												 in worst case scenario */

	/* For printing and debugging purporses ------------------------------- */
	//static const std::unordered_map<State, std::string> state_to_string_map;

	std::string log_path = "autonomous_vehicle_logs";
	enum class Member {
		creation_time,
		id,
		length,
		width,
		color,
		category,
		desired_velocity,
		lane,
		link,
		preferred_relative_lane,
		velocity,
		acceleration,
		desired_acceleration,
		vissim_acceleration,
		leader_id,
		state,
		active_lane_change_direction,
		lane_end_distance,
		ttc,
		drac,
		collision_severity_risk,
		type,
	};

	void write_simulation_log(std::vector<Member> members);
	std::string write_header(std::vector<Member> members,
		bool write_size = false);
	std::string write_members(std::vector<Member> members);
	int get_member_size(Member member);
	std::string member_enum_to_string(Member member);
};
