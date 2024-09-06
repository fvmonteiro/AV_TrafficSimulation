/*==========================================================================*/
/*  EgoVehicle.h	    													*/
/*  TODO																    */
/*                                                                          */
/*  Version of xxxx-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <memory>
#include <vector>

#include "VehicleController.h"
#include "NearbyVehicle.h"
#include "TrafficLight.h"
#include "Vehicle.h"
#include "VehicleState.h"

class Platoon;

class EgoVehicle : public Vehicle
{
public:
	/* Constructors ------------------------------------------------------- */
	EgoVehicle() = default;
	virtual ~EgoVehicle();

	/* Getters and setters ------------------------------------------------ */

	double get_simulation_time_step() const { return simulation_time_step; };
	double get_current_time() const { return current_time; };
	double get_desired_velocity() const { return desired_velocity; };
	long get_lane() const { return lane; };
	double get_distance_traveled() const { return distance_traveled; };
	long get_link() const { return link; };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	double get_lateral_position() const { return lateral_position; }
	RelativeLane get_preferred_relative_lane() const {
		return preferred_relative_lane;
	};
	double get_velocity() const { return velocity; };
	double get_acceleration() const { return acceleration; };
	double get_desired_acceleration() const { return desired_acceleration; };
	double get_vissim_acceleration() const { return vissim_acceleration; };
	RelativeLane get_active_lane_change_direction() const {
		return active_lane_change_direction;
	};
	double get_lane_end_distance() const { return lane_end_distance; };
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
	bool get_has_completed_lane_change() const {
		return has_completed_lane_change;
	};
	double get_ttc() const { return ttc; }
	double get_drac() const { return drac; }
	/* delta vel. at collision under the worst case scenario */
	double get_collision_risk() const { return collision_severity_risk; }

	void set_desired_velocity(double value) { desired_velocity = value; };
	void set_lane(long value) { lane = value; };
	void set_distance_traveled(double value) { distance_traveled = value; };
	void set_link(long value) { link = value; };
	void set_lateral_position(double value) { lateral_position = value; };
	void set_velocity(double value) { velocity = value; };
	void set_acceleration(double value) { acceleration = value; };
	void set_vissim_acceleration(double value) {
		vissim_acceleration = value;
	};
	void set_number_of_lanes(long value) {number_of_lanes = value; };
	void set_desired_lane_angle(double value) { desired_lane_angle = value; };
	void set_turning_indicator(long value) { turning_indicator = value; };
	void set_vissim_use_preferred_lane(long value) {
		vissim_use_preferred_lane = value;
	};
	void set_lane_change_direction(RelativeLane value) {
		this->lane_change_direction = value;
	};

	/* Non-trivial Getters and setters ---------------------------------------- */

	/* Returns zero if no leader */
	long get_leader_id() const;
	long get_old_leader_id() const;
	const Platoon* get_platoon() const;
	std::shared_ptr<Platoon> share_platoon() const;
	double get_safe_time_headway() const;
	/* Gap error (gap minus reference gap) of active longitudinal
	controller */
	double get_gap_error() const;
	double get_current_desired_time_headway() const;
	/* Returns a nullptr if there is no leader at the destination lane */
	const std::shared_ptr<NearbyVehicle> get_destination_lane_leader() const {
		return implement_get_destination_lane_leader();
	};
	/* Returns a nullptr if there is no follower at the destination lane */
	const std::shared_ptr<NearbyVehicle> get_destination_lane_follower() 
		const { return implement_get_destination_lane_follower();
	};
	/* Returns a nullptr if there is no assisted vehicle */
	const std::shared_ptr<NearbyVehicle> get_assisted_vehicle() const {
		return implement_get_assisted_vehicle();
	};
	double get_gap_variation_to(
		const NearbyVehicle* nearby_vehicle) const;
	double get_collision_free_gap_to(
		const NearbyVehicle* nearby_vehicle) const;
	/* Returns the state of the Finite State Machine */
	const VehicleState* get_state() const;
	/* Lateral position w.r.t. the right boundary of right-most lane */
	double get_road_reference_lateral_position() const;
	/* Returns distance traveled, current lane, orientation angle,
	and speed */
	ContinuousStateVector get_state_vector() const;

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
	void set_verbose(bool value);
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

	/* Getters for visual debuggin in VISSIM ------------------------------ */
	/* Most of these could be included only in the derived concrete classes,   
	but then we wouldn't be able to visualize them in the simulator
	*/
	virtual double get_dest_follower_time_headway() const {
		return -1.0;
	};
	virtual double get_time_headway_to_assisted_vehicle() const { 
		return -1.0; 
	};
	/* Returns the transient lane changing gap between ego vehicle
	and other. */
	virtual double compute_transient_gap(const NearbyVehicle* nearby_vehicle) {
		return -1.0;
	};

	/* Dealing with nearby vehicles --------------------------------------- */

	/* Clears the vector of pointers and the individually named pointers */
	void clear_nearby_vehicles();
	/* Creates an instance of nearby vehicle and populates it with
	the given data. */
	void emplace_nearby_vehicle(long nv_id, long relative_lane,
		long relative_position);
	void add_nearby_vehicle(NearbyVehicle nearby_vehicle);
	/* Returns the most recently added nearby vehicle */
	//std::shared_ptr<NearbyVehicle> peek_nearby_vehicles() const;
	/* Sets the nearby vehicle if both the ego and nearby vehicles are connected.
	Otherwise, set the type as unknown. */
	void set_nearby_vehicle_type(long nv_id, long type);
	/* Looks at all nearby vehicles to find the relevant ones, such
	* as the leader. */
	void analyze_nearby_vehicles() { implement_analyze_nearby_vehicles(); };
	/* Returns true if a new platoon was created. */
	bool analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy, float max_computation_time)
	{
		return implement_analyze_platoons(platoons, new_platoon_id,
			platoon_lc_strategy, max_computation_time);
	};
	bool has_leader() const;
	/* Returns a nullptr if there is no leader */
	std::shared_ptr<NearbyVehicle> get_leader() const;
	/* Returns a nullptr if vehicle not found */
	std::shared_ptr<NearbyVehicle> get_nearby_vehicle_by_id(long nv_id) const;
	/* Returns an empty state vector if nv_id not found */
	ContinuousStateVector get_nearby_vehicle_relative_states(long nv_id) const;
	/* Returns a state vector at MAX DIST, desired velocity and "expected_lane",
	if nv_id not found leader */
	ContinuousStateVector get_nearby_vehicle_relative_states(long nv_id,
		long expected_lane) const;
	
	double compute_nearby_vehicle_velocity(
		const NearbyVehicle& nearby_vehicle) const;
	/* Returns the nearby vehicle's velocity if the pointer is not null.
	Otherwise, returns the default value. */
	double compute_nearby_vehicle_velocity(
		const NearbyVehicle* nearby_vehicle, double default_value) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is empty. */
	double compute_gap_to_a_leader(const NearbyVehicle& nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is a nullptr. */
	double compute_gap_to_a_leader(const NearbyVehicle* nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance (absolute value) between
	vehicles. Returns MAX_DISTANCE if nearby_vehicle is empty. */
	double compute_gap_to_a_follower(
		const NearbyVehicle& nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance (absolute value) between 
	vehicles. Returns MAX_DISTANCE if nearby_vehicle is a nullptr. */
	double compute_gap_to_a_follower(
		const NearbyVehicle* nearby_vehicle) const;
	/* 'Exact' nonlinear safe gap. Returns 0 if there is no leader */
	double compute_safe_gap_to_leader();

	/* Ego velocity minus leader velocity. Returns zero if there
	* is no leader */
	double get_relative_velocity_to_leader();
	/* The lane change request is the id of the vehicle in front of which
	the lane changing vehicle wants to merge. Only connected vehicles
	can create a lane change request*/
	long get_lane_change_request() const;

	/* Checks if the given id is in the ego vehicle's nearby
	vehicles map. */
	bool is_vehicle_in_sight(long nearby_vehicle_id) const;
	bool has_destination_lane_leader() const;
	bool has_destination_lane_follower() const;
	bool has_assisted_vehicle() const;


	bool is_in_a_platoon() const;
	long get_platoon_id() const;

	/* Methods to debug nearby vehicles information */

	// Returns zero if no dest lane leader
	long get_destination_lane_leader_id() const;
	// Returns zero if no dest lane follower
	long get_destination_lane_follower_id() const;
	// Returns zero if no assisted vehicle
	long get_assisted_veh_id() const;
	// Returns zero if no virtual leader
	long get_virtual_leader_id() const;

	bool has_next_traffic_light() const {
		return implement_has_next_traffic_light();
	};

	/* Computation of surrogate safety measurements ----------------------- */
	/* 'Exact' nonlinear safe gap */
	double compute_safe_gap_to_a_leader(const NearbyVehicle& a_leader) const;
	double compute_risky_gap_to_leader(const NearbyVehicle& a_leader, 
		double accepted_risk) const;
	
	double compute_ttc(const NearbyVehicle& nearby_vehicle);
	double compute_drac(const NearbyVehicle& nearby_vehicle);

	/* State-machine related methods ----------------------------------------- */

	void update_state();
	void set_state(std::unique_ptr<VehicleState> new_state);
	/* Sets the new state (must be a lane keeping state) and resets the 
	desired lane change direction and the longitudinal controllers. */
	void reset_state(std::unique_ptr<VehicleState> new_lane_keeping_state);
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
	void prepare_to_start_long_adjustments();
	void prepare_to_restart_lane_keeping(bool was_lane_change_successful);

	/* Control related methods ----------------------------------------------- */
	void create_controller() { implement_create_controller(); };

	void compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	double get_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle);

	/* Methods to access internal values. Used for quicker debugging --------- */

	/* Returns the current reference gap to the leader */
	double get_reference_gap() const;
	/* Computes the desired time headway based on lane change intention
	TODO [Nov 17]: double check if it needs to be public*/
	double compute_current_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const;
	/* Returns the desired time headway gap between the ego vehicle and the 
	nearby vehicle based on their relative positions. */
	double compute_time_headway_gap(
		const NearbyVehicle* nearby_vehicle) const;
	/* Sets the desired time headway of the origin lane controller based 
	on the vehicle's current intention and current maneuver */
	void update_time_headway_to_leader();
	///* Sets the desired time headway of the origin lane controller to the 
	//lane-changing time headway value */
	//void increase_time_headway_to_leader();
	///* Sets the desired time headway of the origin lane controller to the
	//lane-keeping time headway value */
	//void decrease_time_headway_to_leader();
	void reset_origin_lane_velocity_controller();

	/* Methods for logging --------------------------------------------------- */

	bool is_verbose() const { return verbose; };

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out,
		const EgoVehicle& vehicle);

protected:
	bool verbose{ false }; /* when true, will print results to
						  the default log file and
						  create a specific log file for this
						  vehicle */

	/* Keeps track of stopped time waiting for lane change */
	double lane_change_waiting_time{ 0.0 };
	std::unique_ptr<VehicleState> state{ nullptr };

	EgoVehicle(long id, VehicleType type, double desired_velocity,
		double brake_delay, bool is_lane_change_autonomous, bool is_connected,
		double simulation_time_step, double creation_time, bool verbose);

	virtual double compute_vehicle_following_safe_time_headway(
		const NearbyVehicle& nearby_vehicle) const;
	void compute_lane_change_gap_parameters();

	/* Proportional maximum expected relative speed */
	double get_rho() const { return rho; };
	double get_lambda_0_lane_change() const { return lambda_0_lane_change; };
	double get_lambda_1_lane_change() const { return lambda_1_lane_change; };
	/* Gets the max brake value based on whether the vehicle is lane 
	changing */
	double get_current_max_brake() const;
	/* Gets parameters lambda 0 and lambda 1 based on whether the vehicle 
	is lane changing */
	std::pair<double, double> get_current_safe_gap_parameters() const;
	std::pair<double, double> get_lane_changing_safe_gap_parameters() const;
	const std::unordered_map<long, std::shared_ptr<NearbyVehicle>>
		get_nearby_vehicles() const;
	void set_controller(std::shared_ptr<VehicleController> controller);
	void set_has_completed_lane_change(bool value);
	void set_gap_variation_during_lane_change(int nv_id, double value);
	void set_collision_free_gap(int nv_id, double value);

	void set_leader_by_id(long new_leader_id);
	void find_leader();
	bool check_if_is_leader(const NearbyVehicle& nearby_vehicle) const;
	bool is_destination_lane_follower(
		const NearbyVehicle& nearby_vehicle);
	bool is_destination_lane_leader(const NearbyVehicle& nearby_vehicle);
	bool is_leader_of_destination_lane_leader(
		const NearbyVehicle& nearby_vehicle);
	
private:
	/* Estimated parameters used for safe gap computations (no direct
	equivalent in VISSIM's simulation dynamics) --------------------------- */

	double tau{ ACTUATOR_CONSTANT }; // actuator constant [s].
	double rho{ 0.2 }; // proportional maximum expected relative speed
	/* constant used in the discrete approximation of
	the vehicle first order actuator dynamics */
	double tau_d{ 0.0 };
	double comfortable_brake{ COMFORTABLE_BRAKE }; // [m/s^2]
	/* Emergency braking parameter during lane change */
	double lambda_1_lane_change{ 0.0 }; // [m/s]
	/* Emergency braking parameter during lane change */
	double lambda_0_lane_change{ 0.0 }; // [m/s]
	std::unordered_map<long, std::shared_ptr<NearbyVehicle>> nearby_vehicles;
	std::shared_ptr<NearbyVehicle> leader{ nullptr };
	long old_leader_id{ 0 };

	/* Data obtained from VISSIM or generated by internal computations ---- */
	double creation_time{ 0.0 };
	double current_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	double desired_velocity{ 0.0 }; /* from VISSIM's desired
								  velocity distribution */
	long lane{ 0 };
	double distance_traveled{ 0.0 };
	long link{ 0 };
	long number_of_lanes{ 0 };
	RelativeLane preferred_relative_lane{ RelativeLane::same };
	/* 0 = only preferable (e.g. European highway)
	   1 = necessary (e.g. before a connector)     */
	long vissim_use_preferred_lane{ 0 };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	double lateral_position{ 0.0 };
	double velocity{ 0.0 };
	double acceleration{ 0.0 };
	double desired_acceleration{ 0.0 };
	/* VISSIM suggested acceleration */
	double vissim_acceleration{ 0.0 };
	/* Value of lane change determined by our internal decision method
	+1 = to the left, 0 = none, -1 = to the right */
	RelativeLane lane_change_direction{ RelativeLane::same };
	/* Value of active lane change in VISSIM:
	+1 = to the left, 0 = none, -1 = to the right */
	RelativeLane active_lane_change_direction{ RelativeLane::same };
	/* Becomes true after a lane change and is set to false again
	every time the vehicle has another lane change intention. */
	bool has_completed_lane_change{ false };
	/* Determines if we use our lane change decision model or VISSIM's */
	bool is_lane_change_autonomous{ true };
	bool is_connected{ false };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	double lane_end_distance{ 0.0 };
	double desired_lane_angle{ 0.0 };
	RelativeLane vissim_lane_suggestion{ RelativeLane::same };
	long turning_indicator{ 0 };

	/* Controller --------------------------------------------------------- */
	std::shared_ptr<VehicleController> controller{ nullptr };

	/* Safe lane change decision parameters ------------------------------- */
	/* Variables are only needed if we want to be able to see values in
	VISSIM. Otherwise there is no need to save these values.
	[May 6, 2022] Not being used. The choice is to keep measuring safety based
	only on the time headway */

	std::unordered_map<int, double> gap_variation_during_lane_change;
	std::unordered_map<int, double> collision_free_gap;

	/*Surrogate Safety Measurements (SSMs) -------------------------------- */

	double ttc{ 0.0 }; // time-to-collision
	double drac; // deceleration rate to avoid collision
	double collision_severity_risk; /* delta vel. at collision
												 in worst case scenario */

	/* Methods ------------------------------------------------------------ */

	virtual void implement_create_controller() = 0;
	/* Computes the longitudinal controller input */
	/*virtual double implement_compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) = 0;*/
	virtual bool give_lane_change_control_to_vissim() const = 0;
	/* Decides whether the vehicle can start a
	lane change. Returns -1 for right lane changes, +1 for left lane
	changes and 0 for lane keeping. */
	virtual bool implement_check_lane_change_gaps() = 0;
	virtual long implement_get_lane_change_request() const = 0;
	virtual void set_traffic_light_information(int traffic_light_id,
		double distance) {};
	virtual bool implement_has_next_traffic_light() const { return false; };
	//virtual void compute_lane_change_risks() {};
	/* Min gap toward a nearby vehicle assuming the ego vehicle will
	change lanes at lane_change_speed. */
	virtual double compute_accepted_lane_change_gap(
		const NearbyVehicle* nearby_vehicle, double lane_change_speed
	) const = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_leader() const = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_destination_lane_follower() const = 0;
	virtual std::shared_ptr<NearbyVehicle>
		implement_get_assisted_vehicle() const = 0;
	virtual long implement_get_virtual_leader_id() const = 0;
	virtual void implement_set_accepted_lane_change_risk_to_leaders(
		double value) = 0;
	virtual void implement_set_accepted_lane_change_risk_to_follower(
		double value) = 0;
	virtual void implement_set_use_linear_lane_change_gap(long value) = 0;
	virtual void implement_prepare_to_start_long_adjustments() = 0;
	//virtual void implement_prepare_to_start_lane_change() = 0;
	virtual void implement_prepare_to_restart_lane_keeping(
		bool was_lane_change_successful) = 0;
	virtual bool implement_analyze_platoons(
		std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
		long new_platoon_id, int platoon_lc_strategy, 
		double max_computation_time) = 0;
	/* Takes the desired acceleration given by the controller and
	returns the feasible acceleration given the approximated low level
	dynamics */
	virtual double apply_low_level_dynamics(double unfiltered_acceleration);
	virtual void pass_this_to_state();
	virtual const Platoon* implement_get_platoon() const;
	virtual std::shared_ptr<Platoon> implement_share_platoon() const;

	/* Finds the current leader */
	virtual void implement_analyze_nearby_vehicles();
	virtual void set_desired_lane_change_direction();

	virtual double compute_lane_changing_desired_time_headway(
		const NearbyVehicle& nearby_vehicle) const = 0;

	void update_leader(const NearbyVehicle* old_leader);

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

	//void write_simulation_log(std::vector<Member> members);
	/*std::string write_header(std::vector<Member> members,
		bool write_size = false);*/
	//std::string write_members(std::vector<Member> members);
	//int get_member_size(Member member);
	std::string member_enum_to_string(Member member);
};
