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
#include "Vehicle.h"



class EgoVehicle : public Vehicle {
public:
	//using Vehicle::set_category;

	enum class State {
		lane_keeping,
		intention_to_change_lanes,
	};

	/* Constructors */
	EgoVehicle() = default;
	EgoVehicle(long id, double simulation_time_step, double creation_time,
		bool verbose);
	EgoVehicle(long id, double simulation_time_step, double creation_time);
	~EgoVehicle();


	/* Getters and setters */

	double get_sampling_interval() const { return simulation_time_step; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_comfortable_acceleration() const { 
		return comfortable_acceleration;
	};
	double get_lane_change_max_brake() const { 
		return max_brake / 2;
	};
	double get_comfortable_brake() const { return comfortable_brake; };
	double get_max_jerk() const { return max_jerk; };
	double get_brake_delay() const { return brake_delay; };
	// double get_lambda_0() const { return lambda_0; };
	double get_lambda_1() const { return lambda_1; };
	double get_lambda_1_lane_change() const { return lambda_1_lane_change; };
	bool get_use_internal_lane_change_decision() const {
		return use_internal_lane_change_decision;
	};
	/*double get_adjustment_speed_factor() const {
		return adjustment_speed_factor;
	};*/
	double get_desired_lane_angle() const { return desired_lane_angle; };
	RelativeLane get_rel_target_lane() const { 
		return relative_target_lane;
	};
	long get_turning_indicator() const { return turning_indicator; };
	/* If the ego vehicle is not connected, returns lambda_1 */
	double get_lambda_1_connected() const {
		return is_connected() ? lambda_1_connected : lambda_1;
	};

	void EgoVehicle::set_desired_velocity(double desired_velocity) {
		this->desired_velocity = desired_velocity;
	};
	//void set_color(long color) { this->color = color; };
	void set_use_internal_lane_change_decision(long use) {
		this->use_internal_lane_change_decision = use > 0;
	};
	void set_desired_lane_angle(double desired_lane_angle) {
		this->desired_lane_angle = desired_lane_angle;
	};
	void set_turning_indicator(long turning_indicator) {
		this->turning_indicator = turning_indicator;
	};
	void set_verbose(long value) {
		this->verbose = value > 0;
	};

	/* Getters of most recent values */
	
	double get_time() const;
	long get_lane() const;
	long get_link() const;
	double get_lateral_position() const;
	RelativeLane get_preferred_relative_lane() const;
	double get_velocity() const;
	double get_acceleration() const;
	double get_desired_acceleration() const;
	double get_vissim_acceleration() const;
	RelativeLane get_active_lane_change_direction() const;
	long get_vissim_active_lane_change() const;
	double get_lane_end_distance() const;
	long get_leader_id() const;
	State get_state() const;
	double get_ttc() const;
	double get_drac() const;
	/* delta vel. at collision under the worst case scenario*/
	double get_collision_risk() const;

	/* Other getters and setters */
	VehicleParameters get_static_parameters();

	void set_lane(long lane);
	void set_link(long link);
	void set_lateral_position(double lateral_position);
	void set_velocity(double velocity);
	void set_acceleration(double acceleration);
	void set_vissim_acceleration(double vissim_acceleration);
	void set_active_lane_change_direction(long direction);
	void set_vissim_active_lane_change(int active_lane_change);
	/* Also sets the estimated maximum braking of the
	vehicle. */
	//void set_category(VehicleCategory category) override;
	void set_type(long type) /*override*/;
	/* Mandatory/route related lane changes */
	void set_preferred_relative_lane(long preferred_relative_lane);
	/* Discretionary lane changes */
	void set_rel_target_lane(long target_relative_lane);
	void set_lane_end_distance(double lane_end_distance,
		long lane_number);


	/* Dealing with nearby vehicles */
	
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
	//void set_nearby_vehicle_lc_intention(long relative_lane);
	/* Searches the nearby_vehicles array. NO LONGER IN USE */
	//std::shared_ptr<NearbyVehicle> find_nearby_vehicle(RelativeLane relative_lane,
	//	int relative_position) const;
	/* Looks at all nearby vehicles, defines pointers to the leader 
	(if it exits), and destination lane leader and follower (if there 
	is lane change intention and if they exist). Also performs time 
	computations */
	void analyze_nearby_vehicles();
	//bool is_cutting_in(const NearbyVehicle& nearby_vehicle) const;
	bool has_leader() const;
	bool has_follower() const;
	bool has_destination_lane_leader() const;
	bool has_destination_lane_follower() const;
	/* Returns a nullptr if there is no leader */
	std::shared_ptr<NearbyVehicle> get_leader() const;
	/* Returns a nullptr if there is no follower */
	std::shared_ptr<NearbyVehicle> get_follower() const;
	/* Returns a nullptr if there is no leader at the destination lane */
	std::shared_ptr<NearbyVehicle> get_destination_lane_leader() const;
	/* Returns a nullptr if there is no follower at the destination lane */
	std::shared_ptr<NearbyVehicle> get_destination_lane_follower() const;
	/* Returns a nullptr if the ego is no vehicle being assisted 
	(for gap generation) */
	std::shared_ptr<NearbyVehicle> get_assisted_vehicle() const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is empty. */
	double compute_gap(const NearbyVehicle& nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is a nullptr. */
	double compute_gap(
		const std::shared_ptr<NearbyVehicle> nearby_vehicle) const;
	bool has_lane_change_conflict() const;
	bool is_cooperating_to_generate_gap() const;
	
	/* The following two methods are useful to avoid postprocessing
	the Vehicle Record generated by VISSIM:
	- The relative velocity that VISSIM outputs is sometimes incorrect
	- The leader type is not automatically exported. */

	/* Ego velocity minus leader velocity. Returns zero if there
	is no leader */
	double get_relative_velocity_to_leader();
	/* Returns 0 if no leader. */
	//long get_leader_type();

	/* Computation of surrogate safety measurements */

	void compute_all_ssms();
	double compute_ttc(const NearbyVehicle& other_vehicle);
	double compute_drac(const NearbyVehicle& other_vehicle);
	/* The collision free gap is computed assuming a worst case braking
	scenario */
	double compute_exact_collision_free_gap(double ego_velocity,
		const NearbyVehicle& other_vehicle);
	/* Relative velocity at collision time under the worst case scenario*/
	double compute_collision_severity_risk(
		const NearbyVehicle& other_vehicle);


	/* State-machine related methods */

	void update_state();
	//bool has_lane_change_intention() const;
	bool is_lane_changing() const override;
	//State get_previous_state() const;
	/* Returns the color equivalent to the current state as a long */
	long get_color_by_controller_state();
	std::string print_detailed_state();

	/* Control related methods */

	/* Computes the longitudinal controller input */
	double compute_desired_acceleration();
	/* Takes the desired acceleration given by the controller and 
	returns the feasible acceleration given the approximated low level
	dynamics */
	double consider_vehicle_dynamics(double desired_acceleration);
	//RelativeLane get_lane_change_direction();
	double compute_safe_lane_change_gap(
		std::shared_ptr<NearbyVehicle> other_vehicle);
	/*double compute_safe_gap_to_destination_lane_leader();
	double compute_safe_gap_to_destination_lane_follower();*/
	/* Calls the controller to decide whether the vehicle can start a 
	lane change. Returns -1 for right lane changes, +1 for left lane 
	changes and 0 for lane keeping. */
	long decide_lane_change_direction();
	std::string state_to_string(State vehicle_state);

	/* Methods to access internal values. Used for quicker debugging */

	/* Returns the current reference gap to the leader */
	double get_reference_gap();
	/* Returns the time headway gap from the ego to the other vehicle
	if the ego vehicle is behind and from the other to the ego vehicle
	if the other vehicle is behind. */
	double compute_time_headway_gap(
		std::shared_ptr<NearbyVehicle> other_vehicle);
	/* Returns the transient lane changing gap between ego vehicle 
	and other. */
	double compute_transient_gap(
		std::shared_ptr<NearbyVehicle> other_vehicle);


	/* Methods for logging */

	/* TODO: find better names for the methods */

	bool is_verbose() const { return verbose; };

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out, const EgoVehicle& vehicle);

private:
	/* Estimated parameters used for safe gap computations (no direct equivalent
	in VISSIM's simulation dynamics) */
	
	double tau{ ACTUATOR_CONSTANT }; // actuator constant [s].
	/* constant used in the discrete approximation of
	the vehicle first order actuator dynamics */
	double tau_d{ 0.0 };
	double comfortable_brake{ COMFORTABLE_BRAKE }; // [m/s^2]

	/* Parameter related to emergency braking during lane change [m/s] */
	double lambda_1_lane_change{ 0.0 };
	
	/* Connected vehicles following non-connected vehicles must use
	non-connected parameters. */
	double lambda_1_connected{ 0.0 };
	/* Connected vehicles following non-connected vehicles must use
	non-connected parameters. */
	double lambda_0_connected{ 0.0 };

	/* Control related members */

	ControlManager controller;
	/*double adjustment_speed_factor{ 0.6 }; /* when adjusting for 
	lane changes, the vehicle has a minimum accepted speed based on its
	speed at the start of the adjustment. */
	
	/* Nearby vehicles data */
	/* TODO: I'm no longer sure a vector of pointer is the way to go.
	Maybe we should use a simple vector of objects. Or maybe some smart
	pointer stuff */
	std::vector<std::shared_ptr<NearbyVehicle>> nearby_vehicles;
	std::vector<long> leader_id;
	long dest_lane_leader_id = 0;
	long dest_lane_follower_id = 0;
	/*int leader_idx = -1;
	int follower_idx = -1;
	int dest_lane_leader_idx = -1;
	int dest_lane_follower_idx = -1;*/

	std::shared_ptr<NearbyVehicle> leader{ nullptr };
	std::shared_ptr<NearbyVehicle> follower{ nullptr };
	std::shared_ptr<NearbyVehicle> destination_lane_leader{ nullptr };
	std::shared_ptr<NearbyVehicle> destination_lane_follower{ nullptr };
	/* Vehicle for which the ego vehicle will help generate a safe
	lane change gap */
	std::shared_ptr<NearbyVehicle> assisted_vehicle{ nullptr };
	//RelevantNearbyVehicles relevant_nearby_vehicles;

	/* Colors for easy visualization in VISSIM:
	General rule: bright colors represent vel control, 
	darker colors represent vehicle following. */
	color_t orig_lane_vel_control_color = GREEN;
	color_t orig_lane_veh_foll_color = DARK_GREEN;
	color_t gap_generation_vel_control_color = YELLOW;
	color_t gap_generation_veh_foll_color = DARK_YELLOW;
	color_t dest_lane_vel_control_color = LIGHT_BLUE;
	color_t dest_lane_veh_foll_color = BLUE;
	color_t end_of_lane_vel_control_color = MAGENTA;
	color_t end_of_lane_veh_foll_color = DARK_MAGENTA;

	/* Data obtained from VISSIM or generated by internal computations */
	//std::vector<double> simulation_time;
	double creation_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	long color{ 0 };
	double desired_velocity{ 0 };
	std::vector<long> lane;
	std::vector<long> link;
	std::vector<RelativeLane> preferred_relative_lane;
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	std::vector<double> lateral_position;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> desired_acceleration;
	/* VISSIM suggested acceleration */
	std::vector<double> vissim_acceleration;
	/* +1 = to the left, 0 = none, -1 = to the right */
	std::vector<RelativeLane> active_lane_change_direction;
	/* VISSIM suggested active lane change */
	std::vector<long> vissim_active_lane_change;
	/* Determines if we use our lane change decision model or VISSIM's */
	bool use_internal_lane_change_decision{ true };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	std::vector<double> lane_end_distance;
	std::vector<State> state;
	double desired_lane_angle{ 0.0 };
	RelativeLane relative_target_lane{ RelativeLane::same };
	long turning_indicator{ 0 };

	/*Surrogate Safety Measurements (SSMs)*/

	std::vector<double> ttc; // time-to-collision
	std::vector<double> drac; // deceleration rate to avoid collision
	std::vector<double> collision_severity_risk; /* delta vel. at collision under the
											worst case scenario*/
	
	/* Vehicle internal methods */
	/* Computes members lambda_0, lambda_1 and lane_change_lambda_1 */
	void compute_safe_gap_parameters() override;
	/* TODO: state and desired_lane_change_direction members
	can become a single member. They are redundant. */
	
	void set_desired_lane_change_direction();

	/* For printing and debugging purporses */
	bool verbose = false; /* when true, will print results to 
						  the default log file and
						  create a specific log file for this
						  vehicle */
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
		vissim_active_lane_change_direction,
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