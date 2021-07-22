/*==========================================================================*/
/*  Vehicle.h	    													    */
/*  Class to manage simualated vehicles                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once
#include <unordered_map>
#include <vector>

#include "Constants.h"
#include "ControlManager.h"
#include "NearbyVehicle.h"

typedef unsigned long color_t;
/* Based on the _WINGDI_ RGB definition and some tests on VISSIM */
//#define ARGB(a,r,g,b)   ((COLORREF)((((BYTE)(b) | ((WORD)((BYTE)(g)) << 8)) | (((DWORD)(BYTE)(r)) << 16))|((BYTE)(a) << 24)))
#define ARGB(a, r, g, b) ((color_t)((long)(b) + (long)(g)*256 + (long)(r)*256*256 + (unsigned long)(a)*256*256*256))

const color_t RED = ARGB(255, 255, 0, 0);
const color_t GREEN = ARGB(255, 0, 255, 0);
const color_t BLUE = ARGB(255, 0, 0, 255);
const color_t CYAN = ARGB(255, 0, 255, 255);
const color_t MAGENTA = ARGB(255, 255, 0, 255);
const color_t YELLOW = ARGB(255, 255, 255, 0);
const color_t BLACK = ARGB(255, 0, 0, 0);
const color_t WHITE = ARGB(255, 255, 255, 255);
const color_t DARK_YELLOW = ARGB(255, 128, 128, 0);
const color_t PURPLE = ARGB(255, 128, 0, 128);
const color_t DARK_GREEN = ARGB(255, 0, 192, 0);

struct RelevantNearbyVehicles {
	NearbyVehicle* leader{ nullptr };
	NearbyVehicle* follower{ nullptr };
	NearbyVehicle* destination_lane_leader{ nullptr };
	NearbyVehicle* destination_lane_follower{ nullptr };
};

/* Class helps to organize all vehicle parameters in one place */
/* TODO: is it better to have an abstract Vehicle class and then two derived
	classes EgoVehicle and NearbyVehicle? */
class Vehicle {
public:

	enum class State {
		lane_keeping,
		intention_to_change_lanes,
	};

	/* Constructors 
	TODO: have a single non default constructor with a 
	default parameter? */
	Vehicle() = default;
	Vehicle(long id, double simulation_time_step, double creation_time,
		bool verbose);
	Vehicle(long id, double simulation_time_step, double creation_time);
	~Vehicle();


	/* Getters and setters */

	double get_sampling_interval() const { return simulation_time_step; };
	long get_id() const{ return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	VehicleCategory get_category() const { return category; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_comfortable_acceleration() const { 
		return comfortable_acceleration;
	};
	double get_max_brake() const { return max_brake; };
	double get_comfortable_brake() const { return comfortable_brake; };
	double get_max_jerk() const { return max_jerk; };
	double get_brake_delay() const { return brake_delay; };
	/*double get_lambda_0() const { return lambda_0; };
	double get_lambda_1() const { return lambda_1; };*/
	bool get_use_internal_lane_change_decision() const {
		return use_internal_lane_change_decision;
	};
	/*RelativeLane get_desired_lane_change_direction() const {
		return desired_lane_change_direction;
	};*/
	double get_adjustment_speed_factor() const {
		return adjustment_speed_factor;
	}
	/* The controller should not be accessed directly by external
	functions. This getter should be used only for simpler debugging.*/
	//ControlManager get_controller() const { return controller; };
	
	void Vehicle::set_desired_velocity(double desired_velocity) {
		this->desired_velocity = desired_velocity;
	};
	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	void set_color(long color) { this->color = color; };
	void set_use_internal_lane_change_decision(long use) {
		this->use_internal_lane_change_decision = use > 0;
	}
	/* set_category also sets the estimated maximum braking of the
	nearby vehicle and computes lambda_0 and lambda_1. */
	void set_category(long category);
	/* set_category also sets the estimated maximum braking of the
	nearby vehicle and computes lambda_0 and lambda_1. */
	void set_category(VehicleCategory category);


	/* Getters of most recent values
	TODO: should we check if vehicle is initialized before returning values? */

	double get_current_time() const;
	long get_current_lane() const;
	long get_current_preferred_relative_lane() const;
	double get_current_velocity() const;
	double get_current_acceleration() const;
	double get_current_desired_acceleration() const;
	double get_current_vissim_acceleration() const;
	long get_current_active_lane_change() const;
	long get_current_vissim_active_lane_change() const;
	double get_current_lane_end_distance() const;
	long get_current_leader_id() const;
	State get_current_state() const;

	/* Setters of vector-type members*/

	void set_lane(long lane);
	/* Also updates the state member */
	void set_preferred_relative_lane(long preferred_relative_lane);
	void set_velocity(double velocity);
	void set_acceleration(double acceleration);
	void set_vissim_acceleration(double vissim_acceleration);
	void set_vissim_active_lane_change(int active_lane_change);
	void set_lane_end_distance(double lane_end_distance,
		long lane_number);

	/* Dealing with nearby vehicles*/
	
	void clear_nearby_vehicles();
	/* Creates an instance of nearby vehicle and populates it with 
	the given data. */
	void emplace_nearby_vehicle(long id, long relative_lane,
		long relative_position);
	/* Returns the most recently added nearby vehicle */
	NearbyVehicle* peek_nearby_vehicles() const;
	/* Searches the nearby_vehicles array. NO LONGER IN USE */
	NearbyVehicle* find_nearby_vehicle(RelativeLane relative_lane,
		int relative_position) const;
	/* Looks at all nearby vehicles, defines pointers to the leader 
	(if it exits), and destination lane leader and follower (if there 
	is lane change intention and if they exist). Also performs time 
	computations */
	void analyze_nearby_vehicles();
	/* Returns a nullptr if there is no leader */
	NearbyVehicle* get_leader() const;
	/* Returns a nullptr if there is no follower */
	NearbyVehicle* get_follower() const;
	/* Returns a nullptr if there is no leader at the destination lane */
	NearbyVehicle* get_destination_lane_leader() const;
	/* Returns a nullptr if there is no follower at the destination lane */
	NearbyVehicle* get_destination_lane_follower() const;
	/* Computes the bumper-to-bumper distance between vehicles */
	double compute_gap(const NearbyVehicle& nearby_vehicle) const;
	double compute_gap(const NearbyVehicle* nearby_vehicle) const;
	bool find_lane_change_conflicts();
	

	/* State-machine related methods */

	bool has_lane_change_intention() const;
	RelativeLane determine_desired_lane_change_direction();
	State get_previous_state() const;
	/* Returns the color equivalent to the current state as a long */
	long get_color_by_controller_state();

	/* Control related methods */

	/* Computes the longitudinal controller input */
	double compute_desired_acceleration();
	/* Takes the desired acceleration given by the controller and 
	returns the feasible acceleration given the approximated low level
	dynamics */
	double consider_vehicle_dynamics(double desired_acceleration);
	RelativeLane get_lane_change_direction();
	double compute_safe_lane_change_gap(NearbyVehicle* other_vehicle);
	double compute_safe_gap_to_destination_lane_leader();
	double compute_safe_gap_to_destination_lane_follower();
	/* Calls the controller to decide whether the vehicle can start a 
	lane change. Returns -1 for right lane changes, +1 for left lane 
	changes and 0 for lane keeping. 
	TODO: return member of enum class relative_lane*/
	long decide_active_lane_change_direction();
	std::string state_to_string(State vehicle_state);


	/* Methods to access internal values. Used for quicker debugging */

	/* Returns the current reference gap to the leader */
	double get_reference_gap();
	/* Returns the time headway gap from the ego to the other vehicle
	if the ego vehicle is behind and from the other to the ego vehicle
	if the other vehicle is behind. */
	double compute_time_headway_gap(NearbyVehicle* other_vehicle);
	/* Returns the transient lane changing gap between ego vehicle 
	and other. */
	double compute_transient_gap(NearbyVehicle* other_vehicle);
	/* Returns the vehicle following gap from the
	destination lane follower to the ego vehicle. */
	//double compute_time_headway_gap_to_destination_lane_follower();
	/* Returns the transiend gap from the destination lane
	follower to the ego vehicle. */
	//double compute_transient_gap_to_destination_lane_follower();


	/* Methods for logging */

	/* TODO: find better names for the methods */
	bool get_should_log() const { return should_log; };
	void log_vehicle_states() { this->should_log = true; };
	/* The log should be adapted depending on what we are trying to debug */
	void write_vehicle_log();

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out, const Vehicle& vehicle);

private:
	/* Estimated parameters used for safe gap computations (no direct equivalent
	in VISSIM's simulation dynamics) */
	/* TODO : parameters below should vary based on vehicle category */
	
	double tau{ ACTUATOR_CONSTANT }; // actuator constant [s].
	double max_brake{ CAR_MAX_BRAKE }; // [m/s^2] TODO: vary with speed?
	double comfortable_brake{ -COMFORTABLE_ACCELERATION }; // [m/s^2] TODO: vary with speed?
	double comfortable_acceleration{ COMFORTABLE_ACCELERATION }; // [m/s^2] TODO: vary with speed?
	double max_jerk{ MAX_JERK }; // [m/s^3]
	double brake_delay{ AUTONOMOUS_BRAKE_DELAY }; // [s]

	//double lambda_0{ 0.0 }; // [m]
	//double lambda_1{ 0.0 }; // [m/s]

	/* Control related members */

	ControlManager controller;
	double adjustment_speed_factor{ 0.6 }; /* when adjusting for 
	lane changes, the vehicle has a minimum accepted speed based on its
	speed at the start of the adjustment. */
	
	/* Nearby vehicles data */
	/* TODO: I'm no longer sure a vector of pointer is the way to go.
	Maybe we should use a simple vector of objects. Or maybe some smart
	pointer stuff */
	std::vector<NearbyVehicle*> nearby_vehicles;
	std::vector<long> leader_id;
	long dest_lane_leader_id = 0;
	long dest_lane_follower_id = 0;
	/*int leader_idx = -1;
	int follower_idx = -1;
	int dest_lane_leader_idx = -1;
	int dest_lane_follower_idx = -1;*/
	RelevantNearbyVehicles relevant_nearby_vehicles;

	/* Colors for easy visualization in VISSIM */
	color_t velocity_control_color = GREEN;
	color_t vehicle_following_color = DARK_GREEN;
	color_t emergency_braking_color = RED;
	color_t lane_change_adjustment_origin_lane_color = BLUE;
	color_t lane_change_adjustment_destination_lane_color = CYAN;
	color_t lane_change_adjustment_end_of_lane_color = PURPLE;

	/* Data obtained from VISSIM or generated by internal computations */
	//std::vector<double> simulation_time;
	double creation_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	long id{ 0 };
	double length{ 0 };
	double width{ 0 };
	long color{ 0 };
	VehicleCategory category{ VehicleCategory::undefined };
	double desired_velocity{ 0 };
	std::vector<long> lane;
	std::vector<long> preferred_relative_lane;
	/* Updated together with preferred_relative_lane */
	//RelativeLane desired_lane_change_direction{ RelativeLane::same };
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> desired_acceleration;
	/* VISSIM suggested acceleration */
	std::vector<double> vissim_acceleration;
	/* +1 = to the left, 0 = none, -1 = to the right */
	std::vector<long> active_lane_change;
	/* VISSIM suggested active lane change */
	std::vector<long> vissim_active_lane_change;
	/* Determines if we use our lane change decision model or VISSIM's */
	bool use_internal_lane_change_decision{ false };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	std::vector<double> lane_end_distance;
	std::vector<State> state;
	
	/* Vehicle internal methods */
	void update_state();

	/* For printing and debugging purporses */
	bool verbose = false; /* when true, will print results to 
						  the default log file */
	bool should_log = false; /* when true, will create a specific 
							 log file for this vehicle */
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
	};

	std::string create_header(std::vector<Member> members,
		bool write_size = false);
	std::ostringstream write_members(std::vector<Member> members,
		bool write_size = false);
	int get_member_size(Member member);
	std::string member_enum_to_string(Member member);
};