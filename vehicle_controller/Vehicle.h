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
const color_t YELLOW = ARGB(255, 255, 255, 96);
const color_t BLACK = ARGB(255, 0, 0, 0);
const color_t WHITE = ARGB(255, 255, 255, 255);

/* Class helps to organize all vehicle parameters in one place */
/* TODO: is it better to have an abstract Vehicle class and then two derived
	classes EgoVehicle and NearbyVehicle? */
class Vehicle {
public:

	//enum class States {
	//	velocity_control,
	//	vehicle_following,
	//	emergency_braking,
	//	intention_to_change_lane,
	//};

	/* Constructors 
	TODO: have a single non default constructor with a 
	default parameter? */
	Vehicle() = default;
	Vehicle(long id, double simulation_time_step, double creation_time,
		bool verbose);
	Vehicle(long id, double simulation_time_step, double creation_time);
	~Vehicle();

	/* Typical getters and setters */

	double get_sampling_interval() const { return simulation_time_step; };
	long get_id() const{ return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	VehicleCategory get_category() const { return category; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	NearbyVehicle& get_leader() { return leader; };
	double get_comfortable_acceleration() const { 
		return comfortable_acceleration;
	};
	double get_max_brake() const { return max_brake; };
	double get_lambda_0() const { return lambda_0; };
	double get_lambda_1() const { return lambda_1; };
	bool get_use_internal_lane_change_decision() const {
		return use_internal_lane_change_decision;
	};
	RelativeLane get_desired_lane_change_direction() const {
		return desired_lane_change_direction;
	};
	/* The controller should not be accessed directly by external
	functions. This getter should be used only for simpler debugging.*/
	//ControlManager get_controller() const { return controller; };
	
	void Vehicle::set_desired_velocity(double desired_velocity) {
		this->desired_velocity = desired_velocity;
	};
	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	void set_category(long category) { 
		this->category = VehicleCategory(category); 
	};
	void set_category(VehicleCategory category) {
		this->category = category;
	};
	void set_color(long color) { this->color = color; };
	void set_use_internal_lane_change_decision(long use) {
		this->use_internal_lane_change_decision = use > 0;
	}

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

	/* Setters of vector-type members*/
	void set_lane(long lane);
	/* Adds a value to the preferred relative lane vector AND 
	sets the value of desired_lane_change_direction */
	void set_preferred_relative_lane(long preferred_relative_lane);
	void set_velocity(double velocity);
	void set_acceleration(double acceleration);
	void set_vissim_acceleration(double vissim_acceleration);
	void set_vissim_active_lane_change(int active_lane_change);
	void set_lane_end_distance(double lane_end_distance,
		long lane_number);

	/* Dealing with nearby vehicles*/
	
	void clear_nearby_vehicles();
	/* Adds a vehicle at the end of the nearby_vehicles container
	TODO: consider whether we should also check whether nearby vehicle is the
	leader. This would avoid having to search for it later on.*/
	void push_nearby_vehicle(NearbyVehicle* nearby_vehicle);
	/* Return the most recently added nearby vehicle */
	NearbyVehicle* peek_nearby_vehicles() const;
	/* Write data to the NearbyVehicle leader. If there is no leader, writes 
	all zeros. Returns true if a leader is found, false otherwise*/
	bool update_leader();
	/* Computes the bumper-to-bumper distance between vehicles */
	double compute_gap(const NearbyVehicle& nearby_vehicle) const;
	double compute_gap(const NearbyVehicle* nearby_vehicle) const;
	double compute_gap_to_destination_lane_leader() const;
	double compute_gap_to_destination_lane_follower() const;
	NearbyVehicle* find_destination_lane_leader() const;
	NearbyVehicle* find_destination_lane_follower() const;
	/*NearbyVehicle* find_left_lane_leader() const;
	NearbyVehicle* find_left_lane_follower() const;
	NearbyVehicle* find_right_lane_leader() const;
	NearbyVehicle* find_right_lane_follower() const;*/
	/* Finds closest downstream nearby_vehicle which is on the same lane and
	writes its address to parameter leader. Function returns a boolean indicating whether
	the vehicle has a leader */
	//bool find_leader(NearbyVehicle*& leader) const;

	
	/* Control related methods */

	/* Accesses the controller state */
	ControlManager::State get_current_vehicle_state();
	/* Returns the color equivalent to the current state as a long */
	long get_color_by_controller_state();
	/* Computes the longitudinal controller input */
	double compute_desired_acceleration();
	/* Takes the desired acceleration given by the controller and 
	returns the feasible acceleration given the approximated low level
	dynamics */
	double consider_vehicle_dynamics(double desired_acceleration);
	RelativeLane get_lane_change_direction();
	double compute_safe_gap_to_destination_lane_leader();
	double compute_safe_gap_to_destination_lane_follower();
	/* Returns the vehicle following collision free gap from the
	destination lane follower to the ego vehicle. Temporary function 
	for debugging. */
	double compute_collision_free_gap_to_destination_lane_follower();
	/* Returns the transiend gap from the destination lane
	follower to the ego vehicle. Temporary function for debugging. */
	double compute_transient_gap_to_destination_lane_follower();
	/* Calls the controller to decide whether the vehicle can start a 
	lane change. Returns -1 for right lane changes, +1 for left lane 
	changes and 0 for lane keeping. 
	TODO: return member of enum class relative_lane*/
	long decide_active_lane_change_direction();
	std::string controller_state_to_string(ControlManager::State vehicle_state);

	/* Methods for debugging */
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
	double tau{ 0.5 }; // actuator constant [s].
	double max_brake{ 7.5 }; // [m/s^2] TODO: vary with speed?
	double comfortable_acceleration{ 2.0 }; // [m/s^2] TODO: vary with speed?
	double max_jerk{ 50.0 }; // [m/s^3]
	double brake_delay{ 0.1 }; // [s]

	/* Derivated parameters for safe gap computation */
	double lambda_0{ 0.0 }; // [m]
	double lambda_1{ 1.0 }; // [m/s]

	ControlManager controller;
	//std::vector<States> vehicle_state; // = VehicleStates::velocity_control;
	std::vector<NearbyVehicle*> nearby_vehicles;
	NearbyVehicle leader; // should this be just a pointer?
	color_t velocity_control_color = BLUE;
	color_t vehicle_following_color = GREEN;
	color_t emergency_braking_color = RED;
	color_t intention_to_change_lane_color = YELLOW;

	/* Data obtained from VISSIM or generated by internal computations */
	//std::vector<double> simulation_time;
	double creation_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	long id{ 0 };
	double length{ 0 };
	double width{ 0 };
	long color{ 0 };
	VehicleCategory category{ VehicleCategory::car };
	double desired_velocity{ 0 };
	std::vector<long> lane;
	std::vector<long> preferred_relative_lane;
	/* Updated together with preferred_relative_lane */
	RelativeLane desired_lane_change_direction{ RelativeLane::same };
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
	std::vector<double> lane_end_distance;
	/* VISSIM sometimes writes some data twice in the same time step.
	To avoid data vectors of different sizes, we only write data once per 
	time step. This boolean determines when to write new data */
	//bool is_new_data{ false };

	/* Vehicle internal methods */
	void compute_safe_gap_parameters();
	NearbyVehicle* find_nearby_vehicle(
		RelativeLane relative_lane,
		int relative_position) const;

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