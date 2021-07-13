/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that does nothing (uses Vissim's internal model).         */
/*                                                                          */
/*  Version of 2017-09-15                                   Lukas Kautzsch  */
/*==========================================================================*/

#include <iostream>
#include <unordered_set>
#include "Constants.h"
#include "DriverModel.h"
#include "SimulationLogger.h"
#include "Vehicle.h"

/*==========================================================================*/

const size_t LOGGED_VEHICLE_NO = 3; // to "debug" code logic
const std::unordered_set<long> LOGGED_VEHICLES_IDS{ 1, 3, 7 };

SimulationLogger simulation_logger;
std::unordered_map<long, Vehicle> vehicles;
long current_vehicle_id = 0;
double simulation_time_step = -1;
double current_time = 0;
//long logged_vehicle_id = -1; // used together with LOGGED_VEH_NO
double desired_lane_angle = 0.0;
long rel_target_lane = 0;
long turning_indicator = 0;

/*==========================================================================*/

/*VISSIM's interface documentation does not mention this function, but it 
seems to initiate and finish communication between the simulator and the DLL.
We can use it to initialize and destruct objects related to each simulation 
run.*/
BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
          simulation_logger.createLogFile();
          break;
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
          break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
    /* Sets the value of a data object of type <type>, selected by <index1> */
    /* and possibly <index2>, to <long_value>, <double_value> or            */
    /* <*string_value> (object and value selection depending on <type>).    */
    /* Return value is 1 on success, otherwise 0.                           */

    /* Note that we can check the order in which each case is accessed at the 
    API documentation. */
    
    /* TODO: should we take some action if something weird happens and data 
    is passed to the DLL before we create the Vehicle object?*/

    switch (type) {
    case DRIVER_DATA_PATH                   :
        return 1;
    case DRIVER_DATA_TIMESTEP               :
        if (simulation_time_step < 0) {
            simulation_time_step = double_value;
        }
        return 1;
    case DRIVER_DATA_TIME                   :
        // BAD PRACTICE
        /* DRIVER_DATA_TIME is NOT called only once per vehicle,
        so this piece of code will update current_vehicle_id too
        often.
        SOLUTION:
        Save the vehicle initial time. When printing, simply sum
        the time_step i times to the initial time. Important! Remember
        that all variables are set twice at the moment of the vehicle
        creation, so this must be taken into account. */
        current_time = double_value;
        /*if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_simulation_time(double_value);
        }*/
        return 1;
    case DRIVER_DATA_USE_UDA                :
        if (index1 >= static_cast<int>(UDA::first)) { 
            return 1;
        }
        return 0; /* 0 doesn't use any UDAs */
                /* must return 1 for desired values of index1 if UDA values 
                are to be sent from/to Vissim */
    case DRIVER_DATA_VEH_ID                 :
        current_vehicle_id = long_value;
        if (vehicles.find(long_value) == vehicles.end()) {
            
            bool verbose = false;
            if (LOGGED_VEHICLES_IDS.find(current_vehicle_id) 
                != LOGGED_VEHICLES_IDS.end()) verbose = true;
            
            vehicles[current_vehicle_id] = Vehicle(current_vehicle_id,
                simulation_time_step, current_time, verbose);
            if (verbose) {
                vehicles[current_vehicle_id].log_vehicle_states();
            }
        }
        else {
            vehicles[current_vehicle_id].clear_nearby_vehicles();
        }
        return 1;
    case DRIVER_DATA_VEH_LANE               :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_lane(long_value);
        }
        return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
        return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_velocity(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_acceleration(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_LENGTH             :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_length(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_WIDTH              :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_width(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
        turning_indicator = long_value;
        return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_category(long_value);
        }
        return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_preferred_relative_lane(
                long_value);
        }
        return 1;
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_desired_velocity(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
    case DRIVER_DATA_VEH_TYPE               :
        return 1;
    case DRIVER_DATA_VEH_COLOR              :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_color(long_value);
        }
        return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
        return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS
                messages) */
                /* Must return 1 if these messages are to be sent from
                VISSIM! */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
        return 1;
    case DRIVER_DATA_VEH_INTAC_STATE        :
        return 1;
    case DRIVER_DATA_VEH_INTAC_TARGET_TYPE  :
        return 1;
    case DRIVER_DATA_VEH_INTAC_TARGET_ID    :
        return 1;
    case DRIVER_DATA_VEH_INTAC_HEADWAY      :
        return 1;
    case DRIVER_DATA_VEH_UDA                :
        if (UDA(index1) == UDA::use_internal_lane_change_decision) {
            vehicles[current_vehicle_id].
                set_use_internal_lane_change_decision(long_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_ID                :
        if (vehicles.find(current_vehicle_id) != vehicles.end() 
            && (long_value > 0)) { 
            /* TODO: the following code doesn't match well the 
            characteristics of the NearbyVehicle class.
            NearbyVehicle has vectors of data (id, lane, etc), but 
            by creating new instances at every time step, this feature
            becomes useless. */
            NearbyVehicle* nearby_vehicle = new NearbyVehicle;
            nearby_vehicle->set_id(long_value);
            nearby_vehicle->set_relative_lane(index1);
            nearby_vehicle->set_relative_position(index2);
            vehicles[current_vehicle_id].push_nearby_vehicle(nearby_vehicle);
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
        return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_distance(double_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_relative_velocity(double_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_acceleration(double_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_length(double_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_width(double_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].peek_nearby_vehicles()
                ->set_category(long_value);
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NVEH_TYPE              :
    case DRIVER_DATA_NVEH_UDA               :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
        return 1;
    case DRIVER_DATA_LANE_END_DISTANCE      :
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_lane_end_distance(
                double_value, index1);
        }
        return 1;
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
        return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION   :
        //desired_acceleration = double_value;
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_vissim_acceleration(
                double_value);
        }
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
        desired_lane_angle = double_value;
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
        //active_lane_change = long_value;
        if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id].set_vissim_active_lane_change(
                long_value);
        }
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
        rel_target_lane = long_value;
        return 1;
    default :
        return 0;
    }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
    /* Gets the value of a data object of type <type>, selected by <index1> */
    /* and possibly <index2>, and writes that value to <*double_value>,     */
    /* <*float_value> or <**string_value> (object and value selection       */
    /* depending on <type>).                                                */
    /* Return value is 1 on success, otherwise 0.                           */

    /* Note that we can check the order in which each case is accessed at the
    API documentation. */
    //double controller_acceleration;

    Vehicle* current_vehicle = &vehicles[current_vehicle_id];

    switch (type) {
    case DRIVER_DATA_STATUS :
        *long_value = 0;
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
        *long_value = turning_indicator;
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        *double_value = current_vehicle->get_desired_velocity();
        return 1;
    case DRIVER_DATA_VEH_COLOR :
        *long_value = current_vehicle->get_color_by_controller_state();
        return 1;
    case DRIVER_DATA_VEH_UDA :
        switch (UDA(index1))
        {
        case UDA::gap_to_dest_lane_leader:
            *double_value = current_vehicle->
                compute_gap_to_destination_lane_leader();
            break;
        case UDA::gap_to_dest_lane_follower:
            *double_value = current_vehicle->
                compute_gap_to_destination_lane_follower();
            break;
        case UDA::safe_gap_to_dest_lane_leader:
            *double_value = current_vehicle->
                compute_safe_gap_to_destination_lane_leader();
            break;
        case UDA::safe_gap_to_dest_lane_follower:
            *double_value = current_vehicle->
                compute_safe_gap_to_destination_lane_follower();
            break;
        case UDA::gap_to_leader:
            *double_value = current_vehicle->
                compute_gap(current_vehicle->get_leader());
            break;
        case UDA::leader_id:
            *long_value = current_vehicle->get_leader().get_current_id();
            break;
        case UDA::veh_following_collision_free_gap_to_fd:
            *double_value = current_vehicle->
                compute_collision_free_gap_to_destination_lane_follower();
            break;
        case UDA::transient_gap_to_fd:
            *double_value = current_vehicle->
                compute_transient_gap_to_destination_lane_follower();
            break;
        default:
            return 0; /* doesn't set any UDA values */
        }
        return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
        *long_value = 1;
        return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
        *double_value = current_vehicle->compute_desired_acceleration();
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
        *double_value = desired_lane_angle;
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
        *long_value = current_vehicle->decide_active_lane_change_direction();
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
        *long_value = rel_target_lane;
        return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
        *long_value = 1;
        return 1;
    case DRIVER_DATA_USE_INTERNAL_MODEL:
        *long_value = 0; /* must be set to 0 if external model is to be 
                        applied */
        return 1;
    case DRIVER_DATA_WANTS_ALL_NVEHS:
        *long_value = 0; /* must be set to 1 if data for more than 2 nearby 
                        vehicles per lane and upstream/downstream is to be 
                        passed from Vissim */
        return 1;
    case DRIVER_DATA_ALLOW_MULTITHREADING:
        *long_value = 0; /* must be set to 1 to allow a simulation run to be
                        started with multiple cores used in the simulation 
                        parameters */
        return 1;
    default:
        return 0;
    }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
    /* Executes the command <number> if that is available in the driver */
    /* module. Return value is 1 on success, otherwise 0.               */

    bool has_leader = false;

    switch (number) {
    case DRIVER_COMMAND_INIT :
        return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
        return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
        if (vehicles[current_vehicle_id].get_should_log()) {
            std::clog << "Vehicle " << current_vehicle_id
                <<" end time " << current_time << std::endl;
            vehicles[current_vehicle_id].write_vehicle_log();
        }
        return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
        has_leader = vehicles[current_vehicle_id].update_leader();
        return 1;
    default :
        return 0;
    }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
