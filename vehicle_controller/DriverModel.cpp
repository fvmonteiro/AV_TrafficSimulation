/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*                                                                          */
/* Based on example from Version of 2017-09-15 by Lukas Kautzsch            */
/*==========================================================================*/

#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "Constants.h"
#include "DriverModel.h"
#include "SimulationLogger.h"
#include "EgoVehicle.h"
#include "LoggedVehicle.h"

/*==========================================================================*/

const std::unordered_set<long> LOGGED_VEHICLES_IDS{ 0 };
const bool CLUELESS_DEBUGGING{ false };

SimulationLogger simulation_logger;
std::unordered_map<long, EgoVehicle> vehicles;
long current_vehicle_id = 0;
double simulation_time_step = -1.0;
double current_time = 0.0;

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
          simulation_logger.create_log_file();
          /*simulation_logger.write_to_persistent_log(
              std::to_string(vehicles.size()) + " vehicles in memory "
                + "(before start)");*/
          break;
      case DLL_THREAD_ATTACH:
          break;
      case DLL_THREAD_DETACH:
          break;
      case DLL_PROCESS_DETACH:
          /*simulation_logger.write_to_persistent_log(
              std::to_string(vehicles.size()) + " vehicles in memory "
                + "(end of simulation)");*/
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

    switch (type) {
    case DRIVER_DATA_PATH                   :
        std::clog << "DLL path: " 
            << string_value
            << std::endl;
        return 1;
    case DRIVER_DATA_TIMESTEP               :
        if (simulation_time_step < 0) {
            simulation_time_step = double_value;
        }
        return 1;
    case DRIVER_DATA_TIME                   :
        if (CLUELESS_DEBUGGING && (double_value != current_time)) {
            std::clog << "t=" << current_time << std::endl;
        }
        // Only for the first time step
        /*if ((double_value != current_time)
            && (std::abs(double_value - simulation_time_step)
                < simulation_time_step / 2)) {
            simulation_logger.write_to_persistent_log(
                std::to_string(vehicles.size()) + " vehicles in memory.");
        }*/
        current_time = double_value;
        return 1;
    case DRIVER_DATA_USE_UDA                :
        /* NOTE: TO AVOID UDA ISSUES BETWEEN DIFFERENT NETWORKS
        WE ARE IGNORING ALL UDAs */

        /* must return 1 for desired values of index1 if UDA values
        are to be sent from/to Vissim */
        if (index1 >= static_cast<int>(UDA::first)) { 
            UDA uda = UDA(index1);
            switch (uda) {
            case UDA::gap_to_dest_lane_leader:
            case UDA::gap_to_dest_lane_follower:
            case UDA::safe_gap_to_dest_lane_leader:
            case UDA::safe_gap_to_dest_lane_follower:
            case UDA::gap_to_leader:
                return 0;
            case UDA::leader_id:
                return 1;
            case UDA::use_internal_lane_change_decision:
                return 1;
            case UDA::veh_following_gap_to_fd:
            case UDA::transient_gap_to_fd:
            case UDA::veh_following_gap_to_ld:
            case UDA::transient_gap_to_ld:
            case UDA::reference_gap:
            case UDA::ttc:
            case UDA::drac:
            case UDA::collision_severity_risk:
                return 0;
            case UDA::write_veh_log:
                return 0;
            case UDA::relative_velocity_to_leader:
            case UDA::leader_type:
                return 0;
            default:
                return 0;
            }
        }
        else {
            return 0;
        }
    case DRIVER_DATA_VEH_ID                 :
        if (CLUELESS_DEBUGGING) {
            std::clog << "getting data for veh. " << long_value << std::endl;
        }

        current_vehicle_id = long_value;
        if (vehicles.find(long_value) == vehicles.end()) {
            
            bool verbose = false;
            if (LOGGED_VEHICLES_IDS.find(current_vehicle_id)
                != LOGGED_VEHICLES_IDS.end()) verbose = true;
            
            /* This cumbersome use of emplace guarantees a single 
            creation (and destruction) of each vehicle */
            vehicles.emplace(std::piecewise_construct,
                std::forward_as_tuple(current_vehicle_id),
                std::forward_as_tuple(current_vehicle_id, 
                    simulation_time_step, current_time, verbose));
        }
        else {
            vehicles[current_vehicle_id].clear_nearby_vehicles();
        }
        return 1;
    case DRIVER_DATA_VEH_LANE               :
        vehicles[current_vehicle_id].set_lane(long_value);
        return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
        return 1;
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
        vehicles[current_vehicle_id].set_lateral_position(double_value);
        return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
        vehicles[current_vehicle_id].set_velocity(double_value);
        return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
        vehicles[current_vehicle_id].set_acceleration(double_value);
        return 1;
    case DRIVER_DATA_VEH_LENGTH             :
        vehicles[current_vehicle_id].set_length(double_value);
        return 1;
    case DRIVER_DATA_VEH_WIDTH              :
        vehicles[current_vehicle_id].set_width(double_value);
        return 1;
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
        vehicles[current_vehicle_id].set_turning_indicator(long_value);
        return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
        vehicles[current_vehicle_id].set_category(long_value);
        return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
        vehicles[current_vehicle_id].set_preferred_relative_lane(
            long_value);
        return 1;
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        vehicles[current_vehicle_id].set_desired_velocity(double_value);
        return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
        return 1;
    case DRIVER_DATA_VEH_TYPE               :
        vehicles[current_vehicle_id].set_type(long_value);
        return 1;
    case DRIVER_DATA_VEH_COLOR              :
        vehicles[current_vehicle_id].set_color(long_value);
        return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
        vehicles[current_vehicle_id].set_link(long_value);
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
        switch (UDA(index1)) {
        case UDA::use_internal_lane_change_decision:
            vehicles[current_vehicle_id].
                set_use_internal_lane_change_decision(long_value);
            break;
        case UDA::write_veh_log:
            //vehicles[current_vehicle_id].set_verbose(long_value);
            break;
        default: // do nothing
            break;
        }
        return 1;
    case DRIVER_DATA_NVEH_ID                :
        if (long_value > 0) { 
            vehicles[current_vehicle_id].emplace_nearby_vehicle(
                long_value, index1, index2);
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
        return 1;
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_lateral_position(double_value);
        return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_distance(double_value);
        return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_relative_velocity(double_value);
        return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_acceleration(double_value);
        return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_length(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_width(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_category(long_value);
        return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_lane_change_direction(long_value);
        return 1;
    case DRIVER_DATA_NVEH_TYPE              :
        vehicles[current_vehicle_id].peek_nearby_vehicles()
            ->set_type(long_value);
        return 1;
    case DRIVER_DATA_NVEH_UDA               :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
        return 1;
    case DRIVER_DATA_LANE_END_DISTANCE      :
        vehicles[current_vehicle_id].set_lane_end_distance(
            double_value, index1);
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
        /* We save this value to compare it to our own algorithm.
        It can also be used in a situation where some assumption
        breaks or when the vehicle gets stuck. */
        vehicles[current_vehicle_id].set_vissim_acceleration(
            double_value);
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
        vehicles[current_vehicle_id].set_desired_lane_angle(double_value);
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
        /* We save this value to compare it to our own algorithm.
        It can also be used in a situation where some assumption
        breaks or when the vehicle gets stuck. */
        vehicles[current_vehicle_id].set_vissim_active_lane_change(
            long_value);
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
        vehicles[current_vehicle_id].set_rel_target_lane(long_value);
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

    EgoVehicle& ego_vehicle = vehicles[current_vehicle_id];
    /*if (LOGGED_VEHICLES_IDS.find(current_vehicle_id) !=
        LOGGED_VEHICLES_IDS.end()) {
        LoggedVehicle& ego_vehicle = (LoggedVehicle)vehicles[current_vehicle_id];;
    }*/

    switch (type) {
    case DRIVER_DATA_STATUS :
        *long_value = 0;
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
        *long_value = ego_vehicle.get_turning_indicator();
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        *double_value = ego_vehicle.get_desired_velocity();
        return 1;
    case DRIVER_DATA_VEH_COLOR :
        *long_value = ego_vehicle.get_color_by_controller_state();
        return 1;
    case DRIVER_DATA_VEH_UDA :
        switch (UDA(index1))
        {
        case UDA::gap_to_dest_lane_leader:
            *double_value = ego_vehicle.
                compute_gap(ego_vehicle.get_destination_lane_leader());
            break;
        case UDA::gap_to_dest_lane_follower:
            *double_value = ego_vehicle.
                compute_gap(ego_vehicle.get_destination_lane_follower());
            break;
        case UDA::safe_gap_to_dest_lane_leader:
            if (ego_vehicle.has_lane_change_intention()) {
                *double_value = ego_vehicle.compute_safe_lane_change_gap(
                    ego_vehicle.get_destination_lane_leader());
            }
            else {
                *double_value = 0.0;
            }
            break;
        case UDA::safe_gap_to_dest_lane_follower:
            if (ego_vehicle.has_lane_change_intention()) {
                *double_value = ego_vehicle.compute_safe_lane_change_gap(
                    ego_vehicle.get_destination_lane_follower());
            }
            else {
                *double_value = 0.0;
            }
            break;
        case UDA::gap_to_leader:
            *double_value = ego_vehicle.
                compute_gap(ego_vehicle.get_leader());
            break;
        case UDA::leader_id:
            *long_value = ego_vehicle.get_leader_id();
            break;
        case UDA::veh_following_gap_to_fd:
            *double_value = ego_vehicle.
                compute_time_headway_gap(
                    ego_vehicle.get_destination_lane_follower());
            break;
        case UDA::transient_gap_to_fd:
            *double_value = ego_vehicle.
                compute_transient_gap(
                    ego_vehicle.get_destination_lane_follower());
            break;
        case UDA::veh_following_gap_to_ld:
            *double_value = ego_vehicle.
                compute_time_headway_gap(
                    ego_vehicle.get_destination_lane_leader());
            break;
        case UDA::transient_gap_to_ld:
            *double_value = ego_vehicle.
                compute_transient_gap(
                    ego_vehicle.get_destination_lane_leader());
            break;
        case UDA::reference_gap:
            *double_value = ego_vehicle.get_reference_gap();
            break;
        case UDA::ttc:
            *double_value = ego_vehicle.get_ttc();
            break;
        case UDA::drac:
            *double_value = ego_vehicle.get_drac();
            break;
        case UDA::collision_severity_risk:
            *double_value = ego_vehicle.get_collision_risk();
            break;
        case UDA::write_veh_log:
            //*long_value = ego_vehicle.is_verbose();
            break;
        case UDA::relative_velocity_to_leader:
            *double_value = 
                ego_vehicle.get_relative_velocity_to_leader();
            break;
        case UDA::leader_type:
            *long_value = ego_vehicle.get_leader_type();
            break;
        default:
            return 0; /* doesn't set any UDA values */
        }
        return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
        *long_value = 1;
        return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
        if (CLUELESS_DEBUGGING) {
            std::clog << "deciding acceleration for veh. "
                << ego_vehicle.get_id() << std::endl;
        }
        *double_value = ego_vehicle.compute_desired_acceleration();
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
        *double_value = ego_vehicle.get_desired_lane_angle();
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
        if (CLUELESS_DEBUGGING) {
            std::clog << "deciding lane change for veh. "
                << ego_vehicle.get_id() << std::endl;
        }
        *long_value = ego_vehicle.decide_active_lane_change_direction();
        if (ego_vehicle.is_verbose()) {
            std::clog << "t=" << ego_vehicle.get_time()
                << ", id=" << ego_vehicle.get_id()
                << ", lane=" << ego_vehicle.get_lane()
                << ", pref. lane=" << ego_vehicle.get_preferred_relative_lane()
                << ", target lane=" << ego_vehicle.get_rel_target_lane()
                << ", active lc.=" << ego_vehicle.get_active_lane_change()
                << ", vissim active lc=" << ego_vehicle.get_vissim_active_lane_change()
                << ", lat pos.=" << ego_vehicle.get_lateral_position()
                << ", vel=" << ego_vehicle.get_velocity()
                << std::endl;
        }
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
        *long_value = ego_vehicle.get_rel_target_lane();
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

    EgoVehicle& ego_vehicle = vehicles[current_vehicle_id];

    switch (number) {
    case DRIVER_COMMAND_INIT :
        return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
        return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
        /*if (ego_vehicle.is_verbose()) {
        }*/
        vehicles.erase(current_vehicle_id);
        return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
        /* This is executed after all the set commands and before 
        any get command. */
        ego_vehicle.update_state();
        ego_vehicle.analyze_nearby_vehicles();
        //vehicles[current_vehicle_id].compute_all_ssms();
        return 1;
    default :
        return 0;
    }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
