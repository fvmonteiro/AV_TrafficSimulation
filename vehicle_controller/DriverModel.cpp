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
#include "EgoVehicle.h"
#include "EgoVehicleFactory.h"
#include "SimulationLogger.h"
#include "TrafficLight.h"
#include "TrafficLightFileReader.h"

/*==========================================================================*/

const std::unordered_set<long> LOGGED_VEHICLES_IDS{ 9 };
const bool CLUELESS_DEBUGGING{ false };
//const double DEBUGGING_START_TIME{ 249.0 };

SimulationLogger simulation_logger;
std::unordered_map<long, std::unique_ptr<EgoVehicle>> vehicles;
std::unordered_map<int, TrafficLight> traffic_lights;
int i, j;  // temp.; used for debugging
double simulation_time_step{ -1.0 };
double current_time{ 0.0 };
long current_vehicle_type{ 0 };
long current_vehicle_id{ 0 };
double current_desired_velocity{ 0 };

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
    case DRIVER_DATA_PARAMETERFILE          :
        if (string_value != NULL && string_value[0] != '\0')
        std::clog << "Parameter file path: "
            << string_value << std::endl;
        /* Only the traffic light ACC has a parameter file */
        TrafficLightFileReader::from_file_to_objects(
            std::string(string_value), traffic_lights);
        for (const std::pair<int, TrafficLight>& pair : traffic_lights)
        {
            std::clog << pair.second << "\n";
        }
        return 1;
    case DRIVER_DATA_TIMESTEP               :
        if (simulation_time_step < 0) {
            simulation_time_step = double_value;
        }
        return 1;
    case DRIVER_DATA_TIME                   :
        /*if (double_value > DEBUGGING_START_TIME)
        {
            CLUELESS_DEBUGGING = true;
        }*/
        if (CLUELESS_DEBUGGING && (double_value != current_time))
        {
            std::clog << "t=" << current_time 
                << ", " << vehicles.size() << " vehicles." << std::endl;
        }
        current_time = double_value;
        return 1;
    case DRIVER_DATA_USE_UDA                :
        /* must return 1 for desired values of index1 if UDA values
        are to be sent from/to Vissim */
        if (index1 >= static_cast<int>(UDA::first)) 
        { 
            UDA uda = UDA(index1);
            switch (uda) {
            /* The first three are necessary */
            case UDA::h_to_assited_veh:
            case UDA::lane_change_request: 
            case UDA::give_control_to_vissim:
            case UDA::max_lane_change_risk:
                return 1;
            /* Debugging: leader */
            case UDA::leader_id:
                return 1;
            case UDA::leader_type:
            case UDA::gap_to_leader:
            case UDA::reference_gap:
            case UDA::relative_velocity_to_leader:
                return 0;
            /* Debugging: dest lane leader */
            case UDA::dest_leader_id:
                return 1;
            case UDA::gap_to_dest_lane_leader:
                return 1;
            case UDA::transient_gap_to_ld:
                return 1;
            case UDA::veh_following_gap_to_ld:
                return 1;
            case UDA::safe_gap_to_dest_lane_leader:
                return 1;
            case UDA::delta_gap_to_ld:
                return 1;
            case UDA::lc_collision_free_gap_to_ld:
                return 1;
            /* Debugging: dest lane follower */
            case UDA::dest_follower_id:
                return 1;
            case UDA::gap_to_dest_lane_follower:
                return 1;
            case UDA::transient_gap_to_fd:
                return 1;
            case UDA::veh_following_gap_to_fd:
                return 1;
            case UDA::safe_gap_to_dest_lane_follower:
                return 1;
            case UDA::dest_follower_time_headway:
                return 0;
            case UDA::delta_gap_to_fd:
                return 1;
            case UDA::lc_collision_free_gap_to_fd:
                return 1;
            /* Debugging: assisted vehicle */
            case UDA::assisted_veh_id:
                return 1;
            /* Debugging: other */
            case UDA::waiting_time:
                return 0;
            case UDA::risk:
                return 0;
            case UDA::safe_time_headway:
                return 0;
            case UDA::gap_error:
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
            std::clog << "t=" << current_time
                << ", getting data for veh. " << long_value << std::endl;
        }

        current_vehicle_id = long_value;
        if (vehicles.find(current_vehicle_id) != vehicles.end()) 
        {
            vehicles[current_vehicle_id]->clear_nearby_vehicles();
        }
        return 1;
    case DRIVER_DATA_VEH_LANE               :
        vehicles[current_vehicle_id]->set_lane(long_value);
        return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
        return 1;
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
        vehicles[current_vehicle_id]->set_lateral_position(double_value);
        return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
        vehicles[current_vehicle_id]->set_velocity(double_value);
        return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
        vehicles[current_vehicle_id]->set_acceleration(double_value);
        return 1;
    case DRIVER_DATA_VEH_LENGTH             :
        vehicles[current_vehicle_id]->set_length(double_value);
        return 1;
    case DRIVER_DATA_VEH_WIDTH              :
        vehicles[current_vehicle_id]->set_width(double_value);
        return 1;
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
        vehicles[current_vehicle_id]->set_turning_indicator(long_value);
        return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
        vehicles[current_vehicle_id]->set_category(long_value);
        return 1;
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
        vehicles[current_vehicle_id]->set_preferred_relative_lane(
            long_value);
        return 1;
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
        vehicles[current_vehicle_id]->set_vissim_use_preferred_lane(
            long_value);
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        current_desired_velocity = double_value;
        if (vehicles.find(current_vehicle_id) != vehicles.end())
        {
            vehicles[current_vehicle_id]->set_desired_velocity(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
        return 1;
    case DRIVER_DATA_VEH_TYPE               :
        /* We only use this information when a new vehicle is created */
        current_vehicle_type = long_value;
        /*if (vehicles.find(current_vehicle_id) != vehicles.end()) {
            vehicles[current_vehicle_id]->set_type(long_value);
        }*/
        return 1;
    case DRIVER_DATA_VEH_COLOR              :
        // We define the vehicle color instead
        //vehicles[current_vehicle_id]->set_color(long_value);
        return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
        vehicles[current_vehicle_id]->set_link(long_value);
        return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS
                messages) */
                /* Must return 1 if these messages are to be sent from
                VISSIM! */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
        return 0;
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
        vehicles[current_vehicle_id]->set_active_lane_change_direction(
            long_value);
        return 1;
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
        case UDA::max_lane_change_risk:
            vehicles[current_vehicle_id]->set_maximum_lane_change_risk(
                double_value);
            break;
        default: // do nothing
            break;
        }
        return 1;
    case DRIVER_DATA_NVEH_ID                :
        if (long_value > 0) 
        {
            vehicles[current_vehicle_id]->emplace_nearby_vehicle(
                long_value, index1, index2);
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
        return 1;
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_lateral_position(double_value);
        return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_distance(double_value);
        return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_relative_velocity(double_value);
        return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_acceleration(double_value);
        return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_length(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_width(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
        return 1;
    case DRIVER_DATA_NVEH_CATEGORY          :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_category(long_value);
        return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
        vehicles[current_vehicle_id]->peek_nearby_vehicles()
            ->set_lane_change_direction(long_value);
        return 1;
    case DRIVER_DATA_NVEH_TYPE              :
        vehicles[current_vehicle_id]->set_nearby_vehicle_type(long_value);
        return 1;
    case DRIVER_DATA_NVEH_UDA               :
        switch (UDA(index1))
        {
        case UDA::lane_change_request:
            vehicles[current_vehicle_id]->peek_nearby_vehicles()
                ->read_lane_change_request(long_value);
            break;
        case UDA::h_to_assited_veh:
            vehicles[current_vehicle_id]->peek_nearby_vehicles()
                ->set_h_to_incoming_vehicle(double_value);
            break;
        default:
            break;
        }
        return 1;
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
        return 1;
    case DRIVER_DATA_LANE_END_DISTANCE      :
        vehicles[current_vehicle_id]->set_lane_end_distance(
            double_value, index1);
        return 1;
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
        return 1;
    case DRIVER_DATA_SIGNAL_DISTANCE        :
        vehicles[current_vehicle_id]->read_traffic_light(
            index1, double_value);
        return 1;
    case DRIVER_DATA_SIGNAL_STATE           :
        /* This is called once for each signal head at the start of 
        every simulation step. And then once again for each vehicle. */
        traffic_lights[index1].set_current_state(long_value);
        return 1;
    case DRIVER_DATA_SIGNAL_STATE_START     :
        /* Called once for each vehicle close to the signal head, so
        we may set the same value several times. */
        traffic_lights[index1].set_current_state_start_time(double_value);
        return 1;
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
        return 1;
    /* IMPORTANT: Following are behavior data suggested for the current time
    step by Vissim's internal model */
    case DRIVER_DATA_DESIRED_ACCELERATION   :
        vehicles[current_vehicle_id]->set_vissim_acceleration(
            double_value);
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
        vehicles[current_vehicle_id]->set_desired_lane_angle(double_value);
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
        /*vehicles[current_vehicle_id]->set_vissim_active_lane_change(
            long_value);*/
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
        /* Apparently this is VISSIM's suggestion of target lane */
        vehicles[current_vehicle_id]->set_relative_target_lane(long_value);
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

    //EgoVehicle& ego_vehicle = vehicles[current_vehicle_id];

    switch (type) {
    case DRIVER_DATA_STATUS :
        *long_value = 0;
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
        *long_value = vehicles[current_vehicle_id]->get_turning_indicator();
        return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
        *double_value = vehicles[current_vehicle_id]->get_desired_velocity();
        return 1;
    case DRIVER_DATA_VEH_COLOR :
        *long_value = vehicles[current_vehicle_id]->
            get_color_by_controller_state();
        return 1;
    case DRIVER_DATA_VEH_UDA :
        switch (UDA(index1))
        {
        /* The first three are necessary */
        case UDA::h_to_assited_veh:
           *double_value = vehicles[current_vehicle_id]->
               get_time_headway_to_assisted_vehicle();
            break;
        case UDA::lane_change_request:
            *long_value = vehicles[current_vehicle_id]->
                get_lane_change_request();
                //get_desired_lane_change_direction().to_int();
            break;
        case UDA::give_control_to_vissim:
            *long_value = vehicles[current_vehicle_id]->
                is_vissim_controlling_lane_change();
            break;

        /* Debugging: leader */
        case UDA::leader_id:
            *long_value = vehicles[current_vehicle_id]->get_leader_id();
            break;
        case UDA::leader_type:
            // TODO: delete this UDA
            if (vehicles[current_vehicle_id]->has_leader()) {
                *long_value = static_cast<int>(
                    vehicles[current_vehicle_id]->get_leader()->get_type());
            }
            else {
                *long_value = 0;
            }
            break;
        case UDA::gap_to_leader:
            *double_value = vehicles[current_vehicle_id]->
                compute_gap(vehicles[current_vehicle_id]->get_leader());
            break;
        case UDA::reference_gap:
            *double_value = vehicles[current_vehicle_id]->get_reference_gap();
            break;
        case UDA::relative_velocity_to_leader:
            *double_value =
                vehicles[current_vehicle_id]->get_relative_velocity_to_leader()
                * 3.6;  // we want to display it in km/h
            break;
        /* Debugging: destination lane leader */
        case UDA::dest_leader_id:
            *long_value = vehicles[current_vehicle_id]->
                get_dest_lane_leader_id();
            break;
        case UDA::gap_to_dest_lane_leader:
            *double_value = vehicles[current_vehicle_id]->compute_gap(
                vehicles[current_vehicle_id]->get_destination_lane_leader()
                );
            break;
        case UDA::transient_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                compute_transient_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader()
                    );
            break;
        case UDA::veh_following_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                compute_time_headway_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader()
                    );
            break;
        case UDA::safe_gap_to_dest_lane_leader:
            *double_value = vehicles[current_vehicle_id]->
                compute_safe_lane_change_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader()
                );
            break;
        /*case UDA::delta_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                get_gap_variation_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader()
                );
            break;
        case UDA::lc_collision_free_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                get_collision_free_gap_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader()
                );
            break;*/
        /* Debugging: destination lane follower */
        case UDA::dest_follower_id:
            *long_value = vehicles[current_vehicle_id]->
                get_dest_lane_follower_id();
            break;
        case UDA::gap_to_dest_lane_follower:
            *double_value = vehicles[current_vehicle_id]->
                compute_gap(vehicles[current_vehicle_id]->
                    get_destination_lane_follower()
                );
            break;
        case UDA::transient_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                compute_transient_gap(
                    vehicles[current_vehicle_id]->get_destination_lane_follower()
                );
            break;
        case UDA::veh_following_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                compute_time_headway_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower()
                );
            break;
        case UDA::safe_gap_to_dest_lane_follower:
            *double_value = vehicles[current_vehicle_id]->
                compute_safe_lane_change_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower()
                );
            break;
        case UDA::dest_follower_time_headway:
            *double_value = vehicles[current_vehicle_id]->
                get_dest_follower_time_headway();
            break;
        /*case UDA::delta_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                get_gap_variation_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower()
                );
            break;
        case UDA::lc_collision_free_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                get_collision_free_gap_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower()
                );
            break;*/
        /* Debugging: assited vehicle */
        case UDA::assisted_veh_id:
            *long_value = vehicles[current_vehicle_id]->
                get_assisted_veh_id();
            break;
        /* Debugging: others */
        case UDA::waiting_time:
            *double_value = vehicles[current_vehicle_id]->get_waiting_time();
            break;
        case UDA::risk:
            *double_value = 0.0;
            /*vehicles[current_vehicle_id]->
                compute_collision_severity_risk_to_leader() * 3.6;  */
            // cause we want to display it in km/h;
            break;
        case UDA::safe_time_headway:
            *double_value = vehicles[current_vehicle_id]->
                get_safe_time_headway();
            break;
        case UDA::gap_error:
            *double_value = vehicles[current_vehicle_id]->
                get_gap_error();
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
                << vehicles[current_vehicle_id]->get_id() << std::endl;
        }
        *double_value = 
            vehicles[current_vehicle_id]->get_desired_acceleration(
            traffic_lights);
        if (CLUELESS_DEBUGGING) {
            std::clog << "decided acceleration for veh. "
                << vehicles[current_vehicle_id]->get_id() << std::endl;
        }
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
        *double_value = 
            vehicles[current_vehicle_id]->get_desired_lane_angle();
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
        if (CLUELESS_DEBUGGING) {
            std::clog << "deciding lane change for veh. "
                << vehicles[current_vehicle_id]->get_id() << std::endl;
        }
        *long_value = 
            vehicles[current_vehicle_id]->decide_lane_change_direction();
        if (CLUELESS_DEBUGGING) {
            std::clog << "decided lane change " << *long_value
                << " for veh. " 
                << vehicles[current_vehicle_id]->get_id() << std::endl;
        }

        if (vehicles[current_vehicle_id]->is_verbose()) 
        {
            std::clog << *vehicles[current_vehicle_id] << std::endl;
        }
        
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
        /* This is used by Vissim only if *long_value was set to 0 in the 
        call of DriverModelGetValue (DRIVER_DATA_SIMPLE_LANECHANGE) */
        /**long_value = 
            vehicles[current_vehicle_id]->get_relative_target_lane();*/
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

    switch (number) {
    case DRIVER_COMMAND_INIT :
        return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
    {
        bool verbose = false;
        if (LOGGED_VEHICLES_IDS.find(current_vehicle_id)
            != LOGGED_VEHICLES_IDS.end()) verbose = true;
        vehicles[current_vehicle_id] = std::move(
            EgoVehicleFactory::create_ego_vehicle(current_vehicle_id, 
                current_vehicle_type, current_desired_velocity, 
                simulation_time_step, current_time, verbose)
            );
        current_vehicle_id = 0;
        return 1;
    }
    case DRIVER_COMMAND_KILL_DRIVER :
        if (CLUELESS_DEBUGGING)
        {
            std::clog << "Erasing veh. " << current_vehicle_id << std::endl;
        }
        vehicles.erase(current_vehicle_id);
        return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
    {
        /* This is executed after all the set commands and before
        any get command. */
        if (CLUELESS_DEBUGGING) {
            std::clog << "Updating states" << std::endl;
        }
        vehicles[current_vehicle_id]->update_state();
        
        if (CLUELESS_DEBUGGING)
        {
            std::clog << "Analyzing nearby vehicles" << std::endl;
        }
        vehicles[current_vehicle_id]->analyze_nearby_vehicles();
        return 1;
    }
    default :
        return 0;
    }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
