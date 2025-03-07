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
#include "Platoon.h"
#include "PlatoonVehicle.h"
#include "PlatoonLCStrategyManager.h"
#include "SimulationLogger.h"
#include "TrafficLight.h"
#include "TrafficLightFileReader.h"

/*==========================================================================*/

std::unordered_set<long> logged_vehicles_ids{ };
const int LOGGED_PLATOON_ID{ 0 };
bool verbose_simulation{ false };
//const double DEBUGGING_START_TIME{ 249.0 };

SimulationLogger simulation_logger;
std::unordered_map<long, std::shared_ptr<EgoVehicle>> vehicles;
std::unordered_map<int, TrafficLight> traffic_lights;
std::unordered_map<int, std::shared_ptr<Platoon>> platoons;
double simulation_time_step{ -1.0 };
double current_time{ 0.0 };
long current_vehicle_type{ 0 };
long current_vehicle_id{ 0 };
double current_desired_velocity{ 0 };
long current_nearby_vehicle_id{ 0 };
long platoon_id{ 1 };
int platoon_lc_strategy{ 0 };
double max_computation_time{ 1.e5 }; // inf in practice

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
          //std::cout << platoon_lc_strategy_manager << std::endl;
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

    //if (verbose_simulation)
    //{
    //    std::cout << "Setting type " << type << std::endl;
    //}

    switch (type) {
    case DRIVER_DATA_PATH                   :
        std::cout << "DLL path: "
            << string_value
            << std::endl;
        return 1;
    case DRIVER_DATA_PARAMETERFILE          :
        if (string_value != NULL && string_value[0] != '\0')
        std::cout << "Parameter file path: "
            << string_value << std::endl;
        /* Only the traffic light ACC has a parameter file */
        TrafficLightFileReader::from_file_to_objects(
            std::string(string_value), traffic_lights);
        for (const std::pair<int, TrafficLight>& pair : traffic_lights)
        {
            std::cout << pair.second << "\n";
        }
        return 1;
    case DRIVER_DATA_TIMESTEP               :
        if (simulation_time_step < 0) 
        {
            simulation_time_step = double_value;
        }
        return 1;
    case DRIVER_DATA_TIME                   :
        if (verbose_simulation && (double_value != current_time))
        {
            std::cout << "t=" << double_value
                << ", " << vehicles.size() << " controlled vehicles.\n";
            /*std::cout << "t=" << current_time
                << ", " << platoons.size() << " platoons" << std::endl;
            for (auto& it : platoons)
            {
                std::cout << *it.second << "\n";
            }*/
        }
        current_time = double_value;
        return 1;
    case DRIVER_DATA_USE_UDA                :
        /* must return 1 for desired values of index1 if UDA values
        are to be sent from/to Vissim */
        if (index1 >= static_cast<int>(UDA::first))
        {
            UDA uda = UDA(index1);
            switch (uda)
            {
            /* The first UDAs are necessary */
            case UDA::h_to_assited_veh:
            case UDA::lane_change_request:
            case UDA::give_control_to_vissim:
            case UDA::max_lane_change_risk_to_leaders:
            case UDA::max_lane_change_risk_to_follower:
            case UDA::use_linear_lane_change_gap:
            case UDA::platoon_id:
            case UDA::platoon_strategy:
            case UDA::max_computation_time:
                return 1;
            case UDA::verbose_simulation:
                return 1;
            case UDA::logged_vehicle_id:
                return 1;
            /* Debugging: leader */
            case UDA::leader_id:
                return 1;
            case UDA::leader_type:
                return 0;
            case UDA::gap_to_leader:
                return 0;
            case UDA::reference_gap:
                return 0;
            case UDA::relative_velocity_to_leader:
                return 0;
            case UDA::safe_gap_to_leader:
                return 0;
            /* Debugging: dest lane leader */
            case UDA::dest_leader_id:
                return 1;
            case UDA::gap_to_dest_lane_leader:
            case UDA::transient_gap_to_ld:
            case UDA::veh_following_gap_to_ld:
            case UDA::safe_gap_to_dest_lane_leader:
            case UDA::delta_gap_to_ld:
            case UDA::lc_collision_free_gap_to_ld:
                return 0;
            /* Debugging: dest lane follower */
            case UDA::dest_follower_id:
                return 1;
            case UDA::gap_to_dest_lane_follower:
            case UDA::transient_gap_to_fd:
            case UDA::veh_following_gap_to_fd:
            case UDA::safe_gap_to_dest_lane_follower:
            case UDA::dest_follower_time_headway:
            case UDA::delta_gap_to_fd:
            case UDA::lc_collision_free_gap_to_fd:
                return 0;
            /* Debugging: virtual leaders */
            case UDA::assisted_veh_id:
                return 1;
            case UDA::virtual_leader_id:
                return 1;
            /* Debugging: other */
            case UDA::waiting_time:
                return 0;
            case UDA::risk:
                return 0;
            case UDA::safe_time_headway:
                return 0;
            case UDA::gap_error:
                return 1;
            case UDA::current_state:
                return 1;
            default:
                return 0;
            }
        }
        else {
            return 0;
        }
    case DRIVER_DATA_VEH_ID                 :
        if (verbose_simulation) {
            std::cout << "t=" << current_time
                << ", setting data for veh. " << long_value << std::endl;
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
        vehicles[current_vehicle_id]->set_distance_traveled(double_value);
        return 1;
    case DRIVER_DATA_VEH_LANE_ANGLE         :
        vehicles[current_vehicle_id]->set_orientation_angle(double_value);
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
        if (vehicles.find(current_nearby_vehicle_id) != vehicles.end())
        {
            vehicles[current_vehicle_id]->set_front_x(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_Y_COORDINATE       :
        if (vehicles.find(current_nearby_vehicle_id) != vehicles.end())
        {
            vehicles[current_vehicle_id]->set_front_y(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
        if (vehicles.find(current_nearby_vehicle_id) != vehicles.end())
        {
            vehicles[current_vehicle_id]->set_rear_x(double_value);
        }
        return 1;
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
        if (vehicles.find(current_nearby_vehicle_id) != vehicles.end())
        {
            vehicles[current_vehicle_id]->set_rear_y(double_value);
        }
        return 1;
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
        /* This seems to only indicate the target lane of an
        already active (or about to be active) lane change. */
        //vehicles[current_vehicle_id]->set_relative_target_lane(long_value);
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
        switch (UDA(index1))
        {
        case UDA::max_lane_change_risk_to_leaders:
            vehicles[current_vehicle_id]->
                set_max_lane_change_risk_to_leaders(double_value);
            break;
        case UDA::max_lane_change_risk_to_follower:
            vehicles[current_vehicle_id]->
                set_max_lane_change_risk_to_follower(double_value);
            break;
        case UDA::use_linear_lane_change_gap:
            vehicles[current_vehicle_id]->set_use_linear_lane_change_gap(
                long_value);
            break;
        case UDA::platoon_strategy:
            platoon_lc_strategy = long_value;
            break;
        case UDA::max_computation_time:
            max_computation_time = double_value;
            break;
        case UDA::verbose_simulation:
            verbose_simulation = long_value > 0;
            break;
        case UDA::logged_vehicle_id:
            /* If possible, we create the vehicle as verbose. This is done
            using the logged_vehicles_ids. If the vehicle is created before
            we have the chance to read logged ids from VISSIM, then using 
            set_verbose ensures it becomes verbose afterwards. */
            logged_vehicles_ids.insert(long_value);
            vehicles[current_vehicle_id]->set_verbose(
                current_vehicle_id == long_value);
            break;
        default: // do nothing
            break;
        }
        return 1;
    case DRIVER_DATA_NVEH_ID                :
        if (long_value > 0)
        {
            current_nearby_vehicle_id = long_value;
            vehicles[current_vehicle_id]->emplace_nearby_vehicle(
                long_value, index1, index2);
        }
        return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_orientation_angle(double_value);
        return 1;
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_lateral_position(double_value);
        return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_distance(double_value);
        return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_relative_velocity(double_value);
        return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_acceleration(double_value);
        return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_length(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_width(double_value);
        return 1;
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
        return 1;
    case DRIVER_DATA_NVEH_CATEGORY          :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_category(long_value);
        return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
        vehicles[current_vehicle_id]
            ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
            ->set_lane_change_direction(long_value);
        return 1;
    case DRIVER_DATA_NVEH_TYPE              :
        vehicles[current_vehicle_id]->set_nearby_vehicle_type(
            current_nearby_vehicle_id, long_value);
        return 1;
    case DRIVER_DATA_NVEH_UDA               :
        switch (UDA(index1))
        {
        case UDA::lane_change_request:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->read_lane_change_request(long_value);
            break;
        case UDA::h_to_assited_veh:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_h_to_incoming_vehicle(double_value);
            break;
        case UDA::max_lane_change_risk_to_follower:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_max_lane_change_risk_to_follower(double_value);
            break;
        case UDA::platoon_id:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_platoon_id(long_value);
            break;
        case UDA::dest_leader_id:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_destination_lane_leader_id(long_value);
            break;
        case UDA::dest_follower_id:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_destination_lane_follower_id(long_value);
            break;
        case UDA::assisted_veh_id:
            vehicles[current_vehicle_id]
                ->get_nearby_vehicle_by_id(current_nearby_vehicle_id)
                ->set_assisted_vehicle_id(long_value);
            break;
        default:
            break;
        }
        return 1;
    case DRIVER_DATA_NO_OF_LANES            :
        vehicles[current_vehicle_id]->set_number_of_lanes(long_value);
        return 1;
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
        return 1;
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
        return 1;
    /* IMPORTANT: Following are behavior data suggested for the current time
    step by Vissim's internal model */
    case DRIVER_DATA_DESIRED_ACCELERATION   :
        vehicles[current_vehicle_id]->set_vissim_acceleration(
            double_value);
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE     :
        /* This value is saved for debugging purposes since we usually set
        DRIVER_DATA_SIMPLE_LANECHANGE to 1 and return 0 when calling
        get data for DRIVER_DATA_DESIRED_LANE_ANGLE*/
        vehicles[current_vehicle_id]->set_desired_lane_angle(double_value);
        return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE     :
        /*vehicles[current_vehicle_id]->set_vissim_active_lane_change(
            long_value);*/
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE        :
        /* Apparently this is indicates when VISSIM considers the lane
        change can start. */
        vehicles[current_vehicle_id]->set_vissim_lane_suggestion(long_value);
        return 1;
    default :
        return 0;
    }
}

/*==========================================================================*/

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

    //if (verbose_simulation)
    //{
    //    std::cout << "Getting type " << type << "\n";
    //}

    switch (type) {
    case DRIVER_DATA_STATUS :
        *long_value = 0;
        return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
        /*if (verbose_simulation) {
            std::cout << "t=" << current_time
                << ", getting data for veh. " << current_vehicle_id 
                << std::endl;
        }*/
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
        /* The first ones are necessary */
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
        case UDA::platoon_id:
            *long_value = vehicles[current_vehicle_id]->get_platoon_id();
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
                compute_gap_to_a_leader(
                    vehicles[current_vehicle_id]->get_leader().get());
            break;
        case UDA::reference_gap:
            *double_value = vehicles[current_vehicle_id]->get_reference_gap();
            break;
        case UDA::relative_velocity_to_leader:
            *double_value =
                vehicles[current_vehicle_id]->get_relative_velocity_to_leader()
                * 3.6;  // we want to display it in km/h
            break;
        case UDA::safe_gap_to_leader:
            *double_value = vehicles[current_vehicle_id]->
                compute_safe_gap_to_leader();
            break;
        /* Debugging: destination lane leader */
        case UDA::dest_leader_id:
            *long_value = vehicles[current_vehicle_id]->
                get_destination_lane_leader_id();
            break;
        case UDA::gap_to_dest_lane_leader:
            *double_value = 
                vehicles[current_vehicle_id]->compute_gap_to_a_leader(
                vehicles[current_vehicle_id]->get_destination_lane_leader()
                    .get()
                );
            break;
        case UDA::transient_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                compute_transient_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader().get()
                    );
            break;
        case UDA::veh_following_gap_to_ld:
            *double_value = vehicles[current_vehicle_id]->
                compute_time_headway_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader().get()
                    );
            break;
        case UDA::safe_gap_to_dest_lane_leader:
            *double_value = vehicles[current_vehicle_id]->
                get_accepted_lane_change_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_leader().get()
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
                get_destination_lane_follower_id();
            break;
        case UDA::gap_to_dest_lane_follower:
            *double_value = vehicles[current_vehicle_id]->
                compute_gap_to_a_follower(vehicles[current_vehicle_id]->
                    get_destination_lane_follower().get()
                );
            break;
        case UDA::transient_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                compute_transient_gap(vehicles[current_vehicle_id]
                    ->get_destination_lane_follower().get()
                );
            break;
        case UDA::veh_following_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                compute_time_headway_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower().get()
                );
            break;
        case UDA::safe_gap_to_dest_lane_follower:
            *double_value = vehicles[current_vehicle_id]->
                get_accepted_lane_change_gap(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower().get()
                );
            break;
        case UDA::dest_follower_time_headway:
            *double_value = vehicles[current_vehicle_id]->
                get_dest_follower_time_headway();
            break;
        case UDA::delta_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                get_gap_variation_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower().get()
                );
            break;
        case UDA::lc_collision_free_gap_to_fd:
            *double_value = vehicles[current_vehicle_id]->
                get_collision_free_gap_to(
                    vehicles[current_vehicle_id]->
                    get_destination_lane_follower().get()
                );
            break;
        /* Debugging: assited vehicle */
        case UDA::assisted_veh_id:
            *long_value = vehicles[current_vehicle_id]->
                get_assisted_veh_id();
            break;
        case UDA::virtual_leader_id:
            *long_value = vehicles[current_vehicle_id]->
                get_virtual_leader_id();
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
        case UDA::current_state:
        {
            std::string state_name =
                vehicles[current_vehicle_id]->get_state()->get_state_name();
            /*std::cout << "*string_value size " << sizeof * string_value
                << ", string_value size " << sizeof string_value
                << ", *string_value strlen " << strlen(*string_value)
                << ", state_name size " << sizeof state_name.c_str()
                << ", state_name strlen " << strlen(state_name.c_str()) << "\n";*/
            strcpy(*string_value, state_name.c_str());
            break;
        }
        default:
            return 0; /* doesn't set any UDA values */
        }
        return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
        *long_value = 1;
        return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
        *double_value = vehicles[current_vehicle_id]
            ->get_desired_acceleration();
        return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
        /* Since we set DRIVER_DATA_SIMPLE_LANECHANGE to 1, we don't
        need to pass any values here. */
        /**double_value =
            vehicles[current_vehicle_id]->get_desired_lane_angle();*/
        return 0;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
        *long_value =
            vehicles[current_vehicle_id]->get_lane_change_direction_to_int();
        return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
        /* This is used by Vissim only if *long_value was set to 0 in the
        call of DriverModelGetValue (DRIVER_DATA_SIMPLE_LANECHANGE) */
        /**long_value =
            vehicles[current_vehicle_id]->get_relative_target_lane();*/
        return 0;
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

    if (verbose_simulation)
    {
        std::cout << "Executing command " << number << "\n";
    }

    switch (number) {
    case DRIVER_COMMAND_INIT :
        return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
    {
        bool verbose = logged_vehicles_ids.find(current_vehicle_id)
            != logged_vehicles_ids.end();
        vehicles[current_vehicle_id] = std::move(
            EgoVehicleFactory::create_ego_vehicle(current_vehicle_id,
                current_vehicle_type, current_desired_velocity,
                simulation_time_step, current_time, verbose)
            );

        current_vehicle_id = 0;
        return 1;
    }
    case DRIVER_COMMAND_KILL_DRIVER :
        if (verbose_simulation)
        {
            std::cout << "Erasing veh. " << current_vehicle_id << std::endl;
        }
        if (vehicles[current_vehicle_id]->is_in_a_platoon())
        {
            long current_platoon_id =
                vehicles[current_vehicle_id]->get_platoon()->get_id();
            platoons[current_platoon_id]->remove_vehicle_by_id(
                current_vehicle_id, true);
            if (platoons[current_platoon_id]->is_empty())
            {
                platoons.erase(current_platoon_id);
            }
        }
        vehicles.erase(current_vehicle_id);
        return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
    {
        /* This is executed after all the set commands and before
        any get command. */
        bool will_print = verbose_simulation
            || vehicles[current_vehicle_id]->is_verbose();
        if (will_print) 
        {
            std::cout << "----------- Veh: " << current_vehicle_id
                << " -----------\nUpdating states\n";
        }
        vehicles[current_vehicle_id]->update_state();

        if (will_print)
        {
            std::cout << "Analyzing nearby vehicles" << std::endl;
        }
        vehicles[current_vehicle_id]->analyze_nearby_vehicles();

        if (will_print)
        {
            std::cout << "Analyzing platoons" << std::endl;
        }

        if (vehicles[current_vehicle_id]->analyze_platoons(platoons,
            platoon_id, platoon_lc_strategy, max_computation_time))
        {
            platoons[platoon_id] =
                vehicles[current_vehicle_id]->share_platoon();
            if (platoon_id == LOGGED_PLATOON_ID)
            {
                platoons[platoon_id]->set_verbose(true);
            }
            platoon_id++;
        }

        if (will_print)
        {
            std::cout << "Deciding acceleration" << std::endl;
        }
        vehicles[current_vehicle_id]->compute_desired_acceleration(
            traffic_lights);

        if (will_print)
        {
            std::cout << "Command 'Move Driver' done." << std::endl;
        }

        if (vehicles[current_vehicle_id]->is_verbose())
        {
            std::cout << *vehicles[current_vehicle_id] << std::endl;
        }

        return 1;
    }
    default :
        return 0;
    }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
