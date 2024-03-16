#include <algorithm>
#include <fstream>
#include <map>
#include <numeric>


#include "Platoon.h"
#include "PlatoonLaneChangeApproach.h"
#include "PlatoonVehicle.h"
#include "PlatoonVehicleState.h"

PlatoonLaneChangeApproach::PlatoonLaneChangeApproach(int id, std::string name,
	bool verbose) 
	: id(id), name(name), verbose(verbose) {}

long PlatoonLaneChangeApproach::get_desired_destination_lane_leader(
	int ego_position) 
{
	PlatoonVehicle* ego_vehicle = 
		platoon->get_a_vehicle_by_position(ego_position);

	if (!ego_vehicle->has_lane_change_intention() || !is_initialized
		|| !is_vehicle_turn_to_lane_change(ego_position)) return 0;

	long virtual_leader_id{ 0 };
	
	if (maneuver_step == 0)
	{
		/* The first vehicles to simultaneously change lanes do so behind the
		destination lane leader of the front-most vehicle (similar to
		single vehicle lane change) */
		int first_lc_pos = *std::max_element(
			platoon_lane_change_order.lc_order[0].begin(),
			platoon_lane_change_order.lc_order[0].end());
		const PlatoonVehicle* front_most_veh =
			platoon->get_a_vehicle_by_position(first_lc_pos);
		virtual_leader_id = 
			front_most_veh->get_suitable_destination_lane_leader_id();
		platoon_destination_lane_leader_id = virtual_leader_id;
		if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id)
			&& virtual_leader_id > 0)
		{
			ego_vehicle->add_nearby_vehicle_from_another(*front_most_veh, 
				virtual_leader_id);
		}
	}
	else
	{
		int coop_pos = get_current_coop_vehicle_position();
		if (coop_pos == -1)
		{
			const PlatoonVehicle* veh = platoon->get_a_vehicle_by_position(
				last_dest_lane_vehicle_pos);
			virtual_leader_id = veh->get_id();
			if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
			{
				ego_vehicle->add_another_as_nearby_vehicle(*veh);
			}
		}
		else
		{
			const PlatoonVehicle* coop_veh =
				platoon->get_a_vehicle_by_position(coop_pos);
			virtual_leader_id = coop_veh->get_leader_id();
			if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id)
				&& virtual_leader_id > 0)
			{
				ego_vehicle->add_nearby_vehicle_from_another(*coop_veh,
					virtual_leader_id);
			}
		}
	}

	return virtual_leader_id;
}

long PlatoonLaneChangeApproach::get_assisted_vehicle_id(int ego_position) 
const
{
	const PlatoonVehicle* ego_vehicle =
		platoon->get_a_vehicle_by_position(ego_position);

	if (ego_vehicle->has_lane_change_intention() || !is_initialized
		|| ego_position != get_current_coop_vehicle_position()) 
	{
		return 0;
	}

	int rear_most_pos = get_rearmost_lane_changing_vehicle_position();
	const PlatoonVehicle* assisted_vehicle =
		platoon->get_a_vehicle_by_position(rear_most_pos);
	return assisted_vehicle->has_lane_change_intention() ?
		assisted_vehicle->get_id() : 0;
}

long PlatoonLaneChangeApproach::get_cooperating_vehicle_id() const
{
	if (is_initialized)
	{
		int coop_veh_position = get_current_coop_vehicle_position();
		if (coop_veh_position >= 0)
		{
			return platoon->get_a_vehicle_by_position(
				coop_veh_position)->get_id();
		}
	}
	return 0;
}

long PlatoonLaneChangeApproach
::get_platoon_destination_lane_leader_id() const
{
	return platoon_destination_lane_leader_id;
}

bool PlatoonLaneChangeApproach::can_vehicle_start_lane_change(
	int ego_position)
{	
	if (verbose) std::clog << "[PlatoonLaneChangeApproach::"
		"can_vehicle_start_lane_change]\n";

	if (!is_initialized && ego_position == 0) decide_lane_change_order();
	if (!is_initialized) return false;

	if (maneuver_step >= platoon_lane_change_order.number_of_steps())
	{
		std::clog << "[WARNING] maneuver step counter greater than "
			"total maneuver steps\n";
		return false;
	}

	std::unordered_set<int> next_to_move = get_current_lc_vehicle_positions();
	bool is_my_turn = is_vehicle_turn_to_lane_change(ego_position);

	if (verbose)
	{
		std::string message = "[PlatoonLaneChangeApproach::"
			"can_vehicle_start_lane_change] Next to move: ";
		for (int i : next_to_move) message += std::to_string(i) + ", ";
		message += "my pos. = " + std::to_string(ego_position)
			+ "-> is my turn? " + boolean_to_string(is_my_turn);
		std::clog << message << "\n";
	}

	check_maneuver_step_done(next_to_move);

	if (is_my_turn)
	{
		for (int i : next_to_move)
		{
			const PlatoonVehicle* vehicle = 
				platoon->get_a_vehicle_by_position(i);
			if (!vehicle->is_at_right_lane_change_gap())
			{
				if (verbose) std::clog << "\tnot at right lc gap\n";
				return false;
			}
			if (!vehicle->are_surrounding_gaps_safe_for_lane_change())
			{
				if (verbose) std::clog << "\tnot safe gap\n";
				return false;
			}
		}
		return true;
	}
	return false;
}

bool PlatoonLaneChangeApproach::can_vehicle_leave_platoon(int veh_position) 
const
{
	return implement_can_vehicle_leave_platoon(veh_position);
}

long PlatoonLaneChangeApproach::create_platoon_lane_change_request(
	int ego_position) const
{
	/* We're not running scenarios with non-platoon cooperative vehicles, 
	* so this is irrelevant for now (Jan 2024). If we need to run cooperative
	* scenarios, this function must return the id of the vehicle behind the
	* platoon vehicle's virtual
	*/

	// Skeleton
	//if (is_initialized && is_vehicle_turn_to_lane_change(ego_position))
	//{
	//	if (platoon_vehicle.has_virtual_leader())
	//	{
	//		// figure out the vehicle behind the virtual leader
	//	}
	//}
	return 0;
}

void PlatoonLaneChangeApproach::set_platoon_lane_change_order(
	LCOrder lc_order, std::vector<int> coop_order)
{
	set_platoon_lane_change_order(
		PlatoonLaneChangeOrder(lc_order, coop_order));
}

void PlatoonLaneChangeApproach::set_platoon_lane_change_order(
	PlatoonLaneChangeOrder plco)
{
	maneuver_step = 0;
	platoon_lane_change_order = plco;

	if (verbose) std::clog << "Order set to: "
		<< platoon_lane_change_order << "\n";

	last_dest_lane_vehicle_pos =
		get_rearmost_lane_changing_vehicle_position();
	is_initialized = true;
}

bool PlatoonLaneChangeApproach::implement_can_vehicle_leave_platoon(
	int ego_position) const
{
	return false;
}

bool PlatoonLaneChangeApproach::is_vehicle_turn_to_lane_change(
	int veh_position) const
{
	std::unordered_set<int> current_movers =
		get_current_lc_vehicle_positions();
	return current_movers.find(veh_position) != current_movers.end();
}

void PlatoonLaneChangeApproach::check_maneuver_step_done(
	const std::unordered_set<int>& lane_changing_veh_ids)
{
	bool all_are_done = true;
	for (int i : lane_changing_veh_ids)
	{
		const PlatoonVehicle* vehicle =
			platoon->get_a_vehicle_by_position(i);
		if (!vehicle->get_has_completed_lane_change())
		{
			all_are_done = false;
			break;
		}
	}
	if (all_are_done)
	{
		if (get_current_coop_vehicle_position() == -1)
		{
			/* If the vehicle completed a maneuver behind all others (no
			coop), it is now the last vehicle */
			last_dest_lane_vehicle_pos =
				get_rearmost_lane_changing_vehicle_position();
		}
		maneuver_step++;
	}

	if (verbose) std::clog << "[PlatoonLaneChangeApproach] is maneuver "
		"step done? " << boolean_to_string(maneuver_step) << "\n";
}

int PlatoonLaneChangeApproach
::get_rearmost_lane_changing_vehicle_position() const
{
	if (verbose) std::clog << "[[PlatoonLaneChangeApproach]"
		<< "Looking for rearmost lc veh.\n";

	double min_x = INFINITY;
	int rear_most_pos = 0;
	for (int pos : get_current_lc_vehicle_positions())
	{
		const PlatoonVehicle* vehicle = 
			platoon->get_a_vehicle_by_position(pos);
		if (vehicle->get_distance_traveled() < min_x)
		{
			min_x = vehicle->get_distance_traveled();
			rear_most_pos = pos;
		}
	}

	if (verbose) std::clog << "\tFound at pos. " << rear_most_pos << "\n";
	return rear_most_pos;
}

/* ------------------------------------------------------------------------ */
/* Fixed Order Approaches ------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void SynchoronousApproach::decide_lane_change_order()
{
	/*std::vector<int> l1(platoon->get_size());
	std::iota(l1.begin(), l1.end(), 0);*/
	std::unordered_set<int> lc_set_1;
	std::vector<int> coop_order{ {-1} };
	for (int i = 0; i < platoon->get_size(); i++)
	{
		lc_set_1.insert(i);
	}
	LCOrder lc_order = { lc_set_1 };
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LeaderFirstApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	std::vector<int> coop_order;
	for (int i = 0; i < platoon->get_size(); i++)
	{
		std::unordered_set<int> lc_set;
		lc_set.insert(i);
		lc_order.push_back(lc_set);
		coop_order.push_back(-1);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LastVehicleFirstApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	std::vector<int> coop_order;
	int n = platoon->get_size();
	for (int i = 0; i < n; i++)
	{
		if (lc_order.empty()) coop_order.push_back(-1);
		else coop_order.push_back(*lc_order.back().begin());

		std::unordered_set<int> lc_set;
		lc_set.insert(n - i - 1);
		lc_order.push_back(lc_set);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}

void LeaderFirstReverseApproach::decide_lane_change_order()
{
	LCOrder lc_order;
	std::vector<int> coop_order;
	int n = platoon->get_size();
	for (int i = 0; i < n; i++)
	{
		if (lc_order.empty()) coop_order.push_back(-1);
		else coop_order.push_back(*lc_order.back().begin());

		std::unordered_set<int> lc_set;
		lc_set.insert(i);
		lc_order.push_back(lc_set);
	}
	set_platoon_lane_change_order(lc_order, coop_order);
}

/* ------------------------------------------------------------------------ */
/* Graph Approach --------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

void GraphApproach::decide_lane_change_order()
{
	if (verbose) std::clog << "[GraphApproach] decide_lane_change_order\n";
	// TODO: cost name as simulation parameter
	if (!is_data_loaded)
	{
		std::string cost_name = "accel";
		strategy_manager = PlatoonLCStrategyManager(cost_name, verbose);
		strategy_manager.initialize(platoon->get_size());
	}

	std::vector<Query> queries = create_all_queries();
	if (verbose)
	{
		for (const Query& q : queries)
		{
			std::clog << "Query: " << "qx=" << vector_to_string(q.first)
				<< " , first_movers= " << set_to_string(q.second) << "\n";
		}
	}
	PlatoonLaneChangeOrder plco = get_query_results_from_map(queries);
	
	//set_maneuver_initial_state_for_all_vehicles();
	//PlatoonLaneChangeOrder plco2 = find_best_order_in_map();
	if (plco.cost > -1)  // an order was found
	{
		set_platoon_lane_change_order(plco);
	}
}

std::vector<Query> GraphApproach::create_all_queries()
{
	if (verbose) std::clog << "[GraphApproach] Getting all queries \n";

	/* Now we check which platoon vehicles can move and set possible
	destination lane leaders */
	std::vector<Query> queries_safe_start;
	std::vector<Query> queries_delayed_start;
	int veh_pos = 0;
	while (veh_pos < platoon->get_size())
	{
		int front_most_lc_veh_pos = veh_pos;
		const PlatoonVehicle* vehicle = 
			platoon->get_a_vehicle_by_position(veh_pos);
		std::set<int> first_movers;
		bool are_lc_gaps_safe = false;
		/* We try to create the largest possible set with consecutive
		vehicles in safe-to-start-lc positions */
		while (veh_pos < platoon->get_size()
			&& (vehicle->are_surrounding_gaps_safe_for_lane_change()))
		{
			are_lc_gaps_safe = true;
			first_movers.insert(veh_pos);
			veh_pos++;
			vehicle = platoon->get_a_vehicle_by_position(veh_pos);
		}
		/* If no vehicle so far is safe to start a lane change, we store
		queries of vehicles with suitable (safe after adjustment)
		lane change gaps. */
		if (first_movers.size() == 0 && queries_safe_start.size() == 0
			&& vehicle->get_is_space_suitable_for_lane_change())
		{
			are_lc_gaps_safe = false;
			first_movers.insert(veh_pos);
		}

		if (first_movers.size() > 0)
		{
			std::vector<ContinuousStateVector> system_state_matrix =
				platoon->get_traffic_states_around_a_vehicle(
					front_most_lc_veh_pos);
			OuterKey quantized_state_vector = flatten_state_matrix<int>(
				state_quantizer.quantize_states(system_state_matrix));
			if (verbose)
			{
				std::clog << "\t[GraphApproach] Pre-query\n\t-state:\n";
				for (const ContinuousStateVector& s : system_state_matrix)
				{
					std::clog << "\t\t" << s.to_string() << "\n";
				}
				std::clog << "\t-first_movers: " 
					<< set_to_string(first_movers) << "\n";
			}
			/* Add query to the proper query vector */
			std::vector<Query>* current_query = are_lc_gaps_safe ?
				&queries_safe_start : &queries_delayed_start;
			current_query->push_back(std::make_pair(
					quantized_state_vector, first_movers));
		}

		/* It's always OK to skip the current not-safe-to-start vehicle */
		veh_pos++;
	}

	/* We prioritize queries where vehicles can start maneuver right away */
	return queries_safe_start.size() > 0 ?
		queries_safe_start : queries_delayed_start;
}

void GraphApproach::set_maneuver_initial_state_for_all_vehicles()
{
	if (verbose) std::clog << "[GraphApproach] Setting all initial states \n";

	/* Now we check which platoon vehicles can move and set possible
	destination lane leaders */
	bool all_states_found = true;
	const std::vector<PlatoonVehicle*> all_vehicles =
		platoon->get_vehicles_by_position();
	for (int veh_pos = 0; veh_pos < all_vehicles.size(); veh_pos++)
	{
		const PlatoonVehicle* vehicle = all_vehicles[veh_pos];

		if (vehicle->get_is_space_suitable_for_lane_change())
		{
			std::vector<ContinuousStateVector> system_state_matrix =
				platoon->get_traffic_states_around_a_vehicle(veh_pos);
			
			if (verbose)
			{
				std::clog << "[GraphApproach] Setting initial state"
					"for veh at pos " << veh_pos << ":\n";
				for (const ContinuousStateVector& s : system_state_matrix)
				{
					std::clog << "\t\t" << s.to_string() << "\n";
				}
			}

			try
			{
				strategy_manager.set_maneuver_initial_state(veh_pos,
					system_state_matrix);
				if (verbose) std::clog << "\tState found.\n";
			}
			catch (const StateNotFoundException& e)
			{
				all_states_found = false;
				//std::vector<double> states_array = 
				//	flatten_state_matrix(system_state_matrix);
				std::clog << "\tState " << vector_to_string(e.state_vector)
					<< " not found.\n";
				ContinuousStateVector& lo_states = system_state_matrix.front();
				ContinuousStateVector& ld_states = system_state_matrix.back();
				save_not_found_state_to_file(e.state_vector,
					lo_states.get_vel(), ld_states.get_vel());
			}
			system_state_matrix.pop_back();
		}
	}

	/* Not sure this design is good. The goal is to save all exception found
	during the loop and only throw again once the loop is done. */
	if (!all_states_found)
	{
		throw StateNotFoundException();
	}
}

PlatoonLaneChangeOrder GraphApproach::get_query_results_from_map(
	std::vector<Query>& queries)
{
	if (verbose) std::clog << "[GraphApproach] Getting query results\n";

	double best_cost{ INFINITY };
	PlatoonLaneChangeOrder best_order;
	bool all_queries_answered = true;
	int n = platoon->get_size();

	std::vector<PlatoonLaneChangeOrder> all_orders; // for debugging
	for (Query q : queries)
	{
		try
		{
			PlatoonLaneChangeOrder an_order = strategy_manager.answer_query(q);
			all_orders.push_back(an_order);
			if (an_order.cost < best_cost)
			{
				best_cost = an_order.cost;
				best_order = an_order;
				if (verbose) std::clog << "\tbest order so far: "
					<< best_order.to_string() << "\n";
			}
		}
		catch (const std::out_of_range&)
		{
			/* we want to save all the non answered queries
			so we keep the loop running */
			all_queries_answered = false;
		}
	}

	if (!all_queries_answered)
	{
		for (PlatoonVehicle* vehicle : platoon->get_vehicles_by_position())
		{
			vehicle->give_up_lane_change();
		}
		return PlatoonLaneChangeOrder();
		//throw std::out_of_range("Some queries not found in stratey map");
	}

	return best_order;
}

PlatoonLaneChangeOrder GraphApproach::find_best_order_in_map()
{
	if (verbose) std::clog << "[GraphApproach] Looking for best order\n";

	double best_cost{ INFINITY };
	PlatoonLaneChangeOrder best_order;
	int n = platoon->get_size();

	/* First, we check if any vehicles are already at safe position to
	start the maneuver */
	std::vector<PlatoonLaneChangeOrder> all_orders; // for debugging
	for (int pos1 = 0; pos1 < n; pos1++)
	{
		std::set<int> first_movers;
		int pos2 = pos1;
		while (pos2 < n
			&& (platoon->get_a_vehicle_by_position(pos2)
				->are_surrounding_gaps_safe_for_lane_change()))
		{
			first_movers.insert(pos2);
			PlatoonLaneChangeOrder an_order =
				strategy_manager.find_minimum_cost_order_given_first_mover(
					first_movers);
			all_orders.push_back(an_order);
			pos2++;
			if (an_order.cost < best_cost)
			{
				best_cost = an_order.cost;
				best_order = an_order;
				if (verbose) std::clog << "\tbest order so far: " 
					<< best_order.to_string() << "\n";
			}
		}
	}

	/* If there are no vehicles at safe positions, we check if any are
	close to a suitable gap */
	if (std::isinf(best_cost))
	{
		for (int pos1 = 0; pos1 < n; pos1++)
		{
			if (platoon->get_a_vehicle_by_position(pos1)
				->get_is_space_suitable_for_lane_change())
			{
				std::set<int> single_veh_set { pos1 };
				PlatoonLaneChangeOrder an_order =
					strategy_manager.find_minimum_cost_order_given_first_mover(
						single_veh_set );
				all_orders.push_back(an_order);
				if (an_order.cost < best_cost)
				{
					best_cost = an_order.cost;
					best_order = an_order;
					if (verbose) std::clog << "\tbest order so far: " 
						<< best_order.to_string() << "\n";
				}
			}
		}
	}
	return best_order;
}

template <typename T>
void GraphApproach::save_not_found_state_to_file(
	std::vector<T> state_vector, double free_flow_speed_orig, 
	double free_flow_speed_dest)
{
	std::clog << "[GraphApproach] Saving not found state to file\n";

	std::map<std::string, double> free_flow_speeds{
		{"dest", free_flow_speed_dest},
		{"platoon", platoon->get_platoon_leader()->get_desired_velocity()},
		{"orig", free_flow_speed_orig}
	};

	std::string file_name = "vissim_x0_" 
		+ std::to_string(platoon->get_size()) + "_vehicles.csv";
	std::string folder = "C:\\Users\\fvall\\Documents\\Research"
		"\\data\\vehicle_state_graphs\\";
	std::string file_path = folder + file_name;
	/* Entry starts with the free-flow speeds in alphabetical order
	(dest, orig, platoon) and then come the states */
	if (!std::ifstream(file_path.c_str()).good())
	{
		std::ofstream outfile{ file_path };
		std::string header;
		for (const auto& item : free_flow_speeds)
		{
			header += item.first + ",";
		}
		for (int i = 0; i < state_vector.size(); i++)
		{
			header += "x" + std::to_string(i) + ",";
		}
		header.erase(header.size() - 1);
		outfile << header << "\n";
	}

	std::string info;
	for (const auto& item : free_flow_speeds)
	{
		info += std::to_string(item.second) + ",";
	}
	for (int i = 0; i < state_vector.size(); i++)
	{
		info += std::to_string(state_vector[i]) + ",";
	}
	info.erase(info.size() - 1); // remove last comma
	std::ofstream outfile{ file_path, std::ios::app };
	outfile << info << "\n";
}