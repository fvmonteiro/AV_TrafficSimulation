#include <algorithm>
#include <fstream>
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
		if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
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
			if (!ego_vehicle->is_vehicle_in_sight(virtual_leader_id))
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
	if (verbose) std::clog << "[PlatoonLaneChangeApproach] "
		"can_vehicle_start_lane_change\n";

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
		std::string message = "Next to move: ";
		for (int i : next_to_move) message += std::to_string(i) + ", ";
		message += "my pos. = " + std::to_string(ego_position)
			+ "-> is my turn? " + (is_my_turn ? "yes" : "no");
		std::clog << message << "\n";
	}

	/* TODO: change where this check happens.The current lc vehicles should
	call a new method when they finish their maneuvers. In this new
	method we advance the idx */
	check_maneuver_step_done(next_to_move);

	if (is_my_turn)
	{
		for (int i : next_to_move)
		{
			const PlatoonVehicle* vehicle = 
				platoon->get_a_vehicle_by_position(i);
			if (!vehicle->is_at_right_lane_change_gap())
			{
				return false;
			}
			if (!vehicle->are_surrounding_gaps_safe_for_lane_change())
			{
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
	LCOrder lc_order, CoopOrder coop_order)
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
}

int PlatoonLaneChangeApproach
::get_rearmost_lane_changing_vehicle_position() const
{
	if (verbose) std::clog << "Looking for rearmost lc veh.\n";

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

	if (verbose) std::clog << "Found at pos." << rear_most_pos << "\n";
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
	CoopOrder coop_order{ {-1} };
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
	CoopOrder coop_order;
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
	CoopOrder coop_order;
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
	CoopOrder coop_order;
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
		strategy_manager = PlatoonLCStrategyManager(cost_name);
		strategy_manager.initialize(platoon->get_size());
	}
	if (verbose) std::clog << "[GraphApproach] about to set initial states \n";
	set_maneuver_initial_state_for_all_vehicles();
	PlatoonLaneChangeOrder plco = find_best_order_in_map();
	if (plco.cost > -1)  // an order was found
	{
		set_platoon_lane_change_order(plco);
	}
	// else: the approach continues to be not initialized.
}

void GraphApproach::set_maneuver_initial_state_for_all_vehicles()
{
	/* TODO [Jan 16 2024]: change how to deal with no leaders once these
	situations are included in the graph */

	// We need to center all vehicles' states around the leader
	const PlatoonVehicle* platoon_leader = platoon->get_platoon_leader();
	double leader_x = platoon_leader->get_state_vector().get_x();
	double leader_y = platoon_leader->get_state_vector().get_y();

	std::vector<ContinuousStateVector> system_state_matrix;

	/* Orig lane leader's and platoon vehicles' states don't depend on
	who's changing lanes first */
	ContinuousStateVector lo_states;
	if (platoon_leader->has_leader())
	{
		lo_states = platoon_leader->get_leader()->get_relative_state_vector(
			platoon_leader->get_velocity());
	}
	else
	{
		lo_states = platoon_leader->get_state_vector();
		lo_states.add_to_x(MAX_DISTANCE);
		lo_states.offset(leader_x, leader_y);
	}
	system_state_matrix.push_back(lo_states);
	if (verbose) std::clog << "lo states="
		<< system_state_matrix.back().to_string() << "\n";

	int i = 0;
	for (ContinuousStateVector& v : platoon->get_vehicles_states())
	{
		system_state_matrix.push_back(v);
		if (verbose) std::clog << "platoon states (" << i++ << ")="
			<< system_state_matrix.back().to_string() << "\n";
	/*}

	for (const auto& item : platoon->get_vehicles_by_position())
	{
		ContinuousStateVector veh_state_vector = 
			item.second->get_state_vector();
		if (verbose) std::clog << "platoon states before (" << item.first << ")="
			<< veh_state_vector.to_string() << "\n";
		veh_state_vector.offset(leader_x, leader_y);
		system_state_matrix.push_back(veh_state_vector);
		if (verbose) std::clog << "platoon states (" << item.first << ")="
			<< system_state_matrix.back().to_string() << "\n"; */
	}

	/* Now we check which platoon vehicles can move and set possible
	destination lane leaders */
	for (std::pair<int, const PlatoonVehicle*> pos_and_veh 
		: platoon->get_vehicles_by_position())
	{
		int veh_pos = pos_and_veh.first;
		const PlatoonVehicle* vehicle = pos_and_veh.second;

		if (vehicle->get_is_space_suitable_for_lane_change())
		{
			ContinuousStateVector ld_states;
			if (vehicle->has_destination_lane_leader())
			{
				const NearbyVehicle& ld = 
					*vehicle->get_destination_lane_leader();
				double vehicle_vel = vehicle->get_velocity();
				ld_states = ld.get_relative_state_vector(
					vehicle->get_velocity());
			}
			else
			{
				ld_states = platoon_leader->get_state_vector();
				ld_states.add_to_x(MAX_DISTANCE);
				ld_states.add_to_y(
					platoon_leader->get_desired_lane_change_direction().to_int()
					* LANE_WIDTH);
				ld_states.offset(leader_x, leader_y);
			}
			system_state_matrix.push_back(ld_states);
			if (verbose) std::clog << "ld states ("
				<< veh_pos << ")" << system_state_matrix.back().to_string()
				<< "\n";
			try
			{
				if (verbose) std::clog << "Setting initial state...";
				strategy_manager.set_maneuver_initial_state(veh_pos,
					system_state_matrix);
			}
			catch (const StateNotFoundException& e)
			{
				if (verbose) std::clog << "done.\nState not found.\n";
				double free_flow_speed_dest =
					vehicle->has_destination_lane_leader() ?
					vehicle->get_destination_lane_leader()->compute_velocity(
						vehicle->get_velocity())
					: platoon_leader->get_desired_velocity();
				save_not_found_state_to_file(e.state_vector, 
					free_flow_speed_dest);
				throw;
			}
			system_state_matrix.pop_back();
		}

		
	}
}

PlatoonLaneChangeOrder GraphApproach::find_best_order_in_map()
{
	bool found{ false };
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
				found = true;
				best_cost = an_order.cost;
				best_order = an_order;
			}
		}
	}

	/* If there are no vehicles at safe positions, we check if any are
	close to a suitable gap */
	if (!found)
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
					found = true;
					best_cost = an_order.cost;
					best_order = an_order;
				}
			}
		}
	}
	return best_order;
}

void GraphApproach::save_not_found_state_to_file(
	std::vector<int> state_vector, double free_flow_speed_dest)
{
	if (verbose) std::clog << "[GraphApproach] Saving not found state to "
		"file\n";

	const PlatoonVehicle* platoon_leader = platoon->get_platoon_leader();
	std::unordered_map<std::string, double> free_flow_speeds{
		{"platoon", platoon_leader->get_desired_velocity()},
		{"dest", free_flow_speed_dest}
	};
	free_flow_speeds["orig"] = platoon_leader->has_leader() ?
		platoon_leader->get_leader()
		->compute_velocity(platoon_leader->get_velocity())
		: free_flow_speeds["platoon"];

	std::string file_name = "unsolved_x0_" 
		+ std::to_string(platoon->get_size()) + "_vehicles.csv";
	std::string file_path = "C:\\Users\\fvall\\Documents\\Research"
		"\\data\\vehicle_state_graphs\\";
	/* Entry starts with the free-flow speeds in alphabetical order
	(dest, orig, platoon) and then come the states */
	std::ofstream outfile{ file_path + file_name, std::ios::app };
	outfile << free_flow_speeds["dest"] << ","
		<< free_flow_speeds["orig"] << ","
		<< free_flow_speeds["platoon"] << ",";
	for (int i = 0; i < state_vector.size() - 1; i++)
	{
		outfile << state_vector[i] << ",";
	}
	outfile << state_vector[state_vector.size() - 1] << "\n";
}