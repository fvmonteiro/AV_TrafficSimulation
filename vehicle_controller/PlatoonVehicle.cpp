
#include "PlatoonVehicle.h"
#include "Platoon.h"
#include "PlatoonLaneChangeStrategy.h"

PlatoonVehicle::PlatoonVehicle(long id, double desired_velocity,
	double simulation_time_step, double creation_time,
	bool verbose) :
	ConnectedAutonomousVehicle(id, VehicleType::platoon_car,
		desired_velocity, simulation_time_step, creation_time,
		verbose),
	alone_desired_velocity{ desired_velocity }
{
	compute_platoon_safe_gap_parameters();
	//controller.add_in_platoon_controller(*this);
	if (verbose)
	{
		std::clog << "lambda1_platoon = " << lambda_1_platoon
			<< ", lambda1_lc_platoon = " << lambda_1_lane_change_platoon
			<< "\n[PlatoonVehicle] constructor done" << std::endl;
	}
}

PlatoonVehicle::~PlatoonVehicle()
{
	if (verbose) std::clog << "[PlatoonVehicle] destructor" << std::endl;
}

bool PlatoonVehicle::is_platoon_leader() const
{
	return !is_in_a_platoon() || (platoon->get_leader_id() == get_id());
}

bool PlatoonVehicle::is_last_platoon_vehicle() const
{
	return !is_in_a_platoon() || (platoon->get_last_veh_id() == get_id());
}

bool PlatoonVehicle::can_start_adjustment_to_destination_lane_leader() const
{
	return !is_in_a_platoon()
		|| platoon->can_vehicle_start_adjustment_to_dest_lane_leader(
			get_id());
}

std::shared_ptr<PlatoonVehicle> 
PlatoonVehicle::get_preceding_vehicle_in_platoon() const
{
	return platoon->get_preceding_vehicle(get_id());
}

std::shared_ptr<PlatoonVehicle>
PlatoonVehicle::get_following_vehicle_in_platoon() const
{
	return get_platoon()->get_following_vehicle(get_id());
}

double PlatoonVehicle::implement_compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double a_desired_acceleration =
		controller.get_desired_acceleration(*this);
	return consider_vehicle_dynamics(a_desired_acceleration);
}

double PlatoonVehicle::compute_vehicle_following_safe_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1;
	double rho;
	if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	{
		if (verbose) std::clog << "Leader identified as platoon veh.\n";
		current_lambda_1 = lambda_1_platoon;
		rho = in_platoon_rho;
	}
	else
	{
		current_lambda_1 = ConnectedAutonomousVehicle::get_lambda_1(
			nearby_vehicle.is_connected());
		rho = get_rho();
	}

	return compute_time_headway_with_risk(get_desired_velocity(),
		get_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, rho, 0);
}

double PlatoonVehicle::compute_lane_changing_desired_time_headway(
	const NearbyVehicle& nearby_vehicle) const
{
	double current_lambda_1;
	double rho;
	if (nearby_vehicle.get_type() == VehicleType::platoon_car)
	{
		current_lambda_1 = lambda_1_lane_change_platoon;
		rho = in_platoon_rho;
	}
	else
	{
		current_lambda_1 = ConnectedAutonomousVehicle::
			get_lambda_1_lane_change(nearby_vehicle.is_connected());
		rho = get_rho();
	}
	double h_lc = compute_time_headway_with_risk(get_desired_velocity(),
		get_lane_change_max_brake(), nearby_vehicle.get_max_brake(),
		current_lambda_1, rho, 0);
	return h_lc;
}

void PlatoonVehicle::set_desired_lane_change_direction()
{
	bool should_change_lane = (get_link() == MAIN_LINK_NUMBER)
		&& (get_lane() == 1);
	desired_lane_change_direction = should_change_lane ?
		RelativeLane::left : RelativeLane::same;
	/*std::shared_ptr<Platoon> platoon = get_platoon();
	if (platoon == nullptr)
	{
		desired_lane_change_direction = should_change_lane ?
			RelativeLane::left : RelativeLane::same;
	}
	else
	{
		long my_id = get_id();
		platoon->set_vehicle_lane_change_state(*this, should_change_lane);
		desired_lane_change_direction = 
			platoon->can_vehicle_start_longitudinal_adjustment(my_id)?
			RelativeLane::left : RelativeLane::same;
	}*/
}

void PlatoonVehicle::implement_analyze_nearby_vehicles()
{
	find_leader();
	find_destination_lane_vehicles();
	find_cooperation_request_from_platoon();
}

void PlatoonVehicle::find_cooperation_request_from_platoon()
{
	if (is_in_a_platoon())
	{
		long assisted_vehicle_id =
			get_platoon()->get_assisted_vehicle_id(get_id());
		set_assisted_vehicle_by_id(assisted_vehicle_id);
	}
}

bool PlatoonVehicle::implement_analyze_platoons(
	std::unordered_map<int, std::shared_ptr<Platoon>>& platoons,
	std::shared_ptr<EgoVehicle> pointer_to_me, long new_platoon_id)
{
	bool new_platoon_created = false;
	bool am_in_a_platoon = is_in_a_platoon();
	bool leader_is_in_a_platoon = 
		has_leader() && get_leader()->is_in_a_platoon();
	bool may_join_leader_platoon =
		leader_is_in_a_platoon && (get_leader()->get_distance() < 60);

	std::shared_ptr<PlatoonVehicle> pointer_to_me_my_type =
		std::dynamic_pointer_cast<PlatoonVehicle>(pointer_to_me);

	if (!am_in_a_platoon && !may_join_leader_platoon)
	{
		// Create platoon
		create_platoon(new_platoon_id, pointer_to_me_my_type);
		new_platoon_created = true;
		/*if (alone_time >= max_time_looking_for_platoon)
		{
			create_platoon(new_platoon_id, pointer_to_me_my_type);
			alone_time = 0.0;
			new_platoon_created = true;
		}
		else
		{
			alone_time += get_sampling_interval();
		}*/
	}
	else if (!am_in_a_platoon && may_join_leader_platoon)
	{
		// Join the platoon of the vehicle ahead
		long leader_platoon_id = get_leader()->get_platoon_id();
		add_myself_to_leader_platoon(platoons.at(leader_platoon_id),
			pointer_to_me_my_type);
	}
	else if (am_in_a_platoon && may_join_leader_platoon)
	{
		// Leave my platoon and join the platoon of the vehicle ahead
		long old_platoon_id = get_platoon_id();
		long leader_platoon_id = get_leader()->get_platoon_id();
		if (old_platoon_id != leader_platoon_id)
		{
			// remove myself
			platoon->remove_vehicle_by_id(get_id());
			// add myself to leader platoon
			add_myself_to_leader_platoon(platoons.at(leader_platoon_id),
				pointer_to_me_my_type);
			// delete my old platoon if it is empty
			if (platoons.at(old_platoon_id)->is_empty())
			{
				platoons.erase(old_platoon_id);
			}
		}
	}
	else // am_in_a_platoon && !may_join_leader_platoon
	{
		if (platoon->can_vehicle_leave_platoon(*this))
		{
			std::clog << "t=" << get_time() << " id=" << get_id()
				<< ": leaving platoon " << platoon->get_id() << "\n";
			platoon->remove_vehicle_by_id(get_id());
			//platoon.reset();
			create_platoon(new_platoon_id, pointer_to_me_my_type);
			new_platoon_created = true;
		}
	}

	return new_platoon_created;
}

std::shared_ptr<Platoon> PlatoonVehicle::implement_get_platoon() const
{
	/*if (platoon == nullptr)
	{
		std::clog << "[ERROR] Trying to read nullptr platoon\n"
			<< "Vehicle: " << *this << std::endl;
	}*/
	return platoon;
}

void PlatoonVehicle::pass_this_to_state()
{
	state->set_ego_vehicle(this);
}

void PlatoonVehicle::create_platoon(long platoon_id,
	std::shared_ptr<PlatoonVehicle> pointer_to_me)
{
	if (verbose)
	{
		std::clog << "Veh id " << get_id()
			<< ". Creating platoon id " << platoon_id << std::endl;
	}

	platoon = std::make_shared<Platoon>(platoon_id,
		pointer_to_me);
}

void PlatoonVehicle::add_myself_to_leader_platoon(
	std::shared_ptr<Platoon> leader_platoon,
	std::shared_ptr<PlatoonVehicle> pointer_to_me)
{
	// Add my pointer to the platoon vehicles list
	leader_platoon->add_last_vehicle(pointer_to_me);
	// Update my platoon pointer
	platoon = leader_platoon;
	// Update my desired velocity
	set_desired_velocity(
		platoon->get_platoon_leader()->get_desired_velocity());
	/* Possibly more stuff:
	 Decrease desired time headway?
	 Deactivate velocity control?
	 Deactivate cooperation (so no one cuts in) ?
	*/
	alone_time = 0.0;
}

void PlatoonVehicle::compute_platoon_safe_gap_parameters()
{
	lambda_0_platoon =
		compute_lambda_0(max_jerk, in_platoon_comf_accel,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_platoon =
		compute_lambda_1(max_jerk, in_platoon_comf_accel,
			max_brake, CONNECTED_BRAKE_DELAY);
	lambda_1_lane_change_platoon =
		compute_lambda_1(max_jerk, in_platoon_comf_accel,
			get_lane_change_max_brake(), CONNECTED_BRAKE_DELAY);
}
