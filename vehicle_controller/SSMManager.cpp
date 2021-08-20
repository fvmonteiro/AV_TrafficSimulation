#include "SSMManager.h"

SSMManager::SSMManager(double time_frequency) 
	: time_frequency{ time_frequency } {}

void SSMManager::aggregate_ssms(double new_time, 
	std::unordered_map<long, EgoVehicle>& vehicles) {

	if (new_time > current_time) { // only update at new time steps
		current_time = new_time;
		accumulate_ssms(vehicles);
		if ((current_time - initial_aggregation_time) >= time_frequency) {
			// end aggregation for this time interval
			time.push_back(current_time);
			average_ssms_over_time_period();
			reset_accumulated_ssms();
			time_step_counter = 0;
		}
		else {
			time_step_counter++;
		}
	}
}

void SSMManager::accumulate_ssms(
	std::unordered_map<long, EgoVehicle>& vehicles) {

	for (const auto& it : vehicles) {
		const EgoVehicle& vehicle = it.second;
		accumulated_ssms[SSMs::ttc] += vehicle.get_ttc();
		accumulated_ssms[SSMs::drac] += vehicle.get_drac();
		accumulated_ssms[SSMs::collision_severity_risk] += 
			vehicle.get_collision_risk();
	}

	int n_vehicles = (int)vehicles.size();
	for (auto it : accumulated_ssms) {
		accumulated_ssms[it.first] /= n_vehicles;
	}
}

void SSMManager::average_ssms_over_time_period() {
	for (const auto& it : accumulated_ssms) {
		ssms[it.first].push_back(it.second / time_step_counter);
	}
}

void SSMManager::reset_accumulated_ssms() {
	for (const auto& it : accumulated_ssms) {
		accumulated_ssms[it.first] = 0;
	}
}