/*==========================================================================*/
/*  SimulationLogger.h 													    */
/*  Class to help debugging simulations                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>

#include "SimulationLogger.h"

void SimulationLogger::create_log_file() 
{	
	freopen_s(&log_file, log_file_name, "w", stdout);
	/* clog writes to stderr. Note that cerr will write to that file too. 
	If we want to write error to a different file, 
	use the merhod writeToErrorLog */
	freopen_s(&persistent_log_file, persistent_log_file_name, "a", stderr);

	std::cout << "------- Start of log -------" << std::endl;
	std::time_t start_time = std::chrono::system_clock::to_time_t(
		std::chrono::system_clock::now());
	char time_str[26];
	ctime_s(time_str, sizeof time_str, &start_time);
	std::clog << "--------------------------------------------" 
		<< std::endl << "Simulation started on: " << time_str;

	//persistent_log.open(persistent_log_file_name, std::ios::app);
	//if (persistent_log.is_open()) {
	//	std::time_t start_time = std::chrono::system_clock::to_time_t(
	//		std::chrono::system_clock::now());
	//	char time_str[26];
	//	ctime_s(time_str, sizeof time_str, &start_time);
	//	persistent_log << "--------------------------------------------"
	//		<< std::endl << "Simulation started on: " << time_str;
	//}
	//else {
	//	std::cout << "Unable to open file the persistent log file."
	//		<< std::endl;
	//}
}

//void SimulationLogger::write_to_persistent_log(std::string& message) {
//	if (persistent_log.is_open()) {
//		persistent_log << message << std::endl;
//	}
//}

void SimulationLogger::write_to_error_log(std::string& message) {
	std::streambuf* error_buffer = std::cerr.rdbuf(); /* save the default 
													  error buffer to restore
													  it later */
	std::ofstream output_stream(error_log_file_name);
	std::cerr.rdbuf(output_stream.rdbuf());
	std::cerr << message << std::endl;
	std::cerr.rdbuf(error_buffer);
};

void SimulationLogger::write_no_vehicle_object_message(const char* type_of_data, 
	long vehicle_id) {
	char buffer[150];
	sprintf_s(buffer, "%s data for vehicle %d set before vehicle "
		"object was created and added to the vehicle map", type_of_data, 
		vehicle_id);
	std::cout << buffer << std::endl;
}

SimulationLogger::~SimulationLogger() {
	std::cout << "-------- End of log --------" << std::endl;
	fclose(log_file);

	std::time_t end_time = std::chrono::system_clock::to_time_t(
		std::chrono::system_clock::now());
	char time_str[26];
	ctime_s(time_str, sizeof time_str, &end_time);
	std::clog << "Simulation ended on: " << time_str;
	fclose(persistent_log_file);
	//persistent_log.close();
}