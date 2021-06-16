/*==========================================================================*/
/*  SimulationLogger.h 													    */
/*  Class to help debugging simulations                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#include <iostream>
#include <fstream>
#include "SimulationLogger.h"

void SimulationLogger::createLogFile() {
	/* clog writes to stderr, so that's the destination we change
	Note that cerr will write to that file too. If we want to write error
	to a different file, use the merhod writeToErrorLog*/
	freopen_s(&log_file, log_file_name, "w", stderr);
	std::clog << "------- Start of log -------" << std::endl;
};

void SimulationLogger::writeToErrorLog(const char* message) {
	std::streambuf* error_buffer = std::cerr.rdbuf(); /* save the default 
													  error buffer to restore
													  it later */
	std::ofstream output_stream(error_log_file_name);
	std::cerr.rdbuf(output_stream.rdbuf());
	std::cerr << message << std::endl;
	std::cerr.rdbuf(error_buffer);
};

void SimulationLogger::writeNoVehicleObjectMessage(const char* type_of_data, 
	long vehicle_id) {
	char buffer[150];
	sprintf_s(buffer, "%s data for vehicle %d set before vehicle "
		"object was created and added to the vehicle map", type_of_data, 
		vehicle_id);
	std::clog << buffer << std::endl;
}

SimulationLogger::~SimulationLogger() {
	std::clog << "-------- End of log --------" << std::endl;
	fclose(log_file);
};