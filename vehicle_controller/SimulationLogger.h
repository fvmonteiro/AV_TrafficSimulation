/*==========================================================================*/
/*  SimulationLogger.h 													    */
/*  Class to help debugging simulations                                     */
/*                                                                          */
/*  Version of 2021-xx-xx                             Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once
#include <iostream>
#include <fstream>

/* Class to help debugging algorithm and code.
TODO: should this class be a singleton? */
class SimulationLogger {
public:
	//SimulationLogger();
	
	/* Sets the default output of clog command to a log file. */
	void createLogFile();
	
	/* Sets the output of cerr to a specific "error_log" file to make it 
	separate from the log used for behavior checking, and writes the error 
	to that file*/
	void writeToErrorLog(const char* message);

	/* Writes warning to log file when vehicle data is read from VISSIM 
	before the respective Vehicle object is created
	Not sure this is needed*/
	void writeNoVehicleObjectMessage(const char* type_of_data, 
		long vehicle_id);

	~SimulationLogger();

private:
	FILE* log_file{ nullptr };
	const char* log_file_name{ "log.txt" };
	const char* error_log_file_name{ "error_log.txt" };
};