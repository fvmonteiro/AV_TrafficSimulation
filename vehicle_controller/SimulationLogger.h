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
	void create_log_file();

	void write_to_persistent_log(std::string& message);
	
	/* Sets the output of cerr to a specific "error_log" file to make it 
	separate from the log used for behavior checking, and writes the error 
	to that file*/
	void write_to_error_log(std::string& message);

	/* Writes warning to log file when vehicle data is read from VISSIM 
	before the respective Vehicle object is created
	Not sure this is needed*/
	void write_no_vehicle_object_message(const char* type_of_data, 
		long vehicle_id);

	~SimulationLogger();

private:
	/* file to store (detailed) results of a single simulation */
	FILE* log_file{ nullptr };
	/* file to store information accross several simulations. This is
	used to debug possible memory issues over several runs. */
	std::ofstream persistent_log; 
	const char* log_file_name{ "dll_log.txt" };
	const char* persistent_log_file_name{ "dll_persistent_log.txt" };
	const char* error_log_file_name{ "error_log.txt" }; // not being used
};