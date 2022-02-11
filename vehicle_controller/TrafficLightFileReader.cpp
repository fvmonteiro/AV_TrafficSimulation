#include <fstream>
#include <sstream>

#include "TrafficLightFileReader.h"

void TrafficLightFileReader::from_file_to_objects(std::string full_address,
	std::unordered_map<int, TrafficLight>& traffic_lights) 
{
	std::ifstream data_file(full_address);
	std::string line;
	int id, position, red_duration, green_duration, amber_duration;

	std::getline(data_file, line); // discard the header
	/* The data is ordered as: 
	id, position, red duration, green duration, amber duration */
	while (std::getline(data_file, line))
	{
		std::istringstream s(line);
		std::string field;
		std::getline(s, field, ',');
		id = std::stoi(field);
		std::getline(s, field, ',');
		position = std::stoi(field);
		std::getline(s, field, ',');
		red_duration = std::stoi(field);
		std::getline(s, field, ',');
		green_duration = std::stoi(field);
		std::getline(s, field, ',');
		amber_duration = std::stoi(field);
		traffic_lights.emplace(std::piecewise_construct,
			std::forward_as_tuple(id),
			std::forward_as_tuple(id, position, red_duration,
				green_duration, amber_duration));
	}

	// An "empty" traffic ligh object for when no traffic light is visible
	// We still have to check if this is really necessary
	traffic_lights.emplace(std::piecewise_construct,
		std::forward_as_tuple(0), std::forward_as_tuple(0, 0, 0, 0, 0));
}
