#include "incl/dataInput/dataStructures/locations.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "evaExceptions.h"

const eva::Location& eva::Locations::get_locationFromId(const uint32_t& id) const
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Locations::get_locationFromId", "Given id does not exist.");

	return _vec[it->second];
}

eva::Location& eva::Locations::get_locationFromId(const uint32_t& id)
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Locations::get_locationFromId", "Given id does not exist.");

	return _vec[it->second];
}

void eva::Locations::read(const std::string& fileName)
{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Locations::read","File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	uint32_t id;
	Location::LocationType type;
	std::string name;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the locations.
	// ----------------------------------------------------------------------------------------------
	while (std::getline(in, line)) {
		// Remove carriage return symbol:
		auto pos = line.find_first_of("\r\n");
		if (pos != std::string::npos)
			line.erase(pos);

		// Stop if we have reached the end of file
		if (in.eof())
			break;

		// If this is the first line, then we may have a "sep=," command.
		// If so, then skip the line.
		if (first_line) {
			first_line = false;
			if (line.find("sep") != std::string::npos) {
				continue;
			}
		}

		// Check if this is the header. If so, then skip the line.
		if (line.find("Id") != std::string::npos)
			continue;

		// Check for comment
		if (line.find_first_of('#') != std::string::npos) {
			first_line = false; continue;
		}

		// Convert the line into a stringstream for easier access
		std::stringstream lineStream(line);

		// Id:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Locations::read","Missing Id.");
		id = atoi(str.c_str());

		// Type:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Locations::read", "Missing Type.");
		type = Location::string2type(str);

		// Id:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Locations::read", "Missing Name.");
		name = str;

		// Push new location object to the vector:
		_vec.push_back(
			Location(
				_vec.size(),
				id,
				type,
				name
			)
		);
	}

	// Initialise the map to store the id<>index reference:
	_mapId.reserve(_vec.size());
	for (Types::Index index = 0; index < _vec.size(); index++)
	{
		_mapId.insert({ _vec[index].get_id(), index });
		_vec[index].get_vecMeasures().resize(_vec.size());
	}
		

}

void eva::Locations::readTravel(const std::string& fileName)
{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Locations::readTravel", "File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	uint32_t fromId;
	uint32_t toId;
	uint32_t duration;
	uint32_t distance;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the locations.
	// ----------------------------------------------------------------------------------------------
	while (std::getline(in, line)) {
		// Remove carriage return symbol:
		auto pos = line.find_first_of("\r\n");
		if (pos != std::string::npos)
			line.erase(pos);

		// Stop if we have reached the end of file
		if (in.eof())
			break;

		// If this is the first line, then we may have a "sep=," command.
		// If so, then skip the line.
		if (first_line) {
			first_line = false;
			if (line.find("sep") != std::string::npos) {
				continue;
			}
		}

		// Check if this is the header. If so, then skip the line.
		if (line.find("Id") != std::string::npos)
			continue;

		// Check for comment
		if (line.find_first_of('#') != std::string::npos) {
			first_line = false; continue;
		}

		// Convert the line into a stringstream for easier access
		std::stringstream lineStream(line);

		// Extract the Id of the start location
		// --------------------------------------------------------------------------------------------
		getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Locations::readTravel", line + "\" has an empty start Id!\n");
		fromId = atoi(str.c_str());

		// Extract the Id of the end location
		// --------------------------------------------------------------------------------------------
		getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Locations::readTravel", line + "\" has an empty end Id!\n");
		toId = atoi(str.c_str());

		// Extract the duration between start and end
		// --------------------------------------------------------------------------------------------
		getline(lineStream, str, ',');
		if (str.length() < 1)
			continue;
		else
			duration = (int64_t)(atof(str.c_str()));

		// Extract the distance between start and end
		// --------------------------------------------------------------------------------------------
		getline(lineStream, str, ',');
		if (str.length() < 1)
			continue;
		else
			distance = atof(str.c_str());	

		// Get the location object, and add the measurement:
		// Only add, if both, distance and duration are specified.
		// Then, write code for function to access, or return non-exist.
		this->get_locationFromId(fromId).get_vecMeasures()[this->get_locationFromId(toId).get_index()] = Location::Measures(distance,duration);
	}

}

void eva::Locations::clear()
{
	_vec.clear();
	_mapId.clear();
}
