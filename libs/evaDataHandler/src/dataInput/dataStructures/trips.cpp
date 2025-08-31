#include "incl/dataInput/dataStructures/trips.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "evaExceptions.h"

const eva::Trip& eva::Trips::get_tripFromId(const uint32_t& id) const
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Trips::get_tripFromId", "Given id does not exist.");

	return _vec[it->second];
}

void eva::Trips::read(const std::string& fileName, const Locations& locations, const Config& config)
{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Trips::read", "File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	uint32_t id;
	Types::DateTime startTime;
	Types::DateTime endTime;
	uint32_t startLocationId;
	uint32_t endLocationId;
	uint32_t lineId;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the trips.
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
			throw DataError("eva::Trips::read", "Missing Id.");
		id = atoi(str.c_str());

		// startTime:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Trips::read", "Missing startTime.");
		startTime = Helper::StringToDateTime(str);

		// endTime:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Trips::read", "Missing endTime.");
		endTime = Helper::StringToDateTime(str);

		// startLocationId:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Trips::read", "Missing startLocationId.");
		startLocationId = atoi(str.c_str());

		// endLocationId:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Trips::read", "Missing endLocationId.");
		endLocationId = atoi(str.c_str());

		// lineId
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Trips::read", "Missing lineId.");
		lineId = atoi(str.c_str());

		// All data extracted, add trip to the list
		// Only add the trip if it is relevant to this optimisation run,
		// and the start time lies after the start bound, and before the end bound:
		if (Helper::diffDateTime(config.get_date_start(), startTime) >= 0
			&& Helper::diffDateTime(startTime, config.get_date_end()) > 0)
		{
			// Push new trip object to the vector:
			_vec.push_back(
				Trip(
					id,
					startTime,
					endTime,
					locations.get_locationFromId(startLocationId),
					locations.get_locationFromId(endLocationId),
					lineId
				)
			);
		}

		
	}

	// Initialise the map to store the id<>index reference:
	_mapId.reserve(_vec.size());
	for (Types::Index index = 0; index < _vec.size(); index++)
		_mapId.insert({ _vec[index].get_id(), index });
}

void eva::Trips::clear()
{
	_vec.clear();
	_mapId.clear();
}
