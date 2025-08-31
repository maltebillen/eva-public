#include "incl/dataInput/dataStructures/chargers.h"



#include <iostream>
#include <sstream>
#include <fstream>

#include "evaExceptions.h"

const eva::Charger& eva::Chargers::get_chargerFromId(const uint32_t& id) const
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Chargers::get_chargerFromId", "Given id does not exist.");

	return _vec[it->second];
}

void eva::Chargers::read(const std::string& fileName, const Locations& locations)
{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Chargers::read", "File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	Types::Index index;
	uint32_t id;
	uint32_t locationId;
	uint32_t capacity;
	Types::BatteryCharge chargingSpeedVolts;
	Types::BatteryCharge chargingSpeedAmps;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the chargers.
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

		// Index:
		index = _vec.size();

		// Id:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Chargers::read", "Missing Id.");
		id = atoi(str.c_str());

		// IdLocation:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Chargers::read", "Missing LocationId.");
		locationId = atoi(str.c_str());

		// Capacity:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Chargers::read", "Missing Capacity.");
		capacity = atoi(str.c_str());

		// chargingSpeedVolts:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Chargers::read", "Missing ChargingSpeedVolts.");
		chargingSpeedVolts = atoi(str.c_str());

		// chargingSpeedAmps:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Chargers::read", "Missing ChargingSpeedAmps.");
		chargingSpeedAmps = atoi(str.c_str());

		// Push new location object to the vector:
		_vec.push_back(
			Charger(
				index,
				id,
				locations.get_locationFromId(locationId),
				capacity,
				chargingSpeedVolts,
				chargingSpeedAmps
			)
		);
	}

	// Initialise the map to store the id<>index reference:
	_mapId.reserve(_vec.size());
	for (Types::Index index = 0; index < _vec.size(); index++)
		_mapId.insert({ _vec[index].get_id(), index });
}

void eva::Chargers::clear()
{
	_vec.clear();
	_mapId.clear();
}
