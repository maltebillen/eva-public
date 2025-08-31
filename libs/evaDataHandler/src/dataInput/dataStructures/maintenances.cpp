#include "incl/dataInput/dataStructures/maintenances.h"

#include "evaExceptions.h"

#include <iostream>
#include <sstream>
#include <fstream>

const eva::Maintenance& eva::Maintenances::get_maintenanceFromId(const uint32_t& id) const
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Maintenances::get_maintenanceFromId", "Given id does not exist.");

	return _vec[it->second];
}

void eva::Maintenances::read(const std::string& fileName, const Locations& locations, const Config& config, const Vehicles& vehicles)
{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Maintenances::read", "File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	uint32_t id;
	Types::DateTime startTime;
	Types::DateTime endTime;
	uint32_t maintenanceLocationId;
	Types::Index indexVehicle;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the maintenances.
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
			throw DataError("eva::Maintenances::read", "Missing Id.");
		id = atoi(str.c_str());

		// startTime:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Maintenances::read", "Missing startTime.");
		startTime = Helper::StringToDateTime(str);

		// endTime:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Maintenances::read", "Missing endTime.");
		endTime = Helper::StringToDateTime(str);

		// maintenanceLocationId:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Maintenances::read", "Missing maintenanceLocationId.");
		maintenanceLocationId = atoi(str.c_str());

		// vehicleId:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			indexVehicle = Constants::BIG_INDEX;
		else
			indexVehicle = vehicles.get_vehicleFromId(atoi(str.c_str())).get_index();

		// All data extracted, add maintenance to the list
		// Only add the maintenance if it is relevant to this optimisation run,
		// and the start time lies after the start bound, and before the end bound:
		if (Helper::diffDateTime(config.get_date_start(), startTime) >= 0
			&& Helper::diffDateTime(startTime, config.get_date_end()) > 0)
		{
			// Push new maintenance object to the vector:
			_vec.push_back(
				Maintenance(
					id,
					startTime,
					endTime,
					locations.get_locationFromId(maintenanceLocationId),
					indexVehicle
				)
			);
		}
	}

	// Initialise the map to store the id<>index reference:
	_mapId.reserve(_vec.size());
	_vecVehicleMaintenances.resize(vehicles.get_vec().size());

	for (Types::Index index = 0; index < _vec.size(); index++)
	{
		_mapId.insert({ _vec[index].get_id(), index });
		
		if(_vec[index].is_assigned())
			_vecVehicleMaintenances[_vec[index].get_indexVehicle()].push_back(index);
		else
			_vecUnassignedMaintenances.push_back(index);
	}
}

void eva::Maintenances::clear()
{
	_vec.clear();
	_mapId.clear();
	_vecVehicleMaintenances.clear();
	_vecUnassignedMaintenances.clear();
}
