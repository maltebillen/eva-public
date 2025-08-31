#include "incl/dataInput/dataStructures/vehicles.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "evaExceptions.h"

const eva::Vehicle& eva::Vehicles::get_vehicleFromId(const uint32_t& id) const
{
	auto it = _mapId.find(id);
	if (it == _mapId.end())
		throw DataError("eva::Vehicles::get_vehicleFromId", "Given id does not exist.");

	return _vec[it->second];
}

void eva::Vehicles::read(const std::string& fileName, const Chargers& chargers)

{
	// Open the file and check for success
	// ----------------------------------------------------------------------------------------------
	std::ifstream in(fileName.c_str(), std::ios::in);
	if (!in)
		throw FileError("eva::Vehicles::read", "File \"" + fileName + "\" does not exist!!!");

	// Definitions
	bool first_line = true;
	std::string line, str;

	// Columns:
	Types::Index index;
	uint32_t id;
	Types::BatteryCharge batteryMinKWh;
	Types::BatteryCharge batteryMaxKWh;
	uint32_t initialChargerId;
	Types::DateTime initialStartTime;
	Types::BatteryCharge initialSOC;
	Types::BatteryCharge chargingSpeedVolts;
	Types::BatteryCharge chargingSpeedAmps;
	std::string numberPlate;
	uint32_t odometerReading;
	uint32_t odometerLastMaintenance;
	bool inRotation;
	double cost;
	double kwh_per_km;

	// ----------------------------------------------------------------------------------------------
	// Now, go over all lines in the file and read the Vehicles.
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
			throw DataError("eva::Vehicles::read", "Missing Id.");
		id = atoi(str.c_str());

		// batteryMinKWh:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing batteryMinKWh.");
		batteryMinKWh = atof(str.c_str());

		// batteryMaxKWh:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing batteryMaxKWh.");
		batteryMaxKWh = atof(str.c_str());

		// initialChargerId:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing initialChargerId.");
		initialChargerId = atoi(str.c_str());

		// initialStartTime:
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing initialStartTime.");
		initialStartTime = Helper::StringToDateTime(str);

		// initialSOC
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing initialSOC.");
		initialSOC = atof(str.c_str());

		// chargingSpeedVolts
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing chargingSpeedVolts.");
		chargingSpeedVolts = atof(str.c_str());

		// chargingSpeedAmps
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing chargingSpeedAmps.");
		chargingSpeedAmps = atof(str.c_str());

		// numberPlate
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing numberPlate.");
		numberPlate = str;

		// odometerReading
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing odometerReading.");
		odometerReading = atoi(str.c_str());

		// odometerLastMaintenance
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing odometerLastMaintenance.");
		odometerLastMaintenance = atoi(str.c_str());

		// inRotation
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing inRotation.");
		inRotation = Helper::stringToBoolean(str);

		// cost
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing Cost.");
		cost = atof(str.c_str());

		// kwh_per_km
		std::getline(lineStream, str, ',');
		if (str.length() < 1)
			throw DataError("eva::Vehicles::read", "Missing Kwh_per_km.");
		kwh_per_km = atof(str.c_str());

		// Push new trip object to the vector:
		_vec.push_back(
			Vehicle(
				index,
				id,
				batteryMinKWh,
				batteryMaxKWh,
				chargers.get_chargerFromId(initialChargerId),
				initialStartTime,
				initialSOC,
				chargingSpeedVolts,
				chargingSpeedAmps,
				numberPlate,
				odometerReading,
				odometerLastMaintenance,
				inRotation,
				cost,
				std::nearbyint(kwh_per_km * 1000)
			)
		);
	}

	// Initialise the map to store the id<>index reference:
	_mapId.reserve(_vec.size());
	for (Types::Index index = 0; index < _vec.size(); index++)
		_mapId.insert({ _vec[index].get_id(), index });
}

void eva::Vehicles::clear()
{
	_vec.clear();
	_mapId.clear();
}
