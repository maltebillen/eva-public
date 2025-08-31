#include "incl/dataInput/dataInput.h"

#include <iostream>

void eva::DataInput::clear()
{
	_locations.clear();
}

void eva::DataInput::initialiseFromCsv(const Types::CommandInput& commandInput)
{
	std::cout << "Reading Data: " << std::endl;

	std::string file_name;
	std::string path_to_data_files = commandInput.PathToData;

	// Read the config parameters:
	file_name = commandInput.PathToConfig + "config.csv";
	_config.set_path_to_data(path_to_data_files);
	_config.set_path_to_config(commandInput.PathToConfig);
	_config.set_path_to_output(commandInput.PathToConfig + "outputs/");
	_config.read_override(file_name);
	std::cout << "...read " << " config:" << std::endl;
	std::cout << " - " << "[Path] Data: " << _config.get_path_to_data() << std::endl;
	std::cout << " - " << "[Path] Config: " << _config.get_path_to_config() << std::endl;
	std::cout << " - " << "[Path] Output: " << _config.get_path_to_output() << std::endl;
	std::cout << " - " << "[Date] Start: " << Helper::DateTimeToString(_config.get_date_start()) << std::endl;
	std::cout << " - " << "[Date] End: " << Helper::DateTimeToString(_config.get_date_end()) << std::endl;
	std::cout << std::endl;

	// Read locations
	file_name = path_to_data_files + "locations.csv";
	_locations.read(file_name);
	std::cout << "...read " << _locations.get_vec().size() << " locations." << std::endl;

	// Read distances, durations, and battery consumptions
	file_name = path_to_data_files + "locations_distances.csv";
	_locations.readTravel(file_name);
	std::cout << "...read " << " location_distances." << std::endl;

	// Read chargers
	file_name = path_to_data_files + "chargers.csv";
	_chargers.read(file_name, _locations);
	std::cout << "...read " << _chargers.get_vec().size() << " chargers." << std::endl;

	// Read trips
	file_name = path_to_data_files + "trips.csv";
	_trips.read(file_name, _locations, _config);
	std::cout << "...read " << _trips.get_vec().size() << " trips." << std::endl;

	// Read vehicles
	file_name = path_to_data_files + "vehicles.csv";
	_vehicles.read(file_name, _chargers);
	std::cout << "...read " << _vehicles.get_vec().size() << " vehicles." << std::endl;

	// Read maintenance
	file_name = path_to_data_files + "maintenances.csv";
	_maintenances.read(file_name, _locations, _config, _vehicles);
	std::cout << "...read " << _maintenances.get_vec().size() << " maintenances." << std::endl;

	// Output Summary:
	std::cout << std::endl;
}
