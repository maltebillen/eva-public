#ifndef EVA_DATA_INPUT_H
#define EVA_DATA_INPUT_H

#include "dataStructures/config.h"
#include "dataStructures/locations.h"
#include "dataStructures/chargers.h"
#include "dataStructures/trips.h"
#include "dataStructures/vehicles.h"
#include "dataStructures/maintenances.h"

namespace eva {
	class DataInput
	{
		// ATTRIBUTES:
		Config _config;
		Locations _locations;
		Chargers _chargers;
		Trips _trips;
		Vehicles _vehicles;
		Maintenances _maintenances;

	public:
		DataInput()	{};

		~DataInput()
		{
			clear();
		};

		// FUNCTIONS DEFINITIONS:

		void clear();
		void initialiseFromCsv(const Types::CommandInput& commandInput);


		// GETTERS:

		inline const Config& get_config() const { return _config; };
		inline const Locations& get_locations() const { return _locations; };
		inline const Chargers& get_chargers() const { return _chargers; };
		inline const Trips& get_trips() const { return _trips; };
		inline const Vehicles& get_vehicles() const { return _vehicles; };
		inline const Maintenances& get_maintenances() const { return _maintenances; };
	};
}

#endif // EVA_INPUT_H