#ifndef EVA_DATA_MAINTENANCES_H
#define EVA_DATA_MAINTENANCES_H

#include "evaConstants.h"
#include "maintenance.h"
#include "config.h"
#include "locations.h"
#include "vehicles.h"

#include <vector>
#include <unordered_map>

namespace eva
{
	class Maintenances
	{
	private:
		// ATTRIBUTES

		std::vector<Maintenance> _vec;
		
		std::unordered_map<uint32_t, Types::Index> _mapId;
		std::vector<std::vector<Types::Index>> _vecVehicleMaintenances;
		std::vector<Types::Index> _vecUnassignedMaintenances;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @ brief Default Constructor
		Maintenances() {};

		// DESTRUCTOR

		// @ brief Destructor
		~Maintenances() { clear(); }

		// FUNCTION DEFINITIONS

		const Maintenance& get_maintenanceFromId(const uint32_t& id) const;
		void read(const std::string& fileName, const Locations& locations, const Config& config, const Vehicles& vehicles);
		void clear();

		// INLINE
		// GETTERS

		const std::vector<Maintenance>& get_vec() const { return _vec; }
		const std::vector<Types::Index>& get_vehicleMaintenances(const Types::Index& indexVehicle) const { return _vecVehicleMaintenances[indexVehicle];};
		const std::vector<Types::Index>& get_unassignedMaintenances() const { return _vecUnassignedMaintenances;};

	};
}

#endif /* EVA_DATA_MAINTENANCES_H */