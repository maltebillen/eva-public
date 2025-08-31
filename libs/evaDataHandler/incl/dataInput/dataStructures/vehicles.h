#ifndef EVA_DATA_VEHICLES_H
#define EVA_DATA_VEHICLES_H

#include "evaConstants.h"
#include "vehicle.h"
#include "chargers.h"

#include <vector>
#include <unordered_map>

namespace eva
{
	class Vehicles
	{
	private:
		// ATTRIBUTES

		std::vector<Vehicle> _vec;
		std::unordered_map<uint32_t, Types::Index> _mapId;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @ brief Default Constructor
		Vehicles() {};

		// DESTRUCTOR

		// @ brief Destructor
		~Vehicles() { clear(); }

		// FUNCTION DEFINITIONS

		const Vehicle& get_vehicleFromId(const uint32_t& id) const;
		void read(const std::string& fileName, const Chargers& chargers);
		void clear();

		// INLINE
		// GETTERS

		const std::vector<Vehicle>& get_vec() const { return _vec; }
		const Vehicle& get_vehicle(const Types::Index& indexVehicle) const { return _vec[indexVehicle]; };

	};
}

#endif /* EVA_DATA_VEHICLES_H */