#ifndef EVA_DATA_TRIPS_H
#define EVA_DATA_TRIPS_H

#include "evaConstants.h"
#include "trip.h"
#include "config.h"
#include "locations.h"

#include <vector>
#include <unordered_map>

namespace eva
{
	class Trips
	{
	private:
		// ATTRIBUTES

		std::vector<Trip> _vec;
		std::unordered_map<uint32_t, Types::Index> _mapId;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @ brief Default Constructor
		Trips() {};

		// DESTRUCTOR

		// @ brief Destructor
		~Trips() { clear(); }

		// FUNCTION DEFINITIONS

		const Trip& get_tripFromId(const uint32_t& id) const;
		void read(const std::string& fileName, const Locations& locations, const Config& config);
		void clear();

		// INLINE
		// GETTERS

		const std::vector<Trip>& get_vec() const { return _vec; }

	};
}

#endif /* EVA_DATA_TRIPS_H */