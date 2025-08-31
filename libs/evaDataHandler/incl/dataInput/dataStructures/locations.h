#ifndef EVA_LOCATIONS_H
#define EVA_LOCATIONS_H

#include "evaConstants.h"
#include "location.h"

#include <vector>
#include <unordered_map>

namespace eva
{
	class Locations
	{
	private:
		// ATTRIBUTES

		std::vector<Location> _vec;
		std::unordered_map<uint32_t, Types::Index> _mapId;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @ brief Default Constructor
		Locations() {};

		// DESTRUCTOR

		// @ brief Destructor
		~Locations() { clear(); }

		// FUNCTION DEFINITIONS

		const Location& get_locationFromId(const uint32_t& id) const;
		Location& get_locationFromId(const uint32_t& id);
		void read(const std::string& fileName);
		void readTravel(const std::string& fileName);
		void clear();

		// INLINE
		// GETTERS

		const std::vector<Location>& get_vec() const { return _vec; }
		const Location& get_location(const Types::Index& indexLocation) const { return _vec[indexLocation]; }
		
	};
}

#endif // EVA_LOCATIONS_H