#ifndef EVA_DATA_CHARGERS_H
#define EVA_DATA_CHARGERS_H

#include "evaConstants.h"
#include "charger.h"

#include "locations.h"

#include <vector>
#include <unordered_map>

namespace eva
{
	class Chargers
	{
	private:
		// ATTRIBUTES

		std::vector<Charger> _vec;
		std::unordered_map<uint32_t, Types::Index> _mapId;

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @ brief Default Constructor
		Chargers() {};

		// DESTRUCTOR

		// @ brief Destructor
		~Chargers() { clear(); }

		// FUNCTION DEFINITIONS

		const Charger& get_chargerFromId(const uint32_t& id) const;
		void read(const std::string& fileName, const Locations& locations);
		void clear();

		// INLINE
		// GETTERS

		const std::vector<Charger>& get_vec() const { return _vec; }

	};
}

#endif /* EVA_DATA_CHARGERS_H */