#ifndef EVA_LOCATION_H
#define EVA_LOCATION_H

#include "evaConstants.h"
#include <string>
#include <vector>

namespace eva
{
	class Location
	{
	public:
		// TYPE DEFINITIONS
		enum class LocationType : uint8_t {
			Undefined = 0,
			Stop = 1,
			Charger = 5,
			Maintenance = 11
		};

		struct Measures
		{
			uint32_t distance = Constants::BIG_UINTEGER;
			uint32_t duration = Constants::BIG_UINTEGER;

			Measures() {};

			Measures(
				const uint32_t& dist,
				const uint32_t& dur
			) :
				distance(dist),
				duration(dur)
			{};
		};

	private:
		// ATTRIBUTES

		Types::Index _index;
		uint32_t _id;
		Location::LocationType _type;
		std::string _name;

		std::vector<Measures> _vecMeasures;

		// FUNCTION DEFINITIONS	

	public:
		// CONSTRUCTORS
		Location() {};

		Location(
			Types::Index index,
			uint32_t id,
			Location::LocationType type,
			const std::string& name
		) :
			_index(index),
			_id(id),
			_type(type),
			_name(name)
		{};

		// FUNCTION DEFINITIONS

		static const Location::LocationType string2type(const std::string& str);
		
		// GETTERS

		std::vector<Measures>& get_vecMeasures() { return _vecMeasures; };

		inline const Types::Index& get_index() const { return _index; };
		inline const uint32_t& get_id() const { return _id; };
		inline const Location::LocationType& get_type() const { return _type; };
		inline const std::string& get_name() const { return _name; };

		inline const uint32_t get_distanceToLocation(const Location& location) const { return _vecMeasures[location.get_index()].distance; };
		inline const uint32_t get_distanceToLocation(const Types::Index& locationIndex) const { return _vecMeasures[locationIndex].distance; };
		inline const uint32_t get_durationToLocation(const Location& location) const { return _vecMeasures[location.get_index()].duration; };
		inline const uint32_t get_durationToLocation(const Types::Index& locationIndex) const { return _vecMeasures[locationIndex].duration; };

	};
}

#endif // EVA_LOCATION_H