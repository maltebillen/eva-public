#ifndef EVA_TIME_SPACE_ARCS_H
#define EVA_TIME_SPACE_ARCS_H

#include "evaConstants.h"

namespace eva
{
	namespace tsn
	{
		class TimeSpaceArcData
		{
			// ATTRIBUTES

			uint32_t _duration = Constants::BIG_INTEGER;
			uint32_t _distance = Constants::BIG_INTEGER;
			double _cost = Constants::BIG_DOUBLE;
			std::vector<Types::AccessType> _vecAccess;

		public:
			// ATTRIBUTES

			Types::Index index = Constants::BIG_INDEX;

			// CONSTRUCTORS

			TimeSpaceArcData() {}

			TimeSpaceArcData(
				const Types::Index& index,
				const uint32_t duration,
				const uint32_t distance,
				double cost
			) :
				index(index),
				_duration(duration),
				_distance(distance),
				_cost(cost)
			{}

			inline void init_access(const size_t numberVehicles, Types::AccessType val_access) { _vecAccess.resize(numberVehicles, val_access); }
			inline void set_access(const Types::Index& indexVehicle, Types::AccessType val_access) { _vecAccess[indexVehicle] = val_access; };
			inline void reset_access(Types::AccessType val_access) { std::fill(_vecAccess.begin(), _vecAccess.end(), val_access); };

			// GETTERS

			inline const bool has_access(const Types::Index& indexVehicle) const { return _vecAccess[indexVehicle] == Types::AccessType::ALLOWED; };
			inline const Types::Index& get_index() const { return index; };
			inline const uint32_t& get_duration() const { return _duration; };
			inline const uint32_t& get_distance() const { return _distance; };
			inline const double& get_cost() const { return _cost; };
		};
	}
}


#endif // !EVA_TIME_SPACE_ARCS_H
