#ifndef EVA_DATA_MAINTENANCE_H
#define EVA_DATA_MAINTENANCE_H

#include "evaConstants.h"

#include "location.h"
#include "vehicle.h"

namespace eva
{
	class Maintenance
	{
		// ATTRIBUTES:

		uint32_t _id;
		Types::DateTime _startTime;
		Types::DateTime _endTime;
		const Location& _maintenanceLocation;
		Types::Index _indexVehicle;

	public:
		// CONSTRUCTORS:

		// @brief Default Constructor
		Maintenance() = delete;

		// @brief Single-argument constructor:
		Maintenance(
			const uint32_t& id,
			const Types::DateTime& startTime,
			const Types::DateTime& endTime,
			const Location& maintenanceLocation,
			const Types::Index& indexVehicle
		) :
			_id(id),
			_startTime(startTime),
			_endTime(endTime),
			_maintenanceLocation(maintenanceLocation),
			_indexVehicle(indexVehicle)
		{};

		// GETTERS:
		inline const uint32_t& get_id() const { return _id; };
		inline const Types::DateTime& get_startTime() const { return _startTime; };
		inline const Types::DateTime& get_endTime() const { return _endTime; };
		inline const Location& get_maintenanceLocation() const { return _maintenanceLocation; };
		inline const Types::Index& get_indexVehicle() const { return _indexVehicle; };

		inline const bool is_unassigned() const { return _indexVehicle == Constants::BIG_INDEX; }; 
		inline const bool is_assigned() const { return _indexVehicle != Constants::BIG_INDEX; }; 

	};
}

#endif /* EVA_DATA_MAINTENANCE_H */
