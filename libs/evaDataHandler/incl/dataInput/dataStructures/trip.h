#ifndef EVA_DATA_TRIP_H
#define EVA_DATA_TRIP_H

#include "evaConstants.h"

#include "location.h"

namespace eva
{
	class Trip
	{
		// ATTRIBUTES:
		uint32_t _id;
		Types::DateTime _startTime;
		Types::DateTime _endTime;
		const Location& _startLocation;
		const Location& _endLocation;
		uint32_t _lineId;

	public:
		// CONSTRUCTORS:

		// @brief Default Constructor
		Trip() = delete;

		// @brief Single-argument constructor:
		Trip(
			const uint32_t& id,
			const Types::DateTime& startTime,
			const Types::DateTime& endTime,
			const Location& startLocation,
			const Location& endLocation,
			const uint32_t& lineId
		) :
			_id(id),
			_startTime(startTime),
			_endTime(endTime),
			_startLocation(startLocation),
			_endLocation(endLocation),
			_lineId(lineId)
		{};

		// GETTERS:
		inline const uint32_t& get_id() const { return _id; };
		inline const Types::DateTime& get_startTime() const { return _startTime; };
		inline const Types::DateTime& get_endTime() const { return _endTime; };
		inline const Location& get_startLocation() const { return _startLocation; };
		inline const Location& get_endLocation() const { return _endLocation; };
		inline const uint32_t& get_lineId() const { return _lineId; };

	};
}

#endif /* EVA_DATA_TRIP_H */
