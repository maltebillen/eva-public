#ifndef EVA_SCHEDULE_GRAPH_ARCS_H
#define EVA_SCHEDULE_GRAPH_ARCS_H

#include "evaConstants.h"

namespace eva
{
	class ScheduleArcData
	{
		// ATTRIBUTES

		uint32_t _duration = Constants::BIG_INTEGER;

	public:
		// ATTRIBUTES

		Types::Index index = Constants::BIG_INDEX;

		// CONSTRUCTORS

		ScheduleArcData() {};

		ScheduleArcData(
			const Types::Index& index,
			const uint32_t& duration
		) :
			index(index),
			_duration(duration)
		{};

		// GETTERS

		inline const Types::Index get_index() const { return index; };
		inline const uint32_t& get_duration() const { return _duration; };
	};

}


#endif // ! EVA_SCHEDULE_GRAPH_ARCS_H
