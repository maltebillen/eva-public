#ifndef EVA_SEGMENTS_H
#define EVA_SEGMENTS_H

#include "moderator/OptimisationInput.h"
#include "segment.h"

namespace eva
{
	namespace sbn
	{
		class Segments
		{
			// MEMBERS

			std::vector<Segment> _vecSegments;

			Types::Index _ctrIndex{0};

			// PRIVATE FUNCTIONS

			void _createSegments(const OptimisationInput& optinput);
			void _recursionGenerateSegments(
				const OptimisationInput& optinput,
				const std::vector<SegmentActivity>& vecAllActivities, 
				const Charger& startCharger, 
				const Charger& endCharger, 
				const uint32_t& range, 
				const Location curLocation, 
				const uint32_t distance,
				std::vector<SegmentActivity> vecThisSegmentActivities
			);

			inline Types::Index _getNextIndex() { return _ctrIndex++; }


		public:
			Segments() {};

			// PUBLIC FUNCTIONS:

			void initialise(const OptimisationInput& _optinput);
			

			// GETTERS:

			inline const std::vector<Segment>& get_vec() const { return _vecSegments; };
			inline std::vector<Segment>& get_vec() { return _vecSegments; };
		};
	}
}

#endif // !EVA_SEGMENTS_H
