#ifndef EVA_SEGMENTS_SUB_GRAPH_ARCS_H
#define EVA_SEGMENTS_SUB_GRAPH_ARCS_H

#include "evaConstants.h"

namespace eva
{
	namespace sbn
	{
		namespace subgraph
		{
			struct ArcData
			{
				// ATTRIBUTES 

				Types::Index index = Constants::BIG_INDEX;

				// CONSTRUCTORS

				ArcData() {}

				ArcData(
					const Types::Index& index
				) :
					index(index)
				{}

				// GETTERS:

				inline const Types::Index get_index() const { return index; };
			};
		}
	}
}

#endif // EVA_SEGMENTS_SUB_GRAPH_ARCS_H
