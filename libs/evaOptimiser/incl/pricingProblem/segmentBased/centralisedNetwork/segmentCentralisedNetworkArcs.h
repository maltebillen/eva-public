#ifndef EVA_SEGMENT_CENTRALISED_NETWORK_ARCS_H
#define EVA_SEGMENT_CENTRALISED_NETWORK_ARCS_H

#include "evaConstants.h"
#include "moderator/OptimisationInput.h"

#include <boost/dynamic_bitset.hpp>

namespace eva
{
    namespace sbn
    {
        namespace cen
        {
            class CenArcData
			{
				std::vector<Types::AccessType> _vecAccess;
				Types::Index _fromNode = Constants::BIG_INDEX;
				Types::Index _toNode = Constants::BIG_INDEX;

				inline void _reset_access(Types::AccessType val_access) { std::fill(_vecAccess.begin(), _vecAccess.end(), val_access); };
				inline void _set_vehicle_access(const Types::Index &indexVehicle, Types::AccessType val_access) { _vecAccess[indexVehicle] = val_access; };

			public:
				CenArcData() {};
				CenArcData(
					const uint32_t &nrVehicles, 
					const Types::Index& fromNode,
					const Types::Index& toNode
				) :
					_fromNode(fromNode),
					_toNode(toNode)
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				}

				Types::Index index = Constants::BIG_INDEX;

				inline const Types::Index get_index() const { return index; };

				inline const bool is_feasible() const { return std::any_of(_vecAccess.begin(), _vecAccess.end(), [](Types::AccessType val) { return val == Types::AccessType::ALLOWED; });};

				inline void reset_default_fixings(const std::vector<Types::AccessType>& vecDefault) { _vecAccess = vecDefault; }
				inline void fix_vehicle(const Types::Index& indexVehicle) { _reset_access(Types::AccessType::NOT_ALLOWED); _set_vehicle_access(indexVehicle, Types::AccessType::ALLOWED);};
				inline void revoke_access(const Types::Index& indexVehicle) { _set_vehicle_access(indexVehicle, Types::AccessType::NOT_ALLOWED);}
				inline const bool has_access(const Types::Index &indexVehicle) const { return _vecAccess[indexVehicle] == Types::AccessType::ALLOWED; };
				inline const std::vector<Types::AccessType>& get_vecAccess() const { return _vecAccess; };
				inline void fix_infeasible() { _reset_access(Types::AccessType::NOT_ALLOWED);}
				
			
				inline const Types::Index get_fromNode() const {return _fromNode;};
				inline const Types::Index get_toNode() const {return _toNode;};
			};

        }
    }
}

#endif //! EVA_SEGMENT_CENTRALISED_NETWORK_ARCS_H