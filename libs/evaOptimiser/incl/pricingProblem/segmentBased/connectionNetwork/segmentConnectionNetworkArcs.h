#ifndef EVA_SEGMENT_CONNECTION_NETWORK_ARCS_H
#define EVA_SEGMENT_CONNECTION_NETWORK_ARCS_H

#include "evaConstants.h"
#include "moderator/OptimisationInput.h"

#include <boost/dynamic_bitset.hpp>

namespace eva
{
	namespace sbn
	{
		namespace con
		{
			class ConArcData
			{
				std::vector<Types::AccessType> _vecAccess;
				Types::Index _indexFixedVehicle = Constants::BIG_INDEX;

				inline void _reset_access(Types::AccessType val_access) { std::fill(_vecAccess.begin(), _vecAccess.end(), val_access); };
				inline void _set_vehicle_access(const Types::Index &indexVehicle, Types::AccessType val_access) { _vecAccess[indexVehicle] = val_access; };

			public:
				ConArcData() {};
				ConArcData(
					const uint32_t &nrVehicles
				)
				{
					_vecAccess.resize(nrVehicles, Types::AccessType::ALLOWED);
				}

				Types::Index index = Constants::BIG_INDEX;
				Types::Index fromNode = Constants::BIG_INDEX;
				Types::Index toNode = Constants::BIG_INDEX;

				inline const Types::Index get_index() const { return index; };
				inline const bool is_feasible() const { return std::any_of(_vecAccess.begin(), _vecAccess.end(), [](Types::AccessType val) { return val == Types::AccessType::ALLOWED; });};

				inline void reset_default_fixings(const std::vector<Types::AccessType>& vecDefault) { _vecAccess = vecDefault; _indexFixedVehicle = Constants::BIG_INDEX;}
				inline void fix_vehicle(const Types::Index& indexVehicle) { _indexFixedVehicle = indexVehicle; _reset_access(Types::AccessType::NOT_ALLOWED); _set_vehicle_access(indexVehicle, Types::AccessType::ALLOWED);};
				inline void revoke_access(const Types::Index& indexVehicle) { _set_vehicle_access(indexVehicle, Types::AccessType::NOT_ALLOWED);}
				inline const bool has_access(const Types::Index &indexVehicle) const { return _vecAccess[indexVehicle] == Types::AccessType::ALLOWED; };
				inline const std::vector<Types::AccessType>& get_vecAccess() const { return _vecAccess; };
				inline const Types::Index& get_indexFixedVehicle() const { return _indexFixedVehicle;};
			};

			struct FullConArcData : public ConArcData
			{
				// ATTRIBUTES

				// DOMINATE ATTRIBUTES (SINK):
				double accDuals = Constants::BIG_DOUBLE;
				double minChargingDuals = Constants::BIG_DOUBLE;
				double maxChargingDuals = Constants::BIG_DOUBLE;
				double bestCaseCost = Constants::BIG_DOUBLE;
				Types::DateTime sinkEndTime = Constants::MAX_TIMESTAMP;
				int64_t chargingDuration = 0;

				// CONSTRUCTORS
				FullConArcData(
					const uint32_t &nrVehicles) : ConArcData(nrVehicles) {};

				// GETTERS:

				inline const bool dominates(const FullConArcData &dominatedArc) const
				{
					// Either both are NA, or both are fixed to the same vehicle. Otherwise, no dominance can be featured:
					if(Helper::compare_is_subset(get_vecAccess(), dominatedArc.get_vecAccess())
						&& get_indexFixedVehicle() == dominatedArc.get_indexFixedVehicle())
					{
						return chargingDuration >= dominatedArc.chargingDuration
						&& sinkEndTime <= dominatedArc.sinkEndTime // If the end-time is smaller, than more or equal connection are possible
						&& Helper::compare_floats_smaller_equal(dominatedArc.accDuals + dominatedArc.maxChargingDuals, accDuals + minChargingDuals); // If the best-case duals are worse than the worst-case duals
					}
					else
					{
						return false;
					};
				};
			};

		}
	}
}

#endif // !EVA_SEGMENT_CONNECTION_NETWORK_ARCS_H
