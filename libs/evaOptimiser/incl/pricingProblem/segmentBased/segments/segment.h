#ifndef EVA_SEGMENT_H
#define EVA_SEGMENT_H

#include "evaConstants.h"
#include "moderator/OptimisationInput.h"
#include "moderator/moderator.h"

#include "subGraph/subGraph.h"

namespace eva
{
	namespace sbn
	{
		enum class SegmentActivityType : uint8_t
		{
			DEFAULT = 0,
			TRIP,
			MAINTENANCE
		};

		class SegmentActivity
		{
			Types::Index _startLocationIndex = Constants::BIG_INDEX;
			Types::Index _endLocationIndex = Constants::BIG_INDEX;
			uint32_t _distance = Constants::BIG_UINTEGER;
			SegmentActivityType _type = SegmentActivityType::DEFAULT;

			std::vector<Types::Index> _vecSegmentReferenceIndexes;

		public:
			SegmentActivity(
				const Types::Index& startLocationIndex,
				const Types::Index& endLocationIndex,
				const uint32_t& distance,
				const SegmentActivityType& type
			) :
				_startLocationIndex(startLocationIndex),
				_endLocationIndex(endLocationIndex),
				_distance(distance),
				_type(type)
			{};

			// FUNCTION DEFINITONS

			inline bool operator==(const SegmentActivity& comp)
			{
				return this->_startLocationIndex == comp._startLocationIndex
					&& this->_endLocationIndex == comp._endLocationIndex
					&& this->_type == comp._type
					&& this->_distance == comp._distance;
			}

			static Types::BatteryCharge sumDistance(uint32_t lhs, const SegmentActivity& rhs)
			{
				return lhs + rhs._distance;
			}

			// GETTERS:

			inline void appendReferenceIndex(const Types::Index refIndex) { _vecSegmentReferenceIndexes.push_back(refIndex); };

			inline const Types::Index& get_startLocationIndex() const { return _startLocationIndex; };
			inline const Types::Index& get_endLocationIndex() const { return _endLocationIndex; };
			inline const uint32_t& get_distance() const { return _distance; };
			inline const SegmentActivityType& get_type() const { return _type; };
			inline const std::vector<Types::Index>& get_vecSegmentReferenceIndexes() const { return _vecSegmentReferenceIndexes; };
		};

		class Segment
		{
			Types::Index _index;
			const Charger& _startCharger;
			const Charger& _endCharger;

			subgraph::SegmentSubGraph _subgraph;

			double _deadlegFixCost = 0.0;
			double _maintenanceFixCost = 0.0;
			double _maintenanceVariableCostCoeff = 0.0;

			uint32_t _distanceBeforeMaintenance = 0;
			uint32_t _distanceAfterMaintenance = 0;
			bool _hasMaintenance = false;

			uint32_t _minlb_rechargeDuration = Constants::BIG_UINTEGER;
			uint32_t _minlb_fullPresenceAtCharger = Constants::BIG_UINTEGER;
			uint32_t _maxub_rechargeDuration = 0;
			uint32_t _maxub_fullPresenceAtCharger = 0;

			std::vector<Types::AccessType> _vecFeasibleVehicle;

			// PRIVATE FUNCTIONS:

			void _initSegmentMetrics(const OptimisationInput& optinput, const std::vector<SegmentActivity>& vecActivities);
			void _initChargingBounds(const OptimisationInput& optinput);
			void _initFeasibileVehicles(const OptimisationInput& optinput);
			void _initSubgraph(const OptimisationInput& optinput, const std::vector<SegmentActivity>& vecActivities);

			void _initialise(const OptimisationInput& optinput, const std::vector<SegmentActivity>& vecActivities);

		public:
			Segment() = delete;

			Segment(
				const Types::Index& index,
				const OptimisationInput& optinput,
				const Charger& startCharger,
				const Charger& endCharger,
				const std::vector<SegmentActivity>& vecActivities
			) :
				_index(index),
				_startCharger(startCharger),
				_endCharger(endCharger),
				_subgraph(optinput)
			{
				_initialise(optinput, vecActivities);
			};

			// PUBLIC FUNCTIONS
			
			inline void updateNonDominatedSchedulePieces(const BranchNode& brn, const Duals& duals) { _subgraph.updateCurrentNonDominatedSchedulePieces(brn, duals); };
			inline void updateVehicleFixings(const BranchNode& brn) { _subgraph.updateFixings(brn);};

			// GETTERS:

			inline const Charger& get_startCharger() const { return _startCharger; };
			inline const Charger& get_endCharger() const { return _endCharger; };

			inline const double& get_deadlegFixCost() const { return _deadlegFixCost; };
			inline const double& get_maintenanceFixCost() const { return _maintenanceFixCost; };
			inline const double& get_maintenanceVariableCostCoeff() const { return _maintenanceVariableCostCoeff; };
			inline const double get_cost(const uint32_t& prev_distance) const { return _deadlegFixCost + _maintenanceFixCost + _maintenanceVariableCostCoeff * prev_distance; };

			inline const uint32_t get_updated_distance(const uint32_t& prev_distance) const { return _hasMaintenance ? _distanceAfterMaintenance : prev_distance + _distanceBeforeMaintenance; };
			inline const uint32_t& get_distanceBeforeMaintenance() const { return _distanceBeforeMaintenance; };
			inline const uint32_t& get_distanceAfterMaintenance() const { return _distanceAfterMaintenance; };
			inline const uint32_t get_total_distance() const { return _distanceBeforeMaintenance + _distanceAfterMaintenance;};
			inline const bool get_hasMaintenance() const { return _hasMaintenance; };
			inline const bool get_isVehicleFeasible(const Types::Index& indexVehicle) const { return _vecFeasibleVehicle[indexVehicle] == Types::AccessType::ALLOWED; };

			inline const uint32_t& get_minlb_rechargeDuration() const { return _minlb_rechargeDuration; };
			inline const uint32_t& get_maxub_rechargeDuration() const { return _maxub_rechargeDuration; };
			inline const uint32_t& get_minlb_fullPresenceAtCharger() const { return _minlb_fullPresenceAtCharger; };
			inline const uint32_t& get_maxub_fullPresenceAtCharger() const { return _maxub_fullPresenceAtCharger; };

			inline const Types::Index& get_index() const { return _index; };

			inline const std::vector<subgraph::NonDominatedSchedulePiece>& get_vecNonDominatedSchedulePieces() const { return _subgraph.get_vecCurrentNonDominatedSchedulePieces(); };
		};
	}
}
#endif // !EVA_SEGMENT_H
