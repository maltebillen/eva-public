#ifndef EVA_SEGMENTS_SUB_GRAPH_NODES_H
#define EVA_SEGMENTS_SUB_GRAPH_NODES_H

#include "evaConstants.h"
#include "moderator/OptimisationInput.h"

namespace eva
{
	namespace sbn
	{
		namespace subgraph
		{
			enum class NodeType : uint8_t
			{
				TRIP = 1,
				MAINTENANCE,
				START_SEGMENT,
				END_SEGMENT,
				COLLECTIVE_START_SEGMENT,
				COLLECTIVE_END_SEGMENT,
				UNDEFINED
			};
			
			class BaseNodeData
			{
			public:
				// INLINE FUNCTIONS 
				inline std::unique_ptr<BaseNodeData> clone() const { return std::unique_ptr<BaseNodeData>(deepClone()); };

				virtual inline const Types::Index get_startLocationIndex() const = 0;
				virtual inline const Types::Index get_endLocationIndex() const = 0;
				virtual inline const Types::DateTime get_startTime() const = 0;
				virtual inline const Types::DateTime get_endTime() const = 0;

				virtual ~BaseNodeData() {};

			protected:
				virtual inline BaseNodeData* deepClone() const = 0;
			};

			class UndefinedNodeData : public BaseNodeData
			{
			public:

				// CONSTRUCTORS
				UndefinedNodeData() {}

				// GETTERS

				virtual inline const Types::Index get_startLocationIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::Index get_endLocationIndex() const override { return Constants::BIG_INDEX; };
				virtual inline const Types::DateTime get_startTime() const override { return Types::DateTime(Constants::MAX_TIMESTAMP); };
				virtual inline const Types::DateTime get_endTime() const override { return Types::DateTime(Constants::MAX_TIMESTAMP); };

			protected:
				virtual inline UndefinedNodeData* deepClone() const override { return new UndefinedNodeData(*this); };
			};

			class TripNodeData : public BaseNodeData
			{
				// ATTRIBUTES

				const SubScheduleTripNodeData& _trip;

			public:
				// CONSTRUCTORS

				TripNodeData(
					const SubScheduleTripNodeData& trip
				) :
					_trip(trip)
				{}

				// GETTERS

				inline const SubScheduleTripNodeData& get_subScheduleTripNodeData() const { return _trip; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _trip.get_scheduleNodeData().get_startLocation().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _trip.get_scheduleNodeData().get_endLocation().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return _trip.get_scheduleNodeData().get_startTime(); };
				virtual inline const Types::DateTime get_endTime() const override { return _trip.get_scheduleNodeData().get_endTime(); };

			protected:
				virtual TripNodeData* deepClone() const override { return new TripNodeData(*this); };
			};

			class MaintenanceNodeData : public BaseNodeData
			{
				// ATTRIBUTES

				const SubScheduleMaintenanceNodeData& _maintenance;

			public:
				// CONSTRUCTORS

				MaintenanceNodeData(
					const SubScheduleMaintenanceNodeData& maintenance
				) :
					_maintenance(maintenance)
				{}

				// GETTERS

				inline const SubScheduleMaintenanceNodeData& get_subScheduleMaintenanceNodeData() const { return _maintenance; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _maintenance.get_scheduleNodeData().get_startLocation().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _maintenance.get_scheduleNodeData().get_endLocation().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return _maintenance.get_scheduleNodeData().get_startTime(); };
				virtual inline const Types::DateTime get_endTime() const override { return _maintenance.get_scheduleNodeData().get_endTime(); };


			protected:
				virtual MaintenanceNodeData* deepClone() const override { return new MaintenanceNodeData(*this); };
			};

			class StartSegmentNodeData : public BaseNodeData {

				// ATTRIBUTES

				const Charger& _charger;
				const Types::DateTime _time;

				const Types::Index _earliestChargingStartIndex;
				const Types::Index _latestChargingStartIndex;
				const Types::Index _latestChargingEndIndex;

			public:
				// CONSTRUCTORS

				StartSegmentNodeData(
					const Charger& charger,
					const Types::DateTime& time,
					const Types::Index& earliestChargingStartIndex,
					const Types::Index& latestChargingStartIndex,
					const Types::Index& latestChargingEndIndex
				) :
					_charger(charger),
					_time(time),
					_earliestChargingStartIndex(earliestChargingStartIndex),
					_latestChargingStartIndex(latestChargingStartIndex),
					_latestChargingEndIndex(latestChargingEndIndex)
				{}

				// FUNCTION:
				
				// LOCAL GETTERS:

				// GETTERS

				inline const Charger& get_charger() const { return _charger; };
				inline const Types::Index& get_earliestChargingStartIndex() const { return _earliestChargingStartIndex; };
				inline const Types::Index& get_latestChargingStartIndex() const { return _latestChargingStartIndex; };
				inline const Types::Index& get_latestChargingEndIndex() const { return _latestChargingEndIndex; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return _time; };
				virtual inline const Types::DateTime get_endTime() const override { return _time; };


			protected:
				virtual StartSegmentNodeData* deepClone() const override { return new StartSegmentNodeData(*this); };
			};

			class EndSegmentNodeData : public BaseNodeData {
				// ATTRIBUTES

				const Charger& _charger;
				const Types::DateTime _time;

			public:
				// CONSTRUCTORS

				EndSegmentNodeData(
					const Charger& charger,
					const Types::DateTime& time
				) :
					_charger(charger),
					_time(time)
				{}

				// GETTERS

				inline const Charger& get_charger() const { return _charger; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return _time; };
				virtual inline const Types::DateTime get_endTime() const override { return _time; };

			protected:
				virtual EndSegmentNodeData* deepClone() const override { return new EndSegmentNodeData(*this); };
			};

			class CollectiveEndSegmentNodeData : public BaseNodeData {
				// ATTRIBUTES

				const Charger& _charger;

			public:
				// CONSTRUCTORS

				CollectiveEndSegmentNodeData(
					const Charger& charger
				) :
					_charger(charger)
				{}

				// GETTERS

				inline const Charger& get_charger() const { return _charger; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
				virtual inline const Types::DateTime get_endTime() const override { return Constants::MAX_TIMESTAMP; };

			protected:
				virtual CollectiveEndSegmentNodeData* deepClone() const override { return new CollectiveEndSegmentNodeData(*this); };
			};

			class CollectiveStartSegmentNodeData : public BaseNodeData {
				// ATTRIBUTES

				const Charger& _charger;

			public:
				// CONSTRUCTORS

				CollectiveStartSegmentNodeData(
					const Charger& charger
				) :
					_charger(charger)
				{
				}

				// GETTERS

				inline const Charger& get_charger() const { return _charger; };

				virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
				virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
				virtual inline const Types::DateTime get_endTime() const override { return Constants::MAX_TIMESTAMP; };

			protected:
				virtual CollectiveStartSegmentNodeData* deepClone() const override { return new CollectiveStartSegmentNodeData(*this); };
			};

			class NodeData
			{
				std::vector<Types::AccessType> _vecAccess;
				Types::Index _indexFixedVehicle = Constants::BIG_INDEX;
				Types::DateTime _fixedPathEndTime = Constants::MAX_TIMESTAMP;
				Types::DateTime _fixedPathStartTime = Constants::MAX_TIMESTAMP;

				inline void _reset_access(Types::AccessType val_access) { std::fill(_vecAccess.begin(), _vecAccess.end(), val_access); };
				inline void _set_vehicle_access(const Types::Index& indexVehicle, Types::AccessType val_access) { _vecAccess[indexVehicle] = val_access;};

			public:
				// ATTRIBUTES:
			
				std::unique_ptr<BaseNodeData> ptrNodeData;
				Types::Index index = Constants::BIG_INDEX;
				NodeType type = NodeType::UNDEFINED;
				double acc_dual = 0.0;
				double min_charging_dual = 0.0;
				double max_charging_dual = 0.0;
				
				// CONSTRUCTORS

				NodeData() :
					index(Constants::BIG_INDEX),
					type(NodeType::UNDEFINED),
					ptrNodeData(std::make_unique<UndefinedNodeData>())
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const TripNodeData& tnd,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::TRIP),
					ptrNodeData(std::make_unique<TripNodeData>(tnd)),
					_vecAccess(vecDefault)
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const MaintenanceNodeData& mnd,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::MAINTENANCE),
					ptrNodeData(std::make_unique<MaintenanceNodeData>(mnd)),
					_vecAccess(vecDefault)
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const StartSegmentNodeData& ss,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::START_SEGMENT),
					ptrNodeData(std::make_unique<StartSegmentNodeData>(ss)),
					_vecAccess(vecDefault)
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const EndSegmentNodeData& ss,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::END_SEGMENT),
					ptrNodeData(std::make_unique<EndSegmentNodeData>(ss)),
					_vecAccess(vecDefault)
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const CollectiveEndSegmentNodeData& ss,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::COLLECTIVE_END_SEGMENT),
					ptrNodeData(std::make_unique<CollectiveEndSegmentNodeData>(ss)),
					_vecAccess(vecDefault)
				{};

				NodeData(
					const Types::Index index,
					const size_t nrVehicles,
					const CollectiveStartSegmentNodeData& ss,
					const std::vector<Types::AccessType>& vecDefault
				) :
					index(index),
					type(NodeType::COLLECTIVE_START_SEGMENT),
					ptrNodeData(std::make_unique<CollectiveStartSegmentNodeData>(ss)),
					_vecAccess(vecDefault)
				{};

				// COPY CONSTRUCTORS:

				NodeData(NodeData const& other) :
					index(other.index),
					type(other.type),
					ptrNodeData(other.ptrNodeData->clone()),
					acc_dual(other.acc_dual),
					min_charging_dual(other.min_charging_dual),
					max_charging_dual(other.max_charging_dual),
				 	_vecAccess(other._vecAccess),
					_indexFixedVehicle(other._indexFixedVehicle),
					_fixedPathEndTime(other._fixedPathEndTime),
					_fixedPathStartTime(other._fixedPathStartTime)
				{}
				NodeData(NodeData&& other) = default;
				NodeData& operator=(NodeData const& other) {
					ptrNodeData = other.ptrNodeData->clone();
					index = other.index;
					type = other.type;
					acc_dual = other.acc_dual;
					min_charging_dual = other.min_charging_dual;
					max_charging_dual = other.max_charging_dual;
					_vecAccess = other._vecAccess;
					_indexFixedVehicle = other._indexFixedVehicle;
					_fixedPathEndTime = other._fixedPathEndTime;
					_fixedPathStartTime = other._fixedPathStartTime;
					return *this;
				}
				NodeData& operator=(NodeData&& other) = default;

				// DESTRUCTOR:

				~NodeData() = default;

				inline const Types::Index get_startLocationIndex() const { return ptrNodeData->get_startLocationIndex(); };
				inline const Types::Index get_endLocationIndex() const { return ptrNodeData->get_endLocationIndex(); };
				inline const Types::DateTime get_startTime() const { return ptrNodeData->get_startTime(); };
				inline const Types::DateTime get_endTime() const { return ptrNodeData->get_endTime(); };

				inline const double& get_acc_dual() const { return acc_dual; };
				inline const double& get_min_charging_dual() const { return min_charging_dual; };
				inline const double& get_max_charging_dual() const { return max_charging_dual; };
				inline const Types::Index get_index() const { return index; };
				inline const NodeType get_type() const { return type; };

				inline void reset_default_fixings(const std::vector<Types::AccessType>& vecDefault) { _vecAccess = vecDefault; _indexFixedVehicle = Constants::BIG_INDEX; }
				inline void fix_infeasible() { _reset_access(Types::AccessType::NOT_ALLOWED); _indexFixedVehicle = Constants::BIG_INDEX; };
				inline void fix_vehicle(const Types::Index& indexVehicle) { _indexFixedVehicle = indexVehicle; _reset_access(Types::AccessType::NOT_ALLOWED); _set_vehicle_access(indexVehicle, Types::AccessType::ALLOWED);};
				inline void revoke_access(const Types::Index& indexVehicle) { _set_vehicle_access(indexVehicle, Types::AccessType::NOT_ALLOWED);}
				inline const Types::DateTime& get_fixedPathEndTime() const { return _fixedPathEndTime; };
				inline const Types::DateTime& get_fixedPathStartTime() const { return _fixedPathStartTime; };
				inline void set_fixedPathEndTime(const Types::DateTime& timestamp) { _fixedPathEndTime = timestamp; };
				inline void set_fixedPathStartTime(const Types::DateTime& timestamp) { _fixedPathStartTime = timestamp; };

				inline const bool is_feasible() const { return std::any_of(_vecAccess.begin(), _vecAccess.end(), [](Types::AccessType val) { return val == Types::AccessType::ALLOWED; });};
				inline const bool has_access(const Types::Index& indexVehicle) const { return _vecAccess[indexVehicle] == Types::AccessType::ALLOWED; };
				inline const std::vector<Types::AccessType>& get_vecAccess() const { return _vecAccess; };
				inline const Types::Index& get_indexFixedVehicle() const { return _indexFixedVehicle;};

				inline const TripNodeData* castTripNodeData() const { return static_cast<const TripNodeData*>(ptrNodeData.get()); };
				inline const MaintenanceNodeData* castMaintenanceNodeData() const { return static_cast<const MaintenanceNodeData*>(ptrNodeData.get()); };
				inline const StartSegmentNodeData* castStartSegmentNodeData() const { return static_cast<const StartSegmentNodeData*>(ptrNodeData.get()); };
				inline const EndSegmentNodeData* castEndSegmentNodeData() const { return static_cast<const EndSegmentNodeData*>(ptrNodeData.get()); };
			};
		}
	}
}
#endif // EVA_SEGMENTS_SUB_GRAPH_NODES_H