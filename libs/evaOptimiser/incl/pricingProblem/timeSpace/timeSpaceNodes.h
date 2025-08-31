#ifndef EVA_TIME_SPACE_NODES_H
#define EVA_TIME_SPACE_NODES_H

#include "moderator/SubScheduleNodes.h"

#include "boost/dynamic_bitset.hpp"

namespace eva
{
	namespace tsn
	{
		enum class TimeSpaceNodeType : uint8_t
		{
			TRIP = 1,
			MAINTENANCE,
			CHARGING,
			START_SCHEDULE,
			CHARGER_END_SCHEDULE,
			COLLECTIVE_END_SCHEDULE,
			UNDEFINED
		};

		enum class TimeSpaceNodeAccessType : uint8_t
		{
			NOT_ALLOWED = 0,
			ALLOWED = 1
		};

		class TimeSpaceBaseNodeData
		{
		public:
			// INLINE FUNCTIONS 
			inline std::unique_ptr<TimeSpaceBaseNodeData> clone() const { return std::unique_ptr<TimeSpaceBaseNodeData>(deepClone()); };

			virtual inline const Types::Index get_startLocationIndex() const = 0;
			virtual inline const Types::Index get_endLocationIndex() const = 0;
			virtual inline const uint32_t get_distance() const = 0;
			virtual inline const Types::DateTime get_startTime() const = 0;
			virtual inline const Types::DateTime get_endTime() const = 0;
			virtual inline const Types::Index get_scheduleNodeIndex() const = 0;

			virtual ~TimeSpaceBaseNodeData() {};

		protected:
			virtual inline TimeSpaceBaseNodeData* deepClone() const = 0;
		};

		class TimeSpaceTripNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

			const SubScheduleTripNodeData& _subTripNodeData;

		public:
			// CONSTRUCTORS

			TimeSpaceTripNodeData(
				const SubScheduleTripNodeData& subTripNodeData
			) :
				_subTripNodeData(subTripNodeData)
			{}

			// GETTERS

			inline const SubScheduleTripNodeData& get_subTripNodeData() const { return _subTripNodeData; };

			virtual inline const Types::Index get_startLocationIndex() const override { return _subTripNodeData.get_scheduleNodeData().get_startLocation().get_index(); };
			virtual inline const Types::Index get_endLocationIndex() const override { return _subTripNodeData.get_scheduleNodeData().get_endLocation().get_index(); };
			virtual inline const uint32_t get_distance() const override { return _subTripNodeData.get_scheduleNodeData().get_distance(); };
			virtual inline const Types::DateTime get_startTime() const override { return _subTripNodeData.get_scheduleNodeData().get_startTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _subTripNodeData.get_scheduleNodeData().get_endTime(); };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return _subTripNodeData.get_scheduleNodeData().get_index(); };

		protected:
			virtual TimeSpaceTripNodeData* deepClone() const override { return new TimeSpaceTripNodeData(*this); };
		};

		class TimeSpaceMaintenanceNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

			const SubScheduleMaintenanceNodeData& _subMaintenanceNodeData;

		public:
			// CONSTRUCTORS

			TimeSpaceMaintenanceNodeData(
				const SubScheduleMaintenanceNodeData& subMaintenanceNodeData
			) :
				_subMaintenanceNodeData(subMaintenanceNodeData)
			{}

			// GETTERS

			inline const SubScheduleMaintenanceNodeData& get_subMaintenanceNodeData() const { return _subMaintenanceNodeData; };

			virtual inline const Types::Index get_startLocationIndex() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_startLocation().get_index(); };
			virtual inline const Types::Index get_endLocationIndex() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_endLocation().get_index(); };
			virtual inline const uint32_t get_distance() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_distance(); };
			virtual inline const Types::DateTime get_startTime() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_startTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_endTime(); };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return _subMaintenanceNodeData.get_scheduleNodeData().get_index(); };

		protected:
			virtual TimeSpaceMaintenanceNodeData* deepClone() const override { return new TimeSpaceMaintenanceNodeData(*this); };
		};

		class TimeSpaceChargingNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

			const Charger& _charger;
			const SubSchedulePutOnChargeNodeData& _earliestPutOnChargeNode;
			const SubScheduleTakeOffChargeNodeData& _latestTakeOffChargeNode;

		public:
			// CONSTRUCTORS

			TimeSpaceChargingNodeData(
				const Charger& charger,
				const SubSchedulePutOnChargeNodeData& earliestPutOnChargeNode,
				const SubScheduleTakeOffChargeNodeData& latestTakeOffChargeNode
			) :
				_charger(charger),
				_earliestPutOnChargeNode(earliestPutOnChargeNode),
				_latestTakeOffChargeNode(latestTakeOffChargeNode)
			{}

			// GETTERS

			inline const Charger& get_charger() const { return _charger; };
			inline const Types::BatteryCharge get_charge(const Types::BatteryCharge cur_charge, const Vehicle& vehicle, const uint32_t& duration) const { return std::min(vehicle.get_batteryMaxKWh() - cur_charge, Types::BatteryCharge(vehicle.get_chargingSpeedKwS(_charger) * duration)); }


			inline const SubSchedulePutOnChargeNodeData& get_earliestPutOnChargeNode() const { return _earliestPutOnChargeNode; };
			inline const SubScheduleTakeOffChargeNodeData& get_latestTakeOffChargeNode() const { return _latestTakeOffChargeNode; };

			virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
			virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return _earliestPutOnChargeNode.get_scheduleNodeData().get_startTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _latestTakeOffChargeNode.get_scheduleNodeData().get_endTime(); };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return Constants::BIG_INDEX; };
		protected:
			virtual TimeSpaceChargingNodeData* deepClone() const override { return new TimeSpaceChargingNodeData(*this); };
		};


		class TimeSpaceStartScheduleNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

			const Location& _location;
			const Types::DateTime _time;
			const Vehicle& _vehicle;

		public:
			// CONSTRUCTORS

			TimeSpaceStartScheduleNodeData(
				const Location& location,
				const Types::DateTime time,
				const Vehicle& vehicle
			) :
				_location(location),
				_time(time),
				_vehicle(vehicle)
			{}

			// 

			// GETTERS
			inline const Vehicle& get_vehicle() const { return _vehicle; };

			virtual inline const Types::Index get_startLocationIndex() const override { return _location.get_index(); };
			virtual inline const Types::Index get_endLocationIndex() const override { return _location.get_index(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return _time; };
			virtual inline const Types::DateTime get_endTime() const override { return _time; };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return Constants::BIG_INDEX; };

		protected:
			virtual TimeSpaceStartScheduleNodeData* deepClone() const override { return new TimeSpaceStartScheduleNodeData(*this); };
		};

		class TimeSpaceChargerEndScheduleNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

			const Charger& _charger;

		public:
			// CONSTRUCTORS

			TimeSpaceChargerEndScheduleNodeData(
				const Charger& charger
			) :
				_charger(charger)
			{}

			// GETTERS

			virtual inline const Types::Index get_startLocationIndex() const override { return _charger.get_location().get_index(); };
			virtual inline const Types::Index get_endLocationIndex() const override { return _charger.get_location().get_index(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
			virtual inline const Types::DateTime get_endTime() const override { return  Constants::MAX_TIMESTAMP; };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return Constants::BIG_INDEX; };

		protected:
			virtual TimeSpaceChargerEndScheduleNodeData* deepClone() const override { return new TimeSpaceChargerEndScheduleNodeData(*this); };
		};


		class TimeSpaceCollectiveEndScheduleNodeData : public TimeSpaceBaseNodeData
		{
			// ATTRIBUTES

		public:
			// CONSTRUCTORS

			TimeSpaceCollectiveEndScheduleNodeData() {};

			// GETTERS

			virtual inline const Types::Index get_startLocationIndex() const override { return Constants::BIG_INDEX; };
			virtual inline const Types::Index get_endLocationIndex() const override { return Constants::BIG_INDEX; };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return Constants::MAX_TIMESTAMP; };
			virtual inline const Types::DateTime get_endTime() const override { return Constants::MAX_TIMESTAMP; };
			virtual inline const Types::Index get_scheduleNodeIndex() const override { return Constants::BIG_INDEX; };
		protected:
			virtual TimeSpaceCollectiveEndScheduleNodeData* deepClone() const override { return new TimeSpaceCollectiveEndScheduleNodeData(*this); };
		};

		class TimeSpaceNodeData
		{	
			Types::Index _index_fixed_vehicle = Constants::BIG_INDEX;
			Types::DateTime _max_rc_start_time = 0;

		public:
			// ATTRIBUTES:

			std::unique_ptr<TimeSpaceBaseNodeData> ptrNodeData;
			Types::Index index;
			TimeSpaceNodeType type;
			std::vector<TimeSpaceNodeAccessType> access;

			// CONSTRUCTORS

			TimeSpaceNodeData() :
				index(Constants::BIG_INDEX),
				type(TimeSpaceNodeType::UNDEFINED)
			{};

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceTripNodeData& tnd
			) :
				index(index),
				type(TimeSpaceNodeType::TRIP),
				ptrNodeData(std::make_unique<TimeSpaceTripNodeData>(tnd))
			{};

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceMaintenanceNodeData& mnd
			) :
				index(index),
				type(TimeSpaceNodeType::MAINTENANCE),
				ptrNodeData(std::make_unique<TimeSpaceMaintenanceNodeData>(mnd))
			{}

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceChargingNodeData& cnd
			) :
				index(index),
				type(TimeSpaceNodeType::CHARGING),
				ptrNodeData(std::make_unique<TimeSpaceChargingNodeData>(cnd))
			{}

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceStartScheduleNodeData& vsnd
			) :
				index(index),
				type(TimeSpaceNodeType::START_SCHEDULE),
				ptrNodeData(std::make_unique<TimeSpaceStartScheduleNodeData>(vsnd))
			{}

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceChargerEndScheduleNodeData& vend
			) :
				index(index),
				type(TimeSpaceNodeType::CHARGER_END_SCHEDULE),
				ptrNodeData(std::make_unique<TimeSpaceChargerEndScheduleNodeData>(vend))
			{}

			TimeSpaceNodeData(
				const Types::Index index,
				const TimeSpaceCollectiveEndScheduleNodeData& vend
			) :
				index(index),
				type(TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE),
				ptrNodeData(std::make_unique<TimeSpaceCollectiveEndScheduleNodeData>(vend))
			{}

			// COPY CONSTRUCTORS:

			TimeSpaceNodeData(TimeSpaceNodeData const& other) :
				index(other.index),
				type(other.type),
				ptrNodeData(other.ptrNodeData->clone()),
				access(other.access),
				_index_fixed_vehicle(other._index_fixed_vehicle),
				_max_rc_start_time(other._max_rc_start_time)
			{}
			TimeSpaceNodeData(TimeSpaceNodeData&& other) = default;
			TimeSpaceNodeData& operator=(TimeSpaceNodeData const& other) {
				ptrNodeData = other.ptrNodeData->clone();
				index = other.index;
				type = other.type;
				access = other.access;
				_index_fixed_vehicle = other._index_fixed_vehicle;
				_max_rc_start_time = other._max_rc_start_time;
				return *this;
			}
			TimeSpaceNodeData& operator=(TimeSpaceNodeData&& other) = default;

			// DESTRUCTOR:

			~TimeSpaceNodeData() = default;

			inline void init_access(const size_t numberVehicles, TimeSpaceNodeAccessType val_access) { access.resize(numberVehicles, val_access); }
			inline void set_access(const Types::Index& indexVehicle, TimeSpaceNodeAccessType val_access) { access[indexVehicle] = val_access; };
			inline void set_access(const Types::Index& indexVehicle, bool val_access) { access[indexVehicle] = static_cast<TimeSpaceNodeAccessType>(val_access); };
			inline void reset_access(TimeSpaceNodeAccessType val_access) { std::fill(access.begin(), access.end(), val_access); };
			inline void reset_access(bool val_access) { std::fill(access.begin(), access.end(), static_cast<TimeSpaceNodeAccessType>(val_access)); };
			inline const bool is_accessible_all_vehicles() const { return std::find(access.begin(), access.end(), TimeSpaceNodeAccessType::NOT_ALLOWED) == access.end(); };

			inline void reset_max_rc_start_time() { _index_fixed_vehicle = Constants::BIG_INDEX; _max_rc_start_time = 0;};
			inline void set_max_rc_start_time(const Types::DateTime& max_rc_start_time, const Types::Index& indexVehicle) { _index_fixed_vehicle = indexVehicle; _max_rc_start_time = max_rc_start_time;};
			inline const bool get_has_fixed_activity(const Types::Index& indexVehicle) const { return _index_fixed_vehicle == indexVehicle;}
			inline const Types::DateTime& get_max_rc_start_time() const { return _max_rc_start_time;};

			inline const Types::Index get_scheduleNodeIndex() const { return ptrNodeData->get_scheduleNodeIndex(); };

			inline const Types::Index get_startLocationIndex() const { return ptrNodeData->get_startLocationIndex(); };
			inline const Types::Index get_endLocationIndex() const { return ptrNodeData->get_endLocationIndex(); };
			inline const uint32_t get_distance() const { return ptrNodeData->get_distance(); };
			inline const Types::DateTime get_startTime() const { return ptrNodeData->get_startTime(); };
			inline const Types::DateTime get_endTime() const { return ptrNodeData->get_endTime(); };

			inline const Types::Index& get_index() const { return index; };
			inline const TimeSpaceNodeType get_type() const { return type; };
			inline const bool has_access(const Types::Index& indexVehicle) const { return access[indexVehicle] == TimeSpaceNodeAccessType::ALLOWED; };


			// TODO: HERE ADD DEBUG / RELEASE VERSION OF CASTING THAT CHECK THE TYPE:
			inline const TimeSpaceTripNodeData* castTripNodeData() const { return static_cast<const TimeSpaceTripNodeData*>(ptrNodeData.get()); };
			inline const TimeSpaceMaintenanceNodeData* castMaintenanceNodeData() const { return static_cast<const TimeSpaceMaintenanceNodeData*>(ptrNodeData.get()); };
			inline const TimeSpaceChargingNodeData* castChargingNodeData() const { return static_cast<const TimeSpaceChargingNodeData*>(ptrNodeData.get()); };
			inline const TimeSpaceStartScheduleNodeData* castVehicleStartNodeData() const { return static_cast<const TimeSpaceStartScheduleNodeData*>(ptrNodeData.get()); };
			inline const TimeSpaceCollectiveEndScheduleNodeData* castCollectiveVehicleEndNodeData() const { return static_cast<const TimeSpaceCollectiveEndScheduleNodeData*>(ptrNodeData.get()); };
			inline const TimeSpaceChargerEndScheduleNodeData* castChargerVehicleEndNodeData() const { return static_cast<const TimeSpaceChargerEndScheduleNodeData*>(ptrNodeData.get()); };
		};
	}
}

#endif // !EVA_TIME_SPACE_NODES_H