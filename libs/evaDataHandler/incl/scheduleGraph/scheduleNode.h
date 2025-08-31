#ifndef EVA_SCHEDULE_GRAPH_NODES_H
#define EVA_SCHEDULE_GRAPH_NODES_H

#include "evaConstants.h"
#include "../dataInput/dataInput.h"
#include <memory>

namespace eva
{
	enum class ScheduleNodeType : uint8_t
	{
		TRIP = 1,
		MAINTENANCE,
		START_SCHEDULE,
		PUT_ON_CHARGE,
		TAKE_OFF_CHARGE,
		DEADLEG,
		CHARGING,
		OUT_OF_ROTATION,
		UNDEFINED
	};

	const std::map<ScheduleNodeType, const char*> ScheduleNodeTypeMap
	{
		{ScheduleNodeType::TRIP, "TRIP"},
		{ScheduleNodeType::MAINTENANCE, "MAINTENANCE"},
		{ScheduleNodeType::START_SCHEDULE, "START_SCHEDULE"},
		{ScheduleNodeType::PUT_ON_CHARGE, "PUT_ON_CHARGE"},
		{ScheduleNodeType::TAKE_OFF_CHARGE, "TAKE_OFF_CHARGE"},
		{ScheduleNodeType::DEADLEG, "DEADLEG"},
		{ScheduleNodeType::CHARGING, "CHARGING"},
		{ScheduleNodeType::OUT_OF_ROTATION, "OUT_OF_ROTATION"},
		{ScheduleNodeType::UNDEFINED, "UNDEFINED"}
	};


	struct ScheduleNodeData
	{
		class ScheduleBaseNodeData
		{
		public:
			// INLINE FUNCTIONS 
			inline std::unique_ptr<ScheduleBaseNodeData> clone() const { return std::unique_ptr<ScheduleBaseNodeData>(deepClone()); };

			virtual inline const Location& get_startLocation() const = 0;
			virtual inline const Location& get_endLocation() const = 0;
			virtual inline const uint32_t get_distance() const = 0;
			virtual inline const uint32_t get_duration() const = 0;
			virtual inline const Types::DateTime get_startTime() const = 0;
			virtual inline const Types::DateTime get_endTime() const = 0;
		
			virtual ~ScheduleBaseNodeData() {};


		protected:
			virtual inline ScheduleBaseNodeData* deepClone() const = 0;
		};

		class ScheduleTripNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Trip& _trip;
			const uint32_t _distance = _trip.get_startLocation().get_distanceToLocation(_trip.get_endLocation());
			const uint32_t _duration = Helper::diffDateTime(_trip.get_startTime(), _trip.get_endTime());

		public:
			// CONSTRUCTORS

			ScheduleTripNodeData(
				const Trip& trip
			) :
				_trip(trip)
			{}

			~ScheduleTripNodeData() = default;

			// GETTERS

			inline const Trip& get_trip() const { return _trip; };

			virtual inline const Location& get_startLocation() const override { return _trip.get_startLocation(); };
			virtual inline const Location& get_endLocation() const override { return _trip.get_endLocation(); };
			virtual inline const uint32_t get_distance() const override { return _distance; };
			virtual inline const uint32_t get_duration() const override { return _duration; };
			virtual inline const Types::DateTime get_startTime() const override { return _trip.get_startTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _trip.get_endTime(); };

		protected:
			virtual ScheduleTripNodeData* deepClone() const override { return new ScheduleTripNodeData(*this); };
		};

		class ScheduleDeadlegNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Location& _startLocation;
			const Location& _endLocation;
			Types::DateTime _startTime;

		public:
			// CONSTRUCTORS

			ScheduleDeadlegNodeData(
				const Location& startLocation,
				const Location& endLocation,
				const Types::DateTime& startTime
			) :
				_startLocation(startLocation),
				_endLocation(endLocation),
				_startTime(startTime)
			{}

			~ScheduleDeadlegNodeData() = default;

			// GETTERS

			virtual inline const Location& get_startLocation() const override { return _startLocation; };
			virtual inline const Location& get_endLocation() const override { return _endLocation; };
			virtual inline const uint32_t get_distance() const override { return _startLocation.get_distanceToLocation(_endLocation);; };
			virtual inline const uint32_t get_duration() const override { return _startLocation.get_durationToLocation(_endLocation); };
			virtual inline const Types::DateTime get_startTime() const override { return _startTime; };
			virtual inline const Types::DateTime get_endTime() const override { return _startTime + get_duration(); };

		protected:
			virtual ScheduleDeadlegNodeData* deepClone() const override { return new ScheduleDeadlegNodeData(*this); };
		};

		class ScheduleOutOfRotationNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Location& _parkingLocation;
			Types::DateTime _time;

		public:
			// CONSTRUCTORS

			ScheduleOutOfRotationNodeData(
				const Location& parkingLocation,
				const Types::DateTime& time
			) :
				_parkingLocation(parkingLocation),
				_time(time)
			{
			}

			~ScheduleOutOfRotationNodeData() = default;
			// GETTERS

			virtual inline const Location& get_startLocation() const override { return _parkingLocation; };
			virtual inline const Location& get_endLocation() const override { return _parkingLocation; };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return _time; };
			virtual inline const Types::DateTime get_endTime() const override { return _time; };

		protected:
			virtual ScheduleOutOfRotationNodeData* deepClone() const override { return new ScheduleOutOfRotationNodeData(*this); };
		};

		class ScheduleMaintenanceNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Maintenance& _maintenance;
			const uint32_t _duration = Helper::diffDateTime(_maintenance.get_startTime(), _maintenance.get_endTime());

		public:
			// CONSTRUCTORS

			ScheduleMaintenanceNodeData(
				const Maintenance& maintenance

			) :
				_maintenance(maintenance)
			{}

			~ScheduleMaintenanceNodeData() = default;
			// GETTERS

			inline const Maintenance& get_maintenance() const { return _maintenance; };

			virtual inline const Location& get_startLocation() const override { return _maintenance.get_maintenanceLocation(); };
			virtual inline const Location& get_endLocation() const override { return _maintenance.get_maintenanceLocation(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return _duration; };
			virtual inline const Types::DateTime get_startTime() const override { return _maintenance.get_startTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _maintenance.get_endTime(); };

		protected:
			virtual ScheduleMaintenanceNodeData* deepClone() const override { return new ScheduleMaintenanceNodeData(*this); };
		};

		class ScheduleStartNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Vehicle& _vehicle;

		public:
			// CONSTRUCTORS

			ScheduleStartNodeData(
				const Vehicle& vehicle

			) :
				_vehicle(vehicle)
			{}

			~ScheduleStartNodeData() = default;

			// GETTERS

			inline const Vehicle& get_vehicle() const { return _vehicle; };

			virtual inline const Location& get_startLocation() const override { return _vehicle.get_initialCharger().get_location(); };
			virtual inline const Location& get_endLocation() const override { return _vehicle.get_initialCharger().get_location(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return 0; };
			virtual inline const Types::DateTime get_startTime() const override { return _vehicle.get_initialStartTime(); };
			virtual inline const Types::DateTime get_endTime() const override { return _vehicle.get_initialStartTime(); };

		protected:
			virtual ScheduleStartNodeData* deepClone() const override { return new ScheduleStartNodeData(*this); };
		};

		class ScheduleChargingNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Charger& _charger;
			const Types::DateTime _startTime;
			const Types::DateTime _endTime;
			const uint32_t _duration;

		public:
			// CONSTRUCTORS

			ScheduleChargingNodeData(
				const Charger& charger,
				const Types::DateTime& startTime,
				const Types::DateTime& endTime
			) :
				_charger(charger),
				_startTime(startTime),
				_endTime(endTime),
				_duration(Helper::diffDateTime(startTime, endTime))
			{};

			~ScheduleChargingNodeData() = default;
			// GETTERS

			inline const Charger& get_charger() const { return _charger; };

			inline Types::BatteryCharge get_charge(const Types::BatteryCharge cur_charge, const Vehicle& vehicle) const { return std::min(vehicle.get_batteryMaxKWh() - cur_charge, Types::BatteryCharge(vehicle.get_chargingSpeedKwS(_charger) * _duration)); }

			virtual inline const Location& get_startLocation() const override { return _charger.get_location(); };
			virtual inline const Location& get_endLocation() const override { return _charger.get_location(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return _duration; };
			virtual inline const Types::DateTime get_startTime() const override { return _startTime; };
			virtual inline const Types::DateTime get_endTime() const override { return _endTime; };

		protected:
			virtual ScheduleChargingNodeData* deepClone() const override { return new ScheduleChargingNodeData(*this); };
		};

		class SchedulePutOnChargeNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Charger& _charger;
			const Types::DateTime _startTime;
			const Types::DateTime _endTime;
			const uint32_t _duration;

		public:
			// CONSTRUCTORS

			SchedulePutOnChargeNodeData(
				const Charger& charger,
				const Types::DateTime& startTime,
				const Types::DateTime& endTime
			) :
				_charger(charger),
				_startTime(startTime),
				_endTime(endTime),
				_duration(Helper::diffDateTime(startTime, endTime))
			{};

			~SchedulePutOnChargeNodeData() = default;
			// GETTERS

			inline const Charger& get_charger() const { return _charger; };

			virtual inline const Location& get_startLocation() const override { return _charger.get_location(); };
			virtual inline const Location& get_endLocation() const override { return _charger.get_location(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return _duration; };
			virtual inline const Types::DateTime get_startTime() const override { return _startTime; };
			virtual inline const Types::DateTime get_endTime() const override { return _endTime; };

		protected:
			virtual SchedulePutOnChargeNodeData* deepClone() const override { return new SchedulePutOnChargeNodeData(*this); };
		};

		class ScheduleTakeOffChargeNodeData : public ScheduleBaseNodeData
		{
			// ATTRIBUTES

			const Charger& _charger;
			const Types::DateTime _startTime;
			const Types::DateTime _endTime;
			const uint32_t _duration;

		public:
			// CONSTRUCTORS

			ScheduleTakeOffChargeNodeData(
				const Charger& charger,
				const Types::DateTime& startTime,
				const Types::DateTime& endTime
			) :
				_charger(charger),
				_startTime(startTime),
				_endTime(endTime),
				_duration(Helper::diffDateTime(startTime, endTime))
			{};

			~ScheduleTakeOffChargeNodeData() = default;
			// GETTERS

			inline const Charger& get_charger() const { return _charger; };

			virtual inline const Location& get_startLocation() const override { return _charger.get_location(); };
			virtual inline const Location& get_endLocation() const override { return _charger.get_location(); };
			virtual inline const uint32_t get_distance() const override { return 0; };
			virtual inline const uint32_t get_duration() const override { return _duration; };
			virtual inline const Types::DateTime get_startTime() const override { return _startTime; };
			virtual inline const Types::DateTime get_endTime() const override { return _endTime; };

		protected:
			virtual ScheduleTakeOffChargeNodeData* deepClone() const override { return new ScheduleTakeOffChargeNodeData(*this); };
		};

		// ATTRIBUTES:

		std::unique_ptr<ScheduleBaseNodeData> ptrNodeData;
		Types::Index index = Constants::BIG_INDEX;
		ScheduleNodeType type = ScheduleNodeType::UNDEFINED;

		// CONSTRUCTORS

		ScheduleNodeData() :
			index(Constants::BIG_INDEX),
			type(ScheduleNodeType::UNDEFINED),
			ptrNodeData(nullptr)
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleTripNodeData& tnd
		) :
			index(index),
			type(ScheduleNodeType::TRIP),
			ptrNodeData(std::make_unique<ScheduleTripNodeData>(tnd))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleMaintenanceNodeData& vand
		) :
			index(index),
			type(ScheduleNodeType::MAINTENANCE),
			ptrNodeData(std::make_unique<ScheduleMaintenanceNodeData>(vand))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleDeadlegNodeData& dnd
		) :
			index(index),
			type(ScheduleNodeType::DEADLEG),
			ptrNodeData(std::make_unique<ScheduleDeadlegNodeData>(dnd))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleOutOfRotationNodeData& oor
		) :
			index(index),
			type(ScheduleNodeType::OUT_OF_ROTATION),
			ptrNodeData(std::make_unique<ScheduleOutOfRotationNodeData>(oor))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleStartNodeData& vsnd
		) :
			index(index),
			type(ScheduleNodeType::START_SCHEDULE),
			ptrNodeData(std::make_unique<ScheduleStartNodeData>(vsnd))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleChargingNodeData& cnd
		) :
			index(index),
			type(ScheduleNodeType::CHARGING),
			ptrNodeData(std::make_unique<ScheduleChargingNodeData>(cnd))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const SchedulePutOnChargeNodeData& pocnd
		) :
			index(index),
			type(ScheduleNodeType::PUT_ON_CHARGE),
			ptrNodeData(std::make_unique<SchedulePutOnChargeNodeData>(pocnd))
		{}

		ScheduleNodeData(
			const Types::Index index,
			const ScheduleTakeOffChargeNodeData& tocnd
		) :
			index(index),
			type(ScheduleNodeType::TAKE_OFF_CHARGE),
			ptrNodeData(std::make_unique<ScheduleTakeOffChargeNodeData>(tocnd))
		{}

		// COPY CONSTRUCTORS:

		ScheduleNodeData(ScheduleNodeData const& other) :
			index(other.index),
			type(other.type),
			ptrNodeData(other.ptrNodeData->clone())
		{}
		ScheduleNodeData(ScheduleNodeData&& other) = default;
		ScheduleNodeData& operator=(ScheduleNodeData const& other) {
			ptrNodeData = other.ptrNodeData->clone();
			index = other.index;
			type = other.type;
			return *this;
		}
		ScheduleNodeData& operator=(ScheduleNodeData&& other) = default;

		// DESTRUCTOR:

		~ScheduleNodeData() = default;

		inline const Types::Index& get_index() const { return index; };
		inline const ScheduleNodeType& get_type() const { return type; };
		inline const Location& get_startLocation() const { return ptrNodeData->get_startLocation(); };
		inline const Location& get_endLocation() const { return ptrNodeData->get_endLocation(); };
		inline const uint32_t get_distance() const { return ptrNodeData->get_distance(); };
		inline const uint32_t get_duration() const { return ptrNodeData->get_duration(); };
		inline const Types::DateTime get_startTime() const { return ptrNodeData->get_startTime(); };
		inline const Types::DateTime get_endTime() const { return ptrNodeData->get_endTime(); };

		inline const ScheduleTripNodeData* castTripNodeData() const { return static_cast<const ScheduleTripNodeData*>(ptrNodeData.get()); };
		inline const ScheduleMaintenanceNodeData* castMaintenanceNodeData() const { return static_cast<const ScheduleMaintenanceNodeData*>(ptrNodeData.get()); };
		inline const ScheduleStartNodeData* castVehicleStartNodeData() const { return static_cast<const ScheduleStartNodeData*>(ptrNodeData.get()); };
		inline const SchedulePutOnChargeNodeData* castPutOnChargeNodeData() const { return static_cast<const SchedulePutOnChargeNodeData*>(ptrNodeData.get()); };
		inline const ScheduleTakeOffChargeNodeData* castTakeOffChargeNodeData() const { return static_cast<const ScheduleTakeOffChargeNodeData*>(ptrNodeData.get()); };
		inline const ScheduleChargingNodeData* castChargingNodeData() const { return static_cast<const ScheduleChargingNodeData*>(ptrNodeData.get()); };
		inline const ScheduleDeadlegNodeData* castDeadlegNodeData() const { return static_cast<const ScheduleDeadlegNodeData*>(ptrNodeData.get()); };


	};

};


#endif // ! EVA_SCHEDULE_GRAPH_NODES_H