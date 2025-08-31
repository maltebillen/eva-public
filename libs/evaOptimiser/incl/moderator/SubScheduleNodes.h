#ifndef EVA_SUB_SCHEDULE_NODES_H
#define EVA_SUB_SCHEDULE_NODES_H

#include "evaConstants.h"
#include "evaDataHandler.h"

namespace eva
{
	class SubScheduleNodeData
	{
	protected:
		Types::Index _index;
		const ScheduleNodeData& _scheduleNodeData;

	public:
		SubScheduleNodeData(
			const Types::Index& idx,
			const ScheduleNodeData& scheduleNodeData
		) :
			_index(idx),
			_scheduleNodeData(scheduleNodeData)
		{};

		inline const Types::Index& get_index() const { return _index; };
		inline const ScheduleNodeData& get_scheduleNodeData() const { return _scheduleNodeData; };
	};

	struct SubScheduleTripNodeData : SubScheduleNodeData
	{
		using SubScheduleNodeData::SubScheduleNodeData;
		inline const ScheduleNodeData::ScheduleTripNodeData* get_ptrTripNodeData() const { return _scheduleNodeData.castTripNodeData(); };
	};

	struct SubScheduleMaintenanceNodeData : SubScheduleNodeData
	{
		using SubScheduleNodeData::SubScheduleNodeData;
		inline const ScheduleNodeData::ScheduleMaintenanceNodeData* get_ptrMaintenanceNodeData() const { return _scheduleNodeData.castMaintenanceNodeData(); };
	};

	struct SubScheduleStartNodeData : SubScheduleNodeData
	{
		using SubScheduleNodeData::SubScheduleNodeData;
		inline const ScheduleNodeData::ScheduleStartNodeData* get_ptrStartNodeData() const { return _scheduleNodeData.castVehicleStartNodeData(); };
	};

	struct SubSchedulePutOnChargeNodeData : SubScheduleNodeData
	{
		using SubScheduleNodeData::SubScheduleNodeData;
		inline const ScheduleNodeData::SchedulePutOnChargeNodeData* get_ptrPutOnChargeNodeData() const { return _scheduleNodeData.castPutOnChargeNodeData(); };
		inline const Types::Index& get_indexCharger() const { return get_ptrPutOnChargeNodeData()->get_charger().get_index(); };
	};

	struct SubScheduleTakeOffChargeNodeData : SubScheduleNodeData
	{
		using SubScheduleNodeData::SubScheduleNodeData;
		inline const ScheduleNodeData::ScheduleTakeOffChargeNodeData* get_ptrTakeOffChargeNodeData() const { return _scheduleNodeData.castTakeOffChargeNodeData(); };
		inline const Types::Index& get_indexCharger() const { return get_ptrTakeOffChargeNodeData()->get_charger().get_index(); };
	};
}

#endif // !EVA_SUB_SCHEDULE_NODES_H
