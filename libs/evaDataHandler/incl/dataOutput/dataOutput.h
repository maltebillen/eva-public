#ifndef EVA_DATA_OUTPUT_H
#define EVA_DATA_OUTPUT_H

#include "../dataInput/dataInput.h"
#include "../scheduleGraph/scheduleGraph.h"
#include "../dataStats/stats.h"

namespace eva
{
	struct DataOutput
	{
		void writeVehicleScheduleToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph);
		void writeUnassignedTripsToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph);
		void writeUnassignedMaintenancesToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph);

		void writeStatsPerformanceDetail(const DataInput& input, const std::vector<Stats::PerformanceDetail>& vecStatsPerformance);
		void writeStatsPlanningHorizon(const DataInput& input, const std::vector<Stats::PlanningHorizon>& vecStatsPlanningHorizon);
		void writeStatsSchedule(const DataInput& input, const std::vector<Stats::Schedule>& vecStatsSchedule);
		void writeStatsVehicles(const DataInput& input, const std::vector<Stats::Vehicles>& vecStatsVehicles);
		void writeStatsChargers(const DataInput& input, const std::vector<Stats::Chargers>& vecStatsChargers);
	};
}




#endif // ! EVA_DATA_OUTPUT_H