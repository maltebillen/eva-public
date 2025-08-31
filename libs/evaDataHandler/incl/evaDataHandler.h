#ifndef EVA_DATA_HANDLER_H
#define EVA_DATA_HANDLER_H

#include "dataInput/dataInput.h"
#include "scheduleGraph/scheduleGraph.h"
#include "dataOutput/dataOutput.h"
#include "dataStats/stats.h"

namespace eva 
{
	struct VehicleSchedule
	{
		Types::Index indexVehicle = Constants::BIG_INDEX;
		Types::Index indexStartLocation = Constants::BIG_INDEX;
		Types::Index indexEndLocation = Constants::BIG_INDEX;
		std::vector<BoostScheduleNode> vecScheduleNodes;

		inline const bool hasScheduleNode(const eva::BoostScheduleNode& scheduleNode) const { return std::find(vecScheduleNodes.begin(), vecScheduleNodes.end(), scheduleNode) != vecScheduleNodes.end(); };
	};

	struct Solution
	{
		// Stats stats;
		Types::DateTime startDecisionHorizon = Constants::MAX_TIMESTAMP;
		Types::DateTime endDecisionHorizon = Constants::MAX_TIMESTAMP;
		std::vector<VehicleSchedule> vecSchedule;
		double objective = Constants::BIG_DOUBLE;

		uint32_t size_unassignedTrips = Constants::BIG_UINTEGER;
		uint32_t size_vehiclesSelected = Constants::BIG_UINTEGER;
	};

	class DataHandler 
	{
		
		DataInput _input; // Stores the data in raw form, just reading out the csv files.
		DataOutput _output; // Stores the functions to write the output.

		Stats _stats; // Stores the stats of the software.
		ScheduleGraph _scheduleGraph; // Stores all schedule activities in graph form.


		void _initialise(const Types::CommandInput& commandInput);

	public:
		// CONSTRUCTORS 

		DataHandler(
			const Types::CommandInput& commandInput
		) 
		{
			_initialise(commandInput);
		}

		// FUNCTION DEFINITIONS
		void storeSolution(const Solution& solution); // In this, add the arcs to the schedule graph, store the schedule path, and store the stats.
		void storeStatsPlanningHorizon(const Stats::PlanningHorizon& ph) { _stats.add_statsPlanningHorizon(ph); };
		void storeStatsPerformanceDetail(const Stats::PerformanceDetail& pd) { _stats.add_statsPerformanceDetail(pd); };
		void storeStatsVehicles();
		void storeStatsChargers();
		void storeStatsSchedules();
		void writeOutputToCsv();

		// GETTERS
		inline const Vehicles& get_vehicles() const { return _input.get_vehicles(); };
		inline const Chargers& get_chargers() const { return _input.get_chargers(); };
		inline const Locations& get_locations() const { return _input.get_locations(); };
		inline const Config& get_config() const { return _input.get_config(); };
		inline const ScheduleGraph& get_scheduleGraph() const { return _scheduleGraph; };

		const ScheduleResourceContainer& get_vehiclePosition(const Vehicle& vehicle) const { return _scheduleGraph.getVehiclePosition(vehicle); };
	};
}

#endif // ! EVA_DATA_HANDLER_H