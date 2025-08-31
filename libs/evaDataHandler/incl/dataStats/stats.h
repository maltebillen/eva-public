#ifndef EVA_DATA_STATS_H
#define EVA_DATA_STATS_H

#include "evaConstants.h"
#include <vector>

namespace eva
{
	class Stats
	{
	public:
		struct PerformanceDetail
		{
			Types::Index indexPlanningHorizon = Constants::BIG_INDEX;
			Types::Index indexBranchingNode = Constants::BIG_INDEX;
			Types::Index iteration = Constants::BIG_INDEX;
			double lb_relaxed = -Constants::BIG_DOUBLE;
			double ub_relaxed = Constants::BIG_DOUBLE;
			bool integerFound = false;
			double lb_integer = 0.0;
			double ub_integer = Constants::BIG_DOUBLE;
			int64_t time_mpSolver = 0;
			int64_t time_ppSolver = 0;
			int64_t time_aux_cg = 0;
			int64_t time_strong_branch = 0;
			uint32_t mp_size_constraints = 0;
			uint32_t mp_size_variables = 0;
			uint32_t columnsAdded = 0;
			int64_t pp_network_construction_ms = 0;
			uint32_t pp_network_size_nodes = 0;
			uint32_t pp_network_size_arcs = 0;
			const char* lazy_constraint_added = NULL;

			const char* branchType = NULL;
			Types::Index indexParentBranchingNode = Constants::BIG_INDEX;
			int64_t time_mpFilterVars = 0;
			int64_t time_ppFilterNodes = 0;
			uint32_t VehicleId = Constants::BIG_UINTEGER;
			uint32_t TripId = Constants::BIG_UINTEGER;
			uint32_t MaintenanceId = Constants::BIG_UINTEGER;
			double fractionalValue = Constants::BIG_DOUBLE;
			double branchValue = Constants::BIG_DOUBLE;

			inline const double gap_relaxed() const { return std::abs(ub_relaxed - lb_relaxed) / std::abs(ub_relaxed); };
			inline const double gap_integer() const { return std::abs(ub_integer - lb_integer) / std::abs(ub_integer); };

		};

		struct PlanningHorizon
		{
			Types::Index indexPlanningHorizon = Constants::BIG_INDEX;
			Types::DateTime startPlanningHorizon = Constants::MAX_TIMESTAMP;
			Types::DateTime endPlanningHorizon = Constants::MAX_TIMESTAMP;
			Types::DateTime endOverlapPlanningHorizon = Constants::MAX_TIMESTAMP;
			double lb_integer = 0.0;
			double ub_integer = Constants::BIG_DOUBLE;
			uint32_t size_schedulesGenerated = Constants::BIG_UINTEGER;
			uint32_t size_unassignedTrips = Constants::BIG_UINTEGER;
			uint32_t size_vehiclesSelected = Constants::BIG_UINTEGER;
			int64_t time_total = 0;
			int64_t time_mpSolver = 0;
			int64_t time_ppSolver = 0;
			int64_t pp_network_construction_ms = 0;
			int64_t pp_nr_segments = 0;
			int64_t time_mpFilterVars = 0;
			int64_t time_ppFilterNodes = 0;
			uint32_t branchingTree_depth = 0;
			uint32_t branchingTree_size = 0;
			std::string algorithm = "";

			inline const double gap_integer() const { return std::abs(ub_integer - lb_integer) / std::abs(ub_integer); };
		};

		struct Schedule
		{
			double cost_vehicles = 0.0;
			double cost_deadlegs = 0.0;
			double cost_maintenance = 0.0;
			double cost_unassignedTrips = 0.0;
			double cost_total = 0.0;
		};

		struct Vehicles
		{
			uint32_t vehicleId = Constants::BIG_UINTEGER;
			bool inRotation = false;
			double cost_vehicle = Constants::BIG_DOUBLE;
			double cost_deadlegs = 0.0;
			double cost_maintenance = 0.0;
			uint32_t km_deadlegs = 0;
			double km_avg_distance_maintenance = 0.0;
			double km_std_distance_maintenance = 0.0;
			Types::BatteryCharge lb_soc = Constants::BIG_BATTERYCHARGE;
			Types::BatteryCharge ub_soc = -Constants::BIG_BATTERYCHARGE;
			int64_t seconds_idle = 0;
			int64_t seconds_productive = 0;
			int64_t seconds_maintenance = 0;
			int64_t seconds_charging = 0;
		};

		struct Chargers
		{
			uint32_t chargerId = Constants::BIG_UINTEGER;
			Types::DateTime timestamp = Constants::MAX_TIMESTAMP;
			uint32_t size_vehiclesAtCharger = 0;
			uint32_t chargerCapacity = Constants::BIG_UINTEGER;
		};

	private:
		std::vector<Stats::PerformanceDetail> _vecStatsPerformanceDetail;
		std::vector<Stats::PlanningHorizon> _vecStatsPlanningHorizon;
		std::vector<Stats::Schedule> _vecStatsSchedule;
		std::vector<Stats::Vehicles> _vecStatsVehicles;
		std::vector<Stats::Chargers> _vecStatsChargers;

	public:
		Stats() {};

		// Functions to add a row to the stats:
		inline void add_statsPerformanceDetail(const Stats::PerformanceDetail& pd) { _vecStatsPerformanceDetail.push_back(pd); };
		inline void add_statsPlanningHorizon(const Stats::PlanningHorizon& ph) { _vecStatsPlanningHorizon.push_back(ph); };
		inline void add_statsSchedule(const Stats::Schedule& sch) { _vecStatsSchedule.push_back(sch); };
		inline void add_statsVehicles(const Stats::Vehicles& ve) { _vecStatsVehicles.push_back(ve); };
		inline void add_statsChargers(const Stats::Chargers& ch) { _vecStatsChargers.push_back(ch); };

		// GETTERS:
		inline const std::vector<Stats::PerformanceDetail>& get_vecStatsPerformanceDetail() const {return _vecStatsPerformanceDetail; };
		inline const std::vector<Stats::PlanningHorizon>& get_vecStatsPlanningHorizon() const {return _vecStatsPlanningHorizon; };
		inline const std::vector<Stats::Schedule>& get_vecStatsSchedule() const {return _vecStatsSchedule; };
		inline const std::vector<Stats::Vehicles>& get_vecStatsVehicles() const {return _vecStatsVehicles; };
		inline const std::vector<Stats::Chargers>& get_vecStatsChargers() const {return _vecStatsChargers; };
	};
}
#endif // !EVA_DATA_STATS_H
