#include "incl/dataOutput/dataOutput.h"

#include <fstream>
#include <iostream>

void eva::DataOutput::writeVehicleScheduleToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph)
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "VS_Output.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "VehicleID,StartLocationID,EndLocationID,StartTime,EndTime,BatteryChange,SoC,DistanceLastMaintenance,Type" << "\n";

	for (const Vehicle& vehicle : input.get_vehicles().get_vec())
	{
		ScheduleResourceContainer old_cont(
			vehicle.get_odometerReading(),
			vehicle.get_odometerLastMaintenance(),
			vehicle.get_initialSOC()
		);
		ScheduleResourceContainer new_cont = old_cont;

		// Start with the vehicle's start node:
		fout << vehicle.get_id() << ","; // VehicleID
		fout << vehicle.get_initialCharger().get_location().get_id() << ","; // StartLocationID
		fout << vehicle.get_initialCharger().get_location().get_id() << ","; // EndLocationID
		fout << Helper::DateTimeToString(vehicle.get_initialStartTime()) << ","; // StartTime
		fout << Helper::DateTimeToString(vehicle.get_initialStartTime()) << ","; // EndTime
		fout << 0 << ","; // BatteryChange
		fout << vehicle.get_initialSOC() << ","; // SoC
		fout << (vehicle.get_odometerReading() - vehicle.get_odometerLastMaintenance()) << ","; // DistanceLastMaintenance
		fout << ScheduleNodeTypeMap.find(ScheduleNodeType::START_SCHEDULE)->second << "\n"; //Type

		// Store the Vehicle Schedule:
		for (const BoostScheduleArc& arc : scheduleGraph.get_vecSchedulePath(vehicle.get_index()))
		{
			scheduleGraph.processArc(input, vehicle, new_cont, old_cont, arc);

			fout << vehicle.get_id() << ","; // VehicleID
			fout << scheduleGraph.get_targetNodeData(arc).get_startLocation().get_id() << ","; // StartLocationID
			fout << scheduleGraph.get_targetNodeData(arc).get_endLocation().get_id() << ","; // EndLocationID
			fout << Helper::DateTimeToString(scheduleGraph.get_targetNodeData(arc).get_startTime()) << ","; // StartTime
			fout << Helper::DateTimeToString(scheduleGraph.get_targetNodeData(arc).get_endTime()) << ","; // EndTime
			fout << new_cont.soc - old_cont.soc << ","; // BatteryChange
			fout << new_cont.soc << ","; // SoC
			fout << new_cont.distanceLastMaintenance() << ","; // DistanceLastMaintenance
			fout << ScheduleNodeTypeMap.find(scheduleGraph.get_targetNodeData(arc).get_type())->second << "\n"; //Type
		}
	}

	fout.close();
}

void eva::DataOutput::writeUnassignedTripsToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph)
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "VS_UnallocatedTrips.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "TripID,StartLocationID,EndLocationID,StartTime,EndTime" << "\n";

	for (const BoostScheduleNode& node : scheduleGraph.get_unassignedTripNodes())
	{
		const ScheduleNodeData::ScheduleTripNodeData* tripNodeData = scheduleGraph.get_nodeData(node).castTripNodeData();

		fout << tripNodeData->get_trip().get_id() << ",";// TripID
		fout << tripNodeData->get_startLocation().get_id() << ",";// StartLocationID
		fout << tripNodeData->get_endLocation().get_id() << ",";// EndLocationID
		fout << Helper::DateTimeToString(tripNodeData->get_startTime()) << ",";// StartTime
		fout << Helper::DateTimeToString(tripNodeData->get_endTime()) << "\n";// EndTime
	};

	fout.close();
}

void eva::DataOutput::writeUnassignedMaintenancesToCsv(const DataInput& input, const ScheduleGraph& scheduleGraph)
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "VS_UnallocatedMaintenances.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "MaintenanceID,VehicleID,MaintenanceLocationID,StartTime,EndTime" << "\n";

	for (const BoostScheduleNode& node : scheduleGraph.get_unassignedMaintenanceNodes())
	{
		const ScheduleNodeData::ScheduleMaintenanceNodeData* maintenanceNodeData = scheduleGraph.get_nodeData(node).castMaintenanceNodeData();

		fout << maintenanceNodeData->get_maintenance().get_id() << ",";// MaintenanceID
		if(maintenanceNodeData->get_maintenance().is_assigned())
			fout << input.get_vehicles().get_vehicle(maintenanceNodeData->get_maintenance().get_indexVehicle()).get_id() << ",";// VehicleID
		else
			fout << ",";// VehicleID
		fout << maintenanceNodeData->get_maintenance().get_maintenanceLocation().get_id() << ",";// MaintenanceLocationID
		fout << Helper::DateTimeToString(maintenanceNodeData->get_startTime()) << ",";// StartTime
		fout << Helper::DateTimeToString(maintenanceNodeData->get_endTime()) << "\n";// EndTime
	};

	fout.close();
}


void eva::DataOutput::writeStatsPerformanceDetail(const DataInput& input, const std::vector<Stats::PerformanceDetail>& vecStatsPerformance)
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "StatsPerformanceDetail.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "IndexPlanningHorizon,IndexBranchingNode,Iteration,LBfrac,UBfrac,GAPfrac,IntFound,LBint,UBint,GAPint,MSecondsMP,MSecondsPP,MSecondsAUXCG,MSecondsStrongBranching,MPSizeConstraints,MPSizeVariables,ColumnsAdded,PPNetworkConstructionTime,PPNetworkSizeNodes,PPNetworkSizeArcs,LazyConstraintAdded,BranchType,IndexParentBranchingNode,MSecondsFilterMPVars,MSecondsFilterPPNodes,VehicleID,TripID,MaintenanceID,FracValue,BranchValue" << "\n";
	
	for (const Stats::PerformanceDetail& st : vecStatsPerformance)
	{
		if (st.branchType != NULL) {
			fout << st.indexPlanningHorizon << ","; // IndexPlanningHorizon, 
			fout << st.indexBranchingNode << ","; // IndexBranchingNode, 
			fout << ",,,,,,,,,,,,,,,,,,,"; // Empty [12]: Iteration,LBfrac,UBfrac,GAPfrac,IntFound,LBint,UBint,GAPint,SecondsMP,SecondsPP,MPSizeConstraints,MPSizeVariables,ColumnsAdded,PPNetworkConstructionTime,PPNetworkSizeNodes,PPNetworkSizeArcs,LazyConstraintAdded
			fout << st.branchType << ","; // BranchType, 
			fout << st.indexParentBranchingNode << ","; // IndexParentBranchingNode, 
			fout << st.time_mpFilterVars << ","; // MSecondsFilterMPVars, 
			fout << st.time_ppFilterNodes << ","; // MSecondsFilterPPNodes, 
			st.VehicleId != Constants::BIG_UINTEGER ? fout << st.VehicleId << "," : fout << ","; // VehicleID, 
			st.TripId != Constants::BIG_UINTEGER ? fout << st.TripId << "," : fout << ","; // TripID, 
			st.MaintenanceId != Constants::BIG_UINTEGER ? fout << st.MaintenanceId << "," : fout << ","; // MaintenanceID, 
			fout << st.fractionalValue << ","; // FracValue, 
			fout << st.branchValue << "\n"; // BranchValue"			
		}
		else {
			fout << st.indexPlanningHorizon << ","; // IndexPlanningHorizon, 
			fout << st.indexBranchingNode << ","; // IndexBranchingNode, 
			fout << st.iteration << ","; // Iteration, 
			fout << st.lb_relaxed << ","; // LBfrac, 
			fout << st.ub_relaxed << ","; // UBfrac, 
			fout << st.gap_relaxed() << ","; // GAPfrac, 
			fout << st.integerFound << ","; // IntFound, 
			fout << st.lb_integer << ","; // LBint, 
			fout << st.ub_integer << ","; // UBint, 
			fout << st.gap_integer() << ","; // GAPint, 
			fout << st.time_mpSolver << ","; // MSecondsMP, 
			fout << st.time_ppSolver << ","; // MSecondsPP, 
			fout << st.time_aux_cg << ","; // MSecondsAUXCG, 
			fout << st.time_strong_branch << ","; // MSecondsStrongBranching, 
			fout << st.mp_size_constraints << ","; // MPSizeConstraints, 
			fout << st.mp_size_variables << ","; // MPSizeVariables, 
			fout << st.columnsAdded << ","; // ColumnsAdded, 
			fout << st.pp_network_construction_ms << ","; // PPNetworkConstructionTime, 
			fout << st.pp_network_size_nodes << ","; // PPNetworkSizeNodes, 
			fout << st.pp_network_size_arcs << ","; // PPNetworkSizeArcs, 
			if (st.lazy_constraint_added != NULL)
				fout << st.lazy_constraint_added << ","; // LazyConstraintAdded
			else
				fout << ","; // LazyConstraintAdded
			fout << ",,,,,,,," << "\n"; // Empty[9]: BranchType,IndexParentBranchingNode,SecondsFilterMPVars,SecondsFilterPPNodes,VehicleID,TripID,MaintenanceID,FracValue,BranchValue
		};
	}

	fout.close();
}

void eva::DataOutput::writeStatsPlanningHorizon(const DataInput& input, const std::vector<Stats::PlanningHorizon>& vecStatsPlanningHorizon) 
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "StatsPlanningHorizon.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "Index,StartTime,EndTime,EndTimeOverlap,LBint,UBint,GAPint,NrSchedulesGenerated,NrUnassignedTrips,NrVehiclesRotation,MSecondsTotal,MSecondsMP,MSecondsPP,PPNetworkConstructionTime,PPNRSegments,MSecondsFilterMPVars,MSecondsFilterPPNodes,BranchingDepth,BranchingSize,Algorithm" << "\n";

	for (const Stats::PlanningHorizon& st : vecStatsPlanningHorizon)
	{
		fout << st.indexPlanningHorizon << ","; // Index,
		fout << Helper::DateTimeToString(st.startPlanningHorizon) << ","; // StartTime,
		fout << Helper::DateTimeToString(st.endPlanningHorizon) << ","; // EndTime,
		fout << Helper::DateTimeToString(st.endOverlapPlanningHorizon) << ","; // EndTimeOverlap,
		fout << st.lb_integer << ","; // LBint,
		fout << st.ub_integer << ","; // UBint,
		fout << st.gap_integer() << ","; // GAPint,
		fout << st.size_schedulesGenerated << ","; // NrSchedulesGenerated,
		fout << st.size_unassignedTrips << ","; // NrUnassignedTrips,
		fout << st.size_vehiclesSelected << ","; // NrVehiclesRotation,
		fout << st.time_total << ","; // MSecondsTotal,
		fout << st.time_mpSolver << ","; // MSecondsMP,
		fout << st.time_ppSolver << ","; // MSecondsPP,
		fout << st.pp_network_construction_ms << ","; // PPNetworkConstructionTime,
		fout << st.pp_nr_segments << ","; // PPNRSegments,
		fout << st.time_mpFilterVars << ","; // MSecondsFilterMPVars,
		fout << st.time_ppFilterNodes << ","; // MSecondsSecondsFilterPPNodesPP,
		fout << st.branchingTree_depth << ","; // BranchingDepth,
		fout << st.branchingTree_size << ","; // BranchingSize,
		fout << st.algorithm << "\n"; // Algorithm
	}

	fout.close();
}

void eva::DataOutput::writeStatsSchedule(const DataInput& input, const std::vector<Stats::Schedule>& vecStatsSchedule) 
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "StatsSchedule.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "CostVehicles,CostDeadlegs,CostMaintenance,CostUnassignedTrips,CostTotal" << "\n";

	for (const Stats::Schedule& st : vecStatsSchedule)
	{
		fout << st.cost_vehicles << ","; // CostVehicles
		fout << st.cost_deadlegs << ","; // CostDeadlegs
		fout << st.cost_maintenance << ","; // CostMaintenance
		fout << st.cost_unassignedTrips << ","; // CostUnassignedTrips
		fout << st.cost_total << "\n"; // CostTotal
	}

	fout.close();
}

void eva::DataOutput::writeStatsVehicles(const DataInput& input, const std::vector<Stats::Vehicles>& vecStatsVehicles) 
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "StatsVehicles.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "VehicleID,InRotation,CostVehicle,CostDeadleg,CostMaintenance,DistDeadlegsKm,DistAvgBetweenMaintenance,DistStdBetweenMaintenance,SOClb,SOCub,SecondsIdle,SecondsProductive,SecondsMaintenance,SecondsCharging" << "\n";

	for (const Stats::Vehicles& st : vecStatsVehicles)
	{
		fout << st.vehicleId << ","; // VehicleID,
		fout << st.inRotation << ","; // InRotation,
		fout << st.cost_vehicle << ","; // CostVehicle,
		fout << st.cost_deadlegs << ","; // CostDeadleg,
		fout << st.cost_maintenance << ","; // CostMaintenance,
		fout << st.km_deadlegs << ","; // DistDeadlegsKm,
		fout << st.km_avg_distance_maintenance << ","; // DistAvgBetweenMaintenance,
		fout << st.km_std_distance_maintenance << ","; // DistStdBetweenMaintenance,
		fout << st.lb_soc << ","; // SOClb,
		fout << st.ub_soc << ","; // SOCub,
		fout << st.seconds_idle << ","; // SecondsIdle,
		fout << st.seconds_productive << ","; // SecondsProductive,
		fout << st.seconds_maintenance << ","; // SecondsMaintenance,
		fout << st.seconds_charging << "\n"; // SecondsCharging"
	}

	fout.close();
}

void eva::DataOutput::writeStatsChargers(const DataInput& input, const std::vector<Stats::Chargers>& vecStatsChargers) 
{
	std::ofstream fout;
	fout.open(input.get_config().get_path_to_output() + "StatsChargers.csv", std::ofstream::out | std::ofstream::trunc);
	fout << "ChargerID,Timestamp,VehiclesAtCharger,Capacity" << "\n";

	for (const Stats::Chargers& st : vecStatsChargers)
	{
		fout << st.chargerId << ","; // ChargerID
		fout << Helper::DateTimeToString(st.timestamp) << ","; // Timestamp
		fout << st.size_vehiclesAtCharger << ","; // VehiclesAtCharger
		fout << st.chargerCapacity << "\n"; //Capacity
	}

	fout.close();
}

