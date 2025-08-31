#include "incl/evaDataHandler.h"

//#include<filesystem>
#include <boost/filesystem.hpp>
#include <numeric>

#include "evaExceptions.h"
#include <iostream>


void eva::DataHandler::_initialise(const Types::CommandInput& commandInput)
{
	// 1. Create all working directories:
	boost::filesystem::create_directory(commandInput.PathToConfig + "outputs");
	boost::filesystem::create_directory(commandInput.PathToConfig + "outputs/model");

	// 2. Initialise the input data:
	_input.initialiseFromCsv(commandInput);

	// 3. Initialise the schedule graph:
	_scheduleGraph.initialise(_input);
}

void eva::DataHandler::storeSolution(const Solution& solution)
{
	BoostScheduleNode fromNode;
	ScheduleNodeData fromNodeData, toNodeData;
	bool addedSuccessfully;
	std::vector<uint8_t> vecInRotation(_input.get_vehicles().get_vec().size(), false);

	for (const VehicleSchedule& vs : solution.vecSchedule)
	{
		const Vehicle& vehicle = _input.get_vehicles().get_vehicle(vs.indexVehicle);

		// Iterate over the schedule nodes and store the nodes:
		fromNode = get_vehiclePosition(vehicle).lastScheduleNode;
		fromNodeData = _scheduleGraph.get_nodeData(fromNode);

		// Iterate over the provided schedule
		// Loop starts at the beginning of the vector and iterates until the end. Keeping the order is important:
		for (const BoostScheduleNode& toNode : vs.vecScheduleNodes)
		{
			toNodeData = _scheduleGraph.get_nodeData(toNode);

			// Check if this is a charging connection:
			if (fromNodeData.get_type() == ScheduleNodeType::PUT_ON_CHARGE
				&& toNodeData.get_type() == ScheduleNodeType::TAKE_OFF_CHARGE)
			{
				addedSuccessfully = _scheduleGraph.add_charging(fromNode, toNode, vehicle);
			}
			else
			{
				addedSuccessfully = _scheduleGraph.add_deadleg(fromNode, toNode, vehicle);
			}

			if (!addedSuccessfully)
				throw LogicError("eva::DataHandler::storeSolution", "Mistake when trying to add the vehicle schedules deadlegs and charging.");

			fromNode = toNode;
			fromNodeData = toNodeData;
		}

		if (vs.vecScheduleNodes.empty())
		{
			// Save that the vehicle was not in rotation:
			vecInRotation[vs.indexVehicle] = false;
		}
		else
		{
			vecInRotation[vs.indexVehicle] = true;

			// Finally, add the last deadleg to the end location of the vehicle schedule.
			// Only if the vehicle was in rotation. Otherwise, we don't add a deadleg as the vehicle is stationary.
			_scheduleGraph.add_deadleg(fromNode, _input.get_locations().get_location(vs.indexEndLocation), vehicle);
		}
	}

	// Store the results of all vehicles that are not in rotation:
	for (const Vehicle& vehicle : _input.get_vehicles().get_vec())
	{
		if (!vecInRotation[vehicle.get_index()])
		{
			_scheduleGraph.add_out_of_rotation(_input, get_vehiclePosition(vehicle).lastScheduleNode, vehicle, solution.startDecisionHorizon, solution.endDecisionHorizon);
		}
	}

	// Finally, update the current vehicle position vector:
	_scheduleGraph.updateVehiclePositions(_input);
}

void eva::DataHandler::storeStatsVehicles()
{
	for (const Vehicle& vehicle : _input.get_vehicles().get_vec())
	{
		ScheduleResourceContainer finalCont = get_vehiclePosition(vehicle);

		Stats::Vehicles statsVehicle;
		statsVehicle.vehicleId = vehicle.get_id();
		
		statsVehicle.inRotation = _scheduleGraph.get_vecSchedulePath(vehicle.get_index()).size() > 0;
		statsVehicle.cost_deadlegs = finalCont.cost_deadlegs;
		statsVehicle.cost_maintenance = finalCont.cost_maintenance;
		statsVehicle.cost_vehicle = finalCont.cost_vehicle;
		statsVehicle.km_deadlegs = finalCont.dist_deadlegs;
		statsVehicle.km_avg_distance_maintenance = finalCont.avg_distance_maintenance();
		statsVehicle.km_std_distance_maintenance = finalCont.std_distance_maintenance();
		statsVehicle.lb_soc = finalCont.lb_soc;
		statsVehicle.ub_soc = finalCont.ub_soc;
		statsVehicle.seconds_productive = finalCont.seconds_productive;
		statsVehicle.seconds_idle = finalCont.seconds_idle;
		statsVehicle.seconds_charging = finalCont.seconds_charging;
		statsVehicle.seconds_maintenance = finalCont.seconds_maintenance;

		_stats.add_statsVehicles(statsVehicle);
	}
}

void eva::DataHandler::storeStatsChargers()
{
	std::vector<uint32_t> chargerPrevSize(_input.get_chargers().get_vec().size(), 0);
	for (Types::Index indexChargerCheck = 0; indexChargerCheck < _scheduleGraph.get_vecSortedPutOnCharges().size(); indexChargerCheck++)
	{
		const Charger& charger = _scheduleGraph.get_nodeData(_scheduleGraph.get_vecSortedPutOnCharges()[indexChargerCheck]).castPutOnChargeNodeData()->get_charger();

		Stats::Chargers statsCharger;
		statsCharger.chargerId = charger.get_id();
		statsCharger.chargerCapacity = charger.get_capacity();
		statsCharger.timestamp = _scheduleGraph.get_nodeData(_scheduleGraph.get_vecSortedPutOnCharges()[indexChargerCheck]).get_endTime();
		statsCharger.size_vehiclesAtCharger = chargerPrevSize[charger.get_index()] + _scheduleGraph.get_numberOutgoingArcs(_scheduleGraph.get_vecSortedPutOnCharges()[indexChargerCheck]) - _scheduleGraph.get_numberOutgoingArcs(_scheduleGraph.get_vecSortedTakeOffCharges()[indexChargerCheck]);

		chargerPrevSize[charger.get_index()] = statsCharger.size_vehiclesAtCharger;

		_stats.add_statsChargers(statsCharger);
	}
}

void eva::DataHandler::storeStatsSchedules()
{
	Stats::Schedule statsSchedule;
	statsSchedule.cost_deadlegs = 0.0;
	statsSchedule.cost_maintenance = 0.0;
	statsSchedule.cost_vehicles = 0.0;
	statsSchedule.cost_total = 0.0;

	for (const Vehicle& vehicle : _input.get_vehicles().get_vec())
	{
		ScheduleResourceContainer finalCont = get_vehiclePosition(vehicle);
		statsSchedule.cost_deadlegs += finalCont.cost_deadlegs;
		statsSchedule.cost_maintenance += finalCont.cost_maintenance;
		statsSchedule.cost_vehicles += finalCont.cost_vehicle;
		statsSchedule.cost_total += finalCont.cost_total();
	}

	statsSchedule.cost_unassignedTrips = _scheduleGraph.get_unassignedTripNodes().size() * _input.get_config().get_cost_uncovered_trip();
	statsSchedule.cost_total += statsSchedule.cost_unassignedTrips;

	_stats.add_statsSchedule(statsSchedule);
}

void eva::DataHandler::writeOutputToCsv()
{
	_output.writeVehicleScheduleToCsv(_input, _scheduleGraph);
	_output.writeUnassignedTripsToCsv(_input, _scheduleGraph);
	_output.writeUnassignedMaintenancesToCsv(_input, _scheduleGraph);

	// write stats:
	_output.writeStatsChargers(_input, _stats.get_vecStatsChargers());
	_output.writeStatsVehicles(_input, _stats.get_vecStatsVehicles());
	_output.writeStatsSchedule(_input, _stats.get_vecStatsSchedule());
	_output.writeStatsPlanningHorizon(_input, _stats.get_vecStatsPlanningHorizon());
	_output.writeStatsPerformanceDetail(_input, _stats.get_vecStatsPerformanceDetail());

	
}