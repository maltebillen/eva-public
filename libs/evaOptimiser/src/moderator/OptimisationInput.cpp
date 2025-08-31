#include "incl/moderator/OptimisationInput.h"

#include "evaExceptions.h"

void eva::OptimisationInput::_initialise()
{
	// 1. Initialise the planning horizon:
	_startPlanningHorizon = _dataHandler.get_config().get_date_start();

	_endPlanningHorizon = std::min(
		_startPlanningHorizon + _dataHandler.get_config().get_const_planning_horizon_length(),
		_dataHandler.get_config().get_date_end()
	);

	_endPlanningHorizonOverlap = std::min(
		_endPlanningHorizon + _dataHandler.get_config().get_const_planning_horizon_overlap(),
		_dataHandler.get_config().get_date_end()
	);

	// Fill the vectors with the nodes currently in the planning horizon:
	_loadScheduleNodes();
}

void eva::OptimisationInput::_reset()
{
	_vecTrips.clear();
	_vecMaintenances.clear();
	_vecPutOnChargeNodes.clear();
	_vecTakeOffChargeNodes.clear();

	_flag_has_unassigned_maintenance = false;
}

void eva::OptimisationInput::_loadScheduleNodes()
{
	// 1. Reset the current loaded schedule nodes:
	_reset();

	// 2. Initialise the earliest vehicle time before loading the vehicles:
	_updateEarliestVehicleTime();

	// 3. Load the nodes in the current planning horizon:
	// a. Trips:
	for (const BoostScheduleNode& scheduleNodeIndex : _dataHandler.get_scheduleGraph().getTripsInDateInterval(_startPlanningHorizon, _endPlanningHorizonOverlap))
	{
		_vecTrips.push_back(SubScheduleTripNodeData(_vecTrips.size(), _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex)));
	}
	
	// b. Maintenances:
	_flag_has_unassigned_maintenance = false;
	for (const BoostScheduleNode& scheduleNodeIndex : _dataHandler.get_scheduleGraph().getMaintenancesInDateInterval(_startPlanningHorizon, _endPlanningHorizonOverlap))
	{
		_vecMaintenances.push_back(SubScheduleMaintenanceNodeData(_vecMaintenances.size(), _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex)));

		// Check if the maintenance is available for assignment:
		// This triggers the algorithm to distribute the unassigned maintenance slots to the most-needing vehicles, based on distance since last being maintained.
		if(_dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex).castMaintenanceNodeData()->get_maintenance().is_unassigned())
			_flag_has_unassigned_maintenance = true;
	}

	// d. Put On Charge:
	Types::Index chargerIndex;
	_vecPutOnChargeNodes.resize(_dataHandler.get_chargers().get_vec().size());
	for (const BoostScheduleNode& scheduleNodeIndex : _dataHandler.get_scheduleGraph().getPutOnChargesInDateInterval(_earliestVehicleTime, _endPlanningHorizonOverlap))
	{
		chargerIndex = _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex).castPutOnChargeNodeData()->get_charger().get_index();
		_vecPutOnChargeNodes[chargerIndex].push_back(SubSchedulePutOnChargeNodeData(_vecPutOnChargeNodes[chargerIndex].size(), _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex)));
	}

	// e. Take Off Charge:
	_vecTakeOffChargeNodes.resize(_dataHandler.get_chargers().get_vec().size());
	for (const BoostScheduleNode& scheduleNodeIndex : _dataHandler.get_scheduleGraph().getTakeOffChargesInDateInterval(_earliestVehicleTime, _endPlanningHorizonOverlap))
	{
		chargerIndex = _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex).castTakeOffChargeNodeData()->get_charger().get_index();
		_vecTakeOffChargeNodes[chargerIndex].push_back(SubScheduleTakeOffChargeNodeData(_vecTakeOffChargeNodes[chargerIndex].size(), _dataHandler.get_scheduleGraph().get_nodeData(scheduleNodeIndex)));
	}
}

void eva::OptimisationInput::_updateEarliestVehicleTime()
{
	// Initialise the earliest vehicle time before loading the vehicles:
	_earliestVehicleTime = _startPlanningHorizon;
	for (const Vehicle& vehicle : _dataHandler.get_vehicles().get_vec())
	{
		_earliestVehicleTime = get_scheduleGraphNodeData(get_vehiclePosition(vehicle).lastScheduleNode).get_endTime() < _earliestVehicleTime ?
			get_scheduleGraphNodeData(get_vehiclePosition(vehicle).lastScheduleNode).get_endTime() : _earliestVehicleTime;
	}
}

bool eva::OptimisationInput::next()
{
	_reset();

	// Move to the next planning horizon:
	// Return if the end of the planning horizon is already reached:
	if (_endPlanningHorizon == _dataHandler.get_config().get_date_end()) return false;

	// Else, start updating the objects:
	_startPlanningHorizon = _endPlanningHorizon;
	_endPlanningHorizon = std::min(
		_endPlanningHorizon + _dataHandler.get_config().get_const_planning_horizon_length(),
		_dataHandler.get_config().get_date_end()
	);

	_endPlanningHorizonOverlap = std::min(
		_endPlanningHorizonOverlap + _dataHandler.get_config().get_const_planning_horizon_length(),
		_dataHandler.get_config().get_date_end()
	);

	_indexPlanningHorizon++;

	// Fill the vectors with the nodes currently in the planning horizon:
	_loadScheduleNodes();

    return true;
}

const eva::Types::Index eva::OptimisationInput::get_nextIdxPutOnChargeAfterStartTime(const Types::Index& indexCharger, const Types::DateTime& starttime) const
{
	// If starttime lies before the first available charging node, it can't do it earlier, and must wait for the first one:
	auto timediff = std::max(int64_t(0),Helper::diffDateTime(_vecPutOnChargeNodes[indexCharger].front().get_scheduleNodeData().get_startTime(), starttime));
	Types::Index result = static_cast<Types::Index>(Helper::intDivisionRoundUp(timediff, get_config().get_const_charger_capacity_check()));

	if (result < _vecPutOnChargeNodes[indexCharger].size())
		return result;
	else
		return Constants::BIG_INDEX;
}

const eva::Types::Index eva::OptimisationInput::get_nextIdxTakeOffChargeAfterStartTime(const Types::Index& indexCharger, const Types::DateTime& starttime) const
{
	auto timediff = std::max(int64_t(0), Helper::diffDateTime(_vecTakeOffChargeNodes[indexCharger].front().get_scheduleNodeData().get_startTime(), starttime));
	Types::Index result = static_cast<Types::Index>(Helper::intDivisionRoundUp(timediff, get_config().get_const_charger_capacity_check()));

	if (result < _vecPutOnChargeNodes[indexCharger].size())
		return result;
	else
		return Constants::BIG_INDEX;
}

const eva::Types::Index eva::OptimisationInput::get_nextIdxPutOnChargeBeforeEndTime(const Types::Index& indexCharger, const Types::DateTime& endtime) const
{
	auto timediff = std::min((int64_t(_vecPutOnChargeNodes[indexCharger].size() * get_config().get_const_charger_capacity_check())),
		Helper::diffDateTime(_vecPutOnChargeNodes[indexCharger].front().get_scheduleNodeData().get_endTime(), endtime));

	if (timediff < 0)
		return Constants::BIG_INDEX;
	else
		return static_cast<Types::Index>(Helper::intDivisionRoundDown(timediff, get_config().get_const_charger_capacity_check()));
}

const eva::Types::Index eva::OptimisationInput::get_nextIdxTakeOffChargeBeforeEndTime(const Types::Index& indexCharger, const Types::DateTime& endtime) const
{
	auto timediff = std::min((int64_t(_vecTakeOffChargeNodes[indexCharger].size() * get_config().get_const_charger_capacity_check())),
		Helper::diffDateTime(_vecTakeOffChargeNodes[indexCharger].front().get_scheduleNodeData().get_endTime(), endtime));

	if (timediff < 0)
		return Constants::BIG_INDEX;
	else
		return static_cast<Types::Index>(Helper::intDivisionRoundDown(timediff, get_config().get_const_charger_capacity_check()));
}
