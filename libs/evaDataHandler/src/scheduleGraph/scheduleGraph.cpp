#include "incl/scheduleGraph/scheduleGraph.h"

#include "evaExceptions.h"
#include <iostream>
#include <numeric>

const double eva::ScheduleResourceContainer::avg_distance_maintenance() const
{
	return vecMaintenanceDistances.empty() ? Constants::BIG_DOUBLE : std::accumulate(vecMaintenanceDistances.begin(), vecMaintenanceDistances.end(), 0.0) / vecMaintenanceDistances.size();
}

const double eva::ScheduleResourceContainer::std_distance_maintenance() const
{
	double mean = avg_distance_maintenance();
	double size = vecMaintenanceDistances.size();
	
	return vecMaintenanceDistances.empty() ? Constants::BIG_DOUBLE : std::sqrt(std::accumulate(vecMaintenanceDistances.begin(), vecMaintenanceDistances.end(), 0.0, [&](double res, const double& val) { return res + std::pow((val - mean), 2.0); }) / vecMaintenanceDistances.size());
}

void eva::ScheduleGraph::_add_trips(const std::vector<Trip>& vecTrips) 
{
	for (const Trip& trip : vecTrips)
	{
		_addNode(
			ScheduleNodeData(
				_getNextIndexNode(),
				ScheduleNodeData::ScheduleTripNodeData(trip)
			),
			_vecSortedTrips
		);
	}
}

void eva::ScheduleGraph::_add_chargers(const std::vector<Charger>& vecChargers, const Config& config)
{
	// The charging nodes discretise the space of time to start charging. 
	// Vehicles can be put on charge, and taken off charge every config.get_const_charger_capacity_check() - minutes. 
	// The smaller the capacity check, the more options exist for the vehicle's to start.
	
	for (const Charger& charger : vecChargers)
	{
		// Smoothen out the start time, and add a few more checks before:
		// Add 6 hours more of put-on-charge nodes:
		Types::DateTime startTime = _earliestVehicleStartTime - 6 * 60 * 60;

		while (Helper::diffDateTime(startTime, config.get_date_end() + config.get_const_planning_horizon_overlap()) > 0)
		{
			// Add the put on charge node:
			_addNode(
				ScheduleNodeData(
					_getNextIndexNode(),
					ScheduleNodeData::SchedulePutOnChargeNodeData(charger, startTime, startTime + config.get_const_put_vehicle_on_charge())
				),
				_vecSortedPutOnCharges
			);

			// Add the take off charge node:
			_addNode(
				ScheduleNodeData(
					_getNextIndexNode(),
					ScheduleNodeData::ScheduleTakeOffChargeNodeData(charger, startTime, startTime + config.get_const_take_vehicle_off_charge())
				),
				_vecSortedTakeOffCharges
			);

			startTime += config.get_const_charger_capacity_check();
		}

	}
}
void eva::ScheduleGraph::_add_maintenances(const std::vector<Maintenance>& vecMaintenances)
{
	for (const Maintenance& maintenance : vecMaintenances)
	{
		_addNode(
			ScheduleNodeData(
				_getNextIndexNode(),
				ScheduleNodeData::ScheduleMaintenanceNodeData(maintenance)
			),
			_vecSortedMaintenances
		);
	}
}

void eva::ScheduleGraph::_add_scheduleStartNodes(const std::vector<Vehicle>& vecVehicles)
{
	_vecStartNodes.resize(vecVehicles.size());
	for (const Vehicle& vehicle : vecVehicles)
	{
		// Save the current node as the only node in the vehicle schedule.
		BoostScheduleNode startNode = _addNode(
			ScheduleNodeData(
				_getNextIndexNode(),
				ScheduleNodeData::ScheduleStartNodeData(vehicle)
			)
		);

		// Store the start node for the vehicle:
		_vecStartNodes[vehicle.get_index()] = startNode;

		// Update the earliest vehicle start time:
		_earliestVehicleStartTime = vehicle.get_initialStartTime() < _earliestVehicleStartTime ? vehicle.get_initialStartTime() : _earliestVehicleStartTime;
	}
}

eva::BoostScheduleNode eva::ScheduleGraph::_addNode(const ScheduleNodeData& nodeData, std::vector<BoostScheduleNode>& vecSorted)
{
	// 0. Add the vertex to the network:
	BoostScheduleNode retNode = _addNode(nodeData);

	// 1. Find position in time-sorted vector, and store the node:
	_storeNodeSorted(retNode, vecSorted);

	return retNode;
}

eva::BoostScheduleNode eva::ScheduleGraph::_addNode(const ScheduleNodeData& nodeData)
{
	return boost::add_vertex(nodeData, _boostScheduleGraph);
}

void eva::ScheduleGraph::_storeNodeSorted(const eva::BoostScheduleNode& node, std::vector<BoostScheduleNode>& vecSorted)
{
	// 1. Find position in time-sorted vector, and store the node:
	auto it = std::lower_bound(vecSorted.begin(), vecSorted.end(), _boostScheduleGraph[node].get_startTime(),
		[&](const BoostScheduleNode& l, Types::DateTime value)
		{
			return _boostScheduleGraph[l].get_startTime() < value;
		});

	vecSorted.insert(it, node);
}

const bool eva::ScheduleGraph::_checkNodeCoverage(const BoostScheduleNode& node) const
{
	switch (get_nodeData(node).type)
	{
		case ScheduleNodeType::DEADLEG:
		case ScheduleNodeType::TRIP:
		case ScheduleNodeType::MAINTENANCE:
		case ScheduleNodeType::START_SCHEDULE:
			return boost::out_degree(node, _boostScheduleGraph) > 0;
		case ScheduleNodeType::PUT_ON_CHARGE:
		case ScheduleNodeType::TAKE_OFF_CHARGE:
		case ScheduleNodeType::UNDEFINED:
		default:
			return false;
	}	
}

bool eva::ScheduleGraph::_checkScheduleTimeSpaceContinuity()
{
	BoostScheduleNode prevNode, curNode;

	for (const auto& vecSchedule : _vecSchedulePaths)
	{
		if (vecSchedule.size() > 0)
		{
			prevNode = boost::source(vecSchedule.front(), _boostScheduleGraph);
			for (const auto& arc : vecSchedule)
			{
				curNode = boost::source(arc, _boostScheduleGraph);
				if (prevNode != curNode)
				{
					// Print the mistake:
					for (const auto& arc : vecSchedule)
					{
						std::cout << get_sourceNodeData(arc).index << " (" << ScheduleNodeTypeMap.find(get_sourceNodeData(arc).get_type())->second << ") -> "
							<< get_targetNodeData(arc).index << " (" << ScheduleNodeTypeMap.find(get_targetNodeData(arc).get_type())->second << ")" << std::endl;
					}
					return false; // Space-continuity wrong.
				} 
				else
				{
					// Check that the time and space is correct:
					// PrevNode == CurNode:
					if (get_sourceNodeData(arc).get_endTime() <= get_targetNodeData(arc).get_startTime())
					{
						prevNode = boost::target(arc, _boostScheduleGraph);
					}
					else
					{
						std::cout << "Time - continuity wrong!!!" << std::endl;
						return false; // Time-continuity wrong.
					}
				}
			}
		}
	}

	return true;
}

const std::pair<std::vector<eva::BoostScheduleNode>::const_iterator, std::vector<eva::BoostScheduleNode>::const_iterator> eva::ScheduleGraph::_getBoundsTimeSortedVector(const std::vector<BoostScheduleNode>& vec, const Types::DateTime& lb, const Types::DateTime& ub) const
{
	// Find range in time sorted vector:
	auto iterBegin = std::lower_bound(vec.begin(), vec.end(), lb,
		[&](const BoostScheduleNode& l, Types::DateTime value)
		{
			return _boostScheduleGraph[l].get_startTime() < value;
		});

	auto iterEnd = std::lower_bound(vec.begin(), vec.end(), ub,
		[&](const BoostScheduleNode& l, Types::DateTime value)
		{
			return  _boostScheduleGraph[l].get_startTime() < value;
		});

	return std::make_pair(iterBegin , iterEnd);
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::_getNodesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate, const std::vector<BoostScheduleNode>& vecSortedNodes) const
{
	std::vector<eva::BoostScheduleNode> result;
	auto& bounds = _getBoundsTimeSortedVector(vecSortedNodes, startDate, endDate);
	result.assign(bounds.first, bounds.second);

	// Filter all nodes that are already covered, and can't have more vehicle leaving:)
	auto it = result.begin();
	while (it != result.end()) {
		if (_checkNodeCoverage(*it)) {
			it = result.erase(it);
			continue;
		}
		it++;
	}

	return result;
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::getTripsInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const
{
	return _getNodesInDateInterval(startDate, endDate, _vecSortedTrips);
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::getMaintenancesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const
{
	return _getNodesInDateInterval(startDate, endDate, _vecSortedMaintenances);
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::getPutOnChargesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const
{
	return _getNodesInDateInterval(startDate, endDate, _vecSortedPutOnCharges);
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::getTakeOffChargesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const
{
	return _getNodesInDateInterval(startDate, endDate, _vecSortedTakeOffCharges);
}

bool eva::ScheduleGraph::add_out_of_rotation(const DataInput& input, const BoostScheduleNode& fromNode, const Vehicle& vehicle, const Types::DateTime& startHorizon, const Types::DateTime& endHorizon)
{
	// Set the time of an out of rotation at either the current optimisation input's start-time or the previous end time:
	// Set the time to the maximum the vehicle could charge before the next planning horizon:
	
	// 1. Get current SoC of vehicle:
	
	const ScheduleNodeData& lastScheduleNodeData = get_nodeData(fromNode);

	// Obtain the charger index:
	Types::Index indexCharger = Constants::BIG_INDEX;
	for (const Charger& charger : input.get_chargers().get_vec())
	{
		if (charger.get_location().get_index() == lastScheduleNodeData.get_endLocation().get_index())
		{
			indexCharger = charger.get_index();
			break;
		}
	}
	if (indexCharger == Constants::BIG_INDEX)
		throw LogicError("eva::ScheduleGraph::add_out_of_rotation", "Vehicle is not out-of-rotation at a charger.");

	auto maxCharge = vehicle.get_batteryMaxKWh() - getVehiclePosition(vehicle).soc;
	auto chargingSpeed = vehicle.get_chargingSpeedKwS(input.get_chargers().get_vec()[indexCharger]);
	auto maxChargingDuration = maxCharge / chargingSpeed
		+ input.get_config().get_const_put_vehicle_on_charge()
		+ input.get_config().get_const_take_vehicle_off_charge();

	// Compute the largest distance deadleg from the end location:
	auto maxDurationDeadleg = 0;
	for (const Location& toLocation : input.get_locations().get_vec())
	{
		if (lastScheduleNodeData.get_endLocation().get_durationToLocation(toLocation) != Constants::BIG_UINTEGER
			&& lastScheduleNodeData.get_endLocation().get_durationToLocation(toLocation) > maxDurationDeadleg)
		{
			maxDurationDeadleg = lastScheduleNodeData.get_endLocation().get_durationToLocation(toLocation);
		}
	}

	Types::DateTime outOfRotationTime = std::max(get_nodeData(fromNode).get_endTime(), Helper::roundToNearestMinute(endHorizon - maxChargingDuration - maxDurationDeadleg - 60));

	// Otherwise, proceed to add the deadleg to the scheduleGraph:
	// 1. Store the deadledNodeData:
	BoostScheduleNode outOfRotationNode = _addNode(
		ScheduleNodeData(
			_getNextIndexNode(),
			ScheduleNodeData::ScheduleOutOfRotationNodeData(get_nodeData(fromNode).get_endLocation(), outOfRotationTime)
		)
	);

	// a. fromNode -> outOfRotationNode:
	_vecSchedulePaths[vehicle.get_index()].push_back(
		boost::add_edge(fromNode, outOfRotationNode,
			ScheduleArcData(
				_getNextIndexArc(),
				0
			),
			_boostScheduleGraph
		).first);

	return _checkScheduleTimeSpaceContinuity();
}

bool eva::ScheduleGraph::add_deadleg(const BoostScheduleNode& fromNode, const BoostScheduleNode& toNode, const Vehicle& vehicle)
{
	// Multi-graph allowed. Hence, adding an edge should always be feasible.
	// However, there may be overlaps at charging nodes, but should never be at trip nodes or maintenance nodes:
	
	// Also, there is no deadleg between a put-on charge and take-off charge node:
	if (get_nodeData(fromNode).type == ScheduleNodeType::PUT_ON_CHARGE
		&& get_nodeData(toNode).type == ScheduleNodeType::TAKE_OFF_CHARGE)
		return false;

	// Otherwise, proceed to add the deadleg to the scheduleGraph:
	// 1. Store the deadledNodeData:
	BoostScheduleNode deadlegNode;
	if (get_nodeData(fromNode).type != ScheduleNodeType::DEADLEG)
	{
		deadlegNode = _addNode(
			ScheduleNodeData(
				_getNextIndexNode(),
				ScheduleNodeData::ScheduleDeadlegNodeData(get_nodeData(fromNode).get_endLocation(), get_nodeData(toNode).get_startLocation(), get_nodeData(fromNode).get_endTime())
			)
		);

		// 2. Store the arcs:
		// a. fromNode -> deadlegNode:
		_vecSchedulePaths[vehicle.get_index()].push_back(
			boost::add_edge(fromNode, deadlegNode,
				ScheduleArcData(
					_getNextIndexArc(),
					0
				),
				_boostScheduleGraph
			).first
		);
	}
	else
	{
		deadlegNode = fromNode;
	}	

	// b. deadlegNode -> toNode:
	_vecSchedulePaths[vehicle.get_index()].push_back(
		boost::add_edge(deadlegNode, toNode,
			ScheduleArcData(
				_getNextIndexArc(),
				Helper::diffDateTime(get_nodeData(deadlegNode).get_endTime(), get_nodeData(toNode).get_startTime())
			),
			_boostScheduleGraph
		).first);

	return _checkScheduleTimeSpaceContinuity();
}

bool eva::ScheduleGraph::add_deadleg(const BoostScheduleNode& fromNode, const Location& toLocation, const Vehicle& vehicle)
{
	// Multi-graph allowed. Hence, adding an edge should always be feasible.
	// However, there may be overlaps at charging nodes, but should never be at trip nodes or maintenance nodes:
	
	// Also, there is no deadleg between a put-on charge and take-off charge node:
	if (get_nodeData(fromNode).type == ScheduleNodeType::PUT_ON_CHARGE)
		return false;

	// Otherwise, proceed to add the deadleg to the scheduleGraph:
	// 1. Store the deadledNodeData:
	BoostScheduleNode deadlegNode = _addNode(
		ScheduleNodeData(
			_getNextIndexNode(),
			ScheduleNodeData::ScheduleDeadlegNodeData(get_nodeData(fromNode).get_endLocation(), toLocation, get_nodeData(fromNode).get_endTime())
		)
	);


	// 2. Store the arcs:
	// a. fromNode -> deadlegNode:
	_vecSchedulePaths[vehicle.get_index()].push_back(
		boost::add_edge(fromNode, deadlegNode,
			ScheduleArcData(
				_getNextIndexArc(),
				0
			),
			_boostScheduleGraph
		).first);


	return _checkScheduleTimeSpaceContinuity();

}

bool eva::ScheduleGraph::add_charging(const BoostScheduleNode& fromNode, const BoostScheduleNode& toNode, const Vehicle& vehicle)
{
	// An arc can only be added directly when it represents charging:
	if (get_nodeData(fromNode).type != ScheduleNodeType::PUT_ON_CHARGE
		|| get_nodeData(toNode).type != ScheduleNodeType::TAKE_OFF_CHARGE)
		return false;

	// 1. Store the charging node:
	BoostScheduleNode chargingNode = _addNode(
		ScheduleNodeData(
			_getNextIndexNode(),
			ScheduleNodeData::ScheduleChargingNodeData(get_nodeData(fromNode).castPutOnChargeNodeData()->get_charger(), get_nodeData(fromNode).get_endTime(), get_nodeData(toNode).get_startTime())
		)
	);

	// 2. Store the connnections:
	// a. FromNode -> ChargingNode:
	_vecSchedulePaths[vehicle.get_index()].push_back(
		boost::add_edge(fromNode, chargingNode,
			ScheduleArcData(
				_getNextIndexArc(),
				0
			),
			_boostScheduleGraph
		).first
	);

	// b. ChargingNode -> toNode:
	_vecSchedulePaths[vehicle.get_index()].push_back(
		boost::add_edge(chargingNode, toNode,
			ScheduleArcData(
				_getNextIndexArc(),
				Helper::diffDateTime(get_nodeData(chargingNode).get_endTime(), get_nodeData(toNode).get_startTime())
			),
			_boostScheduleGraph
		).first);

	return _checkScheduleTimeSpaceContinuity();
}

void eva::ScheduleGraph::updateVehiclePositions(const DataInput& input)
{
	for (const Vehicle& vehicle : input.get_vehicles().get_vec())
	{
		ScheduleResourceContainer old_cont(
			vehicle.get_odometerReading(),
			vehicle.get_odometerLastMaintenance(),
			vehicle.get_initialSOC(),
			_vecStartNodes[vehicle.get_index()]
		);
		ScheduleResourceContainer new_cont = old_cont;

		for (const BoostScheduleArc& arc : _vecSchedulePaths[vehicle.get_index()])
		{
			processArc(input, vehicle, new_cont, old_cont, arc);
		}

		_vecCurrentVehiclePositions[vehicle.get_index()] = new_cont;
	}
}

void eva::ScheduleGraph::initialise(const DataInput& input)
{
	// Start with the vehicle schedule nodes, needed for initialisation of earliest vehicle time:
	_earliestVehicleStartTime = input.get_config().get_date_start();
	_vecSchedulePaths.resize(input.get_vehicles().get_vec().size());
	_vecCurrentVehiclePositions.resize(input.get_vehicles().get_vec().size());

	_add_scheduleStartNodes(input.get_vehicles().get_vec());
	_add_trips(input.get_trips().get_vec());
	_add_maintenances(input.get_maintenances().get_vec());
	_add_chargers(input.get_chargers().get_vec(), input.get_config());
	

	// Update the initial vehicle positions:
	updateVehiclePositions(input);
}

void eva::ScheduleGraph::clear()
{
	_vecSortedMaintenances.clear();
	_vecSortedPutOnCharges.clear();
	_vecSortedTakeOffCharges.clear();
	_vecSortedTrips.clear();
	_vecSchedulePaths.clear();

	_boostScheduleGraph.clear();

	_indexNode = 0;
	_indexArc = 0;
	_earliestVehicleStartTime = Constants::MAX_TIMESTAMP;
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::get_unassignedTripNodes() const
{
	std::vector<BoostScheduleNode> result;

	for (const BoostScheduleNode& node : _vecSortedTrips)
	{
		if (boost::out_degree(node, _boostScheduleGraph) <= 0)
		{
			result.push_back(node);
		}
	}

	return result;
}

const std::vector<eva::BoostScheduleNode> eva::ScheduleGraph::get_unassignedMaintenanceNodes() const
{
	std::vector<BoostScheduleNode> result;

	for (const BoostScheduleNode& node : _vecSortedMaintenances)
	{
		if (boost::out_degree(node, _boostScheduleGraph) <= 0)
		{
			result.push_back(node);
		}
	}

	return result;
}

void eva::ScheduleGraph::processArc(const DataInput& input, const Vehicle& vehicle, ScheduleResourceContainer& new_cont, ScheduleResourceContainer& old_cont, const BoostScheduleArc& arc) const
{
	const ScheduleNodeData& targetNodeData = get_targetNodeData(arc);

	// If the first arc is processed, the vehicle must be active, so update the cost:
	new_cont.cost_vehicle = vehicle.get_cost();

	// Update the new container:
	new_cont.lastScheduleNode = boost::target(arc, _boostScheduleGraph);
	new_cont.odometerReading += targetNodeData.get_distance();
	new_cont.cost_maintenance += 0.5 * input.get_config().get_cost_coefficient_penalty_maintenance() * (std::pow(new_cont.distanceLastMaintenance(), 2.0) - std::pow(old_cont.distanceLastMaintenance(), 2.0));
	new_cont.cost_deadlegs += static_cast<double>(targetNodeData.get_type() == ScheduleNodeType::DEADLEG) * (input.get_config().get_cost_deadleg_fix() + input.get_config().get_cost_deadleg_per_km() * targetNodeData.get_distance());
	new_cont.dist_deadlegs += static_cast<double>(targetNodeData.get_type() == ScheduleNodeType::DEADLEG) * targetNodeData.get_distance();
	new_cont.seconds_idle += get_arcData(arc).get_duration();
	
	if (targetNodeData.get_type() == ScheduleNodeType::DEADLEG
		|| targetNodeData.get_type() == ScheduleNodeType::OUT_OF_ROTATION)
		new_cont.seconds_idle += targetNodeData.get_duration();
	else if (targetNodeData.get_type() == ScheduleNodeType::MAINTENANCE)
		new_cont.seconds_maintenance += targetNodeData.get_duration();
	else if (targetNodeData.get_type() == ScheduleNodeType::CHARGING
		|| targetNodeData.get_type() == ScheduleNodeType::PUT_ON_CHARGE
		|| targetNodeData.get_type() == ScheduleNodeType::TAKE_OFF_CHARGE)
		new_cont.seconds_charging += targetNodeData.get_duration();
	else if (targetNodeData.get_type() == ScheduleNodeType::TRIP)
		new_cont.seconds_productive += targetNodeData.get_duration();

	// Update the distances travelled between maintenances, and the new odometer reading:
	if (targetNodeData.get_type() == ScheduleNodeType::MAINTENANCE) {
		new_cont.odometerLastMaintenance = new_cont.odometerReading;
		new_cont.vecMaintenanceDistances.push_back(new_cont.odometerReading - old_cont.odometerLastMaintenance);
	}

	// Check the SOC:
	new_cont.soc -= vehicle.get_batteryDischarge(targetNodeData.get_distance());
	new_cont.lb_soc = new_cont.soc < new_cont.lb_soc ? new_cont.soc : new_cont.lb_soc;

	// Check that the vehicle remains feasible, and within the bounds:
	if (new_cont.soc < vehicle.get_batteryMinKWh()
		|| new_cont.soc > vehicle.get_batteryMaxKWh())
	{
		throw LogicError("eva::ScheduleGraph::processArc", "SOC " + std::to_string(new_cont.soc) +  "has surpassed the allowed bounds of the vehicle [" + std::to_string(vehicle.get_batteryMinKWh()) + "," + std::to_string(vehicle.get_batteryMaxKWh()) + "]. There must be a mistake in the stored schedule.");
	}

	new_cont.soc += targetNodeData.get_type() == ScheduleNodeType::CHARGING ? targetNodeData.castChargingNodeData()->get_charge(new_cont.soc, vehicle) : 0;
	new_cont.ub_soc = new_cont.soc > new_cont.ub_soc ? new_cont.soc : new_cont.ub_soc;

	// Increment, and store the information in old_cont.
	old_cont = new_cont;
}