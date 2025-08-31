#include "incl/pricingProblem/segmentBased/centralisedNetwork/segmentCentralisedNetwork.h"

#include "evaExceptions.h"
#include <omp.h>

#include <vector>

eva::sbn::cen::CentralisedBasedResourceContainer &eva::sbn::cen::CentralisedBasedResourceContainer::operator=(const CentralisedBasedResourceContainer &other)
{
	if (this == &other)
		return *this;
	this->~CentralisedBasedResourceContainer();
	new (this) CentralisedBasedResourceContainer(other);
	return *this;
}

const bool eva::sbn::cen::CentralisedBasedResourceContainer::operator==(const CentralisedBasedResourceContainer &other) const
{
	if (include_distance)
		return Helper::compare_floats_equal(reducedCost, other.reducedCost) 
					&& distanceLastMaintenance == other.distanceLastMaintenance 
					&& timestamp == other.timestamp;
	else
		return Helper::compare_floats_equal(reducedCost, other.reducedCost) && timestamp == other.timestamp;
}

const bool eva::sbn::cen::CentralisedBasedResourceContainer::operator<(const CentralisedBasedResourceContainer &other) const
{
	if (other.isEndSchedule)
		// never smaller at the sink node:
		return false;

	if (include_distance)
	{
		if (eva::Helper::compare_floats_smaller(other.reducedCost, reducedCost))
			return false;

		if (Helper::compare_floats_equal(other.reducedCost, reducedCost))
		{
			if (distanceLastMaintenance > other.distanceLastMaintenance)
				return false;

			if (distanceLastMaintenance == other.distanceLastMaintenance)
			{
				return timestamp < other.timestamp;
			}
			else
				return timestamp <= other.timestamp;
		}
		else
		{
			return distanceLastMaintenance <= other.distanceLastMaintenance && timestamp <= other.timestamp;
			;
		}
	}
	else
	{
		if (eva::Helper::compare_floats_smaller(other.reducedCost, reducedCost))
			return false;

		if (Helper::compare_floats_equal(other.reducedCost, reducedCost))
		{
			return timestamp < other.timestamp;
		}

		return timestamp <= other.timestamp;
	}
}

bool eva::sbn::cen::CentralisedBasedResourceExtensionFunction::operator()(const BoostCentralisedBasedNetwork &boostSegmentNetwork, CentralisedBasedResourceContainer &new_cont, const CentralisedBasedResourceContainer &old_cont, const BoostCentralisedBasedArc &arc) const
{
	const CenNodeData &targetNodeData = boostSegmentNetwork[boost::target(arc, boostSegmentNetwork)];
	const CenArcData &arcData = boost::get(boost::edge_bundle, boostSegmentNetwork)[arc];

	// Step 1:
	// Check the access to the vertex:
	new_cont.isEndSchedule = (targetNodeData.type == CenNodeType::END_SCHEDULE);

	// Check access
	if (!arcData.has_access(_vehicle.get_index()) || !targetNodeData.has_access(_vehicle.get_index()))
		return false;

	// Check if the node is the sink node:
	// If yes, will be able to skip the dominance function. No dominance check on the sink node.
	if (targetNodeData.type == CenNodeType::END_SCHEDULE)
	{
		// Only if the previous container has unlocked the sink node, a label can be created here:
		if (old_cont.max_rc_start_time == Constants::MAX_TIMESTAMP)
		{
			// Schedule must have negative reduced cost. Otherwise, it is not relevant:
			return Helper::compare_floats_smaller(new_cont.reducedCost, 0.0);
		}
		else
		{
			// Sink is not open because the sink node is currently not the destination vertex.
			// This prevents the algorithm from finishing early, because sufficient labels have been found, without searching the entire tree.
			return false;
		}
	}

	// Check if the vertex is just a centralised charging vertex:
	if (targetNodeData.type == CenNodeType::CENTRALISED_CHARGING)
	{
		// Access has already been checked, so nothing more to do here:
		return true;
	}
	else if (targetNodeData.type == CenNodeType::FIXED_CHARGING)
	{
		// Access has already been checked, so just checking if the node is allowed at this point.
		// Check if the node start time is bigger than currently allowed:
		// If yes, the label cannot be further extended.
		if (targetNodeData.get_startTime() > old_cont.max_rc_start_time)
			return false;

		// If the node remains feasible, then:
		// Update the new container maximum time, if it matches the vehicle:
		// new_cont.timestamp = targetNodeData.get_endTime(); // not updating the timestamp, because the timestamp is already correct from the prev segment node.
		if (targetNodeData.get_has_fixed_activity(_vehicle.get_index()))
			new_cont.max_rc_start_time = targetNodeData.get_max_rc_start_time();

		// Now check if the end time lies beyond the new and updated start-time:
		// If this is bigger, can be pruned too.
		if (targetNodeData.get_endTime() > new_cont.max_rc_start_time)
			return false;

		return true; // Finally return true.
	}
	else if (targetNodeData.type == CenNodeType::SEGMENT)
	{
		// Access has already been checked, so just checking if the node is allowed at this point.
		// Check if the node start time is bigger than currently allowed:
		// If yes, the label cannot be further extended.
		if (targetNodeData.get_startTime() > old_cont.max_rc_start_time)
			return false;

		// If the node remains feasible, then:
		// Update the new container maximum time, if it matches the vehicle:
		if (targetNodeData.get_has_fixed_activity(_vehicle.get_index()))
			new_cont.max_rc_start_time = targetNodeData.get_max_rc_start_time();

		// Now check if the end time lies beyond the new and updated start-time:
		// If this is bigger, can be pruned too.
		if (targetNodeData.get_endTime() > new_cont.max_rc_start_time)
			return false;

		// Finally update the rest of the segment node:
		// 1. Update the SOC, and check if it's feasible:
		ChargingStrategy::Session session = _chargingStrategy.get_chargingSession(
			old_cont.timestamp,
			targetNodeData.get_startTime(),
			_vehicle,
			targetNodeData.get_startChargerIndex(),
			old_cont.soc,
			_vehicle.get_batteryDischarge(targetNodeData.get_distance()));

		if (session.is_feasible)
		{
			// If charging takes place, increase the soc:
			if (session.is_charging)
			{
				new_cont.reducedCost -= _duals.vecCumSumDualsChargerCapacity[targetNodeData.get_startChargerIndex()][session.index_putOnCharge][session.index_takeOffCharge];
			}
		}
		else
		{
			return false;
		}

		// 2. Update the reduced cost, cost and distance:
		new_cont.timestamp = targetNodeData.get_endTime();
		new_cont.distanceLastMaintenance = targetNodeData.get_updated_distance(old_cont.distanceLastMaintenance);
		double cost = targetNodeData.get_cost(old_cont.distanceLastMaintenance);
		new_cont.cost += cost;

		if (_include_cost)
			new_cont.reducedCost += cost - targetNodeData.get_accDuals();
		else
			new_cont.reducedCost -= targetNodeData.get_accDuals(); // Cost coefficients are excluded from computation.

		// finally, return true:
		return true;
	}

	// If any other node exists that goes here, it can't be feasible.
	return false;
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::_clearNetwork()
{
	// 1. Clean the boost network:
	_boostSegmentNetwork.clear();

	// 2. Clean local vectors:
	_vecVehicleStartNodes.erase(_vecVehicleStartNodes.begin(), _vecVehicleStartNodes.end());
	_vecSortedNodesChargerToCharger.erase(_vecSortedNodesChargerToCharger.begin(), _vecSortedNodesChargerToCharger.end());
	_vecCentralChargingNodes.erase(_vecCentralChargingNodes.begin(), _vecCentralChargingNodes.end());
	_vecIncludesTripNodes.erase(_vecIncludesTripNodes.begin(), _vecIncludesTripNodes.end());
	_vecIncludesMaintenanceNodes.erase(_vecIncludesMaintenanceNodes.begin(), _vecIncludesMaintenanceNodes.end());
	_mapIncludesScheduleNode.clear();
	_endNode = 0;

	// 3. Reset the index:
	_indexNode = 0;
	_indexArc = 0;
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::_addNodes(const BranchNode &brn, const Duals &duals)
{
	// 0. Initialisation:
	CenNodeData tmpNodeData;

	// 1. Add single endnode:
	tmpNodeData = CenNodeData(
		EndScheduleNodeData(),
		_optinput.get_vehicles().get_vec().size());
	_endNode = _addNode(tmpNodeData);

	// 2. Add a start node for every vehicle:
	_vecVehicleStartNodes.resize(_optinput.get_vehicles().get_vec().size());
	for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
	{
		const ScheduleResourceContainer &vp = _optinput.get_vehiclePosition(vehicle);
		const ScheduleNodeData &lastScheduleNodeData = _optinput.get_scheduleGraphNodeData(vp.lastScheduleNode);

		// Obtain the charger index:
		Types::Index indexCharger = Constants::BIG_INDEX;
		for (const Charger &charger : _optinput.get_chargers().get_vec())
		{
			if (charger.get_location().get_index() == lastScheduleNodeData.get_endLocation().get_index())
			{
				indexCharger = charger.get_index();
				break;
			}
		}
		if (indexCharger == Constants::BIG_INDEX)
			throw LogicError("eva::sbn::con::ConnectionBasedSegmentNetwork::create_reduced_graph", "Vehicle is not starting at a charger.");

		// Create the vehicle start schedule node:
		tmpNodeData = CenNodeData(
			StartScheduleNodeData(
				indexCharger,
				lastScheduleNodeData.get_endTime()),
			_optinput.get_vehicles().get_vec().size());
		tmpNodeData.set_access_all(Types::AccessType::NOT_ALLOWED);
		tmpNodeData.set_access_vehicle(vehicle.get_index(), Types::AccessType::ALLOWED);
		tmpNodeData.fix_vehicle(vehicle.get_index(), Constants::MAX_TIMESTAMP); // This is done in the branch node. All start nodes are fixed, and assume in the beginning that max_timestamp is allowed.
		_vecVehicleStartNodes[vehicle.get_index()] = _addNode(tmpNodeData);
	};

	// 3. Add the centralised charging vertices:
	_vecCentralChargingNodes.resize(_optinput.get_chargers().get_vec().size());
	for (const Charger &charger : _optinput.get_chargers().get_vec())
	{
		tmpNodeData = CenNodeData(
			CentralisedChargingNodeData(
				charger),
			_optinput.get_vehicles().get_vec().size());

		// Check which vehicles have access to the charger:
		for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
		{
			if (Helper::compare_floats_equal(vehicle.get_chargingSpeedKwS(charger), 0.0))
			{
				tmpNodeData.set_access_vehicle(vehicle.get_index(), Types::AccessType::NOT_ALLOWED);
			}
		}

		// Finally add the charging node:
		_vecCentralChargingNodes[charger.get_index()] = _addNode(tmpNodeData);
	}

// 4. Find all currently non-dominated paths:
// Step A: Solve the subgraphs:
// The OpenMP framework apparently only supports integer types in the for loop, not unsigned integers.
#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD
#pragma omp parallel for
	for (int32_t indexSegment = 0;
		 indexSegment < _segments.get_vec().size();
		 indexSegment++)
	{
		_segments.get_vec()[indexSegment].updateNonDominatedSchedulePieces(brn, duals);
	}

	// Step B: Add the non dominated schedule pieces:
	_vecSortedNodesChargerToCharger.resize(_optinput.get_chargers().get_vec().size(), std::vector<std::vector<std::vector<BoostCentralisedBasedNode>>>(_optinput.get_chargers().get_vec().size()));
	_vecIncludesTripNodes.resize(_optinput.get_vecTrips().size());
	_vecIncludesMaintenanceNodes.resize(_optinput.get_vecMaintenances().size());

	BoostCentralisedBasedNode tmpNode;
	auto iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.begin();
	std::vector<BoostCentralisedBasedNode> vecSegmentFragmentNodes;

	// Step B cont'd: Iterate all segments, and store the non dominated schedule pieces:
	for (const Segment &segment : _segments.get_vec())
	{
		// Iterate the non dominated schedule fragments:
		for (const auto &nonDomPiece : segment.get_vecNonDominatedSchedulePieces())
		{
			// Store the new node in the sorted location:
			tmpNodeData = CenNodeData(
				SegmentPieceNodeData(
					segment,
					nonDomPiece),
				_optinput.get_vehicles().get_vec().size());
			tmpNodeData.init_access(nonDomPiece.get_vecVehicleAccess()); // This does not yet fix the segment node. Only sets access.
			tmpNode = _addNode(tmpNodeData);
			vecSegmentFragmentNodes.push_back(tmpNode);

			// Store a reference to the node of all trip and maintenance nodes that it includes:
			for (const Types::Index indexTrip : nonDomPiece.get_vecTripIndexes())
				_vecIncludesTripNodes[indexTrip].push_back(tmpNode);

			for (const Types::Index indexMaintenance : nonDomPiece.get_vecMaintenanceIndexes())
				_vecIncludesMaintenanceNodes[indexMaintenance].push_back(tmpNode);

			// Store a reference to every schedule node in the segment node:
			for (const Types::Index scheduleNode : nonDomPiece.get_vecScheduleNodeIndexes())
			{
				iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.find(scheduleNode);
				if (iterMapIncludesScheduleNodes != _mapIncludesScheduleNode.end())
					iterMapIncludesScheduleNodes->second.push_back(tmpNode);
				else
					_mapIncludesScheduleNode.insert(std::make_pair(scheduleNode, std::vector<BoostScheduleNode>({tmpNode})));
			}
		}

		// Finally store the segment nodes:
		_vecSortedNodesChargerToCharger[segment.get_startCharger().get_index()][segment.get_endCharger().get_index()].push_back(vecSegmentFragmentNodes);
		vecSegmentFragmentNodes.clear();
	}
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::_addArcs(const BranchNode &brn, const Duals &duals)
{
	// Init:
	CenArcData arcData;

	// Connect the nodes:
	for (int32_t indexFromNode = 0;
		 indexFromNode < _indexNode;
		 ++indexFromNode)
	{
		if (_getNodeData(indexFromNode).type == CenNodeType::SEGMENT)
		{
			// Connect the start-charging node to the start:
			arcData = CenArcData(_optinput.get_vehicles().get_vec().size(), _vecCentralChargingNodes[_getNodeData(indexFromNode).get_startChargerIndex()], indexFromNode);
			_addArc(arcData);

			// Connect the end to the end-charging node:
			arcData = CenArcData(_optinput.get_vehicles().get_vec().size(), indexFromNode, _vecCentralChargingNodes[_getNodeData(indexFromNode).get_endChargerIndex()]);
			_addArc(arcData);

			// Connect the end to the sink node:
			if(_getNodeData(indexFromNode).is_flag_outgoing_to_sink_allowed())
			{
				arcData = CenArcData(_optinput.get_vehicles().get_vec().size(), indexFromNode, _endNode);
				_addArc(arcData);
			}
		}
		else if (_getNodeData(indexFromNode).type == CenNodeType::START_SCHEDULE)
		{
			// Connection is feasible:
			arcData = CenArcData(_optinput.get_vehicles().get_vec().size(), indexFromNode, _vecCentralChargingNodes[_getNodeData(indexFromNode).get_endChargerIndex()]);
			_addArc(arcData);
		}
	}
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::_addBranches(const BranchNode &brn, const Duals &duals)
{
	std::vector<std::vector<CentralisedBasedNodeFixings>> vecNodeFixings(_optinput.get_vehicles().get_vec().size());
	CenNodeData tmpNodeData;
	CenArcData tmpArcData;
	BoostCentralisedBasedNode tmpNode;
	auto iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.end();

	for (const Branch &branch : brn.get_vecBranches())
	{
		switch (branch.get_type())
		{
		case BranchType::VEHICLE_TRIP:
		{
			const BranchVehicleTrip *bvt = branch.castBranchVehicleTrip();

			if (branch.get_branchValueBool())
			{
				CentralisedBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = bvt->get_subTripNodeData().get_scheduleNodeData().get_startTime();
				nodeFixing.vec_fixed_nodes = _vecIncludesTripNodes[bvt->get_subTripNodeData().get_index()];
				vecNodeFixings[bvt->get_vehicle().get_index()].push_back(nodeFixing);
			}
			// ELSE: Access is already taken care of in the segment sub-graph
		}
		break;

		case BranchType::VEHICLE_MAINTENANCE:
		{
			const BranchVehicleMaintenance *bvm = branch.castBranchVehicleMaintenance();

			if (branch.get_branchValueBool())
			{
				CentralisedBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = bvm->get_subMaintenanceNodeData().get_scheduleNodeData().get_startTime();
				nodeFixing.vec_fixed_nodes = _vecIncludesMaintenanceNodes[bvm->get_subMaintenanceNodeData().get_index()];
				vecNodeFixings[bvm->get_vehicle().get_index()].push_back(nodeFixing);
			}
			// ELSE: Access is already taken care of in the segment sub-graph
		}
		break;

		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const BranchVehicleChargingAfter *bvca = branch.castBranchVehicleChargingAfter();

			if (branch.get_branchValueBool())
			{
				CentralisedBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _optinput.get_scheduleGraphNodeData(bvca->get_indexFromScheduleNode()).get_startTime();

				// Iterate all segment vertices that include the schdedule node:
				// They only exist with the schedule node in the end:
				iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.find(bvca->get_indexFromScheduleNode());
				if (iterMapIncludesScheduleNodes != _mapIncludesScheduleNode.end())
				{
					for (const auto &node : iterMapIncludesScheduleNodes->second)
					{
						if (_getNodeData(node).get_endChargerIndex() == bvca->get_charger().get_index() && _getNodeData(node).castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().back() == bvca->get_indexFromScheduleNode())
						{
							nodeFixing.vec_fixed_nodes.push_back(node);
							_getNodeData(node).set_flag_outgoing_to_sink_allowed(false); // A sink vertex must not follow, because charging has to take place.
						}
						else
						{
							throw LogicError("segment_cen::_addBranches()", "Segment node shouldn not exist. Does not end at the required charger, but vehicle_charging_after fixing exists.");
						}
					}
				}
				vecNodeFixings[bvca->get_vehicle().get_index()].push_back(nodeFixing);
			}
			// ELSE: Access is already taken care of in the segment sub-graph
		}
		break;

		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const BranchVehicleChargingBefore *bvcb = branch.castBranchVehicleChargingBefore();

			if (branch.get_branchValueBool())
			{
				CentralisedBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _optinput.get_scheduleGraphNodeData(bvcb->get_indexToScheduleNode()).get_startTime();

				// Iterate all segment vertices that include the schdedule node:
				// They only exist with the schedule node in the end:
				iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.find(bvcb->get_indexToScheduleNode());
				if (iterMapIncludesScheduleNodes != _mapIncludesScheduleNode.end())
				{
					for (const auto &node : iterMapIncludesScheduleNodes->second)
					{
						if (_getNodeData(node).get_startChargerIndex() == bvcb->get_charger().get_index() && _getNodeData(node).castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().front() == bvcb->get_indexToScheduleNode())
						{
							nodeFixing.vec_fixed_nodes.push_back(node);
						}
						else
						{
							throw LogicError("segment_cen::_addBranches()", "Segment node shouldn not exist. Does not end at the required charger, but vehicle_charging_before fixing exists.");
						}
					}
				}
				vecNodeFixings[bvcb->get_vehicle().get_index()].push_back(nodeFixing);
			}
			// ELSE: Access is already taken care of in the segment sub-graph
		}
		break;

		default:
			break;
		}
	}

	// Second, for all vehicles, sort their respective fixed nodes by start time.
	// Then, set the respective max_rc_times for each node:
	std::vector<BoostCentralisedBasedNode> vecPrevNodes;
	for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
	{
		std::sort(vecNodeFixings[vehicle.get_index()].begin(), vecNodeFixings[vehicle.get_index()].end(),
				  [&](const CentralisedBasedNodeFixings &l, const CentralisedBasedNodeFixings &r)
				  {
					  return l.fixed_start_time < r.fixed_start_time;
				  });

		vecPrevNodes.clear();
		vecPrevNodes.push_back(_vecVehicleStartNodes[vehicle.get_index()]);
		for (const CentralisedBasedNodeFixings &nodeFixing : vecNodeFixings[vehicle.get_index()])
		{
			// Update the time of all prev nodes:
			for (const auto &prevNode : vecPrevNodes)
				_getNodeData(prevNode).fix_vehicle(vehicle.get_index(), nodeFixing.fixed_start_time);

			// Update the current node:
			for (const auto &curNode : nodeFixing.vec_fixed_nodes)
				_getNodeData(curNode).fix_vehicle(vehicle.get_index(), Constants::MAX_TIMESTAMP);

			// Update the list of previous nodes:
			vecPrevNodes.clear();
			vecPrevNodes = nodeFixing.vec_fixed_nodes;
		}
	}
}

eva::sbn::cen::BoostCentralisedBasedNode eva::sbn::cen::CentralisedBasedSegmentNetwork::_addNode(CenNodeData &nodeData)
{
	nodeData.index = _getNextIndexNode();
	return boost::add_vertex(nodeData, _boostSegmentNetwork);
}

eva::sbn::cen::BoostCentralisedBasedArc eva::sbn::cen::CentralisedBasedSegmentNetwork::_addArc(CenArcData &arcData)
{
	arcData.index = _getNextIndexArc();

	return boost::add_edge(arcData.get_fromNode(), arcData.get_toNode(), arcData, _boostSegmentNetwork).first;
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::initialise()
{
	_segments.initialise(_optinput);
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::create_reduced_graph(const Duals &duals, const BranchNode &brn)
{
	// 0. Clean-up the network:
	_clearNetwork();

	// Important: Order: nodes -> branches -> arcs 
	_addNodes(brn, duals);
	_addBranches(brn, duals);
	_addArcs(brn, duals);
}

std::vector<eva::SubVehicleSchedule> eva::sbn::cen::CentralisedBasedSegmentNetwork::find_neg_reduced_cost_schedule_vehicle(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal, const std::chrono::high_resolution_clock::time_point &timeOutClock)
{
	BoostCentralisedBasedNode sourceVertex = _vecVehicleStartNodes[vehicle.get_index()];
	BoostCentralisedBasedNode sinkVertex = _endNode;

	const ScheduleResourceContainer &curVehiclePosition = _optinput.get_vehiclePosition(vehicle);
	CentralisedBasedResourceContainer initialResourceContainer(
		0.0,
		0.0 - duals.vecDualsOneSchedulePerVehicle[vehicle.get_index()],
		_optinput.get_scheduleGraphNodeData(curVehiclePosition.lastScheduleNode).get_endTime(),
		curVehiclePosition.odometerReading - curVehiclePosition.odometerLastMaintenance,
		curVehiclePosition.soc,
		_optinput.get_flag_has_unassigned_maintenance(),
		_getNodeData(sourceVertex).get_max_rc_start_time());

	// Solve the resource constraint shortest path problem:
	CentralisedBasedResourceConstraintPaths shortestPaths;
	boost::r_c_shortest_paths(
		_boostSegmentNetwork,
		boost::get(&CenNodeData::index, _boostSegmentNetwork),
		boost::get(&CenArcData::index, _boostSegmentNetwork),
		sourceVertex,
		sinkVertex,
		shortestPaths.pareto_optimal_solutions,
		shortestPaths.pareto_optimal_resource_containers,
		initialResourceContainer,
		CentralisedBasedResourceExtensionFunction(duals, vehicle, _optinput, include_cost),
		CentralisedBasedDominanceCheck(solve_to_optimal, _optinput.get_flag_has_unassigned_maintenance()),
		boost::default_r_c_shortest_paths_allocator(),
		CentralisedBasedResourceExtensionVisitor(_optinput.get_config().get_const_nr_cols_per_vehicle_iter(), solve_to_optimal, timeOutClock));

	// If less than the maximum number of labels are returned, the labelling algorithm must have processed all labels - hence, the subpath is explored optimally. Else, not:
	if ((!solve_to_optimal && shortestPaths.pareto_optimal_solutions.size() >= _optinput.get_config().get_const_nr_cols_per_vehicle_iter()) || std::chrono::high_resolution_clock::now() >= timeOutClock)
	{
		// Indicate that the labelling algorithm for this vehicle was not solved to optimality. Can only be set to false.
		isSolvedOptimal = false;
	}

	// __________________
	// Store the results:
	std::vector<SubVehicleSchedule> result;
	ChargingStrategy::Session chargingSession;
	double cost, reducedCost;
	Types::Index prevScheduleNode = Constants::BIG_INDEX;
	Types::DateTime prevEndTime = Constants::MAX_TIMESTAMP;
	Types::BatteryCharge initSoc = curVehiclePosition.soc;

	for (Types::Index indexResult = 0; indexResult < shortestPaths.pareto_optimal_resource_containers.size(); indexResult++)
	{
		cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
		reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

		if (Helper::compare_floats_smaller(reducedCost, 0))
		{
			SubVehicleSchedule svs;

			svs.indexVehicle = vehicle.get_index();
			svs.indexStartLocation = _optinput.get_charger(_getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].back(), _boostSegmentNetwork)).get_startChargerIndex()).get_location().get_index();
			svs.indexEndLocation = _optinput.get_charger(_getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].front(), _boostSegmentNetwork)).get_endChargerIndex()).get_location().get_index();
			;
			svs.cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
			svs.reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

			prevScheduleNode = Constants::BIG_INDEX;
			prevEndTime = _optinput.get_scheduleGraphNodeData(curVehiclePosition.lastScheduleNode).get_endTime();

			for (auto iterArc = shortestPaths.pareto_optimal_solutions[indexResult].rbegin(); iterArc != shortestPaths.pareto_optimal_solutions[indexResult].rend(); ++iterArc)
			{
				const CenNodeData &targetNodeData = _getNodeData(boost::target(*iterArc, _boostSegmentNetwork));

				if (targetNodeData.type == CenNodeType::SEGMENT)
				{
					const SegmentPieceNodeData *targetSegmentPieceNodeData = targetNodeData.castSegmentPieceNodeData();

					chargingSession = _chargingStrategy.get_chargingSession(
						prevEndTime,
						targetNodeData.get_startTime(),
						vehicle,
						targetNodeData.get_startChargerIndex(),
						initSoc,
						vehicle.get_batteryDischarge(targetNodeData.get_distance()));

					if (!chargingSession.is_feasible)
						throw LogicError("eva::sbn::cen::CentralisedBasedSegmentNetwork::find_neg_reduced_cost_schedule_vehicle", "Charging Session infeasible when reading results!");

					if (chargingSession.is_charging)
					{
						ChargingSchedule cs;
						cs.indexCharger = targetNodeData.get_startChargerIndex();
						cs.indexPutOnCharge = chargingSession.index_putOnCharge;
						cs.indexTakeOffCharge = chargingSession.index_takeOffCharge;
						cs.indexFromScheduleNode = prevScheduleNode;
						cs.indexToScheduleNode = targetNodeData.castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().front();
						svs.vecChargingSchedule.push_back(cs);

						// Update the charging session and add to keep track:
						svs.vecScheduleNodes.push_back(_optinput.get_putOnCharge(targetNodeData.get_startChargerIndex(), chargingSession.index_putOnCharge).get_scheduleNodeData().get_index());
						svs.vecScheduleNodes.push_back(_optinput.get_takeOffCharge(targetNodeData.get_startChargerIndex(), chargingSession.index_takeOffCharge).get_scheduleNodeData().get_index());
					}

					// Update the remainining information of all trips and maintenances visited on the targetnode:
					const SegmentPieceNodeData *segmentPieceNodeData = targetNodeData.castSegmentPieceNodeData();
					svs.vecTripNodeIndexes.insert(svs.vecTripNodeIndexes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecTripIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecTripIndexes().end());
					svs.vecMaintenanceNodesIndexes.insert(svs.vecMaintenanceNodesIndexes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecMaintenanceIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecMaintenanceIndexes().end());
					svs.vecScheduleNodes.insert(svs.vecScheduleNodes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().end());

					prevScheduleNode = targetSegmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().back();
					prevEndTime = targetNodeData.get_endTime();
				}
			}

			result.push_back(svs);
		}
	}

	return result;
}

void eva::sbn::cen::CentralisedBasedSegmentNetwork::update_branch_node_fixings(const BranchNode &brn)
{
	// Iterate over all segments, and update the vehicle fixings:
#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD
#pragma omp parallel for
	for (int32_t indexSegment = 0;
		 indexSegment < _segments.get_vec().size();
		 indexSegment++)
	{
		_segments.get_vec()[indexSegment].updateVehicleFixings(brn);
	}
}
