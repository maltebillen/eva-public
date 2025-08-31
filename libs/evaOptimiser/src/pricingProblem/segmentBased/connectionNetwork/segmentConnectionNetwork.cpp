#include "incl/pricingProblem/segmentBased/connectionNetwork/segmentConnectionNetwork.h"

#include "evaExceptions.h"
#include <omp.h>

#include <vector>

void eva::sbn::con::ConnectionBasedSegmentNetwork::_clearNetwork()
{
	// 1. Clean the boost network:
	_boostSegmentNetwork.clear();

	// 2. Clean local vectors:
	_vecVehicleStartNodes.erase(_vecVehicleStartNodes.begin(), _vecVehicleStartNodes.end());
	_vecSortedNodesChargerToCharger.erase(_vecSortedNodesChargerToCharger.begin(), _vecSortedNodesChargerToCharger.end());
	_vecIncludesTripNodes.erase(_vecIncludesTripNodes.begin(), _vecIncludesTripNodes.end());
	_vecIncludesMaintenanceNodes.erase(_vecIncludesMaintenanceNodes.begin(), _vecIncludesMaintenanceNodes.end());
	_mapIncludesScheduleNode.clear();
	_endNode = 0;

	// 3. Reset the index:
	_indexNode = 0;
	_indexArc = 0;
}

void eva::sbn::con::ConnectionBasedSegmentNetwork::_addNodes(const BranchNode& brn, const Duals& duals)
{
	// 0. Initialisation:
	ConNodeData tmpNodeData;

	// 1. Add single endnode:
	tmpNodeData = ConNodeData(
		EndScheduleNodeData(),
		_optinput.get_vehicles().get_vec().size());
	_endNode = _addNode(tmpNodeData);

	// 2. Add one start node for every vehicle:
	_vecVehicleStartNodes.resize(_optinput.get_vehicles().get_vec().size());
	for (const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
	{
		const ScheduleResourceContainer& vp = _optinput.get_vehiclePosition(vehicle);
		const ScheduleNodeData& lastScheduleNodeData = _optinput.get_scheduleGraphNodeData(vp.lastScheduleNode);

		// Obtain the charger index:
		Types::Index indexCharger = Constants::BIG_INDEX;
		for (const Charger& charger : _optinput.get_chargers().get_vec())
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
		tmpNodeData = ConNodeData(
			StartScheduleNodeData(
				indexCharger,
				lastScheduleNodeData.get_endTime()
			),
			_optinput.get_vehicles().get_vec().size()
		);
		tmpNodeData.set_access_all(Types::AccessType::NOT_ALLOWED);
		tmpNodeData.set_access_vehicle(vehicle.get_index(), Types::AccessType::ALLOWED);
		tmpNodeData.fix_vehicle(vehicle.get_index(), Constants::MAX_TIMESTAMP); // This is done in the branch node. All start nodes are fixed, and assume in the beginning that max_timestamp is allowed.
		_vecVehicleStartNodes[vehicle.get_index()] = _addNode(tmpNodeData);
	}

	// 3. Find all currently non-dominated paths:
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
	_vecSortedNodesChargerToCharger.resize(_optinput.get_chargers().get_vec().size(), std::vector<std::vector<std::vector<BoostConnectionBasedNode>>>(_optinput.get_chargers().get_vec().size()));
	_vecIncludesTripNodes.resize(_optinput.get_vecTrips().size());
	_vecIncludesMaintenanceNodes.resize(_optinput.get_vecMaintenances().size());
	
	BoostConnectionBasedNode tmpNode;
	std::vector<eva::sbn::con::BoostConnectionBasedNode>::iterator iterPosition;
	auto iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.begin();
	std::vector<BoostConnectionBasedNode> vecSegmentFragmentNodes;

	// Step B cont'd: Iterate all segments, and store the non dominated schedule pieces:
	for (const Segment& segment : _segments.get_vec())
	{
		// Iterate the non dominated schedule fragments:
		for (const auto& nonDomPiece : segment.get_vecNonDominatedSchedulePieces())
		{
			// Find position in time-sorted vector, and store the node:
			iterPosition = std::lower_bound(
				vecSegmentFragmentNodes.begin(),
				vecSegmentFragmentNodes.end(),
				nonDomPiece.get_endTime(),
				[&](const BoostConnectionBasedNode& l, Types::DateTime value)
				{
					return _getNodeData(l).get_endTime() < value;
				});

			// Store the new node in the sorted location:
			tmpNodeData = ConNodeData(
				SegmentPieceNodeData(
					segment,
					nonDomPiece
				),
				_optinput.get_vehicles().get_vec().size()
			);
			tmpNodeData.init_access(nonDomPiece.get_vecVehicleAccess()); // This does not yet fix the segment node. Only sets access.
			tmpNode = _addNode(tmpNodeData);
			vecSegmentFragmentNodes.insert(iterPosition, tmpNode);

			// Finally update the look-ups:
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

void eva::sbn::con::ConnectionBasedSegmentNetwork::_addArcs(const BranchNode& brn, const Duals& duals)
{
	// Run the following functions parallel:
	// Function is logically only called after the nodes are added to the network:
	std::vector<std::vector<FullConArcData>> vecArcsOutgoing(_indexNode); // Store the non-dominated nodes:

	// The OpenMP framework apparently only supports integer types in the for loop, not unsigned integers.
#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD
#pragma omp parallel for
	for (int32_t indexFromNode = 0;
		 indexFromNode < _indexNode;
		 indexFromNode++)
	{
		// Add all outgoing edges from segment nodes and start nodes:
			if (_getNodeData(indexFromNode).type == ConNodeType::SEGMENT || _getNodeData(indexFromNode).type == ConNodeType::START_SCHEDULE)
			{
				for (const Charger &toEndCharger : _optinput.get_chargers().get_vec())
				{
					for (const std::vector<BoostConnectionBasedNode> &vecSegmentNodes : _vecSortedNodesChargerToCharger[_getNodeData(indexFromNode).get_endChargerIndex()][toEndCharger.get_index()])
					{
						// Create a new vector for all edges to the segment node:
						std::vector<FullConArcData> vecNonDominatedArcs;

						// Iterate all nodes starting from the same node of the startNode, and finishing at endCharger:
						for (const auto &toNode : vecSegmentNodes)
						{
							// Function checks if the toNode is dominated
							if (_isFeasible(indexFromNode, toNode))
							{
								// Create the arcData object:
								FullConArcData arcData = _createArcData(indexFromNode, toNode, duals);

								// Check if there is at least one vehicle allowed on the arc:
								if (arcData.is_feasible())
								{
									if (!_isDominated(vecNonDominatedArcs, arcData))
										vecNonDominatedArcs.push_back(arcData);
								}
							}
						}

						// Store the non-dominated arcs:
						vecArcsOutgoing[indexFromNode].insert(vecArcsOutgoing[indexFromNode].end(), vecNonDominatedArcs.begin(), vecNonDominatedArcs.end());
					}
				}
			}

			// At last, sort the outgoing arcs based on being most promising:
			// Using an estimate on the reduced cost to determine the order.
			std::sort(vecArcsOutgoing[indexFromNode].begin(), vecArcsOutgoing[indexFromNode].end(), [](const FullConArcData &l, const FullConArcData &r)
					  { return Helper::compare_floats_smaller(
							l.bestCaseCost - (l.accDuals + l.maxChargingDuals),
							r.bestCaseCost - (r.accDuals + r.maxChargingDuals)); });

			// Finally, add one edge to the end-node:
			if (_getNodeData(indexFromNode).type == ConNodeType::SEGMENT
				&& _getNodeData(indexFromNode).is_flag_outgoing_to_sink_allowed())
			{
				vecArcsOutgoing[indexFromNode].push_back(_createArcData(indexFromNode, _endNode, duals));
			}
	}

	// Finally, add all arcs to the boost network:
	for (auto& vecNonDominatedArcs : vecArcsOutgoing)
	{
		for (auto& arcData : vecNonDominatedArcs)
		{
			_addArc(arcData);
		}
	}
}

void eva::sbn::con::ConnectionBasedSegmentNetwork::_addBranches(const BranchNode& brn, const Duals& duals)
{
	std::vector<std::vector<ConnectionBasedNodeFixings>> vecNodeFixings(_optinput.get_vehicles().get_vec().size());
	ConNodeData tmpNodeData;
	ConArcData tmpArcData;

	BoostConnectionBasedNode tmpNode;
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
				ConnectionBasedNodeFixings nodeFixing;
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
				ConnectionBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = bvm->get_subMaintenanceNodeData().get_scheduleNodeData().get_startTime();
				nodeFixing.vec_fixed_nodes = _vecIncludesMaintenanceNodes[bvm->get_subMaintenanceNodeData().get_index()];
				vecNodeFixings[bvm->get_vehicle().get_index()].push_back(nodeFixing);
			}
			// ELSE: Access is already taken care of in the segment sub-graph
		}
		break;

		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const BranchVehicleChargingAfter* bvca = branch.castBranchVehicleChargingAfter();

			if(branch.get_branchValueBool())
			{
				ConnectionBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _optinput.get_scheduleGraphNodeData(bvca->get_indexFromScheduleNode()).get_startTime();

				// Iterate all segment vertices that include the schdedule node:
				// They only exist with the schedule node in the end:
				iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.find(bvca->get_indexFromScheduleNode());
				if (iterMapIncludesScheduleNodes != _mapIncludesScheduleNode.end()) {
					for (const auto& node : iterMapIncludesScheduleNodes->second)
					{
						if (_getNodeData(node).get_endChargerIndex() == bvca->get_charger().get_index()
							&& _getNodeData(node).castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().back() == bvca->get_indexFromScheduleNode())
						{
							nodeFixing.vec_fixed_nodes.push_back(node);
							_getNodeData(node).set_flag_outgoing_to_sink_allowed(false); // A sink vertex must not follow, because charging has to take place.
						}
						else
						{
							throw LogicError("segment_con::_addBranches()","Segment node shouldn not exist. Does not end at the required charger, but vehicle_charging_after fixing exists.");
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
			const BranchVehicleChargingBefore* bvcb = branch.castBranchVehicleChargingBefore();

			if(branch.get_branchValueBool())
			{
				ConnectionBasedNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _optinput.get_scheduleGraphNodeData(bvcb->get_indexToScheduleNode()).get_startTime();

				// Iterate all segment vertices that include the schdedule node:
				// They only exist with the schedule node in the end:
				iterMapIncludesScheduleNodes = _mapIncludesScheduleNode.find(bvcb->get_indexToScheduleNode());
				if (iterMapIncludesScheduleNodes != _mapIncludesScheduleNode.end()) {
					for (const auto& node : iterMapIncludesScheduleNodes->second)
					{
						if (_getNodeData(node).get_startChargerIndex() == bvcb->get_charger().get_index()
							&& _getNodeData(node).castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().front() == bvcb->get_indexToScheduleNode())
						{
							nodeFixing.vec_fixed_nodes.push_back(node);
						}
						else
						{
							throw LogicError("segment_con::_addBranches()","Segment node shouldn not exist. Does not end at the required charger, but vehicle_charging_before fixing exists.");
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
	std::vector<BoostConnectionBasedNode> vecPrevNodes;
	for (const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
	{
		std::sort(vecNodeFixings[vehicle.get_index()].begin(), vecNodeFixings[vehicle.get_index()].end(),
				  [&](const ConnectionBasedNodeFixings &l, const ConnectionBasedNodeFixings &r)
				  {
					  return l.fixed_start_time < r.fixed_start_time;
				  });
		
		vecPrevNodes.clear();
		vecPrevNodes.push_back(_vecVehicleStartNodes[vehicle.get_index()]);
		for (const ConnectionBasedNodeFixings& nodeFixing : vecNodeFixings[vehicle.get_index()]) {
			// Update the time of all prev nodes:
			for(const auto& prevNode : vecPrevNodes)
				_getNodeData(prevNode).fix_vehicle(vehicle.get_index(), nodeFixing.fixed_start_time);
			
			// Update the current node:
			for(const auto& curNode : nodeFixing.vec_fixed_nodes)
				_getNodeData(curNode).fix_vehicle(vehicle.get_index(), Constants::MAX_TIMESTAMP);
			
			// Update the list of previous nodes:
			vecPrevNodes.clear();
			vecPrevNodes = nodeFixing.vec_fixed_nodes;
		}
	}
}

eva::sbn::con::BoostConnectionBasedNode eva::sbn::con::ConnectionBasedSegmentNetwork::_addNode(ConNodeData& nodeData)
{
	nodeData.index = _getNextIndexNode();
	return boost::add_vertex(nodeData, _boostSegmentNetwork);
}

eva::sbn::con::BoostConnectionBasedArc eva::sbn::con::ConnectionBasedSegmentNetwork::_addArc(ConArcData& arcData)
{
	arcData.index = _getNextIndexArc();

	return boost::add_edge(arcData.fromNode, arcData.toNode, arcData, _boostSegmentNetwork).first;
}

eva::sbn::con::BoostConnectionBasedArc eva::sbn::con::ConnectionBasedSegmentNetwork::_addArc(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode)
{
	ConArcData tmpArcData(_optinput.get_vehicles().get_vec().size());
	tmpArcData.fromNode = fromNode;
	tmpArcData.toNode = toNode;
	tmpArcData.index = _getNextIndexArc();

    return boost::add_edge(tmpArcData.fromNode, tmpArcData.toNode, tmpArcData, _boostSegmentNetwork).first;
}

eva::sbn::con::FullConArcData eva::sbn::con::ConnectionBasedSegmentNetwork::_createArcData(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode, const Duals &duals)
{
	FullConArcData arcData(_optinput.get_vehicles().get_vec().size());
	arcData.fromNode = fromNode;
	arcData.toNode = toNode;

	// Compute node access:
	// Intersection of from Node + to Node:
	for(Types::Index idxVehicle = 0; idxVehicle < _getNodeData(fromNode).get_vecAccess().size(); ++idxVehicle)
	{
		// First, check the intersection.
		if(!_getNodeData(fromNode).has_access(idxVehicle) || !_getNodeData(toNode).has_access(idxVehicle))
			arcData.revoke_access(idxVehicle);
	}

	// Check if the arc is feasible:
	if(arcData.is_feasible())
	{
		// Check if there is a fixed vehicle on either side:
		if(_getNodeData(toNode).get_indexFixedVehicle() != Constants::BIG_INDEX)
			arcData.fix_vehicle(_getNodeData(toNode).get_indexFixedVehicle());

		// Finally, compute the rest:
		if (_getNodeData(toNode).type == ConNodeType::SEGMENT)
		{
			const SegmentPieceNodeData *sinkNodeData = _getNodeData(toNode).castSegmentPieceNodeData();

			arcData.accDuals = sinkNodeData->get_accDuals();
			arcData.bestCaseCost = sinkNodeData->get_cost(_getNodeData(fromNode).get_updated_distance(0));
			arcData.minChargingDuals = sinkNodeData->get_nonDominatedSchedulePiece().get_minChargingDuals();
			arcData.maxChargingDuals = sinkNodeData->get_nonDominatedSchedulePiece().get_maxChargingDuals();
			arcData.sinkEndTime = sinkNodeData->get_endTime();

			// Important to use the full presence at the charger as the bound. Otherwise, it cannot be guarenteed that there actually is sufficient time for all vehicles!!!
			// Because we know that no vehicle will be at the charger longer than this upper bound!
			arcData.chargingDuration = std::min(Helper::diffDateTime(_getNodeData(fromNode).get_endTime(), sinkNodeData->get_startTime()), static_cast<int64_t>(sinkNodeData->get_segment().get_maxub_fullPresenceAtCharger()));
		}
	}

	return arcData;
}

const bool eva::sbn::con::ConnectionBasedSegmentNetwork::_isDominated(std::vector<FullConArcData>& vecOutgoingArcs, const FullConArcData& candidateArc)
{
	// Iterate over all arcs and check if the node 
	auto iterDominatingArc = vecOutgoingArcs.begin();
	while (iterDominatingArc != vecOutgoingArcs.end())
	{

#ifdef DEBUG_BUILD
		if (iterDominatingArc->fromNode != candidateArc.fromNode)
			throw LogicError("eva::sbn::con::ConnectionBasedSegmentNetwork::_checkOutgoingDominance", "FromNode not identical!");
#endif // DEBUG_BUILD

		// a. Check if the new arc is dominated:
		// If the dominating arc dominates based on the outgoing arc set, return true:
		if (iterDominatingArc->dominates(candidateArc))
			return true;

		// b. Check if the arc dominates a current dominating arc:
		else if (candidateArc.dominates(*iterDominatingArc))
		{
			iterDominatingArc = vecOutgoingArcs.erase(iterDominatingArc);
		}
		else
		{
			++iterDominatingArc;
		}
	}

	return false;
}

const bool eva::sbn::con::ConnectionBasedSegmentNetwork::_isFeasible(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode)
{
	// Check if the connection is feasible space/and time-wise:
#ifdef DEBUG_BUILD
	// In debug, check the full information, including space:

	// Check that the fromEndCharger = ToStartCharger:
	// Should be already verified by the calling function:
	if (_getNodeData(fromNode).get_endChargerIndex() != _getNodeData(toNode).get_startChargerIndex())
		return false;

#endif // DEBUG_BUILD


	// Check that there is at least sufficient time to minimum charge a vehicle:	
	int64_t timediff = Helper::diffDateTime(_getNodeData(fromNode).get_endTime(), _getNodeData(toNode).get_startTime());
	if (timediff < 0)
		return false;

	// At least one of the vehicles that is feasible on both nodes must have sufficient time to charge:
	ChargingStrategy::Session session;
	if(_getNodeData(toNode).get_type() == ConNodeType::SEGMENT)
	{
		for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
		{
			if (_getNodeData(fromNode).has_access(vehicle.get_index()) && _getNodeData(toNode).has_access(vehicle.get_index()))
			{
				// Check if there is sufficient time charge at least one vehicle that has access:
				session = _chargingStrategy.get_chargingSession(
					_getNodeData(fromNode).get_endTime(),
					_getNodeData(toNode).get_startTime(),
					vehicle,
					_getNodeData(toNode).get_startChargerIndex(),
					vehicle.get_batteryMinKWh(),
					vehicle.get_batteryDischarge(_getNodeData(toNode).get_distance()));

				// if the session is feasible, no further checks required because there is at least one vehicle that has a feasible charging session
				if(session.is_feasible)
					return true;
			}
		}
	}else
	{
		return true;
	}

	return false;
}

void eva::sbn::con::ConnectionBasedSegmentNetwork::initialise()
{
	_segments.initialise(_optinput);
}

void eva::sbn::con::ConnectionBasedSegmentNetwork::create_reduced_graph(const Duals& duals, const BranchNode& brn)
{
	// 0. Clean-up the network:
	_clearNetwork();

	// Important: Order: nodes -> branches -> arcs!!!
	_addNodes(brn, duals);
	_addBranches(brn, duals);
	_addArcs(brn, duals);
}

std::vector<eva::SubVehicleSchedule> eva::sbn::con::ConnectionBasedSegmentNetwork::find_neg_reduced_cost_schedule_vehicle(const Duals& duals, const Vehicle& vehicle, const BranchNode& brn, const bool include_cost, const bool solve_to_optimal, bool& isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
	BoostConnectionBasedNode sourceVertex = _vecVehicleStartNodes[vehicle.get_index()];
	BoostConnectionBasedNode sinkVertex = _endNode;

	const ScheduleResourceContainer &curVehiclePosition = _optinput.get_vehiclePosition(vehicle);
	ConnectionBasedResourceContainer initialResourceContainer(
		0.0,
		0.0 - duals.vecDualsOneSchedulePerVehicle[vehicle.get_index()],
		_optinput.get_scheduleGraphNodeData(curVehiclePosition.lastScheduleNode).get_endTime(),
		curVehiclePosition.odometerReading - curVehiclePosition.odometerLastMaintenance,
		curVehiclePosition.soc,
		_optinput.get_flag_has_unassigned_maintenance(),
		_getNodeData(sourceVertex).get_max_rc_start_time());
	
	// Solve the resource constraint shortest path problem:
	ConnectionBasedResourceConstraintPaths shortestPaths;
	boost::r_c_shortest_paths(
					_boostSegmentNetwork,
					boost::get(&ConNodeData::index, _boostSegmentNetwork),
					boost::get(&ConArcData::index, _boostSegmentNetwork),
					sourceVertex,
					sinkVertex,
					shortestPaths.pareto_optimal_solutions,
					shortestPaths.pareto_optimal_resource_containers,
					initialResourceContainer,
					ConnectionBasedResourceExtensionFunction(duals, vehicle, _optinput, include_cost),
					ConnectionBasedDominanceCheck(solve_to_optimal, _optinput.get_flag_has_unassigned_maintenance()),
					boost::default_r_c_shortest_paths_allocator(),
					ConnectionBasedResourceExtensionVisitor(_optinput.get_config().get_const_nr_cols_per_vehicle_iter(),solve_to_optimal, timeOutClock)
				);
	
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

	for (Types::Index indexResult = 0; indexResult < shortestPaths.pareto_optimal_resource_containers.size(); indexResult++)
	{
		cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
		reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

		if (Helper::compare_floats_smaller(reducedCost, 0))
		{
			SubVehicleSchedule svs;

			svs.indexVehicle = vehicle.get_index();
			svs.indexStartLocation = _optinput.get_charger(_getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].back(), _boostSegmentNetwork)).get_startChargerIndex()).get_location().get_index();
			svs.indexEndLocation = _optinput.get_charger(_getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].front(), _boostSegmentNetwork)).get_endChargerIndex()).get_location().get_index();;
			svs.cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
			svs.reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

			Types::BatteryCharge soc = curVehiclePosition.soc;
			prevScheduleNode = Constants::BIG_INDEX;

			for (auto iterArc = shortestPaths.pareto_optimal_solutions[indexResult].rbegin(); iterArc != shortestPaths.pareto_optimal_solutions[indexResult].rend(); ++iterArc)
			{
				const ConNodeData& sourceNodeData = _getNodeData(boost::source(*iterArc, _boostSegmentNetwork));
				const ConNodeData& targetNodeData = _getNodeData(boost::target(*iterArc, _boostSegmentNetwork));

				if (targetNodeData.type == ConNodeType::SEGMENT)
				{
					chargingSession = _chargingStrategy.get_chargingSession(
						sourceNodeData.get_endTime(),
						targetNodeData.get_startTime(),
						vehicle,
						targetNodeData.get_startChargerIndex(),
						soc,
						vehicle.get_batteryDischarge(targetNodeData.get_distance()));
				
					// Stop charging when the vehicle is full. Then update the following battery discharge:
					// Only charge when there actually is charging taking place between the two activities.
					if(chargingSession.is_charging)
					{
						soc = std::min(vehicle.get_batteryMaxKWh(), soc + chargingSession.get_charge(_optinput, targetNodeData.get_startChargerIndex()));

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

					// Discharge the vehicle by the amount of the next node:
					soc -= vehicle.get_batteryDischarge(targetNodeData.get_distance());
					
					// Update the remainining information of all trips and maintenances visited on the targetnode:
					const SegmentPieceNodeData* segmentPieceNodeData = targetNodeData.castSegmentPieceNodeData();
					svs.vecTripNodeIndexes.insert(svs.vecTripNodeIndexes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecTripIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecTripIndexes().end());
					svs.vecMaintenanceNodesIndexes.insert(svs.vecMaintenanceNodesIndexes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecMaintenanceIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecMaintenanceIndexes().end());
					svs.vecScheduleNodes.insert(svs.vecScheduleNodes.end(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().begin(), segmentPieceNodeData->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().end());
				}

				prevScheduleNode = targetNodeData.type == ConNodeType::SEGMENT ? targetNodeData.castSegmentPieceNodeData()->get_nonDominatedSchedulePiece().get_vecScheduleNodeIndexes().back() : prevScheduleNode;
			}

			result.push_back(svs);
		}
	}

	return result;
}

void eva::sbn::con::ConnectionBasedSegmentNetwork::update_branch_node_fixings(const BranchNode &brn)
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

eva::sbn::con::ConnectionBasedResourceContainer &eva::sbn::con::ConnectionBasedResourceContainer::operator=(const ConnectionBasedResourceContainer &other)
{
	if (this == &other)
		return *this;
	this->~ConnectionBasedResourceContainer();
	new(this) ConnectionBasedResourceContainer(other);
	return *this;
}

const bool eva::sbn::con::ConnectionBasedResourceContainer::operator==(const ConnectionBasedResourceContainer &other) const
{
	if (include_distance)
		return Helper::compare_floats_equal(reducedCost, other.reducedCost)
					&& distanceLastMaintenance == other.distanceLastMaintenance;
	else	
		return Helper::compare_floats_equal(reducedCost, other.reducedCost);
}

const bool eva::sbn::con::ConnectionBasedResourceContainer::operator<(const ConnectionBasedResourceContainer &other) const
{
	if (other.isEndSchedule)
		// never smaller at the sink node:
		return false;

	if (include_distance)
	{
		if (eva::Helper::compare_floats_smaller(other.reducedCost, reducedCost))
			return false;

		if (Helper::compare_floats_equal(other.reducedCost, reducedCost))
			return distanceLastMaintenance < other.distanceLastMaintenance;
		
		// Then reduced cost must be smaller, but distance must be at least smaller or equal as well:
		return distanceLastMaintenance <= other.distanceLastMaintenance;
	}
	else
	{
		return Helper::compare_floats_smaller(reducedCost, other.reducedCost);
	}
}

bool eva::sbn::con::ConnectionBasedResourceExtensionFunction::operator()(const BoostConnectionBasedNetwork &boostSegmentNetwork, ConnectionBasedResourceContainer &new_cont, const ConnectionBasedResourceContainer &old_cont, const BoostConnectionBasedArc &arc) const
{
	const ConNodeData& sourceNodeData = boostSegmentNetwork[boost::source(arc, boostSegmentNetwork)];
	const ConNodeData& targetNodeData = boostSegmentNetwork[boost::target(arc, boostSegmentNetwork)];
	const ConArcData& arcData = boost::get(boost::edge_bundle, boostSegmentNetwork)[arc];

	// Step 1:
	// Check the access to the vertex:
	new_cont.isEndSchedule = (targetNodeData.type == ConNodeType::END_SCHEDULE);

	// Check if the node is the sink node:
	// If yes, will be able to skip the dominance function. No dominance check on the sink node.
	if (targetNodeData.type == ConNodeType::END_SCHEDULE)
	{
		// Only if the previous container has unlocked the sink node, a label can be created here:
		if(old_cont.max_rc_start_time == Constants::MAX_TIMESTAMP)
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

	// Check access
	if (!arcData.has_access(_vehicle.get_index())
		|| !targetNodeData.has_access(_vehicle.get_index()))
		return false;

	// Check if the node start time is bigger than currently allowed:
	// If yes, the label cannot be further extended.
	if (targetNodeData.get_startTime() > old_cont.max_rc_start_time)
		return false;

	// Update the new container maximum time, if it matches the vehicle:
	if (targetNodeData.get_has_fixed_activity(_vehicle.get_index()))
		new_cont.max_rc_start_time = targetNodeData.get_max_rc_start_time();
	
	// Now check if the end time lies beyond the new and updated start-time:
	// If this is bigger, can be pruned too.
	if (targetNodeData.get_endTime() > new_cont.max_rc_start_time)
		return false;

	// ______________________________
	// Node Access check completed!!!


	// Finally, continue with the rest:
	if (targetNodeData.type == ConNodeType::SEGMENT)
	{
		// 1. Update the SOC, and check if it's feasible:
		ChargingStrategy::Session session = _chargingStrategy.get_chargingSession(
			sourceNodeData.get_endTime(),
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
	}

	return true;
}
