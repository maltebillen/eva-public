#include "incl/pricingProblem/timeSpace/timeSpaceNetwork.h"

void eva::tsn::TimeSpaceNetwork::_addTripNodes()
{
	// Add all trip nodes in the planning horizon:
	_vecTripNodes.resize(_optinput.get_vecTrips().size());
	BoostTimeSpaceNode ts_node;
	for (const auto &tripNode : _optinput.get_vecTrips())
	{
		ts_node = boost::add_vertex(
			TimeSpaceNodeData(
				_getNextIndexNode(),
				TimeSpaceTripNodeData(
					tripNode)),
			_boostTimeSpaceNetwork);

		_vecTripNodes[tripNode.get_index()] = ts_node;
		_mapScheduleNodeLookup.insert(std::make_pair(tripNode.get_scheduleNodeData().get_index(), ts_node));
	}
}

void eva::tsn::TimeSpaceNetwork::_addMaintenanceNodes()
{
	// Add all maintenance nodes in the planning horizon:
	_vecMaintenanceNodes.resize(_optinput.get_vecMaintenances().size());
	BoostTimeSpaceNode ts_node;
	for (const auto &maintenanceNode : _optinput.get_vecMaintenances())
	{
		ts_node = boost::add_vertex(
			TimeSpaceNodeData(
				_getNextIndexNode(),
				TimeSpaceMaintenanceNodeData(
					maintenanceNode)),
			_boostTimeSpaceNetwork);

		_vecMaintenanceNodes[maintenanceNode.get_index()] = ts_node;
		_mapScheduleNodeLookup.insert(std::make_pair(maintenanceNode.get_scheduleNodeData().get_index(), ts_node));
	}
}

void eva::tsn::TimeSpaceNetwork::_addVehicleStartEndNodes()
{
	// Add all vehicle start nodes, and one end node:
	// Start:
	_vecStartNodes.resize(_optinput.get_vehicles().get_vec().size());
	for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
	{
		const ScheduleResourceContainer &vp = _optinput.get_vehiclePosition(vehicle);

		_vecStartNodes[vehicle.get_index()] =
			boost::add_vertex(
				TimeSpaceNodeData(
					_getNextIndexNode(),
					TimeSpaceStartScheduleNodeData(
						_optinput.get_scheduleGraphNodeData(vp.lastScheduleNode).get_endLocation(),
						_optinput.get_scheduleGraphNodeData(vp.lastScheduleNode).get_endTime(),
						vehicle)),
				_boostTimeSpaceNetwork);
	}

	// End schedule, must be routed via a charger:
	for (const Charger &charger : _optinput.get_chargers().get_vec())
	{
		boost::add_vertex(
			TimeSpaceNodeData(
				_getNextIndexNode(),
				TimeSpaceChargerEndScheduleNodeData(
					charger)),
			_boostTimeSpaceNetwork);
	}

	// Collective End:
	_endNode = boost::add_vertex(
		TimeSpaceNodeData(
			_getNextIndexNode(),
			TimeSpaceCollectiveEndScheduleNodeData()),
		_boostTimeSpaceNetwork);
}

void eva::tsn::TimeSpaceNetwork::_addDeadlegs()
{
	const auto &allTimeSpaceNodes = boost::make_iterator_range(boost::vertices(_boostTimeSpaceNetwork));

	int64_t duration = Constants::BIG_UINTEGER;
	uint32_t distance = Constants::BIG_UINTEGER;
	double cost = Constants::BIG_DOUBLE;
	int64_t timeDiff = 0;

	for (const BoostTimeSpaceNode &fromNode : allTimeSpaceNodes)
	{
		for (const BoostTimeSpaceNode &toNode : allTimeSpaceNodes)
		{
			if (fromNode != toNode &&
				_getNodeData(fromNode).type != TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE &&
				_getNodeData(toNode).type != TimeSpaceNodeType::START_SCHEDULE &&
				_getNodeData(fromNode).type != TimeSpaceNodeType::START_SCHEDULE)
			{
				if (_getNodeData(toNode).type != TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE)
				{
					// All connections feasible, where:
					// a. Travel allowed
					// b. Time sufficient for connection'
					if (_getNodeData(toNode).type != TimeSpaceNodeType::CHARGER_END_SCHEDULE)
						timeDiff = Helper::diffDateTime(_getNodeData(fromNode).get_endTime(), _getNodeData(toNode).get_startTime());
					else
						timeDiff = 0;

					const Location &fromLocation = _optinput.get_location(_getNodeData(fromNode).get_endLocationIndex());
					const Location &toLocation = _optinput.get_location(_getNodeData(toNode).get_startLocationIndex());

					duration = fromLocation.get_durationToLocation(toLocation);
					distance = fromLocation.get_distanceToLocation(toLocation);

					if (distance != Constants::BIG_UINTEGER && (distance == 0 || _optinput.get_config().get_flag_allow_deadlegs()) && (timeDiff - duration) >= 0)
					{
						cost = _optinput.get_config().get_cost_deadleg_fix() + distance * _optinput.get_config().get_cost_deadleg_per_km();

						// Add the deadleg:
						boost::add_edge(fromNode, toNode,
										TimeSpaceArcData(
											_getNextIndexArc(),
											duration,
											distance,
											cost),
										_boostTimeSpaceNetwork);
					}
				}
				else
				{
					// Connecting to the end schedule only allowed from the charger end schedule nodes:
					if (_getNodeData(fromNode).type == TimeSpaceNodeType::CHARGER_END_SCHEDULE)
					{
						boost::add_edge(fromNode, toNode,
										TimeSpaceArcData(
											_getNextIndexArc(),
											0,
											0,
											0.0),
										_boostTimeSpaceNetwork);
					}
				}
			}
		}
	}
}

void eva::tsn::TimeSpaceNetwork::_addCharging()
{
	const auto &allTimeSpaceNodes = boost::make_iterator_range(boost::vertices(_boostTimeSpaceNetwork));

	int64_t durationToCharger, durationFromCharger;
	uint32_t distanceToCharger, distanceFromCharger;

	double costToCharger, costFromCharger;
	int64_t timeDiff;
	Types::Index earliestPutOnChargeIndex, latestTakeOffChargeIndex;

	// Initialise the vector before adding charging nodes:
	_vecChargingNodesFrom.resize(_optinput.get_chargers().get_vec().size(), std::vector<std::vector<BoostTimeSpaceNode>>(_indexNode));
	_vecChargingNodesTo.resize(_optinput.get_chargers().get_vec().size(), std::vector<std::vector<BoostTimeSpaceNode>>(_indexNode));

	for (const BoostTimeSpaceNode &fromNode : allTimeSpaceNodes)
	{
		for (const BoostTimeSpaceNode &toNode : allTimeSpaceNodes)
		{
			if (fromNode != toNode &&
				_getNodeData(fromNode).type != TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE &&
				_getNodeData(toNode).type != TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE &&
				_getNodeData(toNode).type != TimeSpaceNodeType::CHARGER_END_SCHEDULE &&
				_getNodeData(fromNode).type != TimeSpaceNodeType::CHARGER_END_SCHEDULE &&
				_getNodeData(toNode).type != TimeSpaceNodeType::START_SCHEDULE)
			{
				const Location &fromLocation = _optinput.get_location(_getNodeData(fromNode).get_endLocationIndex());
				const Location &toLocation = _optinput.get_location(_getNodeData(toNode).get_startLocationIndex());

				timeDiff = Helper::diffDateTime(_getNodeData(fromNode).get_endTime(), _getNodeData(toNode).get_startTime());

				// Connect to every charger in reach:
				for (const Charger &charger : _optinput.get_chargers().get_vec())
				{
					durationToCharger = fromLocation.get_durationToLocation(charger.get_location());
					durationFromCharger = charger.get_location().get_durationToLocation(toLocation);
					distanceToCharger = fromLocation.get_distanceToLocation(charger.get_location());
					distanceFromCharger = charger.get_location().get_distanceToLocation(toLocation);

					if (distanceToCharger != Constants::BIG_UINTEGER && (distanceToCharger == 0 || _optinput.get_config().get_flag_allow_deadlegs()) && distanceFromCharger != Constants::BIG_UINTEGER && (distanceFromCharger == 0 || _optinput.get_config().get_flag_allow_deadlegs()) && (timeDiff - durationToCharger - _optinput.get_config().get_const_put_vehicle_on_charge() - durationFromCharger - _optinput.get_config().get_const_take_vehicle_off_charge()) >= 0)
					{
						earliestPutOnChargeIndex = _optinput.get_nextIdxPutOnChargeAfterStartTime(charger.get_index(), _getNodeData(fromNode).get_endTime() + durationToCharger);
						latestTakeOffChargeIndex = _optinput.get_nextIdxTakeOffChargeBeforeEndTime(charger.get_index(), _getNodeData(toNode).get_startTime() - durationFromCharger);

						if (earliestPutOnChargeIndex != Constants::BIG_INDEX && latestTakeOffChargeIndex != Constants::BIG_INDEX)
						{
							const auto &earliestPutOnCharge = _optinput.get_putOnCharge(charger.get_index(), earliestPutOnChargeIndex);
							const auto &latestTakeOffCharge = _optinput.get_takeOffCharge(charger.get_index(), latestTakeOffChargeIndex);

							if (earliestPutOnCharge.get_scheduleNodeData().get_endTime() < latestTakeOffCharge.get_scheduleNodeData().get_startTime())
							{

								if (_getNodeData(fromNode).get_type() == TimeSpaceNodeType::START_SCHEDULE && _getNodeData(fromNode).get_endLocationIndex() == charger.get_location().get_index())
									// With the last activity of a schedule, the deadleg is already accounted for:
									costToCharger = 0;
								else
								{
									costToCharger = _optinput.get_config().get_cost_deadleg_fix() + distanceToCharger * _optinput.get_config().get_cost_deadleg_per_km();
								}
								costFromCharger = _optinput.get_config().get_cost_deadleg_fix() + distanceFromCharger * _optinput.get_config().get_cost_deadleg_per_km();

								// 1. Add the charging node:
								// Final end time to charge is at the end minus the
								BoostTimeSpaceNode chargingNode =
									boost::add_vertex(
										TimeSpaceNodeData(
											_getNextIndexNode(),
											TimeSpaceChargingNodeData(
												charger,
												earliestPutOnCharge,
												latestTakeOffCharge)),
										_boostTimeSpaceNetwork);

								_vecChargingNodesFrom[charger.get_index()][fromNode].push_back(chargingNode);
								_vecChargingNodesTo[charger.get_index()][toNode].push_back(chargingNode);

								// 2. Add the charging connection to the charging node:
								boost::add_edge(fromNode, chargingNode,
												TimeSpaceArcData(
													_getNextIndexArc(),
													durationToCharger,
													distanceToCharger,
													costToCharger),
												_boostTimeSpaceNetwork);

								// 3. Add the charging connection from the charging node:
								boost::add_edge(chargingNode, toNode,
												TimeSpaceArcData(
													_getNextIndexArc(),
													durationFromCharger,
													distanceFromCharger,
													costFromCharger),
												_boostTimeSpaceNetwork);
							}
						}
					}
				}
			}
		}
	}
}

void eva::tsn::TimeSpaceNetwork::_initialiseNodeAccess()
{
	for (auto &node : _boostTimeSpaceNetwork.m_vertices)
	{
		if (node.m_property.get_type() == TimeSpaceNodeType::START_SCHEDULE)
		{
			node.m_property.init_access(_optinput.get_vehicles().get_vec().size(), TimeSpaceNodeAccessType::NOT_ALLOWED);
			node.m_property.set_access(node.m_property.castVehicleStartNodeData()->get_vehicle().get_index(), TimeSpaceNodeAccessType::ALLOWED);
			node.m_property.set_max_rc_start_time(Constants::MAX_TIMESTAMP, node.m_property.castVehicleStartNodeData()->get_vehicle().get_index());
		}
		else
		{
			node.m_property.init_access(_optinput.get_vehicles().get_vec().size(), TimeSpaceNodeAccessType::ALLOWED);
			node.m_property.reset_max_rc_start_time();
		}
	}

	// Init all access to edges
	auto ei = boost::edges(_boostTimeSpaceNetwork);
	for(auto it = ei.first; it != ei.second; ++it)
	{
		_getArcData(*it).init_access(_optinput.get_vehicles().get_vec().size(),Types::AccessType::ALLOWED);
	}
}

void eva::tsn::TimeSpaceNetwork::_resetAccess()
{
	for (auto &node : _boostTimeSpaceNetwork.m_vertices)
	{
		if (node.m_property.get_type() == TimeSpaceNodeType::START_SCHEDULE)
		{
			node.m_property.reset_access(TimeSpaceNodeAccessType::NOT_ALLOWED);
			node.m_property.set_access(node.m_property.castVehicleStartNodeData()->get_vehicle().get_index(), TimeSpaceNodeAccessType::ALLOWED);
			node.m_property.set_max_rc_start_time(Constants::MAX_TIMESTAMP,node.m_property.castVehicleStartNodeData()->get_vehicle().get_index());
		}
		else
		{
			node.m_property.reset_access(TimeSpaceNodeAccessType::ALLOWED);
			node.m_property.reset_max_rc_start_time();
		}
	}

	// Reset all access to edges
	auto ei = boost::edges(_boostTimeSpaceNetwork);
	for(auto it = ei.first; it != ei.second; ++it)
	{
		_getArcData(*it).reset_access(Types::AccessType::ALLOWED);
	}
}

void eva::tsn::TimeSpaceNetwork::initialise()
{
	// 1. Nodes:
	_addVehicleStartEndNodes();
	_addTripNodes();
	_addMaintenanceNodes();

	// 2. Arcs:
	_addDeadlegs();
	_addCharging();

	// 3. Update Node Access:
	_initialiseNodeAccess();
}

void eva::tsn::TimeSpaceNetwork::updateAccess(const BranchNode &brn)
{
	// First, reset the node access and rc start times:
	_resetAccess();

	// Keep a list of the fixed vertices for every vehicle:
	std::vector<std::vector<TimeSpaceNodeFixings>> vecNodeFixings(_optinput.get_vehicles().get_vec().size());

	// Second, update the node access and rc start times based on the current branching node:
	BoostTimeSpaceNode branchedNode;
	Types::Index indexVehicle;
	Types::Index indexCharger;
	for (const Branch &branch : brn.get_vecBranches())
	{
		switch (branch.get_type())
		{
		case BranchType::TRIP_UNASSIGNED:
			branchedNode =_getTripNode(branch.castBranchTripUnassigned()->get_subTripNodeData().get_index());

			if (branch.get_branchValueBool())
			{
				// Trip is fixed to be unassigned. No other schedule that includes this trip will be feasible.
				_getNodeData(branchedNode).reset_access(false);
			}
			break;

		case BranchType::VEHICLE_TRIP:
			branchedNode =_getTripNode(branch.castBranchVehicleTrip()->get_subTripNodeData().get_index());
			indexVehicle = branch.castBranchVehicleTrip()->get_vehicle().get_index();

			if (branch.get_branchValueBool())
			{
				// Vehicle is fixed:
				// No other vehicle should generate a schedule with the trip.
				// 1. Update the vehicle access:
				_getNodeData(branchedNode).reset_access(false);
				_getNodeData(branchedNode).set_access(indexVehicle, true);
			
				// 2. Store the fixed vehicle node:
				TimeSpaceNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _getNodeData(branchedNode).get_startTime();
				nodeFixing.vec_fixed_nodes.push_back(branchedNode);
				vecNodeFixings[indexVehicle].push_back(nodeFixing);
			}
			else
			{
				_getNodeData(branchedNode).set_access(indexVehicle, false);
			}
			break;

		case BranchType::VEHICLE_MAINTENANCE:
			branchedNode = _getMaintenanceNode(branch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_index());
			indexVehicle = branch.castBranchVehicleMaintenance()->get_vehicle().get_index();

			if (branch.get_branchValueBool())
			{
				// Vehicle is fixed:
				// No other vehicle should generate a schedule with the trip.
				// 1. Update the vehicle access:
				_getNodeData(branchedNode).reset_access(false);
				_getNodeData(branchedNode).set_access(indexVehicle, true);

				// 2. Store the fixed vehicle node:
				TimeSpaceNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _getNodeData(branchedNode).get_startTime();
				nodeFixing.vec_fixed_nodes.push_back(branchedNode);
				vecNodeFixings[indexVehicle].push_back(nodeFixing);
			}
			else
			{
				_getNodeData(branchedNode).set_access(indexVehicle, false);
			}
			break;

		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			// Vehicle is fixed:
			const BranchVehicleChargingAfter *bvca = branch.castBranchVehicleChargingAfter();
			const BoostTimeSpaceNode fromNode = _mapScheduleNodeLookup.find(bvca->get_indexFromScheduleNode())->second;
			indexVehicle = bvca->get_vehicle().get_index();
			indexCharger = bvca->get_charger().get_index();

			// Set the fromNode:
			if (branch.get_branchValueBool())
			{
				_getNodeData(fromNode).reset_access(false);
				_getNodeData(fromNode).set_access(indexVehicle, true);

				// Fix the schedule node:
				TimeSpaceNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _getNodeData(fromNode).get_startTime();
				nodeFixing.vec_fixed_nodes.push_back(fromNode);
				vecNodeFixings[indexVehicle].push_back(nodeFixing);

				// Now remove access to all outgoing arcs that are not leading to a charging session at indexCharger:
				// Iterate over all charging nodes when charger at fromNode:
				auto out_edges_it = boost::out_edges(fromNode, _boostTimeSpaceNetwork);
				BoostTimeSpaceNode toNode;
				for (auto iter = out_edges_it.first; iter != out_edges_it.second; ++iter) 
				{
					// Unless the target node is a charging node at the charger, remove access to the arc:
					toNode = boost::target(*iter, _boostTimeSpaceNetwork);
					if(_getNodeData(toNode).type == TimeSpaceNodeType::CHARGING)
					{
						// Must be guarded because there could be end_schedule nodes that are not having any location assigned
						if(_getNodeData(toNode).get_startLocationIndex() != _optinput.get_charger(indexCharger).get_location().get_index())
						{
							_getArcData(*iter).set_access(indexVehicle, Types::AccessType::NOT_ALLOWED);
						}
					}
					else
					{
						_getArcData(*iter).set_access(indexVehicle, Types::AccessType::NOT_ALLOWED);
					}
				}
			}
			else
			{
				// Iterate over all charging nodes when charger at fromNode:
				for(BoostScheduleNode chargingNode :  _vecChargingNodesFrom[indexCharger][fromNode])
				{	
					// Remove access to the charging node for the vehicle:
					_getNodeData(chargingNode).set_access(indexVehicle, false);
				}
			}
		}
		break;

		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			// Vehicle is fixed:
			const BranchVehicleChargingBefore *bvcb = branch.castBranchVehicleChargingBefore();
			const BoostTimeSpaceNode toNode = _mapScheduleNodeLookup.find(bvcb->get_indexToScheduleNode())->second;
			indexVehicle = bvcb->get_vehicle().get_index();
			indexCharger = bvcb->get_charger().get_index();

			// Set the toNode:
			if (branch.get_branchValueBool())
			{
				_getNodeData(toNode).reset_access(false);
				_getNodeData(toNode).set_access(indexVehicle, true);

				// Fix the schedule node:
				TimeSpaceNodeFixings nodeFixing;
				nodeFixing.fixed_start_time = _getNodeData(toNode).get_startTime();
				nodeFixing.vec_fixed_nodes.push_back(toNode);
				vecNodeFixings[indexVehicle].push_back(nodeFixing);

				// Now remove access to all incoming arcs that are not coming from a charging session at indexCharger:
				// Iterate over all charging nodes when charger at toNode:
				auto in_edges_it = boost::in_edges(toNode, _boostTimeSpaceNetwork);
				BoostTimeSpaceNode fromNode;
				for (auto iter = in_edges_it.first; iter != in_edges_it.second; ++iter) 
				{
					// Unless the target node is a charging node at the charger, remove access to the arc:
					fromNode = boost::source(*iter, _boostTimeSpaceNetwork);
					if(_getNodeData(fromNode).type == TimeSpaceNodeType::CHARGING)
					{
						// Must be guarded because there could be end_schedule nodes that are not having any location assigned
						if(_getNodeData(fromNode).get_endLocationIndex() != _optinput.get_charger(indexCharger).get_location().get_index())
						{
							_getArcData(*iter).set_access(indexVehicle, Types::AccessType::NOT_ALLOWED);
						}
					}
					else
					{
						_getArcData(*iter).set_access(indexVehicle, Types::AccessType::NOT_ALLOWED);
					}
				}
			}
			else
			{
				// Iterate over all charging nodes when charger at fromNode:
				for(BoostScheduleNode chargingNode :  _vecChargingNodesTo[indexCharger][toNode])
				{	
					// Remove access to the charging node for the vehicle:
					_getNodeData(chargingNode).set_access(indexVehicle, false);
				}								
			}
		}	
		break;

		default: 
			break;
		}
	}

	// Second, for all vehicles, sort their respective fixed nodes by start time.
	// Then, set the respective max_rc_times for each node:
	std::vector<BoostTimeSpaceNode> vecPrevNodes;
	for (const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
	{
		std::sort(vecNodeFixings[vehicle.get_index()].begin(), vecNodeFixings[vehicle.get_index()].end(),
				  [&](const TimeSpaceNodeFixings &l, const TimeSpaceNodeFixings &r)
				  {
					  return l.fixed_start_time < r.fixed_start_time;
				  });
		
		vecPrevNodes.clear();
		vecPrevNodes.push_back(_vecStartNodes[vehicle.get_index()]);
		for (const TimeSpaceNodeFixings& nodeFixing : vecNodeFixings[vehicle.get_index()]) {
			// Update the time of all prev nodes:
			for(const auto& prevNode : vecPrevNodes)
				_getNodeData(prevNode).set_max_rc_start_time(nodeFixing.fixed_start_time,vehicle.get_index());
					
			
			// Update the current node:
			for(const auto& curNode : nodeFixing.vec_fixed_nodes)
				_getNodeData(curNode).set_max_rc_start_time(Constants::MAX_TIMESTAMP,vehicle.get_index());
			
			// Update the list of previous nodes:
			vecPrevNodes.clear();
			vecPrevNodes = nodeFixing.vec_fixed_nodes;
		}
	}
}

std::vector<eva::SubVehicleSchedule> eva::tsn::TimeSpaceNetwork::find_neg_reduced_cost_schedule_vehicle(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
	BoostTimeSpaceNode sourceVertex = _getStartNode(vehicle.get_index());
	BoostTimeSpaceNode sinkVertex = _endNode;

	const ScheduleResourceContainer &curVehiclePosition = _optinput.get_vehiclePosition(vehicle);
	TimeSpaceResourceContainer initialResourceContainer(
				0.0,
				0.0 - duals.vecDualsOneSchedulePerVehicle[vehicle.get_index()],
				_optinput.get_scheduleGraphNodeData(curVehiclePosition.lastScheduleNode).get_endTime(),
				curVehiclePosition.odometerReading - curVehiclePosition.odometerLastMaintenance,
				curVehiclePosition.soc,
				_optinput.get_flag_has_unassigned_maintenance(),
				_getNodeData(sourceVertex).get_max_rc_start_time());

	// Solve the resource constraint shortest path problem:
	TimeSpaceResourceConstraintPaths shortestPaths;
	boost::r_c_shortest_paths(
		_boostTimeSpaceNetwork,
		boost::get(&TimeSpaceNodeData::index, _boostTimeSpaceNetwork),
		boost::get(&TimeSpaceArcData::index, _boostTimeSpaceNetwork),
		sourceVertex,
		sinkVertex,
		shortestPaths.pareto_optimal_solutions,
		shortestPaths.pareto_optimal_resource_containers,
		initialResourceContainer,
		TimeSpaceResourceExtensionFunction(duals, vehicle, _optinput, include_cost),
		TimeSpaceDominanceCheck(solve_to_optimal, _optinput.get_flag_has_unassigned_maintenance()),
		boost::default_r_c_shortest_paths_allocator(),
		TimeSpaceResourceExtensionVisitor(_optinput.get_config().get_const_nr_cols_per_vehicle_iter(), solve_to_optimal, timeOutClock));
	
	// If less than the maximum number of labels are returned, the labelling algorithm must have processed all labels - hence, the subpath is explored optimally. Else, not:
	if ((!solve_to_optimal && shortestPaths.pareto_optimal_solutions.size() >= _optinput.get_config().get_const_nr_cols_per_vehicle_iter()) || std::chrono::high_resolution_clock::now() >= timeOutClock)
	{
		// Indicate that the labelling algorithm for this vehicle was not solved to optimality. Can only be set to false.
		isSolvedOptimal = false;
	}

	// __________________
	// Store the results:
	std::vector<SubVehicleSchedule> result;
	double cost, reducedCost;
	for (Types::Index indexResult = 0; indexResult < shortestPaths.pareto_optimal_resource_containers.size(); indexResult++)
	{
		cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
		reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

		if (Helper::compare_floats_smaller(reducedCost, 0))
		{
			SubVehicleSchedule svs;

			svs.indexVehicle = vehicle.get_index();
			svs.indexStartLocation = _getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].back(), _boostTimeSpaceNetwork)).get_startLocationIndex();
			svs.indexEndLocation = _getNodeData(boost::source(shortestPaths.pareto_optimal_solutions[indexResult].front(), _boostTimeSpaceNetwork)).get_endLocationIndex();
			svs.cost = shortestPaths.pareto_optimal_resource_containers[indexResult].cost;
			svs.reducedCost = shortestPaths.pareto_optimal_resource_containers[indexResult].reducedCost;

			Types::BatteryCharge soc = curVehiclePosition.soc;

			for (auto iterArc = shortestPaths.pareto_optimal_solutions[indexResult].rbegin(); iterArc != shortestPaths.pareto_optimal_solutions[indexResult].rend(); ++iterArc)
			{
				const TimeSpaceArcData &arcData = boost::get(boost::edge_bundle, _boostTimeSpaceNetwork)[*iterArc];
				const TimeSpaceNodeData &sourceNodeData = _getNodeData(boost::source(*iterArc, _boostTimeSpaceNetwork));
				const TimeSpaceNodeData &targetNodeData = _getNodeData(boost::target(*iterArc, _boostTimeSpaceNetwork));

				soc -= vehicle.get_batteryDischarge(arcData.get_distance() + targetNodeData.get_distance());

				switch (targetNodeData.type)
				{
				case TimeSpaceNodeType::TRIP:
					svs.vecScheduleNodes.push_back(targetNodeData.castTripNodeData()->get_subTripNodeData().get_scheduleNodeData().get_index());
					svs.vecTripNodeIndexes.push_back(targetNodeData.castTripNodeData()->get_subTripNodeData().get_index());
					break;
				case TimeSpaceNodeType::MAINTENANCE:
					svs.vecScheduleNodes.push_back(targetNodeData.castMaintenanceNodeData()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index());
					svs.vecMaintenanceNodesIndexes.push_back(targetNodeData.castMaintenanceNodeData()->get_subMaintenanceNodeData().get_index());
					break;
				case TimeSpaceNodeType::CHARGING:
				{
					const auto &chargingNodeData = targetNodeData.castChargingNodeData();

					ChargingStrategy::Session session = _chargingStrategy.get_chargingSession(
						chargingNodeData->get_earliestPutOnChargeNode().get_scheduleNodeData().get_startTime(),
						chargingNodeData->get_latestTakeOffChargeNode().get_scheduleNodeData().get_endTime(),
						vehicle,
						chargingNodeData->get_charger(),
						soc);

					if (session.is_charging)
					{
						// Stop charging when the vehicle is full:
						soc = std::min(vehicle.get_batteryMaxKWh(), soc + session.get_charge(_optinput, chargingNodeData->get_charger().get_index()));

						ChargingSchedule cs;
						cs.indexCharger = chargingNodeData->get_charger().get_index();
						cs.indexPutOnCharge = session.index_putOnCharge;
						cs.indexTakeOffCharge = session.index_takeOffCharge;
						cs.indexFromScheduleNode = sourceNodeData.get_scheduleNodeIndex(); // If no schedule node, use default -1.
						cs.indexToScheduleNode = (iterArc + 1) != shortestPaths.pareto_optimal_solutions[indexResult].rend() ? _getNodeData(boost::target(*(iterArc + 1), _boostTimeSpaceNetwork)).get_scheduleNodeIndex() : Constants::BIG_INDEX;
						svs.vecChargingSchedule.push_back(cs);

						// At the end, add the charging session nodes:
						svs.vecScheduleNodes.push_back(_optinput.get_putOnCharge(chargingNodeData->get_charger().get_index(), session.index_putOnCharge).get_scheduleNodeData().get_index());
						svs.vecScheduleNodes.push_back(_optinput.get_takeOffCharge(chargingNodeData->get_charger().get_index(), session.index_takeOffCharge).get_scheduleNodeData().get_index());
					}
					break;
				}
				case TimeSpaceNodeType::CHARGER_END_SCHEDULE:
				case TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE:
				case TimeSpaceNodeType::START_SCHEDULE:
				case TimeSpaceNodeType::UNDEFINED:
					break;
				}
			}

			result.push_back(svs);
		}
	}

	return result;
}

eva::tsn::TimeSpaceResourceContainer &eva::tsn::TimeSpaceResourceContainer::operator=(const TimeSpaceResourceContainer &other)
{
	if (this == &other)
		return *this;
	this->~TimeSpaceResourceContainer();
	new (this) TimeSpaceResourceContainer(other);
	return *this;
}

const bool eva::tsn::TimeSpaceResourceContainer::operator==(const TimeSpaceResourceContainer &other) const
{
	if (include_distance)
		return Helper::compare_floats_equal(reducedCost, other.reducedCost) && distanceLastMaintenance == other.distanceLastMaintenance && soc == other.soc;
	else
		return Helper::compare_floats_equal(reducedCost, other.reducedCost) && soc == other.soc;
}

const bool eva::tsn::TimeSpaceResourceContainer::operator<(const TimeSpaceResourceContainer &other) const
{
	if (other.isEndSchedule || other.isExemptFromDominance)
		// never smaller at the sink node:
		return false;

	if (eva::Helper::compare_floats_smaller(other.reducedCost, reducedCost))
		return false;

	if (include_distance)
	{
		if (Helper::compare_floats_equal(other.reducedCost, reducedCost))
		{
			if (other.distanceLastMaintenance < distanceLastMaintenance)
			{
				return false;
			}

			if (other.distanceLastMaintenance == distanceLastMaintenance)
			{
				return soc > other.soc;
			}
		}

		return true;
	}
	else
	{
		if (Helper::compare_floats_equal(other.reducedCost, reducedCost))
			return soc > other.soc;

		return true;
	}
}

bool eva::tsn::TimeSpaceResourceExtensionFunction::_handleTripNode(TimeSpaceResourceContainer &new_cont, const TimeSpaceResourceContainer &old_cont, const TimeSpaceTripNodeData *tripNodeData) const
{
	new_cont.reducedCost -= _duals.vecDualsTripCoverage[tripNodeData->get_subTripNodeData().get_index()];

	return true;
}

bool eva::tsn::TimeSpaceResourceExtensionFunction::_handleMaintenanceNode(TimeSpaceResourceContainer &new_cont, const TimeSpaceResourceContainer &old_cont, const TimeSpaceMaintenanceNodeData *maintenanceNodeData) const
{
	new_cont.distanceLastMaintenance = 0;
	new_cont.reducedCost -= _duals.vecDualsOneVehiclePerMaintenance[maintenanceNodeData->get_subMaintenanceNodeData().get_index()];

	return true;
}

bool eva::tsn::TimeSpaceResourceExtensionFunction::_handleChargingNode(TimeSpaceResourceContainer &new_cont, const TimeSpaceResourceContainer &old_cont, const TimeSpaceChargingNodeData *chargingNodeData) const
{
	ChargingStrategy::Session session = _chargingStrategy.get_chargingSession(
		chargingNodeData->get_earliestPutOnChargeNode().get_scheduleNodeData().get_startTime(),
		chargingNodeData->get_latestTakeOffChargeNode().get_scheduleNodeData().get_endTime(),
		_vehicle,
		chargingNodeData->get_charger(),
		new_cont.soc);

	if (session.is_feasible)
	{
		// If charging takes place, increase the soc:
		if (session.is_charging)
		{
			new_cont.soc = std::min(_vehicle.get_batteryMaxKWh(), new_cont.soc + session.get_charge(_optinput, chargingNodeData->get_charger().get_index()));
			new_cont.reducedCost -= _duals.vecCumSumDualsChargerCapacity[chargingNodeData->get_charger().get_index()][session.index_putOnCharge][session.index_takeOffCharge];
		}

		return true;
	}
	else
		return false;
}

bool eva::tsn::TimeSpaceResourceExtensionFunction::operator()(const BoostTimeSpaceNetwork &boostTimeSpaceNetwork, TimeSpaceResourceContainer &new_cont, const TimeSpaceResourceContainer &old_cont, const BoostTimeSpaceArc &arc) const
{
	const TimeSpaceArcData &arcData = boost::get(boost::edge_bundle, boostTimeSpaceNetwork)[arc];

	const TimeSpaceNodeData &sourceNodeData = boostTimeSpaceNetwork[boost::source(arc, boostTimeSpaceNetwork)];
	const TimeSpaceNodeData &targetNodeData = boostTimeSpaceNetwork[boost::target(arc, boostTimeSpaceNetwork)];

	// Check the access to the vertex, and if the start time is below the label maximum start time.
	new_cont.isEndSchedule = (targetNodeData.type == TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE);
	new_cont.isExemptFromDominance = (targetNodeData.type == TimeSpaceNodeType::CHARGER_END_SCHEDULE);

	// Check vehicle access:
	if (!targetNodeData.has_access(_vehicle.get_index())
			|| !arcData.has_access(_vehicle.get_index()))
		return false;

	if (targetNodeData.type == TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE
	 	|| targetNodeData.type == TimeSpaceNodeType::CHARGER_END_SCHEDULE)
	{
		// Only if the previous container has unlocked the sink node, a label can be created here:
		if(old_cont.max_rc_start_time == Constants::MAX_TIMESTAMP)
		{
			return Helper::compare_floats_smaller(new_cont.reducedCost, 0.0);
		}
		else
		{
			// Sink is not open because the sink node is currently not the destination vertex.
			// This prevents the algorithm from finishing early, before all vehicle fixings have been allocated.
			return false;
		}
	}

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

	
	// ARC + NODE: SOC
	new_cont.soc -= _vehicle.get_batteryDischarge(arcData.get_distance() + targetNodeData.get_distance());
	if (new_cont.soc < static_cast<int32_t>(_vehicle.get_batteryMinKWh()))
		return false;

	// ARC: TIME + COST + RC
	new_cont.distanceLastMaintenance += arcData.get_distance() + targetNodeData.get_distance();

	double cost = arcData.get_cost();
	if (_optinput.get_flag_has_unassigned_maintenance())
		cost += 0.5 * _optinput.get_config().get_cost_coefficient_penalty_maintenance() * (std::pow(new_cont.distanceLastMaintenance, 2.0) - std::pow(old_cont.distanceLastMaintenance, 2.0));

	new_cont.cost += cost;
	if (_include_cost)
		new_cont.reducedCost += cost;

	// NODE: TIME + COST + RC
	switch (targetNodeData.type)
	{
	case TimeSpaceNodeType::TRIP:
		return _handleTripNode(new_cont, old_cont, targetNodeData.castTripNodeData());
	case TimeSpaceNodeType::MAINTENANCE:
		return _handleMaintenanceNode(new_cont, old_cont, targetNodeData.castMaintenanceNodeData());
	case TimeSpaceNodeType::CHARGING:
		return _handleChargingNode(new_cont, old_cont, targetNodeData.castChargingNodeData());
	case TimeSpaceNodeType::CHARGER_END_SCHEDULE:
		return true;
	case TimeSpaceNodeType::COLLECTIVE_END_SCHEDULE:
		return true;
	case TimeSpaceNodeType::START_SCHEDULE:
	case TimeSpaceNodeType::UNDEFINED:
		return false;
	}

	return false;
}
