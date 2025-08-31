#include "incl/pricingProblem/segmentBased/segments/subGraph/subGraph.h"

#include <boost/graph/r_c_shortest_paths.hpp>

void eva::sbn::subgraph::SegmentSubGraph::initialise(const uint32_t& nrLayers)
{
	_vecLayers.resize(nrLayers);
}

void eva::sbn::subgraph::SegmentSubGraph::addTripNode(const SubScheduleTripNodeData& trip, const uint32_t& layer)
{
	auto node = boost::add_vertex(
			NodeData(
				_getNextIndexNode(),
				_optinput.get_vehicles().get_vec().size(),
				TripNodeData(trip),
				_vecDefaultVehicleAccess
			),
			_boostSubGraph
		);

	_vecLayers[layer].push_back(node);

	// Store a lookup for schedule node.
	_mapScheduleNodeLookUp[trip.get_scheduleNodeData().get_index()].push_back(node);
	_mapLayerLookUp.emplace(std::pair<BoostSubGraphNode, Types::Index>(node, layer));
}

void eva::sbn::subgraph::SegmentSubGraph::addMaintenanceNode(const SubScheduleMaintenanceNodeData& maintenance, const uint32_t& layer)
{
	auto node = boost::add_vertex(
			NodeData(
				_getNextIndexNode(),
				_optinput.get_vehicles().get_vec().size(),
				MaintenanceNodeData(maintenance),
				_vecDefaultVehicleAccess
			),
			_boostSubGraph
		);

	_vecLayers[layer].push_back(node);

	// Store a lookup for schedule node.
	_mapScheduleNodeLookUp[maintenance.get_scheduleNodeData().get_index()].push_back(node);
	_mapLayerLookUp.emplace(std::pair<BoostSubGraphNode, Types::Index>(node, layer));
}

void eva::sbn::subgraph::SegmentSubGraph::addAuxiliaryNodes(const Charger& startCharger, const Charger& endCharger)
{
	// 1. Start Segment Nodes:
	// Add a start node for every node in the first layer:
	Types::DateTime segmentStartTime, segmentEndTime, latestChargingEndTime;
	Types::Index latestChargingEndIndex, earliestChargingStartIndex, latestChargingStartIndex;
	for (const BoostSubGraphNode& node : _vecLayers[1])
	{
		segmentStartTime = _getNodeData(node).get_startTime() - startCharger.get_location().get_durationToLocation(_getNodeData(node).get_startLocationIndex());
		latestChargingEndIndex = _optinput.get_nextIdxTakeOffChargeBeforeEndTime(startCharger.get_index(), segmentStartTime);
			
		if (latestChargingEndIndex == Constants::BIG_INDEX)
		{
			_getNodeData(node).fix_infeasible(); // End lies before any vehicle could reach it. Therefore, it must be infeasible and won't appear in any feasible schedule.
			latestChargingEndTime = 0;
			latestChargingEndIndex = 0;
			earliestChargingStartIndex = 0;
			latestChargingStartIndex = 0;
		}
		else
		{
			segmentStartTime = _optinput.get_takeOffCharge(startCharger.get_index(), latestChargingEndIndex).get_scheduleNodeData().get_endTime(); // Update the segment start time to the end of the latest charging session
			latestChargingEndTime = _optinput.get_takeOffCharge(startCharger.get_index(), latestChargingEndIndex).get_scheduleNodeData().get_startTime();
			earliestChargingStartIndex = _optinput.get_nextIdxPutOnChargeBeforeEndTime(startCharger.get_index(), latestChargingEndTime - _maxub_rechargeDuration);
			latestChargingStartIndex = _optinput.get_nextIdxPutOnChargeBeforeEndTime(startCharger.get_index(), latestChargingEndTime - _minlb_rechargeDuration);

			if (earliestChargingStartIndex == Constants::BIG_INDEX)
				earliestChargingStartIndex = 0; // No vehicle will be charging earlier any way.

			if (latestChargingStartIndex == Constants::BIG_INDEX)
				latestChargingStartIndex = 0; // No vehicle will be charging earlier any way.
		}		

		// Create the start segment node 
		_vecLayers.front().push_back(
			boost::add_vertex(
				NodeData(
					_getNextIndexNode(),
					_optinput.get_vehicles().get_vec().size(),
					StartSegmentNodeData(startCharger, segmentStartTime, earliestChargingStartIndex,latestChargingStartIndex, latestChargingEndIndex),
					_vecDefaultVehicleAccess
				),
				_boostSubGraph
			)
		);
	}

	// 2. End Segment Nodes:
	// Add an end node for every node in the last layer:
	for (const BoostSubGraphNode& node : _vecLayers[(_vecLayers.size() - 2)])
	{
		segmentEndTime = _getNodeData(node).get_endTime() + _optinput.get_location(_getNodeData(node).get_endLocationIndex()).get_durationToLocation(endCharger.get_location());
		earliestChargingStartIndex = _optinput.get_nextIdxPutOnChargeAfterStartTime(endCharger.get_index(), segmentEndTime);

		// Update the segment end time to the start of the earliest charging session
		if(earliestChargingStartIndex != Constants::BIG_INDEX)
			segmentEndTime = _optinput.get_putOnCharge(endCharger.get_index(), earliestChargingStartIndex).get_scheduleNodeData().get_startTime(); 
			
		// Create the end segment node 
		_vecLayers.back().push_back(
			boost::add_vertex(
				NodeData(
					_getNextIndexNode(),
					_optinput.get_vehicles().get_vec().size(),
					EndSegmentNodeData(endCharger, segmentEndTime),
					_vecDefaultVehicleAccess
				),
				_boostSubGraph
			)
		);
	}

	// 3. Add one artifical end node purely for computational reasons to run the RCSP-algorithm.
	_sourceNode =
		boost::add_vertex(
			NodeData(
				_getNextIndexNode(),
				_optinput.get_vehicles().get_vec().size(),
				CollectiveStartSegmentNodeData(startCharger),
				_vecDefaultVehicleAccess
			),
			_boostSubGraph
		);

	_sinkNode =
		boost::add_vertex(
			NodeData(
				_getNextIndexNode(),
				_optinput.get_vehicles().get_vec().size(),
				CollectiveEndSegmentNodeData(endCharger),
				_vecDefaultVehicleAccess
			),
			_boostSubGraph
		);
}

void eva::sbn::subgraph::SegmentSubGraph::addConnections()
{
	// Layer 0 -> 1: 1-1
	for (Types::Index startIndex = 0; startIndex < _vecLayers.front().size(); startIndex++)
	{
		if (_getNodeData(_vecLayers[1][startIndex]).is_feasible())
		{
			// Layer -1 -> 0:
			boost::add_edge(_sourceNode, _vecLayers.front()[startIndex],
				ArcData(
					_getNextIndexArc()
				),
				_boostSubGraph
			);

			boost::add_edge(_vecLayers.front()[startIndex], _vecLayers[1][startIndex],
				ArcData(
					_getNextIndexArc()
				),
				_boostSubGraph
			);
		}
	}

	// Midlayer:
	// Layer 1 until n - 2: 1-n
	for (uint32_t layer = 1; layer < _vecLayers.size() - 2; layer++)
	{
		for (const BoostSubGraphNode from : _vecLayers[layer])
		{
			for (const BoostSubGraphNode to : _vecLayers[layer + 1])
			{
				const NodeData& fromData = _getNodeData(from);
				const NodeData& toData = _getNodeData(to);

				int64_t timeDiff = Helper::diffDateTime(fromData.get_endTime(), toData.get_startTime());
				int64_t duration = _optinput.get_location(_getNodeData(from).get_endLocationIndex()).get_durationToLocation(_getNodeData(to).get_startLocationIndex());

				if ((timeDiff - duration) >= 0)
				{
					// Add the one connection and break the inner-loop.
					boost::add_edge(from, to,
						ArcData(
							_getNextIndexArc()
						),
						_boostSubGraph
					);
				}
			}
		}
	}

	// Layer n-1 -> n
	// Layer n -> n+1:
	for (Types::Index endIndex = 0; endIndex < _vecLayers.back().size(); endIndex++)
	{
		boost::add_edge(_vecLayers[(_vecLayers.size() - 2)][endIndex], _vecLayers.back()[endIndex],
			ArcData(
				_getNextIndexArc()
			),
			_boostSubGraph
		);

		// Layer n -> n+1:
		boost::add_edge(_vecLayers.back()[endIndex], _sinkNode,
			ArcData(
				_getNextIndexArc()
			),
			_boostSubGraph
		);
	}
}

void eva::sbn::subgraph::SegmentSubGraph::updateCurrentNonDominatedSchedulePieces(const BranchNode& brn, const Duals& duals)
{
	// Step 0: Clear the current vector:
	_vecCurrentNonDominatedSchedulePieces.clear();

	// Step 1: Update duals:
	_updateDuals(duals);

	// Step 2: Initialise:
	std::vector<Types::AccessType> vecAccess = _vecDefaultVehicleAccess;
	SubGraphResourceExtensionFunction ref;
	SubGraphResourceContainer initialResourceContainer(0.0, 0.0, 0.0, Constants::MAX_TIMESTAMP, Constants::MAX_TIMESTAMP, vecAccess, Constants::BIG_INDEX, Constants::MAX_TIMESTAMP);
	std::vector<std::vector<BoostSubGraphArc>> pareto_optimal_paths;
	std::vector<SubGraphResourceContainer> pareto_optimal_resource_containers;

	// Step 3: Run the search for paths:
	// Solve the shortest paths for the source-sink combination:
	// This creates paths valid for all vehicles (changes could be applied later if branches remove some access to some paths)
	// These paths here are GLOBAL optimal!
	boost::r_c_shortest_paths(
		_boostSubGraph,
		boost::get(&NodeData::index, _boostSubGraph),
		boost::get(&ArcData::index, _boostSubGraph),
		_sourceNode,
		_sinkNode,
		pareto_optimal_paths,
		pareto_optimal_resource_containers,
		initialResourceContainer,
		ref,
		SubGraphDominanceCheck()
	);
	
	// Store all current non-dominated paths:
	for (int32_t i = 0; i < static_cast<int32_t>(pareto_optimal_resource_containers.size()); ++i)
	{
		std::vector<Types::Index> vecTripIndexes;
		std::vector<Types::Index> vecMaintenanceIndexes;
		std::vector<Types::Index> vecScheduleNodeIndexes;
		double minChargingDuals = 0.0;
		double maxChargingDuals = 0.0;

		for (auto iterArc = pareto_optimal_paths[i].rbegin(); iterArc != pareto_optimal_paths[i].rend(); ++iterArc)
		{
			const NodeData& toData = _boostSubGraph[boost::target(*iterArc, _boostSubGraph)];

			switch (toData.type)
			{
			case NodeType::TRIP:
				vecScheduleNodeIndexes.push_back(toData.castTripNodeData()->get_subScheduleTripNodeData().get_scheduleNodeData().get_index());
				vecTripIndexes.push_back(toData.castTripNodeData()->get_subScheduleTripNodeData().get_index());
				break;
			case NodeType::MAINTENANCE:
				vecScheduleNodeIndexes.push_back(toData.castMaintenanceNodeData()->get_subScheduleMaintenanceNodeData().get_scheduleNodeData().get_index());
				vecMaintenanceIndexes.push_back(toData.castMaintenanceNodeData()->get_subScheduleMaintenanceNodeData().get_index());
				break;
			case NodeType::START_SEGMENT:
				minChargingDuals = toData.get_min_charging_dual();
				maxChargingDuals = toData.get_max_charging_dual();
				break;
			default:
				break;
			};
		}

		// Create the non dominated object:
		_vecCurrentNonDominatedSchedulePieces.push_back(
			NonDominatedSchedulePiece(
				pareto_optimal_resource_containers[i].accDuals,
				minChargingDuals,
				maxChargingDuals,
				pareto_optimal_resource_containers[i].timestampStart,
				pareto_optimal_resource_containers[i].timestampEnd,
				pareto_optimal_resource_containers[i].vecAccess,
				vecTripIndexes,
				vecMaintenanceIndexes,
				vecScheduleNodeIndexes,
				pareto_optimal_resource_containers[i].indexFixedVehicle
			)
		);
	}
}

void eva::sbn::subgraph::SegmentSubGraph::_updateDuals(const Duals &duals)
{
	for (uint32_t layer = 0; layer < _vecLayers.size() - 1; layer++)
	{
		for (const BoostSubGraphNode &node : _vecLayers[layer])
		{
			switch (_getNodeData(node).get_type())
			{
			case NodeType::TRIP:
				_getNodeData(node).acc_dual = duals.vecDualsTripCoverage[_getNodeData(node).castTripNodeData()->get_subScheduleTripNodeData().get_index()];
				break;
			case NodeType::MAINTENANCE:
				_getNodeData(node).acc_dual = duals.vecDualsOneVehiclePerMaintenance[_getNodeData(node).castMaintenanceNodeData()->get_subScheduleMaintenanceNodeData().get_index()];
				break;
			case NodeType::START_SEGMENT:
				if (_getNodeData(node).is_feasible())
				{
					// Only if the node is generally feasible will the start and end-index be correct.
					_getNodeData(node).min_charging_dual =
						duals.vecCumSumDualsChargerCapacity
							[_getNodeData(node).castStartSegmentNodeData()->get_charger().get_index()]
							[_getNodeData(node).castStartSegmentNodeData()->get_earliestChargingStartIndex()]
							[_getNodeData(node).castStartSegmentNodeData()->get_latestChargingEndIndex()];

					_getNodeData(node).max_charging_dual =
						duals.vecCumSumDualsChargerCapacity
							[_getNodeData(node).castStartSegmentNodeData()->get_charger().get_index()]
							[_getNodeData(node).castStartSegmentNodeData()->get_latestChargingStartIndex()]
							[_getNodeData(node).castStartSegmentNodeData()->get_latestChargingEndIndex()];
				}
				break;
			default:
				_getNodeData(node).acc_dual = 0.0;
				_getNodeData(node).min_charging_dual = 0.0;
				_getNodeData(node).max_charging_dual = 0.0;
				break;
			}
		}
	}
}

void eva::sbn::subgraph::SegmentSubGraph::updateFixings(const BranchNode &brn)
{
	// Step 1:
	// First, reset the flexible node data:
	for (uint32_t layer = 1; layer < _vecLayers.size() - 1; layer++)
	{
		for (const BoostSubGraphNode &node : _vecLayers[layer])
		{	
			_getNodeData(node).reset_default_fixings(_vecDefaultVehicleAccess);
		}
	}

	// Step 2:
	// Iterate over all branches, and update the node access:
	auto ptrScheduleNode = _mapScheduleNodeLookUp.end();
	for (const Branch &branch : brn.get_vecBranches())
	{
		switch (branch.get_type())
		{
		case BranchType::TRIP_UNASSIGNED:
			// If the trip has been deemed unassigned. Stop producing paths that include the trip.
			// Mark that the trip cannot be used in any path:
			if (branch.get_branchValueBool())
			{
				// Check if any of the fixed schedule nodes are in this segment subgraph, if yes, keep track of all.
				ptrScheduleNode = _mapScheduleNodeLookUp.find(branch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index());
				if (ptrScheduleNode != _mapScheduleNodeLookUp.end())
				{
					for(const auto& node : ptrScheduleNode->second)
					{
						_getNodeData(node).fix_infeasible();
					}
				}
			}
			break;
		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const auto *cba = branch.castBranchVehicleChargingAfter();

			// If the schedule node must conclude with a charging session, then check if this segment allows this:
			// Mark that the maintenance cannot be used in any path:
			ptrScheduleNode = _mapScheduleNodeLookUp.find(cba->get_indexFromScheduleNode());
			if (ptrScheduleNode != _mapScheduleNodeLookUp.end())
			{
				// Iterate all occurences of the schedule node:
				for (const auto &node : ptrScheduleNode->second)
				{
					if(branch.get_branchValueBool())
					{
						// Check if the schedule node is in the last layer, and charging concludes at the respective charger:
						if (_mapLayerLookUp.at(node) == (_vecLayers.size() - 2) 
							&& _getNodeData(_vecLayers.back().back()).castEndSegmentNodeData()->get_charger().get_index() == cba->get_charger().get_index()
							&& _getNodeData(node).has_access(cba->get_vehicle().get_index()))
						{
								// If the node has alrady been previously removed, then this segment is in combination not feasible for the vehicle.
								_getNodeData(node).fix_vehicle(cba->get_vehicle().get_index());
								_getNodeData(node).set_fixedPathEndTime(brn.get_vehicleFixedNodeNextMaxEndTime(cba->get_vehicle().get_index(), cba->get_indexFromScheduleNode()));
								_getNodeData(node).set_fixedPathStartTime(brn.get_vehicleFixedNodePrevMinStartTime(cba->get_vehicle().get_index(), cba->get_indexFromScheduleNode()));
						}
						else
						{
							// Else, no other vehicle or this vehicle can use this node in this subgraph:
							_getNodeData(node).fix_infeasible();
						}
					}
					else
					{
						// Revoke access if for the vehicle, 
						// If it the schedule node is in the last layer, and at the undesired charger.
						if (_mapLayerLookUp.at(node) == (_vecLayers.size() - 2) 
							&& _getNodeData(_vecLayers.back().back()).castEndSegmentNodeData()->get_charger().get_index() == cba->get_charger().get_index())
						{
							_getNodeData(node).revoke_access(cba->get_vehicle().get_index());
						}
					}
				}
			}
		}
		break;

		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const auto *cbb = branch.castBranchVehicleChargingBefore();

			// If the schedule node must conclude with a charging session, then check if this segment allows this:
			// Mark that the maintenance cannot be used in any path:
			ptrScheduleNode = _mapScheduleNodeLookUp.find(cbb->get_indexToScheduleNode());
			if (ptrScheduleNode != _mapScheduleNodeLookUp.end())
			{
				// Iterate all occurences of the schedule node:
				for (const auto &node : ptrScheduleNode->second)
				{
					if(branch.get_branchValueBool())
					{
						// Check if the schedule node is in the last layer, and charging concludes at the respective charger:
						if (_mapLayerLookUp.at(node) == 1 
							&& _getNodeData(_vecLayers.front().back()).castStartSegmentNodeData()->get_charger().get_index() == cbb->get_charger().get_index()
							&& _getNodeData(node).has_access(cbb->get_vehicle().get_index()))
						{
							// If the node has alrady been previously removed, then this segment is in combination not feasible for the vehicle.
							_getNodeData(node).fix_vehicle(cbb->get_vehicle().get_index());
							_getNodeData(node).set_fixedPathEndTime(brn.get_vehicleFixedNodeNextMaxEndTime(cbb->get_vehicle().get_index(), cbb->get_indexToScheduleNode()));
							_getNodeData(node).set_fixedPathStartTime(brn.get_vehicleFixedNodePrevMinStartTime(cbb->get_vehicle().get_index(), cbb->get_indexToScheduleNode()));
						}
						else
						{
							// Else, no other vehicle or this vehicle can use this node in this subgraph:
							_getNodeData(node).fix_infeasible();
						}
					}
					else
					{
						// Revoke access if for the vehicle, 
						// If it the schedule node is in the last layer, and at the undesired charger.
						if (_mapLayerLookUp.at(node) == 1 
								&& _getNodeData(_vecLayers.front().back()).castStartSegmentNodeData()->get_charger().get_index() == cbb->get_charger().get_index())
						{
							_getNodeData(node).revoke_access(cbb->get_vehicle().get_index());
						}
					}
				}
			}
		}
		break;

		case BranchType::VEHICLE_MAINTENANCE:
		{
			const auto *cb = branch.castBranchVehicleMaintenance();

			// If the maintenance has been fixed to a vehicle. Stop producing paths that include the maintenance.
			// Mark that the maintenance cannot be used in any path:
			ptrScheduleNode = _mapScheduleNodeLookUp.find(cb->get_subMaintenanceNodeData().get_scheduleNodeData().get_index());
			if (ptrScheduleNode != _mapScheduleNodeLookUp.end())
			{
				for (const auto &node : ptrScheduleNode->second)
				{
					if (branch.get_branchValueBool())
					{
						// If there exists another branch that has already revoked the access, then it can't be fixed again.
						// For instance, this can happen if the node also appears in a vehicle-charging fixing.
						// Then, the node may be fixed but on an interior node, which makes it infeasible in the context of this subgraph.
						if(_getNodeData(node).has_access(cb->get_vehicle().get_index()))
						{
							_getNodeData(node).fix_vehicle(cb->get_vehicle().get_index());
							_getNodeData(node).set_fixedPathEndTime(brn.get_vehicleFixedNodeNextMaxEndTime(cb->get_vehicle().get_index(), cb->get_subMaintenanceNodeData().get_scheduleNodeData().get_index()));
							_getNodeData(node).set_fixedPathStartTime(brn.get_vehicleFixedNodePrevMinStartTime(cb->get_vehicle().get_index(), cb->get_subMaintenanceNodeData().get_scheduleNodeData().get_index()));
						}
						else
						{
							// Else, no other vehicle or this vehicle can use this node in this subgraph:
							_getNodeData(node).fix_infeasible();
						}
					}
					else
					{
						_getNodeData(node).revoke_access(cb->get_vehicle().get_index());
					}
				}
			}
		}
		break;
		case BranchType::VEHICLE_TRIP:
		{
			const auto *cb = branch.castBranchVehicleTrip();

			// If the trip has been fixed to a vehicle. Stop producing paths that include the trip.
			// Mark that the trip cannot be used in any path:
			ptrScheduleNode = _mapScheduleNodeLookUp.find(cb->get_subTripNodeData().get_scheduleNodeData().get_index());
			if (ptrScheduleNode != _mapScheduleNodeLookUp.end())
			{
				for (const auto &node : ptrScheduleNode->second)
				{
					if (branch.get_branchValueBool())
					{
						// If there exists another branch that has already revoked the access, then it can't be fixed again.
						// For instance, this can happen if the node also appears in a vehicle-charging fixing.
						// Then, the node may be fixed but on an interior node, which makes it infeasible in the context of this subgraph.
						if(_getNodeData(node).has_access(cb->get_vehicle().get_index()))
						{
							_getNodeData(node).fix_vehicle(cb->get_vehicle().get_index());
							_getNodeData(node).set_fixedPathEndTime(brn.get_vehicleFixedNodeNextMaxEndTime(cb->get_vehicle().get_index(), cb->get_subTripNodeData().get_scheduleNodeData().get_index()));
							_getNodeData(node).set_fixedPathStartTime(brn.get_vehicleFixedNodePrevMinStartTime(cb->get_vehicle().get_index(), cb->get_subTripNodeData().get_scheduleNodeData().get_index()));
						}
						else
						{
							// Else, no other vehicle or this vehicle can use this node in this subgraph:
							_getNodeData(node).fix_infeasible();
						}
					}
					else
					{
						_getNodeData(node).revoke_access(cb->get_vehicle().get_index());
					}
				}
			}
		}
		break;
		default:
			break;
		}
	}
}

eva::sbn::subgraph::SubGraphResourceContainer &eva::sbn::subgraph::SubGraphResourceContainer::operator=(const SubGraphResourceContainer &other)
{
	if (this == &other)
		return *this;
	this->~SubGraphResourceContainer();
	new(this) SubGraphResourceContainer(other);
	return *this;
}

bool eva::sbn::subgraph::operator==(const SubGraphResourceContainer& res_cont_1, const SubGraphResourceContainer& res_cont_2)
{
	return eva::Helper::compare_floats_equal(res_cont_1.accDuals, res_cont_2.accDuals)
		&& eva::Helper::compare_floats_equal(res_cont_1.minChargingDuals,  res_cont_2.minChargingDuals)
		&& eva::Helper::compare_floats_equal(res_cont_1.maxChargingDuals,  res_cont_2.maxChargingDuals)
		&& res_cont_1.timestampEnd == res_cont_2.timestampEnd
		&& res_cont_1.timestampStart == res_cont_2.timestampStart
		&& (Helper::compare_is_subset(res_cont_1.vecAccess, res_cont_2.vecAccess) && Helper::compare_is_subset(res_cont_2.vecAccess, res_cont_1.vecAccess))
		&& res_cont_1.indexFixedVehicle == res_cont_2.indexFixedVehicle;
}

bool eva::sbn::subgraph::operator<(const SubGraphResourceContainer& res_cont_1, const SubGraphResourceContainer& res_cont_2)
{
	if (Helper::compare_is_subset(res_cont_1.vecAccess, res_cont_2.vecAccess))
	{
		if (Helper::compare_floats_smaller(res_cont_1.accDuals + res_cont_1.minChargingDuals, res_cont_2.accDuals + res_cont_2.maxChargingDuals) 
		|| res_cont_1.timestampEnd > res_cont_2.timestampEnd 
		|| res_cont_1.timestampStart < res_cont_2.timestampStart)
			return false;
		else
			return Helper::compare_floats_smaller(res_cont_2.accDuals + res_cont_2.maxChargingDuals, res_cont_1.accDuals + res_cont_1.minChargingDuals) 
			|| res_cont_1.timestampEnd < res_cont_2.timestampEnd 
			|| res_cont_1.timestampStart > res_cont_2.timestampStart;
	}
	else
		return false;
}

bool eva::sbn::subgraph::SubGraphResourceExtensionFunction::operator()(const BoostSubGraph& subGraph, SubGraphResourceContainer& new_cont, const SubGraphResourceContainer& old_cont, const BoostSubGraphArc& arc) const
{
	const NodeData& toData = subGraph[boost::target(arc, subGraph)];

	// Extend the access of the next label:
	// The label access is defined by the intersection of old and new:
	new_cont.isFeasible = false;
	for(Types::Index idxVehicle = 0; idxVehicle < toData.get_vecAccess().size(); ++idxVehicle)
	{
		 // Only need to update if the access is different. Otherwise, it's already copied correctly.
		if(old_cont.vecAccess[idxVehicle] != toData.get_vecAccess()[idxVehicle])
			new_cont.vecAccess[idxVehicle] = Types::AccessType::NOT_ALLOWED;

		if(!new_cont.isFeasible && new_cont.vecAccess[idxVehicle] == Types::AccessType::ALLOWED)
			new_cont.isFeasible = true;
	}

	// Check if the node is infeasible:
	// May have become infeasible because there is no vehicle anymore allowed on the path.
	// Use new_cont.vecAccess to check:
	if(!new_cont.isFeasible)
		return false;

	// Check if the end time is bigger than the allowed time:
	// Doesn't matter if label isn't fixed to a vehicle. Then endtime = inf
	if (toData.get_type() == NodeType::COLLECTIVE_END_SEGMENT)
	{
		// This schedule fragment is not allowed because it has skipped a fixed node somewhere along the path:
		if(new_cont.timestampEnd > old_cont.fixedAllowedEndTime)
			return false;
		else
			return true; // Either the end time-stamp is inf or before the max allowed end-time.
	}
	else if(toData.get_startTime() > old_cont.fixedAllowedEndTime)
		return false; // Otherwise, check if the current start time is bigger than the allowed end time.

	// Update and check the indexFixedVehicle:
	if(toData.get_indexFixedVehicle() != Constants::BIG_INDEX)
	{
		new_cont.indexFixedVehicle = toData.get_indexFixedVehicle();
		new_cont.fixedAllowedEndTime = toData.get_fixedPathEndTime();

		// Before fixing the first node. It must be checked if any fixed vertex was skipped.
		// Afterwards, this is not necessary, because the fixedPathEndTime ensures this.
		if(old_cont.indexFixedVehicle == Constants::BIG_INDEX &&
			old_cont.timestampStart < toData.get_fixedPathStartTime())
			return false;
	}

	// Now check if the end time lies beyond the new and updated start-time:
	// If this is bigger, can be pruned too.
	if (toData.get_endTime() > new_cont.fixedAllowedEndTime)
		return false;
	
	// Finally, update the duals and timestamps for the label:
	if (toData.get_type() == NodeType::START_SEGMENT)
	{
		new_cont.timestampStart = toData.get_startTime();

		// Update the lower bound and upper bound duals:
		new_cont.minChargingDuals = toData.get_min_charging_dual();
		new_cont.maxChargingDuals = toData.get_max_charging_dual();
	}
	else if (toData.get_type() == NodeType::END_SEGMENT)
	{
		new_cont.timestampEnd = toData.get_endTime();
	}
	else
	{
		new_cont.accDuals += toData.get_acc_dual();
	}

	return true;
}
