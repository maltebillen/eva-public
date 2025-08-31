#include "incl/moderator/moderator.h"
#include "evaExceptions.h"

void eva::BranchNode::_initialise_root(const OptimisationInput& optinput)
{
	// Initialsie the map with vehicle fixings:
	_vecVehicleFixings.resize(optinput.get_vehicles().get_vec().size());

	// For all maintenances, create a "branch" at initialisition. 
	// These maintenance activities are non-negotiable, and must be part of a feasible schedule:
	for (const SubScheduleMaintenanceNodeData& subMaintenance : optinput.get_vecMaintenances())
	{
		if (subMaintenance.get_ptrMaintenanceNodeData()->get_maintenance().is_assigned())
		{
			_vecBranches.push_back(
				Branch(
					true,
					1.0,
					BranchVehicleMaintenance(
						optinput.get_vehicle(subMaintenance.get_ptrMaintenanceNodeData()->get_maintenance().get_indexVehicle()),
						subMaintenance)));

			// And, ensure the nodes appear in the vehicle fixings:
			_vecVehicleFixings[subMaintenance.get_ptrMaintenanceNodeData()->get_maintenance().get_indexVehicle()].push_back(subMaintenance.get_scheduleNodeData().get_index());
		}
	}
}

void eva::BranchNode::_initialise_child(const Branch &newBranch, const OptimisationInput &optinput)
{
	_vecBranches.push_back(newBranch);
	_update_vehicleFixings(newBranch);
	_prepare_fixings(optinput);
}

void eva::BranchEvaluator::_initialise(const OptimisationInput& optinput)
{
	// Initialise the branch evaluators:
	// TOTAL_VEHICLES
	_eval_branch_total_number_vehicles = std::make_pair(0.0, 0); 

	// TOTAL_TRIPS_UNASSIGNED 
	_eval_branch_total_number_unassigned_trips = std::make_pair(0.0, 0); 

	// VEHICLE_ROTATION
	_vec_eval_branch_vehicle_rotation.resize(optinput.get_vehicles().get_vec().size(), std::make_pair(0.0, 0)); 

	// TRIP_UNASSIGNED
	_vec_eval_branch_trip_unassigned.resize(optinput.get_vecTrips().size(), std::make_pair(0.0, 0)); 

	// VEHICLE_TRIP
	_vec_eval_branch_vehicle_trip.resize(optinput.get_vehicles().get_vec().size(), std::vector<std::pair<double, uint32_t>>(optinput.get_vecTrips().size(), std::make_pair(0.0, 0)));

	// VEHICLE_MAINTENANCE
	_vec_eval_branch_vehicle_maintenance.resize(optinput.get_vehicles().get_vec().size(), std::vector<std::pair<double, uint32_t>>(optinput.get_vecMaintenances().size(), std::make_pair(0.0, 0)));
	
}
void eva::BranchEvaluator::_update_moving_average(std::pair<double, uint32_t> &cur, const double &new_data_point)
{
	cur.second += 1;
	cur.first = ((cur.second - 1) * cur.first + new_data_point)/ cur.second;
}
void eva::BranchEvaluator::update_branch_mean_score(const Branch &branch)
{
	switch(branch.get_type())
	{
		case BranchType::TOTAL_VEHICLES:
			_update_moving_average(_eval_branch_total_number_vehicles, branch.get_strong_branching_score());
			break;
		case BranchType::TOTAL_TRIPS_UNASSIGNED:
			_update_moving_average(_eval_branch_total_number_unassigned_trips, branch.get_strong_branching_score());
		break;
		case BranchType::VEHICLE_ROTATION:
			_update_moving_average(_vec_eval_branch_vehicle_rotation[branch.castBranchVehicleRotation()->get_vehicle().get_index()], branch.get_strong_branching_score());
		break;
		case BranchType::TRIP_UNASSIGNED:
			_update_moving_average(_vec_eval_branch_trip_unassigned[branch.castBranchTripUnassigned()->get_subTripNodeData().get_index()], branch.get_strong_branching_score());
		break;
		case BranchType::VEHICLE_TRIP:
			_update_moving_average(_vec_eval_branch_vehicle_trip[branch.castBranchVehicleTrip()->get_vehicle().get_index()][branch.castBranchVehicleTrip()->get_subTripNodeData().get_index()], branch.get_strong_branching_score());
		break;
		case BranchType::VEHICLE_MAINTENANCE:
			_update_moving_average(_vec_eval_branch_vehicle_maintenance[branch.castBranchVehicleMaintenance()->get_vehicle().get_index()][branch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_index()], branch.get_strong_branching_score());
		break;
		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const auto *cba = branch.castBranchVehicleChargingAfter();
			std::tuple<Types::Index, Types::Index, Types::Index> key = {cba->get_vehicle().get_index(), cba->get_charger().get_index(), cba->get_indexFromScheduleNode()};

			if (_umap_eval_branch_vehicle_charging_after.find(key) != _umap_eval_branch_vehicle_charging_after.end()) {
				_update_moving_average(_umap_eval_branch_vehicle_charging_after.at(key), branch.get_strong_branching_score());
			} else {
				auto res = _umap_eval_branch_vehicle_charging_after.insert({key, std::make_pair(0.0, 0)});
				if (res.second) {
					_update_moving_average(res.first->second, branch.get_strong_branching_score());
				}
			}
		}
		break;
		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const auto *cbb = branch.castBranchVehicleChargingBefore();
			std::tuple<Types::Index, Types::Index, Types::Index> key = {cbb->get_vehicle().get_index(), cbb->get_charger().get_index(), cbb->get_indexToScheduleNode()};

			if (_umap_eval_branch_vehicle_charging_before.find(key) != _umap_eval_branch_vehicle_charging_before.end()) {
				_update_moving_average(_umap_eval_branch_vehicle_charging_before.at(key), branch.get_strong_branching_score());
			} else {
				auto res = _umap_eval_branch_vehicle_charging_before.insert({key, std::make_pair(0.0, 0)});
				if (res.second) {
					_update_moving_average(res.first->second, branch.get_strong_branching_score());
				}
			}
		}
		break;
		default:
		break;
	};
}
const double eva::BranchEvaluator::get_mean_score(const Branch &branch) const
{
	switch(branch.get_type())
	{
		case BranchType::TOTAL_VEHICLES:
			return _eval_branch_total_number_vehicles.first;
		break;
		case BranchType::TOTAL_TRIPS_UNASSIGNED:
			return _eval_branch_total_number_unassigned_trips.first;
		break;
		case BranchType::VEHICLE_ROTATION:
			return _vec_eval_branch_vehicle_rotation[branch.castBranchVehicleRotation()->get_vehicle().get_index()].first;
		break;
		case BranchType::TRIP_UNASSIGNED:
			return _vec_eval_branch_trip_unassigned[branch.castBranchTripUnassigned()->get_subTripNodeData().get_index()].first;
		break;
		case BranchType::VEHICLE_TRIP:
			return _vec_eval_branch_vehicle_trip[branch.castBranchVehicleTrip()->get_vehicle().get_index()][branch.castBranchVehicleTrip()->get_subTripNodeData().get_index()].first;
		break;
		case BranchType::VEHICLE_MAINTENANCE:
			return _vec_eval_branch_vehicle_maintenance[branch.castBranchVehicleMaintenance()->get_vehicle().get_index()][branch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_index()].first;
		break;
		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const auto *cba = branch.castBranchVehicleChargingAfter();
			std::tuple<Types::Index, Types::Index, Types::Index> key = {cba->get_vehicle().get_index(), cba->get_charger().get_index(), cba->get_indexFromScheduleNode()};

			if (_umap_eval_branch_vehicle_charging_after.find(key) != _umap_eval_branch_vehicle_charging_after.end()) {
				return _umap_eval_branch_vehicle_charging_after.at(key).first;
			} else {
				return 0.0;
			}
		}
		break;
		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const auto *cbb = branch.castBranchVehicleChargingBefore();
			std::tuple<Types::Index, Types::Index, Types::Index> key = {cbb->get_vehicle().get_index(), cbb->get_charger().get_index(), cbb->get_indexToScheduleNode()};

			if (_umap_eval_branch_vehicle_charging_before.find(key) != _umap_eval_branch_vehicle_charging_before.end()) {
				return _umap_eval_branch_vehicle_charging_before.at(key).first;
			} else {
				return 0.0;
			}
		}
		break;
		default:
		break;
	};

    return 0.0;
}

void eva::BranchNode::_update_vehicleFixings(const Branch &newBranch)
{
	if (newBranch.get_branchValueBool())
	{
		switch (newBranch.get_type())
		{
		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const auto *cba = newBranch.castBranchVehicleChargingAfter();
			_vecVehicleFixings[cba->get_vehicle().get_index()].push_back(cba->get_indexFromScheduleNode());
		}
		break;
		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const auto *cbb = newBranch.castBranchVehicleChargingBefore();
			_vecVehicleFixings[cbb->get_vehicle().get_index()].push_back(cbb->get_indexToScheduleNode());
		}
		break;
		case BranchType::VEHICLE_MAINTENANCE:
		{
			const auto *cb = newBranch.castBranchVehicleMaintenance();
			_vecVehicleFixings[cb->get_vehicle().get_index()].push_back(cb->get_subMaintenanceNodeData().get_scheduleNodeData().get_index());
		}
		break;
		case BranchType::VEHICLE_TRIP:
		{
			const auto *cb = newBranch.castBranchVehicleTrip();
			_vecVehicleFixings[cb->get_vehicle().get_index()].push_back(cb->get_subTripNodeData().get_scheduleNodeData().get_index());
		}
		break;
		default:
			break;
		}
	}
}

void eva::BranchNode::_prepare_fixings(const OptimisationInput& optinput)
{
	// Create the max end timestamps
	// Sort, and filter duplicates:
	_vecVehicleFixedEndTimeLookup.clear();
	_vecVehicleFixedStartTimeLookup.clear();

	_vecVehicleFixedEndTimeLookup.resize(_vecVehicleFixings.size());
	_vecVehicleFixedStartTimeLookup.resize(_vecVehicleFixings.size());

	Types::DateTime maxEndTime = Constants::MAX_TIMESTAMP;
	Types::DateTime minStartTime = Constants::MAX_TIMESTAMP;
	for (Types::Index indexVec = 0; indexVec < _vecVehicleFixings.size(); ++indexVec )
	{
		// Sort the fixed nodes by time:
		std::sort(_vecVehicleFixings[indexVec].begin(), _vecVehicleFixings[indexVec].end(),
				  [&](const Types::Index &l, const Types::Index &r)
				  {
					  return optinput.get_scheduleGraphNodeData(l).get_startTime() < optinput.get_scheduleGraphNodeData(r).get_startTime();
				  });

		// Remove duplicates:
		auto dup = std::unique(_vecVehicleFixings[indexVec].begin(), _vecVehicleFixings[indexVec].end()); 
   		_vecVehicleFixings[indexVec].erase(dup, _vecVehicleFixings[indexVec].end());

		// Iterate from the back to the front to create the endtime lookups:
		maxEndTime = Constants::MAX_TIMESTAMP;
		for (auto it = _vecVehicleFixings[indexVec].rbegin(); it != _vecVehicleFixings[indexVec].rend(); ++it) {
        	_vecVehicleFixedEndTimeLookup[indexVec].insert(std::make_pair(*it, maxEndTime));
			maxEndTime = optinput.get_scheduleGraphNodeData(*it).get_startTime();
    	}

		// Iterate from the front to the back to create the starttime lookups:
		minStartTime = 0;
		for (auto it = _vecVehicleFixings[indexVec].begin(); it != _vecVehicleFixings[indexVec].end(); ++it) {
			_vecVehicleFixedStartTimeLookup[indexVec].insert(std::make_pair(*it, minStartTime));
			minStartTime = optinput.get_scheduleGraphNodeData(*it).get_endTime();
		}
	}
}

void eva::BranchNode::store_branchOptionsTruncColumnGeneration(const std::vector<Branch> &vecBranchOptions)
{
	// Store and sort the branch options:
	// Filter only vehicle specific branch options. Disregard the other options, and add only to the end.
	std::vector<Branch> vecBackup;
	std::vector<Branch> vecOptions;

	for (const auto& branch : vecBranchOptions)
	{
		switch (branch.get_type())
		{
		case BranchType::VEHICLE_CHARGING_AFTER:
		case BranchType::VEHICLE_CHARGING_BEFORE:
		case BranchType::VEHICLE_TRIP:
		case BranchType::VEHICLE_MAINTENANCE:
			vecOptions.push_back(branch);
			break;
		default:
			vecBackup.push_back(branch);
			break;
		}
	}

	if(vecOptions.empty())
	{
		std::sort(vecBackup.begin(), vecBackup.end(), &Branch::compareLeastFractional);
		_vecSortedBranchOptions = vecBackup;
	}
	else
	{
		std::sort(vecOptions.begin(), vecOptions.end(), &Branch::compareAscending);
		_vecSortedBranchOptions = vecOptions;
	}
}

void eva::BranchNode::store_branchOptionsBranchAndPrice(const std::vector<Branch>& vecBranchOptions)
{
	// Store all branch options:
	// These are dealt with and evaluated during strong branching:
	_vecSortedBranchOptions = vecBranchOptions;
	std::sort(_vecSortedBranchOptions.begin(), _vecSortedBranchOptions.end(), &Branch::compareMostFractional);

	// std::vector<Branch> vecBackup;
	// std::vector<Branch> vecOptions;

	// for (const auto& branch : vecBranchOptions)
	// {
	// 	switch (branch.get_type())
	// 	{
	// 	case BranchType::TOTAL_TRIPS_UNASSIGNED:
	// 	case BranchType::TOTAL_VEHICLES:
	// 	case BranchType::TRIP_UNASSIGNED:
	// 	case BranchType::VEHICLE_ROTATION:
	// 		vecOptions.push_back(branch);
	// 		break; 
	// 	// case BranchType::VEHICLE_CHARGING:
	// 	// case BranchType::VEHICLE_TRIP:
	// 	// case BranchType::VEHICLE_MAINTENANCE:
	// 	// 	vecBackup.push_back(branch);
	// 	// 	break;
	// 	default:
	// 		vecBackup.push_back(branch);
	// 		break;
	// 	}
	// }

	// //Store the branch options
	// _vecSortedBranchOptions = vecOptions;
	// if(vecOptions.empty())
	// {
	// 	_vecSortedBranchOptions = vecBackup;
	// }

	// std::sort(_vecSortedBranchOptions.begin(), _vecSortedBranchOptions.end(), &Branch::compareMostFractional);
	// However, the order is irrelevant, because they will be evaluated during strong branching.

	
}

void eva::BranchNode::writeBranchesToConsole()
{
	for (const Branch &branch : this->_vecBranches)
	{
		switch (branch.get_type())
		{
		case BranchType::TOTAL_TRIPS_UNASSIGNED:
			std::cout << "TOTAL_TRIPS_UNASSIGNED: [" << branch.get_branchValue() << "] "
					  << std::endl;
			break;
		case BranchType::TOTAL_VEHICLES:
			std::cout << "TOTAL_VEHICLES: [" << branch.get_branchValue() << "] "
					  << std::endl;
			break;
		case BranchType::TRIP_UNASSIGNED:
			std::cout << "TRIP_UNASSIGNED: [" << branch.get_branchValue() << "] "
					  << ", Trip: " << branch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index()
					  << std::endl;
			break;
		case BranchType::VEHICLE_CHARGING_AFTER:
			std::cout << "VEHICLE_CHARGING_AFTER: [" << branch.get_branchValue() << "] "
					  << ", Vehicle: " << branch.castBranchVehicleChargingAfter()->get_vehicle().get_index()
					  << ", Charger: " << branch.castBranchVehicleChargingAfter()->get_charger().get_index()
					  << ", Node: " << branch.castBranchVehicleChargingAfter()->get_indexFromScheduleNode()
					  << std::endl;
			break;
		case BranchType::VEHICLE_CHARGING_BEFORE:
			std::cout << "VEHICLE_CHARGING_BEFORE: [" << branch.get_branchValue() << "] "
					  << ", Vehicle: " << branch.castBranchVehicleChargingBefore()->get_vehicle().get_index()
					  << ", Charger: " << branch.castBranchVehicleChargingBefore()->get_charger().get_index()
					  << ", Node: " << branch.castBranchVehicleChargingBefore()->get_indexToScheduleNode()
					  << std::endl;
			break;
		case BranchType::VEHICLE_MAINTENANCE:
			std::cout << "VEHICLE_MAINTENANCE: [" << branch.get_branchValue() << "] "
					  << ", Vehicle: " << branch.castBranchVehicleMaintenance()->get_vehicle().get_index()
					  << ", Node: " << branch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index()
					  << std::endl;
			break;
		case BranchType::VEHICLE_ROTATION:
			std::cout << "VEHICLE_ROTATION: [" << branch.get_branchValue() << "] "
					  << ", Vehicle: " << branch.castBranchVehicleRotation()->get_vehicle().get_index()
					  << std::endl;
			break;
		case BranchType::VEHICLE_TRIP:
			std::cout << "VEHICLE_TRIP: [" << branch.get_branchValue() << "] "
					  << ", Vehicle: " << branch.castBranchVehicleTrip()->get_vehicle().get_index()
					  << ", Node: " << branch.castBranchVehicleTrip()->get_subTripNodeData().get_scheduleNodeData().get_index()
					  << std::endl;
			break;
		default:
			break;
		}
	}
}

void eva::Duals::initialise(const OptimisationInput &optinput)
{
	// Initialise the specific row sizes of the vecDualsChargerCapacity:
	for (const Charger& charger : optinput.get_chargers().get_vec())
	{
		vecCumSumDualsChargerCapacity[charger.get_index()].resize(optinput.get_vecPutOnChargeNodes(charger.get_index()).size());
		for (Types::Index indexPutOn = 0; indexPutOn < vecCumSumDualsChargerCapacity[charger.get_index()].size(); ++indexPutOn)
		{
			vecCumSumDualsChargerCapacity[charger.get_index()][indexPutOn] = std::vector<double>(vecCumSumDualsChargerCapacity[charger.get_index()].size(), Constants::BIG_DOUBLE);
		}
	}
}

const bool eva::SubVehicleSchedule::hasChargingAfter(const Types::Index &indexCharger, const Types::Index &indexFromScheduleNode) const
{
	// Check if the schedule has charging at the charger after the from schedule node:
	for(const ChargingSchedule& cs : this->vecChargingSchedule)
	{
		if(cs.indexCharger == indexCharger && cs.indexFromScheduleNode == indexFromScheduleNode)
		{
			return true;
		}
	}

	return false;
}

const bool eva::SubVehicleSchedule::hasChargingBefore(const Types::Index &indexCharger, const Types::Index &indexToScheduleNode) const
{
	// Check if the schedule has charging at the charger before the to schedule node:
	for(const ChargingSchedule& cs : this->vecChargingSchedule)
	{
		if(cs.indexCharger == indexCharger && cs.indexToScheduleNode == indexToScheduleNode)
		{
			return true;
		}
	}

	return false;
}

const bool eva::SubVehicleSchedule::isSubsetOf(const SubVehicleSchedule &other) const
{
    // 1. Check if generally all sets are smaller or equal.
	// If there is any set that is strictly bigger, there must be one item that is not included in other.
	if(this->vecTripNodeIndexes.size() > other.vecTripNodeIndexes.size()
		|| this->vecMaintenanceNodesIndexes.size() > other.vecMaintenanceNodesIndexes.size()
		|| this->vecChargingSchedule.size() < other.vecChargingSchedule.size()
	)
		return false;
	
	// 2. Check if all of the trips are included in the other schedule:
	for(const auto& idxTrip : this->vecTripNodeIndexes)
	{
		if(!other.hasTrip(idxTrip))
			return false; // Return false if there exists at least one trip that is not included.
	}

	// 3. Check if all of the maintenance slots are included in the other schedule:
	for(const auto& idxMaintenance : this->vecMaintenanceNodesIndexes)
	{
		if(!other.hasMaintenance(idxMaintenance))
			return false; // Return false if there exists at least one maintenance slot that is not included.
	}

	// 5. Check if the charging slots are identical or smaller:
	bool found = false;
	for(const auto& refChargingSession : other.vecChargingSchedule)
	{
		found = false;
		for(const auto& chargingSession : this->vecChargingSchedule)
		{
			if(refChargingSession.indexCharger == chargingSession.indexCharger
				&& refChargingSession.indexPutOnCharge >= chargingSession.indexPutOnCharge
				&& refChargingSession.indexTakeOffCharge <= chargingSession.indexTakeOffCharge)
				{
					found = true;
					break;
				}
		}
		if(!found)
			return false;
	}

	// If it gets here, the column is a subset of "other":
	return true;
}

const bool eva::SubVehicleSchedule::isFeasibleInBranchNode(const BranchNode &brn) const
{
	for (const Branch &branch : brn.get_vecBranches())
	{
		switch (branch.get_type())
		{
		case BranchType::VEHICLE_ROTATION:
		{
			const BranchVehicleRotation *bvr = branch.castBranchVehicleRotation();
			if (bvr->get_vehicle().get_index() == this->indexVehicle && !branch.get_branchValueBool())
			{
				return false;
			}
		}
		break;

		case BranchType::TRIP_UNASSIGNED:
		{
			const BranchTripUnassigned *btu = branch.castBranchTripUnassigned();

			// If a trip is fixed to be unassigned, then all schedules including the trip can be removed as they will never be included in a solution.
			if (branch.get_branchValueBool() && this->hasTrip(btu->get_subTripNodeData().get_index()))
			{
				return false;
			}
		}
		break;

		case BranchType::VEHICLE_CHARGING_AFTER:
		{
			const BranchVehicleChargingAfter *bvca = branch.castBranchVehicleChargingAfter();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			if (bvca->get_vehicle().get_index() == this->indexVehicle)
			{
				// Bitwise XOR:
				// A) Schedule includes charging but shouldn't.
				// B) Schedule doesn't include charging but should.
				if (branch.get_branchValueBool() ^
					this->hasChargingAfter(bvca->get_charger().get_index(), bvca->get_indexFromScheduleNode()))
				{
					return false;
				}
			}
			else
			{
				if (branch.get_branchValueBool())
				{
					// Check if the schedule has a schedule node that is fixed to another vehicle:
					// These schedules can no longer be feasible, and hence can be discarded.
					if (this->hasScheduleNode(bvca->get_indexFromScheduleNode()))
					{
						// The schedule is not relevant for this branching node:
						return false;
					}
				}
			}
		}
		break;

		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			const BranchVehicleChargingBefore *bvcb = branch.castBranchVehicleChargingBefore();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			if (bvcb->get_vehicle().get_index() == this->indexVehicle)
			{
				// Bitwise XOR:
				// A) Schedule includes charging but shouldn't.
				// B) Schedule doesn't include charging but should.
				if (branch.get_branchValueBool() ^
					this->hasChargingBefore(bvcb->get_charger().get_index(), bvcb->get_indexToScheduleNode()))
				{
					// The schedule is not relevant for this branching node:
					return false;
				}
			}
			else
			{
				if (branch.get_branchValueBool())
				{
					// Check if the schedule has a schedule node that is fixed to another vehicle:
					// These schedules can no longer be feasible, and hence can be discarded.
					if (this->hasScheduleNode(bvcb->get_indexToScheduleNode()))
					{
						// The schedule is not relevant for this branching node:
						return false;
					}
				}
			}
		}
		break;

		case BranchType::VEHICLE_TRIP:
		{
			const BranchVehicleTrip *bvt = branch.castBranchVehicleTrip();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			if (bvt->get_vehicle().get_index() == this->indexVehicle)
			{
				// Bitwise XOR:
				// A) Schedule includes trip but shouldn't.
				// B) Schedule doesn't include trip but should.
				if (branch.get_branchValueBool() ^
					this->hasTrip(bvt->get_subTripNodeData().get_index()))
				{
					// The schedule is not relevant for this branching node:
					return false;
				}
			}
			else
			{
				if (branch.get_branchValueBool())
				{
					// Check if the trip is included in any other vehicle schedule. These must be irrelevant for the solution.
					if (this->hasTrip(bvt->get_subTripNodeData().get_index()))
					{
						// The schedule is not relevant for this branching node:
						return false;
					}
				}
			}
		}
		break;

		case BranchType::VEHICLE_MAINTENANCE:
		{
			const BranchVehicleMaintenance *bvm = branch.castBranchVehicleMaintenance();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			if (bvm->get_vehicle().get_index() == this->indexVehicle)
			{
				// Bitwise XOR:
				// A) Schedule includes maintenance but shouldn't.
				// B) Schedule doesn't include maintenance but should.
				if (branch.get_branchValueBool() ^
					this->hasMaintenance(bvm->get_subMaintenanceNodeData().get_index()))
				{
					// The schedule is not relevant for this branching node:
					return false;
				}
			}
			else
			{
				if (branch.get_branchValueBool())
				{
					// Only if the branch value was "=1" for another vehicle, a maintenance activity has been fixed to another vehicle.
					// Check if the schedule has a schedule using the same maintenance. This variable will no longer be available:
					if (this->hasMaintenance(bvm->get_subMaintenanceNodeData().get_index()))
					{
						// The schedule is not relevant for this branching node:
						return false;
					}
				}
			}
		}
		break;

		default:
			break;
		}
	}

	return true;
}

const double eva::SubVehicleSchedule::get_current_reducedCost(const Duals &duals) const
{
	double sumDualValues = 0.0;

	// 1. Constraint: One Schdedule per Vehicle:
	sumDualValues += duals.vecDualsOneSchedulePerVehicle[indexVehicle];

	// 2. Constraint: Trip Coverage:
	for(const auto& idxTrip : vecTripNodeIndexes)
		sumDualValues += duals.vecDualsTripCoverage[idxTrip];

	// 3. Constraint: Maintenance Coverage:
	for(const auto& idxMaintenance : vecMaintenanceNodesIndexes)
		sumDualValues += duals.vecDualsOneVehiclePerMaintenance[idxMaintenance];

	// 4. Constraint: Charger Usage:
	for(const auto& cs : vecChargingSchedule)
		sumDualValues += duals.vecCumSumDualsChargerCapacity[cs.indexCharger][cs.indexPutOnCharge][cs.indexTakeOffCharge];

	// Return the reduced cost rc = cost - duals:
    return cost - sumDualValues;
}