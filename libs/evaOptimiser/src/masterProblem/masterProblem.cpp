#include "incl/masterProblem/masterProblem.h"

#include "evaExceptions.h"
#include <chrono>

#ifdef DEBUG_BUILD
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void) msg, exp))

#endif // DEBUG_BUILD

namespace eva
{
	namespace HelperMP
	{
		static const bool isZero(const double& value) { return Helper::compare_floats_equal(0.0, value); };
		static const bool isOne(const double& value) { return Helper::compare_floats_equal(1.0, value); };
		static const bool isInteger(const double& value) { return Helper::compare_floats_equal(std::nearbyint(value), value); };
		static const bool isFractional(const double& value) { return !isInteger(value); };
	}
}

void eva::MasterProblem::_initialise()
{
	// Set environment parameters. 
	_model.setOptionValue("output_flag", false);
	_model.setOptionValue("solver", "simplex");
	_model.setOptionValue("simplex_strategy", 0); // Strategy "Dual-Simplex"
	_model.setOptionValue("presolve", "off");
	_model.setOptionValue("log_to_console", false);
#ifdef DEBUG_BUILD
	_model.setOptionValue("parallel", "off");
	_model.setOptionValue("threads", static_cast<HighsInt>(1));
#else
	_model.setOptionValue("parallel", "choose");
	_model.setOptionValue("threads", static_cast<HighsInt>(_optinput.get_config().get_const_nr_threads()));
#endif

	// Set general model features:
	// Columns are later added to the lp.
	HighsLp lpmodel;
	lpmodel.sense_ = ObjSense::kMinimize;

	// Pass the created LP model on to Highs:
	_model.passModel(lpmodel);

	// Finally, fill the vectors with the variables and constraints:
	_addConstrs();
	_addVars();

	// Set current solution type to undefined:
	_currentSolutionStatus = MasterProblemSolutionStatus::MP_UNDEFINED;	
}

void eva::MasterProblem::_updateCurrentDuals()
{
	// Check the current duals are valid when updating:
	if (_currentHighsSolution.dual_valid)
	{
		// 1. Constraints: OneSchedulePerVehicle:
		for (Types::Index idxConstr = 0; idxConstr < _currentDuals.vecDualsOneSchedulePerVehicle.size(); idxConstr++)
			_currentDuals.vecDualsOneSchedulePerVehicle[idxConstr] = _getDual(_vecConstrOneSchedulePerVehicle[idxConstr].get_constr());

		// 2. Constraints: TripCoverage
		for (Types::Index idxConstr = 0; idxConstr < _currentDuals.vecDualsTripCoverage.size(); idxConstr++)
			_currentDuals.vecDualsTripCoverage[idxConstr] = _getDual(_vecConstrTripCoverage[idxConstr].get_constr());

		// 3. Constraints: OneVehicleMaintenance
		for (Types::Index idxConstr = 0; idxConstr < _currentDuals.vecDualsOneVehiclePerMaintenance.size(); idxConstr++)
			_currentDuals.vecDualsOneVehiclePerMaintenance[idxConstr] = _getDual(_vecConstrOneVehiclePerMaintenance[idxConstr].get_constr());
	
		// 4. Constraints: Cumulative Sum of Duals Charger Capacity
		double cumSum = 0.0;
		double chargerDual = 0.0;
		for (Types::Index idxCharger = 0; idxCharger < _currentDuals.vecCumSumDualsChargerCapacity.size(); idxCharger++)
		{
			for (Types::Index idxConstrPO = 0; idxConstrPO < _currentDuals.vecCumSumDualsChargerCapacity[idxCharger].size(); ++idxConstrPO)
			{
				cumSum = 0.0;
				for (Types::Index idxConstrTO = idxConstrPO; idxConstrTO < _currentDuals.vecCumSumDualsChargerCapacity[idxCharger][idxConstrPO].size(); ++idxConstrTO)
				{
					// Only if the constraint was actually included in the model, check the dual:
					
					if(_getConstrChargerCapacity(idxCharger, idxConstrTO).is_in_RMP())
						cumSum += _getDual(_getConstrChargerCapacity(idxCharger, idxConstrTO).get_constr());

					_currentDuals.vecCumSumDualsChargerCapacity[idxCharger][idxConstrPO][idxConstrTO] = cumSum;
				}
			}
				
		}
	}
	else
	{
		throw LogicError("eva::MasterProblem::_updateCurrentDuals", "Trying to update invalid duals. Model must be resolved first.");
	}
}

const double& eva::MasterProblem::_getDual(const HighsInt& constr)
{
	return _currentHighsSolution.row_dual[constr];
}

void eva::MasterProblem::_updateSolutionStatus()
{
	auto cur_basis = _model.getBasis();

	// Check all variables of the model if they are integer values:
	if (_currentHighsSolution.value_valid)
	{
		// Check integrality. 
		// Assume it's integer, and if a fractional value is found, change to fractional:
		_currentSolutionStatus = MasterProblemSolutionStatus::MP_INTEGER;
		for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
		{
			for (auto& varSchedule : vecVehicleSchedules)
			{
				// Check if the solution is fractional:
				if (HelperMP::isFractional(_currentHighsSolution.col_value[varSchedule.get_var()]))
				{
					_currentSolutionStatus = MasterProblemSolutionStatus::MP_FRACTIONAL;
				}

				// Update the basis status of the variable:
				varSchedule.set_basisStatus(cur_basis.col_status[varSchedule.get_var()]);
			}
		}
	}
	else
	{
		_currentSolutionStatus = MasterProblemSolutionStatus::MP_INVALID;
	}
}

void eva::MasterProblem::_clean_up(const uint32_t& numberDelete)
{
	// Filter out "numberDelete" amount of columns that must be deleted.
	// a. Find the threshold rc.
	double threshold_rc = -Constants::BIG_DOUBLE;
	double rc = 0.0;
	std::vector<Variable<SubVehicleSchedule>*> vecDeleteVars;
	auto cur_basis = _model.getBasis();

	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (auto& varSchedule : vecVehicleSchedules)
		{
			if(vecDeleteVars.size() < numberDelete && cur_basis.col_status[varSchedule.get_var()] != HighsBasisStatus::kBasic)
			{
				// Must never remove basic variables, even if the rmp is too big otherwise
				// Haven't reached the full size of the delete vector yet:
				vecDeleteVars.push_back(&varSchedule);
			}
			else
			{
				// Else, find the variable with the minimum reduced cost to leave the vector of deleted variables:
				threshold_rc = Constants::BIG_DOUBLE;
				Types::Index idxMin = 0;
				for(Types::Index idxDel = 0; idxDel < vecDeleteVars.size(); ++idxDel)
				{
					rc = _currentHighsSolution.col_dual[vecDeleteVars[idxDel]->get_var()];
					if(Helper::compare_floats_smaller(rc, threshold_rc)
						|| (Helper::compare_floats_equal(rc, threshold_rc)
						&& vecDeleteVars[idxDel]->get_var() > vecDeleteVars[idxMin]->get_var()))
					{
						threshold_rc = rc;
						idxMin = idxDel;
					}
				}

				// Now, compare, and check if the variables rc is better than the best threshold at the moment:
				rc = _currentHighsSolution.col_dual[varSchedule.get_var()];
				if (Helper::compare_floats_smaller(threshold_rc, rc) || (Helper::compare_floats_equal(rc, threshold_rc) && varSchedule.get_var() < vecDeleteVars[idxMin]->get_var()))
				{
					vecDeleteVars[idxMin] = &varSchedule;
				}
			}
		}
	}

	// For all variables in the delete vector, set the flag to delete, and remove from highs:
	std::vector<HighsInt> vecDeleteStatus(_model.getNumCols(), 0);
	for(auto& ptrVarSchedule : vecDeleteVars)
	{
		vecDeleteStatus[ptrVarSchedule->get_var()] = 1;
		ptrVarSchedule->flagDelete = true;

		// Store the schedule in the pool of schedules:
		store_schedule_in_pool(*ptrVarSchedule->get_ptr());
	}
	vecDeleteVars.clear(); // Clear the pointers

	// Then, remove the variables:
	// All variables receive either the new updated index, or 0/-1 to indicate they have been deleted.
	HighsBasis oldBasis = _model.getBasis();
	_model.deleteCols(&vecDeleteStatus[0]);

	// Finally, delete the variable from the vector, and update the index for all remaining ones:
#ifdef DEBUG_BUILD
	uint32_t counter_removed = 0;
#endif // DEBUG_BUILD

	// Use to reset the basis:
	HighsBasis updatedBasis = _model.getBasis();
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (auto iter = vecVehicleSchedules.begin(); iter != vecVehicleSchedules.end();)
		{
			// Update if index for schedule is larger than 0:
			if (iter->flagDelete)
			{
				iter = vecVehicleSchedules.erase(iter);
#ifdef DEBUG_BUILD
				++counter_removed;
#endif // DEBUG_BUILD
			}
			else
			{
				iter->set_var(vecDeleteStatus[iter->get_var()]);
				updatedBasis.col_status[iter->get_var()] = iter->get_basisStatus();
				++iter;
			}
		}
	}

	_model.setBasis(updatedBasis);

#ifdef DEBUG_BUILD
	std::cout << "Clean-up removed " << counter_removed << " columns." << std::endl;
#endif // DEBUG_BUILD
}

std::vector<eva::Branch> eva::MasterProblem::get_vecBranchOptions()
{
	std::vector<Branch> result;

	auto botv = _get_vecBranchOptionsTotalVehicles();
	result.insert(result.end(), botv.begin(), botv.end());

	auto botu = _get_vecBranchOptionsTotalNumberTripsUnassigned();
	result.insert(result.end(), botu.begin(), botu.end());

	auto bovr = _get_vecBranchOptionsVehicleRotation();
	result.insert(result.end(), bovr.begin(), bovr.end());

	auto bovt = _get_vecBranchOptionsVehicleTrip();
	result.insert(result.end(), bovt.begin(), bovt.end());

	auto bovm = _get_vecBranchOptionsVehicleMaintenance();
	result.insert(result.end(), bovm.begin(), bovm.end());

	auto bovca = _get_vecBranchOptionsVehicleChargingAfter();
	result.insert(result.end(), bovca.begin(), bovca.end());

	auto bovcb = _get_vecBranchOptionsVehicleChargingBefore();
	result.insert(result.end(), bovcb.begin(), bovcb.end());

	auto bout = _get_vecBranchOptionsUnassignedTrips();
	result.insert(result.end(), bout.begin(), bout.end());

	return result;
}

eva::Solution eva::MasterProblem::get_currentSolution()
{
	Solution sol;
	
	// 1. Save the schedule nodes currently in the optimal solution:
	if (this->get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER)
	{
		sol.objective = get_currentObjective();
		sol.size_unassignedTrips = std::accumulate(_vecVarUnallocatedTrips.begin(), _vecVarUnallocatedTrips.end(), 0, [&](uint32_t res, const Variable<eva::SubScheduleTripNodeData>& var) { return res + std::nearbyint(_currentHighsSolution.col_value[var.get_var()]); });
		sol.size_vehiclesSelected = 0;

		for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
		{
			for (auto& varSchedule : vecVehicleSchedules)
			{
				if (HelperMP::isOne(_currentHighsSolution.col_value[varSchedule.get_var()]))
				{
					// Cast to parent object, erasing all local indexes:
					sol.vecSchedule.push_back(static_cast<VehicleSchedule>(*(varSchedule.get_ptr())));
					sol.size_vehiclesSelected++;
				}
			}
		}
	}

	return sol;
}

const std::vector<std::pair<double, double>> eva::MasterProblem::get_vecLookupColumnBounds() const
{
	std::vector<std::pair<double, double>> res(_model.getNumCols());
	double lb, ub;

	for (HighsInt var = 0; var < _model.getNumCols(); var++)
	{
		lb = _model.getLp().col_lower_[var];
		ub = _model.getLp().col_upper_[var];
		res[var] = std::make_pair(lb,ub);
	}
	
	return res;
}

const bool eva::MasterProblem::check_aux_variables_feasible(const std::vector<std::pair<double, double>>& vecBounds)
{
	// a. Vehicle Selection:
	for (const auto& var : _vecVarVehicleSelected)
	{
		if (Helper::compare_floats_smaller(_currentHighsSolution.col_value[var.get_var()], vecBounds[var.get_var()].first)
			|| Helper::compare_floats_smaller(vecBounds[var.get_var()].second, _currentHighsSolution.col_value[var.get_var()]))
			return false;
	}

	// b. Trips unassigned:
	for (const auto& var : _vecVarUnallocatedTrips)
	{
		if (Helper::compare_floats_smaller(_currentHighsSolution.col_value[var.get_var()], vecBounds[var.get_var()].first)
			|| Helper::compare_floats_smaller(vecBounds[var.get_var()].second, _currentHighsSolution.col_value[var.get_var()]))
			return false;
	}

	// c. Total Number of Vehicles:
	if (!Helper::compare_floats_equal(_currentHighsSolution.col_value[_varsTotalVehiclesSlacks.first.get_var()], 0.0)
		|| !Helper::compare_floats_equal(_currentHighsSolution.col_value[_varsTotalVehiclesSlacks.second.get_var()], 0.0))
		return false;
	
	// d. Total Number of Trips Unassigned:
	if (!Helper::compare_floats_equal(_currentHighsSolution.col_value[_varsTotalTripsUnassignedSlacks.first.get_var()], 0.0)
		|| !Helper::compare_floats_equal(_currentHighsSolution.col_value[_varsTotalTripsUnassignedSlacks.second.get_var()], 0.0))
		return false;

	return true;
}

void eva::MasterProblem::set_auxiliary_objective()
{
	// Reset all variables to 0.0, and only add the variables relevant to the objective of the auxiliary problem:
	for (HighsInt var = 0; var < _model.getNumCols(); var++)
		_model.changeColCost(var, 0.0);

	// Iterate the auxiliary variables:
	// a. Vehicle Selection:
	for (const auto& var : _vecVarVehicleSelected)
	{
		if (Helper::compare_floats_equal(_model.getLp().col_lower_[var.get_var()], 1.0))
		{
			_model.changeColCost(var.get_var(), -1.0); // new obj: (fixing:1 - var) = 1, if infeasible.
		}
		else if (Helper::compare_floats_equal(_model.getLp().col_upper_[var.get_var()], 0.0))
		{
			_model.changeColCost(var.get_var(), 1.0); // new obj: (var - fixing:0) = 1, if infeasible.
		}
	}

	// b. Trips unassigned:
	for (const auto& var : _vecVarUnallocatedTrips)
	{
		if (Helper::compare_floats_equal(_model.getLp().col_lower_[var.get_var()], 1.0))
		{
			_model.changeColCost(var.get_var(), -1.0); // new obj: (fixing:1 - var) = 1, if infeasible.
		}
		else if (Helper::compare_floats_equal(_model.getLp().col_upper_[var.get_var()], 0.0))
		{
			_model.changeColCost(var.get_var(), 1.0); // new obj: (var - fixing:0) = 1, if infeasible.
		}
	}

	// c. Total Number of Vehicles:
	_model.changeColCost(_varsTotalVehiclesSlacks.first.get_var(), 1.0);
	_model.changeColCost(_varsTotalVehiclesSlacks.second.get_var(), 1.0);

	// d. Total Number of Unassigned Trips:
	_model.changeColCost(_varsTotalTripsUnassignedSlacks.first.get_var(), 1.0);
	_model.changeColCost(_varsTotalTripsUnassignedSlacks.second.get_var(), 1.0);
}

void eva::MasterProblem::filterVars(const BranchNode& brn)
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	// 1. Quick reset the variables and constraints:
	// a. Total Vehicles Constraint:
	_model.changeRowBounds(_constrTotalNumberVehicles.get_constr(), -kHighsInf, _optinput.get_vehicles().get_vec().size());

	// b. Total Trips Unassigned Constraint:
	_model.changeRowBounds(_constrTotalNumberTripsUnassigned.get_constr(), -kHighsInf, _optinput.get_vecTrips().size());

	// b. Vehicles Selected:
	for (const auto& var : _vecVarVehicleSelected)
		_model.changeColBounds(var.get_var(), 0.0, 1.0);

	// c. Trips unassigned:
	for (const auto& var : _vecVarUnallocatedTrips)
		_model.changeColBounds(var.get_var(), 0.0, 1.0);

	// d. Vehicle Schedules:
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
		for (auto& varSchedule : vecVehicleSchedules)
			_model.changeColBounds(varSchedule.get_var(), 0.0, kHighsInf);
	

	// 2. Now that the model is reset, change the bounds according to the branches:
	for (const Branch& branch : brn.get_vecBranches())
	{
		switch (branch.get_type())
		{
		case BranchType::TOTAL_VEHICLES:
		{
			// Update the constraint with number of vehicles:
			if (Helper::compare_floats_smaller(branch.get_branchValue() - branch.get_fractionalValue(), 0))
				_model.changeRowBounds(_constrTotalNumberVehicles.get_constr(), _model.getLp().row_lower_[_constrTotalNumberVehicles.get_constr()], branch.get_branchValue());
			else
				_model.changeRowBounds(_constrTotalNumberVehicles.get_constr(), branch.get_branchValue(), _model.getLp().row_upper_[_constrTotalNumberVehicles.get_constr()]);
		}
		break;

		case BranchType::TOTAL_TRIPS_UNASSIGNED:
		{
			// Update the constraint with number of unassigned trips:
			if (Helper::compare_floats_smaller(branch.get_branchValue() - branch.get_fractionalValue(), 0))
				_model.changeRowBounds(_constrTotalNumberTripsUnassigned.get_constr(), _model.getLp().row_lower_[_constrTotalNumberTripsUnassigned.get_constr()], branch.get_branchValue());
			else
				_model.changeRowBounds(_constrTotalNumberTripsUnassigned.get_constr(), branch.get_branchValue(), _model.getLp().row_upper_[_constrTotalNumberTripsUnassigned.get_constr()]);
		}
		break; 

		case BranchType::VEHICLE_ROTATION:
		{
			_model.changeColBounds(_vecVarVehicleSelected[branch.castBranchVehicleRotation()->get_vehicle().get_index()].get_var(), branch.get_branchValue(), branch.get_branchValue());
		}
		break;

		case BranchType::TRIP_UNASSIGNED:
		{
			const BranchTripUnassigned* btu = branch.castBranchTripUnassigned();

			// If a trip is fixed to be unassigned, then all schedules including the trip can be removed as they will never be included in a solution.
			if(branch.get_branchValueBool())
			{
				for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
				{
					for (auto &varSchedule : _vecVarVehicleSchedules[vehicle.get_index()])
					{
						if (varSchedule.get_ptr()->hasTrip(btu->get_subTripNodeData().get_index()))
						{
							// The schedule is not relevant for this branching node:
							_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
						}
					}
				}
			}

			// Finally, set the fixed bounds of the unallocated trip variable:
			_model.changeColBounds(_vecVarUnallocatedTrips[btu->get_subTripNodeData().get_index()].get_var(), branch.get_branchValue(), branch.get_branchValue());
		}
		break;

		case BranchType::VEHICLE_CHARGING_AFTER:
			{
				const BranchVehicleChargingAfter* bvca = branch.castBranchVehicleChargingAfter();

				for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
				{
					for (auto &varSchedule : _vecVarVehicleSchedules[vehicle.get_index()])
					{
						if (vehicle.get_index() == bvca->get_vehicle().get_index())
						{
							// Check if the vehicle has charging after the schedule node:
							// Bitwise XOR: 
							// A) Schedule includes charging but shouldn't.
							// B) Schedule doesn't include charging but should.
							if (branch.get_branchValueBool() ^
								varSchedule.get_ptr()->hasChargingAfter(bvca->get_charger().get_index(),bvca->get_indexFromScheduleNode()))
							{
								// The schedule is not relevant for this branching node:
								_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
							}
						}
						else
						{
							if (branch.get_branchValueBool())
							{
								// Check if the schedule has a schedule node that is fixed to another vehicle:
								// These schedules can no longer be feasible, and hence can be discarded.
								if (varSchedule.get_ptr()->hasScheduleNode(bvca->get_indexFromScheduleNode()))
								{
									// The schedule is not relevant for this branching node:
									_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
								}
							}
						}
					}
				}

				// Finally, since the vehicle is fixed to the charging, it must select a schedule:
				if (branch.get_branchValueBool())
					_model.changeColBounds(_vecVarVehicleSelected[bvca->get_vehicle().get_index()].get_var(), 1.0, 1.0);
			}
		break;

		case BranchType::VEHICLE_CHARGING_BEFORE:
		{
			// Differ approach between branch being 1 or 0:
			const BranchVehicleChargingBefore* bvcb = branch.castBranchVehicleChargingBefore();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			for(const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
			{
				for (auto& varSchedule : _vecVarVehicleSchedules[vehicle.get_index()])
				{
					if(vehicle.get_index() == bvcb->get_vehicle().get_index())
					{
						// Check if the vehicle has charging after the schedule node:
						// Bitwise XOR:
						// A) Schedule includes charging but shouldn't.
						// B) Schedule doesn't include charging but should.
						if (branch.get_branchValueBool() ^
							varSchedule.get_ptr()->hasChargingBefore(bvcb->get_charger().get_index(), bvcb->get_indexToScheduleNode()))
						{
							// The schedule is not relevant for this branching node:
							_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
						}
					}
					else
					{
						if (branch.get_branchValueBool())
						{
							// Check if the schedule has a schedule node that is fixed to another vehicle:
							// These schedules can no longer be feasible, and hence can be discarded.
							if (varSchedule.get_ptr()->hasScheduleNode(bvcb->get_indexToScheduleNode()))
							{
								// The schedule is not relevant for this branching node:
								_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
							}
						}
					}					
				}
			}

			// Finally, since the vehicle is fixed to the charging, it must select a schedule:
			if(branch.get_branchValueBool())
				_model.changeColBounds(_vecVarVehicleSelected[bvcb->get_vehicle().get_index()].get_var(), 1.0, 1.0);
		}
			break;

		case BranchType::VEHICLE_TRIP:
		{
			const BranchVehicleTrip* bvt = branch.castBranchVehicleTrip();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
			{
				for (auto &varSchedule : _vecVarVehicleSchedules[vehicle.get_index()])
				{
					if (vehicle.get_index() == bvt->get_vehicle().get_index())
					{
						// Bitwise XOR:
						// A) Schedule includes trip but shouldn't.
						// B) Schedule doesn't include trip but should.
						if (branch.get_branchValueBool() ^
							varSchedule.get_ptr()->hasTrip(bvt->get_subTripNodeData().get_index()))
						{
							// The schedule is not relevant for this branching node:
							_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
						}
					}
					else
					{
						if (branch.get_branchValueBool())
						{
							// Check if the trip is included in any other vehicle schedule. These must be irrelevant for the solution.
							if (varSchedule.get_ptr()->hasTrip(bvt->get_subTripNodeData().get_index()))
							{
								// The schedule is not relevant for this branching node:
								_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
							}
						}
					}
				}
			}
		
			// Finally, since the vehicle is fixed to the trip, it must select a schedule:
			if (branch.get_branchValueBool())
				_model.changeColBounds(_vecVarVehicleSelected[bvt->get_vehicle().get_index()].get_var(), 1.0, 1.0);
		}
		break;

		case BranchType::VEHICLE_MAINTENANCE:
		{
			const BranchVehicleMaintenance* bvm = branch.castBranchVehicleMaintenance();

			// Iterate over all vehicle schedules of the branches vehicle, and check if they have the charging activity:
			for (const Vehicle &vehicle : _optinput.get_vehicles().get_vec())
			{
				for (auto &varSchedule : _vecVarVehicleSchedules[vehicle.get_index()])
				{
					if (vehicle.get_index() == bvm->get_vehicle().get_index())
					{
						// Bitwise XOR: 
						// A) Schedule includes maintenance but shouldn't.
						// B) Schedule doesn't include maintenance but should.
						if (branch.get_branchValueBool() ^
						varSchedule.get_ptr()->hasMaintenance(bvm->get_subMaintenanceNodeData().get_index()))
						{
							// The schedule is not relevant for this branching node:
							_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
						}
					}
					else
					{
						if (branch.get_branchValueBool())
						{
							// Only if the branch value was "=1" for another vehicle, a maintenance activity has been fixed to another vehicle.
							// Check if the schedule has a schedule using the same maintenance. This variable will no longer be available:
							if (varSchedule.get_ptr()->hasMaintenance(bvm->get_subMaintenanceNodeData().get_index()))
							{
								// The schedule is not relevant for this branching node:
								_model.changeColBounds(varSchedule.get_var(), 0.0, 0.0);
							}
						}
					}
				}
			}

			// Finally, since the vehicle is fixed to the maintenance, it must select a schedule:
			if (branch.get_branchValueBool())
				_model.changeColBounds(_vecVarVehicleSelected[bvm->get_vehicle().get_index()].get_var(), 1.0, 1.0);
		}
			break;

		default:
			break;
		}
	}

	_mseconds_filterVars += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
}

void eva::MasterProblem::writeModel()
{
#ifdef DEBUG_BUILD
	_model.writeModel(_optinput.get_config().get_path_to_output() + "model/model.lp");
	_model.writeSolution(_optinput.get_config().get_path_to_output() + "model/solution.sol");
	_model.writeBasis(_optinput.get_config().get_path_to_output() + "model/basis.bas");
#endif // DEBUG_BUILD
}

bool eva::MasterProblem::solve()
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	HighsStatus returnStatus = _model.run();
	HighsModelStatus modelStatus = _model.getModelStatus();

	if (modelStatus == HighsModelStatus::kUnknown)
	{
		// Highs might have started cycling. Clear the solver and start run again.
		_model.clearSolver();

		returnStatus = _model.run();
		modelStatus = _model.getModelStatus();
	}

	if (returnStatus == HighsStatus::kOk
		&& modelStatus == HighsModelStatus::kOptimal)
	{
		// 1. Update the current solution measures:
		_currentHighsSolution = _model.getSolution();
		_updateSolutionStatus();

		// 2. If the dual is valid, e.g. the master problem was solved as a continious problem, update the duals:
		if (_currentHighsSolution.dual_valid)
		{
			_updateCurrentDuals();
		}

		_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
	}
	else
	{
		_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();

		_currentSolutionStatus = MasterProblemSolutionStatus::MP_INFEASIBLE;
		
		return false;
	}

	return true;
}

void eva::MasterProblem::solveAsMIP()
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	// Change column integrality:
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
		for (auto& varSchedule : vecVehicleSchedules)
			_model.changeColIntegrality(varSchedule.get_var(), HighsVarType::kInteger);
	
	// Set an upper bound to solve the problem:
	// Step 2: Now, solve MIP.
	_model.setOptionValue("time_limit", static_cast<HighsInt>(_optinput.get_config().get_const_branch_and_price_timelimit()));

	// Solve the masterproblem:
	HighsStatus returnStatus = _model.run();
	const HighsInfo& info = _model.getInfo();

	if (returnStatus == HighsStatus::kOk
		&& info.primal_solution_status)
	{
		_currentHighsSolution = _model.getSolution();
		_updateSolutionStatus();
	}

	// Reset the column integrality:
	_model.clearIntegrality();

	_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
}

eva::StatusVarSchedulesAdded eva::MasterProblem::addVarsSchedule(std::vector<std::vector<SubVehicleSchedule>>& vecSchedules, const bool include_cost)
{
	std::vector<SubVehicleSchedule> nonDominatedSchedules;
	StatusVarSchedulesAdded result;
	result.lb = this->get_currentObjective();

	// Check for dominance among the newly generated columns:
	for(auto& vehicleVecSchedules : vecSchedules)
	{
		for (SubVehicleSchedule& schedule : vehicleVecSchedules)
		{
			bool isDominated = false;
			if (_optinput.get_config().get_flag_use_model_cleanup())
			{
				// 2. Check the current batch of new schedules, and only add the ones that are independent:
				auto iterNonDomSchedules = nonDominatedSchedules.begin();
				while (iterNonDomSchedules != nonDominatedSchedules.end())
				{
					// a. Check if the iternondom is subset of current schedule:
					if (iterNonDomSchedules->isSubsetOf(schedule) && Helper::compare_floats_smaller_equal(schedule.reducedCost, iterNonDomSchedules->reducedCost))
					{
						// Then, schedule dominates the current pointer, making it irrelevant (at this point in CG):
						iterNonDomSchedules = nonDominatedSchedules.erase(iterNonDomSchedules);
					}
					else if (schedule.isSubsetOf(*iterNonDomSchedules) && Helper::compare_floats_smaller_equal(iterNonDomSchedules->reducedCost, schedule.reducedCost))
					{
						isDominated = true;
						break;
					}
					else
					{
						++iterNonDomSchedules;
					}
				}
			}

			// Finally add the schedule, if it is non-dominated:
			if(!isDominated)
			{
				nonDominatedSchedules.push_back(schedule);
			}
		}	
	
		// Determine the lower bound:
		if (vehicleVecSchedules.size() > 0)
		{
			result.lb += std::min_element(vehicleVecSchedules.begin(), vehicleVecSchedules.end(), SubVehicleSchedule::compare_rC)->reducedCost;
		}
	}
	
	// Before adding the new columns, check if the RMP will be above the feasible limit. 
	// If yes, delete the columns currently with the worst reduced cost.
	if (_optinput.get_config().get_flag_use_model_cleanup())
	{
		int32_t toDelete = (_model.getNumCols() - _VAR_SCHEDULES_START) + nonDominatedSchedules.size() - _optinput.get_config().get_const_max_number_cols_mp();
		if(toDelete > 0)
			_clean_up(toDelete);
	}

	// Now, add all the non-dominated schedules:
	for(const SubVehicleSchedule& schedule : nonDominatedSchedules)
	{
		// -------------------------------
		// Finally add the schedule:
		std::vector<HighsInt> row_indices;
		std::vector<double> row_coefficients;
		HighsInt number_nonzero_coefficients = 0;

		// 1. Cstr: One Routing Vehicle:
		row_indices.push_back(_getConstrOneSchedulePerVehicle(schedule.indexVehicle).get_constr());
		row_coefficients.push_back(1.0);
		number_nonzero_coefficients++;

		// 2. Cstr: Trip Coverage:
		for (const Types::Index& indexTrip : schedule.vecTripNodeIndexes)
		{
			row_indices.push_back(_getConstrTripCoverage(indexTrip).get_constr());
			row_coefficients.push_back(1.0);
			number_nonzero_coefficients++;
		}

		// 3. Cstr: Maintenance Coverage:
		for (const Types::Index& indexMaintenance : schedule.vecMaintenanceNodesIndexes)
		{
			row_indices.push_back(_getConstrOneVehiclePerMaintenance(indexMaintenance).get_constr());
			row_coefficients.push_back(1.0);
			number_nonzero_coefficients++;
		}

		// 4. Cstr: Charger Capacity
		for (const ChargingSchedule& chargingSchedule : schedule.vecChargingSchedule)
		{
			for (Types::Index indexCharging = chargingSchedule.indexPutOnCharge; indexCharging <= chargingSchedule.indexTakeOffCharge; indexCharging++)
			{
				if (_getConstrChargerCapacity(chargingSchedule.indexCharger, indexCharging).is_in_RMP())
				{
					row_indices.push_back(_getConstrChargerCapacity(chargingSchedule.indexCharger, indexCharging).get_constr());
					row_coefficients.push_back(1.0);
					number_nonzero_coefficients++;
				}
			}
		}

		// Add the schedule to RMP:
		_vecVarVehicleSchedules[schedule.indexVehicle].push_back(
			Variable<SubVehicleSchedule>
		(
			_addColumn(
				include_cost * schedule.cost,
				0.0,
				kHighsInf,
				number_nonzero_coefficients,
				&row_indices[0],
				&row_coefficients[0]
			),
			schedule
		)
		);
	}

	// Save the added columns
	result.columnsAdded = nonDominatedSchedules.size();

	return result;
}

eva::StatusVarSchedulesAdded eva::MasterProblem::addPoolVarsSchedule(const Duals &duals, const BranchNode &brn, const bool include_cost)
{
	std::vector<std::vector<SubVehicleSchedule>> vecPoolVars(_optinput.get_vehicles().get_vec().size());

#ifdef DEBUG_BUILD
	uint32_t addedCols = 0;
#endif

	// Iterate through the pool of previoulsy discovered schedules. 
	// If they have negative reduced cost, and are feasible in the branching node, then add them to the RMP:
	auto iterPoolVars = _vecPoolVehicleSchedule.begin();
	while(iterPoolVars != _vecPoolVehicleSchedule.end())
	{
		// Check if the schedule has negative reduced cost:
		if(Helper::compare_floats_smaller(iterPoolVars->get_current_reducedCost(duals),0.0))
		{
			// Check if the schedule is feasible with the branching node:
			if(iterPoolVars->isFeasibleInBranchNode(brn))
			{
				// Check if already the maximum number of schedules have been added for the vehicle:
				if(vecPoolVars[iterPoolVars->indexVehicle].size() < (_optinput.get_config().get_const_nr_cols_per_vehicle_iter() - 1))
				{
#ifdef DEBUG_BUILD
				++addedCols;
#endif
					vecPoolVars[iterPoolVars->indexVehicle].push_back(*iterPoolVars);
				
					// Second, delete from the pool of vehicle schedules:
					iterPoolVars = _vecPoolVehicleSchedule.erase(iterPoolVars);
	
					// Continue the loop:
					continue;
				}

			}
		}
		++iterPoolVars;
	}

#ifdef DEBUG_BUILD
	if(addedCols > 0)
		std::cout << "Pre-qualified Pool Columns Added: " << addedCols << std::endl;
#endif

	// Finally, re-add the schedules to the RMP:
    return addVarsSchedule(vecPoolVars, include_cost);
}

void eva::MasterProblem::store_schedule_in_pool(const SubVehicleSchedule &schedule)
{
	// First-in-first-out principle on how to manage the pool schedules.
	// If the number of items exceeds the queue, erase from the front first:
	if(_vecPoolVehicleSchedule.size() >= _optinput.get_config().get_const_max_number_cols_mp_pool())
		_vecPoolVehicleSchedule.pop_front();

	// Now, add the deleted schedule to the pool at the back:
	_vecPoolVehicleSchedule.push_back(schedule);
}

void eva::MasterProblem::set_aux_variable_bounds()
{
	// 1. Reset Variable selected:
	for (const auto& var : _vecVarVehicleSelected)
	{
		_model.changeColBounds(var.get_var(), 0.0, 1.0);
	}

	// 2. Reset trip unassigned bounds:
	for (const auto& var : _vecVarUnallocatedTrips)
	{
		_model.changeColBounds(var.get_var(), 0.0, 1.0);
	}

	// 3. Reset Total Number of Vehicles:
	_model.changeColBounds(_varsTotalVehiclesSlacks.first.get_var(), 0.0, _optinput.get_vehicles().get_vec().size());
	_model.changeColBounds(_varsTotalVehiclesSlacks.second.get_var(), 0.0, _optinput.get_vehicles().get_vec().size());

	// 3. Reset Total Number of Vehicles:
	_model.changeColBounds(_varsTotalTripsUnassignedSlacks.first.get_var(), 0.0, _optinput.get_vecTrips().size());
	_model.changeColBounds(_varsTotalTripsUnassignedSlacks.second.get_var(), 0.0, _optinput.get_vecTrips().size());
}

void eva::MasterProblem::reset_objective()
{
	// 1. Reset Variable selected:
	for (const auto& var : _vecVarVehicleSelected)
	{
		_model.changeColCost(var.get_var(), var.get_ptr()->get_cost());
	}

	// 2. Reset trip unassigned cost:
	for (const auto& var : _vecVarUnallocatedTrips)
	{
		_model.changeColCost(var.get_var(), _optinput.get_config().get_cost_uncovered_trip());
	}

	// 3. Reset the objective of all schedule variables:
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
		for (auto& varSchedule : vecVehicleSchedules)
			_model.changeColCost(varSchedule.get_var(), varSchedule.get_ptr()->cost);

	// 4. Total Number of Vehicles:
	_model.changeColCost(_varsTotalVehiclesSlacks.first.get_var(), 0.0);
	_model.changeColCost(_varsTotalVehiclesSlacks.second.get_var(), 0.0);

	// 5. Total Number of Trips Unassigned:
	_model.changeColCost(_varsTotalTripsUnassignedSlacks.first.get_var(), 0.0);
	_model.changeColCost(_varsTotalTripsUnassignedSlacks.second.get_var(), 0.0);
}

void eva::MasterProblem::reset_variable_bounds(const std::vector<std::pair<double, double>>& vecBounds)
{
	// Only reset the variable bounds of the auxiliary columns
	// All routing columns are not touched in this

	// 1. Reset Variable selected:
	for (const auto& var : _vecVarVehicleSelected)
	{
		_model.changeColBounds(var.get_var(), vecBounds[var.get_var()].first, vecBounds[var.get_var()].second);
	}

	// 2. Reset trip unassigned cost:
	for (const auto& var : _vecVarUnallocatedTrips)
	{
		_model.changeColBounds(var.get_var(), vecBounds[var.get_var()].first, vecBounds[var.get_var()].second);
	}

	// 4. Total Number of Vehicles:
	_model.changeColBounds(_varsTotalVehiclesSlacks.first.get_var(), vecBounds[_varsTotalVehiclesSlacks.first.get_var()].first, vecBounds[_varsTotalVehiclesSlacks.first.get_var()].second);
	_model.changeColBounds(_varsTotalVehiclesSlacks.second.get_var(), vecBounds[_varsTotalVehiclesSlacks.second.get_var()].first, vecBounds[_varsTotalVehiclesSlacks.second.get_var()].second);

	// 5. Total Number of Trips Unassigned:
	_model.changeColBounds(_varsTotalTripsUnassignedSlacks.first.get_var(), vecBounds[_varsTotalTripsUnassignedSlacks.first.get_var()].first, vecBounds[_varsTotalTripsUnassignedSlacks.first.get_var()].second);
	_model.changeColBounds(_varsTotalTripsUnassignedSlacks.second.get_var(), vecBounds[_varsTotalTripsUnassignedSlacks.second.get_var()].first, vecBounds[_varsTotalTripsUnassignedSlacks.second.get_var()].second);
}

bool eva::MasterProblem::check_and_update_charger_capacity(const bool add_rmp_rows)
{
	// Check if the charger capacity constraints are all feasible:
	bool capacities_feasible = true;

	// Step 1: Create the twin-vector of charger capacity to compute the value of how many vehicles are present at the charger:
	std::vector<std::vector<double>> vecVehiclesAtChargers;
	for (const auto& vecChargerIntervals : _vecConstrChargerCapacity)
        vecVehiclesAtChargers.emplace_back(vecChargerIntervals.size(), 0.0);

	// Step 2: Iterate over the current solution to sum how many vehicles are currently present at each charger at each time:
	for (auto &vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (auto &varSchedule : vecVehicleSchedules)
		{
			if(Helper::compare_floats_smaller(0.0, _currentHighsSolution.col_value[varSchedule.get_var()]))
			{
				for (const ChargingSchedule &chargingSchedule : varSchedule.get_ptr()->vecChargingSchedule)
				{
					for (Types::Index indexCharging = chargingSchedule.indexPutOnCharge; indexCharging <= chargingSchedule.indexTakeOffCharge; ++indexCharging)
					{
						vecVehiclesAtChargers[chargingSchedule.indexCharger][indexCharging] += _currentHighsSolution.col_value[varSchedule.get_var()];
					}
				}
			}
		}
	}

	// Step 3: Check in which time-intervals, the capacity limit was exceeded:
	std::vector<std::vector<Types::Index>> vecRelIndexesChargerLimitExceeded(_vecConstrChargerCapacity.size());
	Types::Index indexChargingInterval = 0, indexStartExceeded = Constants::BIG_INDEX, indexEndExceeded = Constants::BIG_INDEX;
	for(Types::Index indexCharger = 0; indexCharger < _vecConstrChargerCapacity.size(); ++indexCharger)
	{
		indexChargingInterval = 0;
		while(indexChargingInterval < _vecConstrChargerCapacity[indexCharger].size())
		{
			if(Helper::compare_floats_smaller(_getConstrChargerCapacity(indexCharger, indexChargingInterval).get_ub(),vecVehiclesAtChargers[indexCharger][indexChargingInterval]))
			{
				capacities_feasible = false;

				// Track the first and last index of the sequence where the limit was exceeded:
				indexStartExceeded = indexChargingInterval;
				while(indexChargingInterval < _vecConstrChargerCapacity[indexCharger].size() && 
						Helper::compare_floats_smaller(_getConstrChargerCapacity(indexCharger, indexChargingInterval).get_ub(),vecVehiclesAtChargers[indexCharger][indexChargingInterval]))
				{
					indexEndExceeded = indexChargingInterval;
					++indexChargingInterval;
				}	
				
				// Once the end has been identified, add the two indices:
				// vecRelIndexesChargerLimitExceeded[indexCharger].push_back(indexStartExceeded);
				//if(indexStartExceeded != indexEndExceeded) 
				vecRelIndexesChargerLimitExceeded[indexCharger].push_back(indexEndExceeded);
			}
			else
			{
				++indexChargingInterval;
			}
		}
	}

	// Step 4: Add the constraints:
	// Only necessary if changes are required to be made: 
	if (add_rmp_rows && !capacities_feasible)
	{
		for (Types::Index indexCharger = 0; indexCharger < vecRelIndexesChargerLimitExceeded.size(); ++indexCharger)
		{
#ifdef DEBUG_BUILD
			if(vecRelIndexesChargerLimitExceeded[indexCharger].size() > 0)
				std::cout << "Charger " << indexCharger << ": added " <<  vecRelIndexesChargerLimitExceeded[indexCharger].size() << "." << std::endl;
#endif // DEBUG_BUILD
			for (const auto &indexChargingInterval : vecRelIndexesChargerLimitExceeded[indexCharger])
			{
				_vecConstrChargerCapacity[indexCharger][indexChargingInterval].set_constr(
					_addRow(_vecConstrChargerCapacity[indexCharger][indexChargingInterval].get_lb(), _vecConstrChargerCapacity[indexCharger][indexChargingInterval].get_ub()));
			}
		}

		// Step 5: Update all column coefficients:
		// a. Update the current columns:
		for (auto &vecVehicleSchedules : _vecVarVehicleSchedules)
		{
			for (auto &varSchedule : vecVehicleSchedules)
			{
				for (const ChargingSchedule &chargingSchedule : varSchedule.get_ptr()->vecChargingSchedule)
				{
					for (Types::Index indexCharging = chargingSchedule.indexPutOnCharge; indexCharging <= chargingSchedule.indexTakeOffCharge; indexCharging++)
					{
						if (_getConstrChargerCapacity(chargingSchedule.indexCharger, indexCharging).is_in_RMP())
						{
							_model.changeCoeff(_getConstrChargerCapacity(chargingSchedule.indexCharger, indexCharging).get_constr(), varSchedule.get_var(), 1.0);
						}
					}
				}
			}
		}

		// Step 6: Update the current status to undefined:
		_currentSolutionStatus = MasterProblemSolutionStatus::MP_UNDEFINED;	
	}

	

    return capacities_feasible;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsTotalVehicles()
{
	std::vector<Branch> result;

	double sumSolutionValue = std::accumulate(_vecVarVehicleSelected.begin(), _vecVarVehicleSelected.end(), 0.0,
		[&](double lhs, const eva::Variable<Vehicle>& rhs) { return lhs + _currentHighsSolution.col_value[rhs.get_var()]; });

	if (HelperMP::isFractional(sumSolutionValue))
	{
		result.push_back(
			Branch(
				std::floor(sumSolutionValue) + Helper::compare_floats_smaller(0.5, sumSolutionValue - std::floor(sumSolutionValue)),
				sumSolutionValue,
				BranchTotalVehicles()
			)
		);
	}

	return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsTotalNumberTripsUnassigned()
{
    std::vector<Branch> result;

	double sumSolutionValue = std::accumulate(_vecVarUnallocatedTrips.begin(), _vecVarUnallocatedTrips.end(), 0.0,
		[&](double lhs, const eva::Variable<SubScheduleTripNodeData>& rhs) { return lhs + _currentHighsSolution.col_value[rhs.get_var()]; });

	if (HelperMP::isFractional(sumSolutionValue))
	{
		result.push_back(
			Branch(
				std::floor(sumSolutionValue) + Helper::compare_floats_smaller(0.5, sumSolutionValue - std::floor(sumSolutionValue)),
				sumSolutionValue,
				BranchTotalTripsUnassigned()
			)
		);
	}

	return result;
}
std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsUnassignedTrips()
{
	std::vector<Branch> result;

	for (const auto& varTripUnassigned : _vecVarUnallocatedTrips)
	{
		if (HelperMP::isFractional(_currentHighsSolution.col_value[varTripUnassigned.get_var()]))
		{
			result.push_back(
				Branch(
					Helper::compare_floats_smaller(0.5, _currentHighsSolution.col_value[varTripUnassigned.get_var()] - std::floor(_currentHighsSolution.col_value[varTripUnassigned.get_var()])),
					_currentHighsSolution.col_value[varTripUnassigned.get_var()],
					BranchTripUnassigned(
						*(varTripUnassigned.get_ptr())
					)
				)
			);
		}
	}
	
	return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsVehicleRotation()
{
	std::vector<Branch> result;

	for (const auto& varVehicleRotation : _vecVarVehicleSelected)
	{
		if (HelperMP::isFractional(_currentHighsSolution.col_value[varVehicleRotation.get_var()]))
		{
			result.push_back(
				Branch(
					Helper::compare_floats_smaller_equal(0.5, _currentHighsSolution.col_value[varVehicleRotation.get_var()] - std::floor(_currentHighsSolution.col_value[varVehicleRotation.get_var()])),
					_currentHighsSolution.col_value[varVehicleRotation.get_var()],
					BranchVehicleRotation(
						*(varVehicleRotation.get_ptr())
					)
				)
			);
		}
	}

	return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsVehicleTrip()
{
	std::vector<Branch> result;
	std::vector<std::vector<double>> vecTripAccValues(_optinput.get_vecTrips().size(), std::vector<double>(_optinput.get_vehicles().get_vec().size(), 0.0));

	// Sum the solution values of all trips for all vehicles:
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (auto& varSchedule : vecVehicleSchedules)
		{
			for (const Types::Index& indexTrip : varSchedule.get_ptr()->vecTripNodeIndexes)
			{
				vecTripAccValues[indexTrip][varSchedule.get_ptr()->indexVehicle] += _currentHighsSolution.col_value[varSchedule.get_var()];
			}
		}
	}

	// If the accumulated solution value is fractional, save this as a possible branch:
	for (Types::Index indexTrip = 0; indexTrip < _optinput.get_vecTrips().size(); indexTrip++)
	{
		for (Types::Index indexVehicle = 0; indexVehicle < _optinput.get_vehicles().get_vec().size(); indexVehicle++)
		{
			if (HelperMP::isFractional(vecTripAccValues[indexTrip][indexVehicle]))
			{
				result.push_back(
					Branch(
						Helper::compare_floats_smaller_equal(0.5, vecTripAccValues[indexTrip][indexVehicle] - std::floor(vecTripAccValues[indexTrip][indexVehicle])),
						vecTripAccValues[indexTrip][indexVehicle],
						BranchVehicleTrip(
							_optinput.get_vehicle(indexVehicle),
							_optinput.get_trip(indexTrip)
						)
					)
				);
			}
		}
	}

	return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsVehicleMaintenance()
{
	std::vector<Branch> result;
	std::vector<std::vector<double>> vecMaintenanceAccValues(_optinput.get_vecMaintenances().size(), std::vector<double>(_optinput.get_vehicles().get_vec().size(), 0.0));

	// Sum the solution values of all maintenance for all vehicles:
	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (auto& varSchedule : vecVehicleSchedules)
		{
			for (const Types::Index& indexMaintenance : varSchedule.get_ptr()->vecMaintenanceNodesIndexes)
			{
				vecMaintenanceAccValues[indexMaintenance][varSchedule.get_ptr()->indexVehicle] += _currentHighsSolution.col_value[varSchedule.get_var()];
			}
		}
	}

	// If the accumulated solution value is fractional, save this as a possible branch:
	for (Types::Index indexMaintenance = 0; indexMaintenance < _optinput.get_vecMaintenances().size(); indexMaintenance++)
	{
		for (Types::Index indexVehicle = 0; indexVehicle < _optinput.get_vehicles().get_vec().size(); indexVehicle++)
		{
			if (HelperMP::isFractional(vecMaintenanceAccValues[indexMaintenance][indexVehicle]))
			{
				result.push_back(
					Branch(
						Helper::compare_floats_smaller_equal(0.5, vecMaintenanceAccValues[indexMaintenance][indexVehicle] - std::floor(vecMaintenanceAccValues[indexMaintenance][indexVehicle])),
						vecMaintenanceAccValues[indexMaintenance][indexVehicle],
						BranchVehicleMaintenance(
							_optinput.get_vehicle(indexVehicle),
							_optinput.get_maintenance(indexMaintenance)
						)
					)
				);
			}
		}
	}

	return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsVehicleChargingBefore()
{
	std::vector<Branch> result;

	// Determine the branches of vehicle charging at charger -> schedule node:
	// Vehicle <> Charger <> {To_SN, Val}
	std::vector<std::vector<std::map<Types::Index, double>>> vecMapValues;
	vecMapValues.resize(_optinput.get_vehicles().get_vec().size(), std::vector<std::map<Types::Index, double>>(_optinput.get_chargers().get_vec().size()));
	std::pair<std::map<Types::Index, double>::iterator, bool> emplaceResult;

	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (const auto& varSchedule : vecVehicleSchedules)
		{
			if (Helper::compare_floats_smaller(0.0, _currentHighsSolution.col_value[varSchedule.get_var()]))
			{
				for (const ChargingSchedule& cs : varSchedule.get_ptr()->vecChargingSchedule)
				{
					// Check if the to-node is feasible:
					if(cs.indexToScheduleNode != Constants::BIG_INDEX)
					{
						// Check if the map currently has the sn already for the vehicle at the charger:
						emplaceResult = vecMapValues[varSchedule.get_ptr()->indexVehicle][cs.indexCharger].emplace(cs.indexToScheduleNode, _currentHighsSolution.col_value[varSchedule.get_var()]);
						if (!emplaceResult.second)
						{
							emplaceResult.first->second += _currentHighsSolution.col_value[varSchedule.get_var()];
						}
					}
				}
			}
		}
	}

	for (const auto& vehicle : _optinput.get_vehicles().get_vec())
	{
		for(const auto& charger : _optinput.get_chargers().get_vec())
		{
			for (const auto& pair : vecMapValues[vehicle.get_index()][charger.get_index()]) {
				if (HelperMP::isFractional(pair.second))
				{
					result.push_back(
						Branch(
							Helper::compare_floats_smaller_equal(0.5, pair.second - std::floor(pair.second)),
							pair.second,
							BranchVehicleChargingBefore(
								vehicle,
								charger,
								pair.first)
							)
						);
				}
			}
		}
	}

    return result;
}

std::vector<eva::Branch> eva::MasterProblem::_get_vecBranchOptionsVehicleChargingAfter()
{
    std::vector<Branch> result;

	// Determine the branches of schedule node -> vehicle charging at charger.

	// Vehicle <> Charger <> {From_SN, Val}
	std::vector<std::vector<std::map<Types::Index, double>>> vecMapValues;
	vecMapValues.resize(_optinput.get_vehicles().get_vec().size(), std::vector<std::map<Types::Index, double>>(_optinput.get_chargers().get_vec().size()));
	std::pair<std::map<Types::Index, double>::iterator, bool> emplaceResult;

	for (auto& vecVehicleSchedules : _vecVarVehicleSchedules)
	{
		for (const auto& varSchedule : vecVehicleSchedules)
		{
			if (Helper::compare_floats_smaller(0.0, _currentHighsSolution.col_value[varSchedule.get_var()]))
			{
				for (const ChargingSchedule& cs : varSchedule.get_ptr()->vecChargingSchedule)
				{
					// Check if the to-node is feasible:
					if(cs.indexFromScheduleNode != Constants::BIG_INDEX)
					{
						// Check if the map currently has the sn already for the vehicle at the charger:
						emplaceResult = vecMapValues[varSchedule.get_ptr()->indexVehicle][cs.indexCharger].emplace(cs.indexFromScheduleNode, _currentHighsSolution.col_value[varSchedule.get_var()]);
						if (!emplaceResult.second)
						{
							emplaceResult.first->second += _currentHighsSolution.col_value[varSchedule.get_var()];
						}
					}
				}
			}
		}
	}

	for (const auto& vehicle : _optinput.get_vehicles().get_vec())
	{
		for(const auto& charger : _optinput.get_chargers().get_vec())
		{
			for (const auto& pair : vecMapValues[vehicle.get_index()][charger.get_index()]) {
				if (HelperMP::isFractional(pair.second))
				{
					result.push_back(
						Branch(
							Helper::compare_floats_smaller_equal(0.5, pair.second - std::floor(pair.second)),
							pair.second,
							BranchVehicleChargingAfter(
								vehicle,
								charger,
								pair.first)
							)
						);
				}
			}
		}
	}

    return result;
}

HighsInt eva::MasterProblem::_addColumn(const double& cost, const double& lb, const double& ub, const HighsInt& number_nz_coeff, const HighsInt* indices, const double* values)
{
	HighsInt indexCol = _model.getNumCol();
	HighsStatus colStatus = _model.addCol(
		cost,
		lb,
		ub,
		number_nz_coeff,
		indices,
		values
	);

	if (colStatus != HighsStatus::kOk)
	{
		throw LogicError("eva::MasterProblem::_addColumn", "Error when trying to add column to highs model.");
	}

	return indexCol;
}

HighsInt eva::MasterProblem::_addRow(const double& lb, const double& ub, const HighsInt& number_nz_coeff, const HighsInt* indices, const double* values)
{
	HighsInt indexRow = _model.getNumRow();
	HighsStatus rowStatus = _model.addRow(
		lb,
		ub,
		number_nz_coeff,
		indices,
		values
	);

	if (rowStatus != HighsStatus::kOk)
	{
		throw LogicError("eva::MasterProblem::_addRow","Error when trying to add row to highs model.");
	}

	return indexRow;
}

HighsInt eva::MasterProblem::_addRow(const double& lb, const double& ub)
{
	return _addRow(lb, ub, 0, nullptr, nullptr);
}

HighsInt eva::MasterProblem::_addRow(const double& val)
{
	return _addRow(val, val, 0, nullptr, nullptr);
}

void eva::MasterProblem::_addVars()
{
	_addVarTotalVehiclesSlack();
	_addVarTotalNumberTripsUnassignedSlack();
	_addVarsVehicleSelected();
	_addVarsUnallocatedTrips();

	_VAR_SCHEDULES_START = _model.getNumCols();

	// Initialise the vector with variables for schedules:
	_vecVarVehicleSchedules.resize(_optinput.get_vehicles().get_vec().size());
}

void eva::MasterProblem::_addConstrs()
{
	_addConstrTotalNumberVehicles();
	_addConstrTotalNumberTripsUnassigned();
	_addConstrOneRoutingPerVehicle();
	_addConstrTripCoverage();
	_addConstrOneVehiclePerMaintenance();
	_addTempConstrChargerCapacity();
}

void eva::MasterProblem::_addConstrTotalNumberVehicles()
{
	_constrTotalNumberVehicles = Constraint<Types::Index>(_addRow(-kHighsInf, _optinput.get_vehicles().get_vec().size()), Constants::BIG_INDEX, -kHighsInf, _optinput.get_vehicles().get_vec().size());
}

void eva::MasterProblem::_addConstrTotalNumberTripsUnassigned()
{
	_constrTotalNumberTripsUnassigned = Constraint<Types::Index>(_addRow(-kHighsInf, _optinput.get_vecTrips().size()), Constants::BIG_INDEX, -kHighsInf, _optinput.get_vecTrips().size());
}

void eva::MasterProblem::_addConstrOneRoutingPerVehicle()
{
	_vecConstrOneSchedulePerVehicle.resize(_optinput.get_vehicles().get_vec().size());

	for (const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
	{
		_vecConstrOneSchedulePerVehicle[vehicle.get_index()] = Constraint<Vehicle>(_addRow(0.0), vehicle, 0.0, 0.0);
	}
}

void eva::MasterProblem::_addConstrTripCoverage()
{
	_vecConstrTripCoverage.resize(_optinput.get_vecTrips().size());

	for (const auto& tripNode : _optinput.get_vecTrips())
	{
		_vecConstrTripCoverage[tripNode.get_index()] = Constraint<SubScheduleTripNodeData>(_addRow(1.0), tripNode, 1.0, 1.0); // Assignment of exactly one vehicle to trip only allowed!
	}
}

void eva::MasterProblem::_addConstrOneVehiclePerMaintenance()
{
	_vecConstrOneVehiclePerMaintenance.resize(_optinput.get_vecMaintenances().size());

	for (const auto& maintenanceNode : _optinput.get_vecMaintenances())
	{
		_vecConstrOneVehiclePerMaintenance[maintenanceNode.get_index()] = Constraint<SubScheduleMaintenanceNodeData>(_addRow(-kHighsInf, 1.0), maintenanceNode, -kHighsInf, 1.0);
	}
}

void eva::MasterProblem::_addTempConstrChargerCapacity()
{
	uint32_t atCharger = 0;
	_vecConstrChargerCapacity.resize(_optinput.get_chargers().get_vec().size());

	for (const Charger& charger : _optinput.get_chargers().get_vec())
	{

#if DEBUG_BUILD
		// PutOnCharge or TakeOffCharge nodes should have the same length. Therefore, it doesn't matter which nodes are used to create the interval.
		assertm(_optinput.get_vecPutOnChargeNodes(charger.get_index()).size() == _optinput.get_vecTakeOffChargeNodes(charger.get_index()).size(), "Constr:ChargerCapacity, different sized vectors. Inconsistency may lead to errors. Stop.");
#endif

		atCharger = 0; 
		_vecConstrChargerCapacity[charger.get_index()].resize(_optinput.get_vecPutOnChargeNodes(charger.get_index()).size());

		for (int32_t indexInterval = _optinput.get_vecPutOnChargeNodes(charger.get_index()).size() - 1; indexInterval >= 0; --indexInterval)
		{
			// Just storing the constraint but not adding to the RMP!
			_vecConstrChargerCapacity[charger.get_index()][indexInterval] 
				= Constraint<SubSchedulePutOnChargeNodeData>(_optinput.get_putOnCharge(charger.get_index(), indexInterval), -kHighsInf, (charger.get_capacity() - atCharger));

			// Prepare next step:
			atCharger += _optinput.get_scheduleGraph().get_numberOutgoingArcs(_optinput.get_takeOffCharge(charger.get_index(), indexInterval).get_scheduleNodeData().get_index())
				- _optinput.get_scheduleGraph().get_numberOutgoingArcs(_optinput.get_putOnCharge(charger.get_index(), indexInterval).get_scheduleNodeData().get_index());
		}
	}
}

void eva::MasterProblem::_addVarTotalVehiclesSlack()
{
	std::vector<HighsInt> row_indices;
	std::vector<double> row_coefficients;
	HighsInt number_nonzero_coefficients = 0;

	// Positive:
	// Add the column coefficients to the respective vector:
	row_indices.push_back(_getConstrTotalNumberVehicles().get_constr());
	row_coefficients.push_back(1.0);
	number_nonzero_coefficients++;

	// Only activated in the auxiliary problem:
	auto varSlackPos = Variable<Types::Index>(
		_addColumn(
			0.0,
			0.0,
			0.0,
			number_nonzero_coefficients,
			&row_indices[0],
			&row_coefficients[0]
		),
		Constants::BIG_INDEX
	);

	// Negative:
	row_coefficients.clear();
	row_coefficients.push_back(-1.0);

	auto varSlackNeg = Variable<Types::Index>(
		_addColumn(
			0.0,
			0.0,
			0.0,
			number_nonzero_coefficients,
			&row_indices[0],
			&row_coefficients[0]
		),
		Constants::BIG_INDEX
	);

	// Store the variables:
	_varsTotalVehiclesSlacks = std::make_pair(varSlackPos, varSlackNeg);
}

void eva::MasterProblem::_addVarTotalNumberTripsUnassignedSlack()
{
	std::vector<HighsInt> row_indices;
	std::vector<double> row_coefficients;
	HighsInt number_nonzero_coefficients = 0;

	// Positive:
	// Add the column coefficients to the respective vector:
	row_indices.push_back(_getConstrTotalNumberTripsUnassigned().get_constr());
	row_coefficients.push_back(1.0);
	number_nonzero_coefficients++;

	// Only activated in the auxiliary problem:
	auto varSlackPos = Variable<Types::Index>(
		_addColumn(
			0.0,
			0.0,
			0.0,
			number_nonzero_coefficients,
			&row_indices[0],
			&row_coefficients[0]
		),
		Constants::BIG_INDEX
	);

	// Negative:
	row_coefficients.clear();
	row_coefficients.push_back(-1.0);

	// Only activated in the auxiliary problem:
	auto varSlackNeg = Variable<Types::Index>(
		_addColumn(
			0.0,
			0.0,
			0.0,
			number_nonzero_coefficients,
			&row_indices[0],
			&row_coefficients[0]
		),
		Constants::BIG_INDEX
	);

	// Store the variables:
	_varsTotalTripsUnassignedSlacks = std::make_pair(varSlackPos, varSlackNeg);
}

void eva::MasterProblem::_addVarsUnallocatedTrips()
{
	_vecVarUnallocatedTrips.resize(_optinput.get_vecTrips().size());

	for (const auto& tripNode : _optinput.get_vecTrips())
	{
		std::vector<HighsInt> row_indices;
		std::vector<double> row_coefficients;
		HighsInt number_nonzero_coefficients = 0;

		// Add the column coefficients to the total number of trips constraint:
		row_indices.push_back(_getConstrTotalNumberTripsUnassigned().get_constr());
		row_coefficients.push_back(1.0);
		number_nonzero_coefficients++;

		// Add the column coefficients to the respective vector:
		row_indices.push_back(_getConstrTripCoverage(tripNode.get_index()).get_constr());
		row_coefficients.push_back(1.0);
		number_nonzero_coefficients++;

		_vecVarUnallocatedTrips[tripNode.get_index()] =
			Variable<SubScheduleTripNodeData>(
				_addColumn(
					_optinput.get_config().get_cost_uncovered_trip(),
					0.0,
					1.0,
					number_nonzero_coefficients,
					&row_indices[0],
					&row_coefficients[0]
				),
				tripNode
			);
	}
}

void eva::MasterProblem::_addVarsVehicleSelected()
{
	_vecVarVehicleSelected.resize(_optinput.get_vehicles().get_vec().size());

	for (const Vehicle& vehicle : _optinput.get_vehicles().get_vec())
	{
		std::vector<HighsInt> row_indices;
		std::vector<double> row_coefficients;
		HighsInt number_nonzero_coefficients = 0;

		// Add the column coefficients to the respective vector:
		row_indices.push_back(_getConstrTotalNumberVehicles().get_constr());
		row_coefficients.push_back(1.0);
		number_nonzero_coefficients++;

		row_indices.push_back(_getConstrOneSchedulePerVehicle(vehicle.get_index()).get_constr());
		row_coefficients.push_back(-1.0);
		number_nonzero_coefficients++;

		_vecVarVehicleSelected[vehicle.get_index()] =
			Variable<Vehicle>(
				_addColumn(
					vehicle.get_cost() * _optinput.get_config().get_flag_minimise_number_vehicles(),
					0.0,
					1.0,
					number_nonzero_coefficients,
					&row_indices[0],
					&row_coefficients[0]
				),
				vehicle
			);
	}
}