#include "incl/pricingProblem/pricingProblem.h"
#include <chrono>
#include <omp.h>
#include <random>

void eva::PricingProblem::_initialise()
{
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	switch (_optinput.get_config().get_const_pricing_problem_type())
	{
	case Types::PricingProblemType::TIME_SPACE_NETWORK:
		_tsn.initialise();

		_network_size_nodes = _tsn.get_number_nodes();
		_network_size_arcs = _tsn.get_number_arcs();
		break;

	case Types::PricingProblemType::CONNECTION_SEGMENT_NETWORK:
		_connection_sbn.initialise();
		break;

	case Types::PricingProblemType::CENTRALISED_SEGMENT_NETWORK:
		_centralised_sbn.initialise();
		break;

	default:
		break;
	};

	_network_construction_ms += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
}

eva::PricingProblemResult eva::PricingProblem::_tsn_find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	// Create the result object of the pricing problem:
	PricingProblemResult result;
	result.resSchedule.resize(_optinput.get_vehicles().get_vec().size());

	// Track if the pricing problem was solved to optimality:
	bool isSolvedOptimal = true;

	// Do other preparational things before solving every vehicle individually:
	// a. Check if there is a vehicle that is taken out of rotation in branches:
	std::vector<uint8_t> vecVehicleRotationBool(_optinput.get_vehicles().get_vec().size(), 1);
	for (const Branch& branch : brn.get_vecBranches())
	{
		if (branch.get_type() == BranchType::VEHICLE_ROTATION)
			vecVehicleRotationBool[branch.castBranchVehicleRotation()->get_vehicle().get_index()] = branch.get_branchValueBool();
	}

	// Create a randomly shuffled vector of all vehicle indexes in the rotation:
	std::vector<Types::Index> vecShuffledVehicleRotation = _shuffleVecVehicleRotation(vecVehicleRotationBool);
	
	// Determine if the pricing problem must be solved to optimality or a shortened version can be solved:
	if (solve_all_vehicles)
	{
		_solve_tsn_pricing_problem(vecShuffledVehicleRotation, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);
		result.isOptimal = isSolvedOptimal; // Only true, if all vehicles are solved AND the pricing problem was solved to optimality.
	}
	else
	{
		// Iterate over the vehicles in batches, and only continue if no optimal solution has been found:
		// Split the shuffled vehicles in two:
		uint32_t totalSelect = std::min(_optinput.get_config().get_const_nr_threads(), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#ifdef DEBUG_BUILD
			totalSelect = std::min(static_cast<uint32_t>(2), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#endif // DEBUG_BUILD	

		std::vector<Types::Index> vecVehicleRotationRandomInclude(vecShuffledVehicleRotation.begin(), vecShuffledVehicleRotation.begin() + totalSelect);
		std::vector<Types::Index> vecVehicleRotationRandomExclude(vecShuffledVehicleRotation.begin() + totalSelect, vecShuffledVehicleRotation.end());

		// Iterate until terminate:
		bool stop_criteria = false;
		while(!stop_criteria)
		{	
			// Step 1: Solve the sampled set of vehicles first, and check if a reduced cost column was found:
			_solve_tsn_pricing_problem(vecVehicleRotationRandomInclude, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);

			// Step 2: Check if the pricing problem was solved to optimality in this iteration:
			// Only if the there is no vehicle left, and all labels were accounted for:
			result.isOptimal = isSolvedOptimal && vecVehicleRotationRandomExclude.empty();

			// Step 2: Check if the pricing problem contains at least one negative reduced cost schedule:
			for (const auto &vecSchedules : result.resSchedule)
			{
				if (!stop_criteria)
				{
					for (const auto &schedule : vecSchedules)
					{
						if (Helper::compare_floats_smaller(schedule.reducedCost, 0))
						{
							stop_criteria = true;
							break;
						}
					}
				}
			}

			// Early exit:
			if(stop_criteria)
				break;

			// Step 3: If there are still excluded vehicles, then include these now, and erase from the excluded list:
			if(vecVehicleRotationRandomExclude.empty())
			{
				stop_criteria = true;
			}
			else
			{
				vecVehicleRotationRandomInclude.clear();
				auto iterFirst = vecVehicleRotationRandomExclude.begin();
				auto iterLast = vecVehicleRotationRandomExclude.size() > totalSelect ? vecVehicleRotationRandomExclude.begin() + totalSelect : vecVehicleRotationRandomExclude.end();

				vecVehicleRotationRandomInclude.insert(vecVehicleRotationRandomInclude.end(), iterFirst, iterLast);
				vecVehicleRotationRandomExclude.erase(iterFirst, iterLast);
			}
		}
	}

	// Exit with the pricing problem results:
	_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
	return result;
}

eva::PricingProblemResult eva::PricingProblem::_connection_sbn_find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point &timeOutClock)
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();
	
	// Create the result object of the pricing problem:
	PricingProblemResult result;
	result.resSchedule.resize(_optinput.get_vehicles().get_vec().size());

	// Track if the pricing problem was solved to optimality:
	bool isSolvedOptimal = true;

	// Do other preparational things before solving every vehicle individually:
	// a. Check if there is a vehicle that is taken out of rotation in branches:
	std::vector<uint8_t> vecVehicleRotationBool(_optinput.get_vehicles().get_vec().size(), 1);
	for (const Branch& branch : brn.get_vecBranches())
	{
		if (branch.get_type() == BranchType::VEHICLE_ROTATION)
			vecVehicleRotationBool[branch.castBranchVehicleRotation()->get_vehicle().get_index()] = branch.get_branchValueBool();
	}

	// Create a randomly shuffled vector of all vehicle indexes in the rotation:
	std::vector<Types::Index> vecShuffledVehicleRotation = _shuffleVecVehicleRotation(vecVehicleRotationBool);

	// b. Create the reduced segment network:
	std::chrono::high_resolution_clock::time_point startClockConstruction = std::chrono::high_resolution_clock::now();
	_connection_sbn.create_reduced_graph(duals, brn);
	_network_construction_ms += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClockConstruction).count();
	_network_size_nodes = _connection_sbn.get_number_nodes();
	_network_size_arcs = _connection_sbn.get_number_arcs();

	// c. Solve the pricing problem with segments:
	if (solve_all_vehicles)
	{
		_solve_connection_sbn_pricing_problem(vecShuffledVehicleRotation, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);
		result.isOptimal = isSolvedOptimal; // Only true, if all vehicles are solved AND the pricing problem was solved to optimality.
	}
	else
	{
		// Iterate over the vehicles in batches, and only continue if no optimal solution has been found:
		// Split the shuffled vehicles in two:
		uint32_t totalSelect = std::min(_optinput.get_config().get_const_nr_threads(), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#ifdef DEBUG_BUILD
			totalSelect = std::min(static_cast<uint32_t>(2), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#endif // DEBUG_BUILD	

		std::vector<Types::Index> vecVehicleRotationRandomInclude(vecShuffledVehicleRotation.begin(), vecShuffledVehicleRotation.begin() + totalSelect);
		std::vector<Types::Index> vecVehicleRotationRandomExclude(vecShuffledVehicleRotation.begin() + totalSelect, vecShuffledVehicleRotation.end());

		// Iterate until terminate:
		bool stop_criteria = false;
		while(!stop_criteria)
		{	
			// Step 1: Solve the sampled set of vehicles first, and check if a reduced cost column was found:
			_solve_connection_sbn_pricing_problem(vecVehicleRotationRandomInclude, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);

			// Step 2: Check if the pricing problem was solved to optimality in this iteration:
			// Only if the there is no vehicle left, and all labels were accounted for:
			result.isOptimal = isSolvedOptimal && vecVehicleRotationRandomExclude.empty();

			// Step 2: Check if the pricing problem contains at least one negative reduced cost schedule:
			for (const auto &vecSchedules : result.resSchedule)
			{
				if (!stop_criteria)
				{
					for (const auto &schedule : vecSchedules)
					{
						if (Helper::compare_floats_smaller(schedule.reducedCost, 0))
						{
							stop_criteria = true;
							break;
						}
					}
				}
			}

			// Early exit:
			if(stop_criteria)
				break;

			// Step 3: If there are still excluded vehicles, then include these now, and erase from the excluded list:
			if(vecVehicleRotationRandomExclude.empty())
			{
				stop_criteria = true;
			}
			else
			{
				vecVehicleRotationRandomInclude.clear();
				auto iterFirst = vecVehicleRotationRandomExclude.begin();
				auto iterLast = vecVehicleRotationRandomExclude.size() > totalSelect ? vecVehicleRotationRandomExclude.begin() + totalSelect : vecVehicleRotationRandomExclude.end();

				vecVehicleRotationRandomInclude.insert(vecVehicleRotationRandomInclude.end(), iterFirst, iterLast);
				vecVehicleRotationRandomExclude.erase(iterFirst, iterLast);
			}
		}
	}
		
	_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();

	return result;
}

eva::PricingProblemResult eva::PricingProblem::_centralised_sbn_find_neg_reduced_cost_schedule(const Duals &duals, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point &timeOutClock)
{
    // Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();
	
	// Create the result object of the pricing problem:
	PricingProblemResult result;
	result.resSchedule.resize(_optinput.get_vehicles().get_vec().size());

	// Track if the pricing problem was solved to optimality:
	bool isSolvedOptimal = true;

	// Do other preparational things before solving every vehicle individually:
	// a. Check if there is a vehicle that is taken out of rotation in branches:
	std::vector<uint8_t> vecVehicleRotationBool(_optinput.get_vehicles().get_vec().size(), 1);
	for (const Branch& branch : brn.get_vecBranches())
	{
		if (branch.get_type() == BranchType::VEHICLE_ROTATION)
			vecVehicleRotationBool[branch.castBranchVehicleRotation()->get_vehicle().get_index()] = branch.get_branchValueBool();
	}

	// Create a randomly shuffled vector of all vehicle indexes in the rotation:
	std::vector<Types::Index> vecShuffledVehicleRotation = _shuffleVecVehicleRotation(vecVehicleRotationBool);

	// b. Create the reduced segment network:
	std::chrono::high_resolution_clock::time_point startClockConstruction = std::chrono::high_resolution_clock::now();
	_centralised_sbn.create_reduced_graph(duals, brn);
	_network_construction_ms += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClockConstruction).count();
	_network_size_nodes = _centralised_sbn.get_number_nodes();
	_network_size_arcs = _centralised_sbn.get_number_arcs();

	// c. Solve the pricing problem with segments:
	if (solve_all_vehicles)
	{
		_solve_centralised_sbn_pricing_problem(vecShuffledVehicleRotation, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);
		result.isOptimal = isSolvedOptimal; // Only true, if all vehicles are solved AND the pricing problem was solved to optimality.
	}
	else
	{
		// Iterate over the vehicles in batches, and only continue if no optimal solution has been found:
		// Split the shuffled vehicles in two:
		uint32_t totalSelect = std::min(_optinput.get_config().get_const_nr_threads(), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#ifdef DEBUG_BUILD
			totalSelect = std::min(static_cast<uint32_t>(2), static_cast<uint32_t>(vecShuffledVehicleRotation.size()));
		#endif // DEBUG_BUILD	

		std::vector<Types::Index> vecVehicleRotationRandomInclude(vecShuffledVehicleRotation.begin(), vecShuffledVehicleRotation.begin() + totalSelect);
		std::vector<Types::Index> vecVehicleRotationRandomExclude(vecShuffledVehicleRotation.begin() + totalSelect, vecShuffledVehicleRotation.end());

		// Iterate until terminate:
		bool stop_criteria = false;
		while(!stop_criteria)
		{	
			// Step 1: Solve the sampled set of vehicles first, and check if a reduced cost column was found:
			_solve_centralised_sbn_pricing_problem(vecVehicleRotationRandomInclude, result, duals, brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);

			// Step 2: Check if the pricing problem was solved to optimality in this iteration:
			// Only if the there is no vehicle left, and all labels were accounted for:
			result.isOptimal = isSolvedOptimal && vecVehicleRotationRandomExclude.empty();

			// Step 2: Check if the pricing problem contains at least one negative reduced cost schedule:
			for (const auto &vecSchedules : result.resSchedule)
			{
				if (!stop_criteria)
				{
					for (const auto &schedule : vecSchedules)
					{
						if (Helper::compare_floats_smaller(schedule.reducedCost, 0))
						{
							stop_criteria = true;
							break;
						}
					}
				}
			}

			// Early exit:
			if(stop_criteria)
				break;

			// Step 3: If there are still excluded vehicles, then include these now, and erase from the excluded list:
			if(vecVehicleRotationRandomExclude.empty())
			{
				stop_criteria = true;
			}
			else
			{
				vecVehicleRotationRandomInclude.clear();
				auto iterFirst = vecVehicleRotationRandomExclude.begin();
				auto iterLast = vecVehicleRotationRandomExclude.size() > totalSelect ? vecVehicleRotationRandomExclude.begin() + totalSelect : vecVehicleRotationRandomExclude.end();

				vecVehicleRotationRandomInclude.insert(vecVehicleRotationRandomInclude.end(), iterFirst, iterLast);
				vecVehicleRotationRandomExclude.erase(iterFirst, iterLast);
			}
		}
	}
		
	_mseconds_runtimeSolver += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();

	return result;
}

void eva::PricingProblem::_tsn_updateNodeAccess(const BranchNode &brn)
{
	// Measure time of the function:
	std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();

	// Update all nodes and arcs that are infeasible, or fixed to a vehicle:
	_tsn.updateAccess(brn);

	_mseconds_filterNodeAccess += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
}

void eva::PricingProblem::_solve_tsn_pricing_problem(const std::vector<Types::Index>& vecVehicleIndexes, PricingProblemResult& result, const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, bool& isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD	
#pragma omp parallel for
	for (int32_t idxIn = 0;
		idxIn < vecVehicleIndexes.size();
		idxIn++)
	{
		result.resSchedule[vecVehicleIndexes[idxIn]] = _tsn.find_neg_reduced_cost_schedule_vehicle(duals, _optinput.get_vehicle(vecVehicleIndexes[idxIn]), brn, include_cost,solve_to_optimal, isSolvedOptimal, timeOutClock);
	}
}

void eva::PricingProblem::_solve_connection_sbn_pricing_problem(const std::vector<Types::Index>& vecVehicleIndexes, PricingProblemResult& result, const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, bool& isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD	
#pragma omp parallel for
	for (int32_t idxIn = 0;
		idxIn < vecVehicleIndexes.size();
		idxIn++)
	{
		result.resSchedule[vecVehicleIndexes[idxIn]] = _connection_sbn.find_neg_reduced_cost_schedule_vehicle(duals, _optinput.get_vehicle(vecVehicleIndexes[idxIn]), brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);
	}
}

void eva::PricingProblem::_solve_centralised_sbn_pricing_problem(const std::vector<Types::Index> &vecVehicleIndexes, PricingProblemResult &result, const Duals &duals, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
	#ifdef DEBUG_BUILD
	omp_set_num_threads(1);
#else
	omp_set_num_threads(_optinput.get_config().get_const_nr_threads());
#endif // DEBUG_BUILD	
#pragma omp parallel for
	for (int32_t idxIn = 0;
		idxIn < vecVehicleIndexes.size();
		idxIn++)
	{
		result.resSchedule[vecVehicleIndexes[idxIn]] = _centralised_sbn.find_neg_reduced_cost_schedule_vehicle(duals, _optinput.get_vehicle(vecVehicleIndexes[idxIn]), brn, include_cost,solve_to_optimal, isSolvedOptimal,timeOutClock);
	}
}

std::vector<eva::Types::Index> eva::PricingProblem::_shuffleVecVehicleRotation(const std::vector<uint8_t>& vecVehicleRotation)
{
	// a. Create a vector with all vehicles in rotation:
	std::vector<Types::Index> vecResult;
	for (Types::Index idx = 0; idx < vecVehicleRotation.size(); ++idx)
	{
		if (vecVehicleRotation[idx] == 1)
			vecResult.push_back(idx);
	}

	// b. Shuffle the vector of indices:
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(vecResult.begin(), vecResult.end(), g);

	return vecResult;
}

eva::PricingProblemResult eva::PricingProblem::find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock)
{
	switch (_optinput.get_config().get_const_pricing_problem_type())
	{
	case Types::PricingProblemType::TIME_SPACE_NETWORK:
		return _tsn_find_neg_reduced_cost_schedule(duals, brn, include_cost,solve_to_optimal, solve_all_vehicles, timeOutClock);

	case Types::PricingProblemType::CONNECTION_SEGMENT_NETWORK:
		return _connection_sbn_find_neg_reduced_cost_schedule(duals, brn, include_cost,solve_to_optimal, solve_all_vehicles, timeOutClock);
	
	case Types::PricingProblemType::CENTRALISED_SEGMENT_NETWORK:
		return _centralised_sbn_find_neg_reduced_cost_schedule(duals, brn, include_cost,solve_to_optimal, solve_all_vehicles, timeOutClock);
		
	default:
		break;
	};

	return PricingProblemResult();
}

void eva::PricingProblem::updateNodeAccess(const BranchNode& brn)
{
	switch (_optinput.get_config().get_const_pricing_problem_type())
	{
	case Types::PricingProblemType::TIME_SPACE_NETWORK:
		_tsn_updateNodeAccess(brn);
		break;

	case Types::PricingProblemType::CONNECTION_SEGMENT_NETWORK:
		_connection_sbn.update_branch_node_fixings(brn);
		break;
		
	case Types::PricingProblemType::CENTRALISED_SEGMENT_NETWORK:
		_centralised_sbn.update_branch_node_fixings(brn);
		break;

	default:
		break;
	};
}

const uint32_t eva::PricingProblem::get_number_segments() const
{
	if (_optinput.get_config().get_const_pricing_problem_type() == Types::PricingProblemType::CONNECTION_SEGMENT_NETWORK)
	{
		return _connection_sbn.get_number_segments();
	}
	else if(_optinput.get_config().get_const_pricing_problem_type() == Types::PricingProblemType::CENTRALISED_SEGMENT_NETWORK)
	{
		return _centralised_sbn.get_number_segments();
	};

	return 0;
}
