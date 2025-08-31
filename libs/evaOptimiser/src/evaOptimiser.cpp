#include "evaOptimiser.h"

#include <iostream>
#include <chrono>
#include <queue>

#include "moderator/branch.h"
#include "moderator/OptimisationInput.h"
#include "evaExceptions.h"

#include "incl/masterProblem/masterProblem.h"
#include "incl/pricingProblem/pricingProblem.h"


namespace eva
{
	namespace Algorithms
	{
		struct ColumnGenerationResults
		{
			double lb = -Constants::BIG_DOUBLE;
			bool isFeasible = false;
		};

		static bool aux_column_generation(OptimisationInput& optinput, const BranchNode& brn, MasterProblem& mp, PricingProblem& pp, const std::chrono::high_resolution_clock::time_point& timeOutClockPH)
		{
			// This runs because an infeasibility is suspected.
			// Therefore, run this auxiliary problem to either confirm infeasibility -> return, false.
			// Or, restore feasibility in MP -> return true.
			// Function must be self-contained and must restore the mp and pp to its previous state with no problems.

			// 0. Initialisation:
			bool stoppingCriteriaReached = false;
			std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();
			std::chrono::high_resolution_clock::time_point timeOutClock = std::min(std::chrono::high_resolution_clock::now() + std::chrono::seconds(optinput.get_config().get_const_column_generation_timelimit()), timeOutClockPH);
			eva::PricingProblemResult pricingProblemResult;
			eva::StatusVarSchedulesAdded mpAddingVarsResult;

			// 0. Stats initialisation:
			Stats::PerformanceDetail stats_pd_aux;
			stats_pd_aux.branchType = "Auxiliary Column Generation";
			optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd_aux);

			Stats::PerformanceDetail stats_pd;
			stats_pd.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
			stats_pd.indexBranchingNode = brn.get_index();
			stats_pd.time_mpSolver = mp.get_totalRuntimeSolver();
			stats_pd.time_ppSolver = pp.get_totalRuntimeSolver();
			stats_pd.mp_size_constraints = mp.get_sizeConstraints();
			stats_pd.mp_size_variables = mp.get_sizeVariables();
			stats_pd.pp_network_construction_ms = pp.get_network_construction_ms();
			stats_pd.columnsAdded = 0;
			stats_pd.lb_relaxed = -Constants::BIG_DOUBLE;
			stats_pd.ub_relaxed = Constants::BIG_DOUBLE;
			stats_pd.iteration = 0;

			// 1. Save the MP bounds of all auxiliary variables:
			auto vecColumnBounds = mp.get_vecLookupColumnBounds(); // may include some routing columns, but they are irrelevnt.
			mp.set_auxiliary_objective(); // update the objective funciton of the MP, and save the number of required fixings to be feasible.
			mp.set_aux_variable_bounds();

			// 2. Solve the Aux-CG to determine feasibility:
			uint32_t columnsAdded = Constants::BIG_UINTEGER;
			bool mp_solved = false, mp_charger_capacity_feasible = false;
			double cur_lb = -Constants::BIG_DOUBLE;

			while ((columnsAdded > 0 || !mp_charger_capacity_feasible))
			{
				++stats_pd.iteration;
				stats_pd.time_mpSolver = mp.get_totalRuntimeSolver(); // Init the runtime, to later adjust.
				stats_pd.time_ppSolver = pp.get_totalRuntimeSolver(); // Init the runtime, to later adjust.
				stats_pd.pp_network_construction_ms = pp.get_network_construction_ms();

				// a. Solve the MP:
				mp_solved = mp.solve();
				if (!mp_solved)
					throw LogicError("eva::Algorithms::aux_column_generation", "Solving Auxiliary MP. Must always be feasible. Somthing went wrong.");

				if(std::chrono::high_resolution_clock::now() >= timeOutClock)
					break;
				
				// a. First, investigate, if any columns from the deleted pool can be added:
				mpAddingVarsResult = mp.addPoolVarsSchedule(mp.get_currentDuals(), brn, false);

				// b. If no variables have already been added to the RMP, then run the pricing problem:
				if (mpAddingVarsResult.columnsAdded <= 0)
				{
					// 1. Solve the pricing problem:
					pricingProblemResult = pp.find_neg_reduced_cost_schedule(mp.get_currentDuals(), brn, false,false, true, timeOutClock);
					mpAddingVarsResult = mp.addVarsSchedule(pricingProblemResult.resSchedule, false);

					if(pricingProblemResult.isOptimal)
						cur_lb = mpAddingVarsResult.lb;
				}

				// c. Update the column counter added:
				columnsAdded = mpAddingVarsResult.columnsAdded;

				stats_pd.ub_relaxed = mp.get_currentObjective();
				stats_pd.lb_relaxed = cur_lb;
				stats_pd.time_mpSolver = mp.get_totalRuntimeSolver() - stats_pd.time_mpSolver;
				stats_pd.time_ppSolver = pp.get_totalRuntimeSolver() - stats_pd.time_ppSolver;
				stats_pd.columnsAdded = mpAddingVarsResult.columnsAdded;
				stats_pd.pp_network_construction_ms = pp.get_network_construction_ms() - stats_pd.pp_network_construction_ms;
				stats_pd.pp_network_size_nodes = pp.get_network_size_nodes();
				stats_pd.pp_network_size_arcs = pp.get_network_size_arcs();
				stats_pd.mp_size_constraints = mp.get_sizeConstraints();
				stats_pd.mp_size_variables = mp.get_sizeVariables();

#ifdef DEBUG_BUILD
				// d. Output:
				std::cout << "AUX-CG: " << " added " << columnsAdded << " columns." << std::endl;
				std::cout << "AUX-CG: " << stats_pd.lb_relaxed << ", " << stats_pd.ub_relaxed << std::endl;
#endif // DEBUG_BUILD	

				// e. Check that the charger capacity constraints are feasible:
				// However, only check when the convergence is reached:
				if (columnsAdded == 0)
				{
					mp_charger_capacity_feasible = mp.check_and_update_charger_capacity(true);

					if (!mp_charger_capacity_feasible)
					{

						stats_pd.lazy_constraint_added = "CSTRS_CHARGER_CAPACITY_ADDED";

						// Have to resolve the RMP with the new charger capacity constraints:
						// c. Resolve the master problem.
						mp_solved = mp.solve(); // Then, resolve the master problem.
						if (!mp_solved)
						{
							throw LogicError("eva::Algorithms::column_generation", "Resolving MP after AUX-CG. Must be feasible. Somthing went wrong.");
						}
					}
				}
#ifdef DEBUG_BUILD
				optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);
#endif // DEBUG_BUILD	
			};
					
			// 3. Set the MP to it's previous state:
			bool result = mp.check_and_update_charger_capacity(false);
			result = mp.check_aux_variables_feasible(vecColumnBounds);
			mp.reset_objective();
			mp.reset_variable_bounds(vecColumnBounds);

			// 4. Print aux-time: 
			stats_pd_aux.branchType = "Auxiliary Column Generation...Finished.";
			stats_pd_aux.time_aux_cg = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count() ;
			stats_pd_aux.iteration = stats_pd.iteration;
			stats_pd_aux.ub_relaxed = stats_pd.ub_relaxed;
			stats_pd_aux.lb_relaxed = stats_pd.lb_relaxed;
			optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd_aux);

			return result;
		};

		// static bool greedy_heuristic(OptimisationInput& optinput, const BranchNode& brn, MasterProblem& mp, PricingProblem& pp, Solution& solution)
		// {
		// 	bool mp_solved = false;
		// 	eva::PricingProblemResult pricingProblemResult;
		// 	eva::StatusVarSchedulesAdded mpAddingVarsResult;
		// 	double min_rc = Constants::BIG_DOUBLE;
		// 	std::vector<std::vector<SubVehicleSchedule>> vec_min_schedule;
		// 	SubVehicleSchedule min_schedule;
		// 	uint32_t size = 0;
		// 	BranchNode greedyBranch = brn;
		// 	// Implement a simple greedy heuristic to route the vehicles.
		// 	while (true)
		// 	{
		// 		// 6. Filter the variables:
		// 		mp.filterVars(greedyBranch);
		// 		pp.updateNodeAccess(greedyBranch);
		// 		// A. Solve MP:
		// 		mp_solved = mp.solve();
		// 		if (!mp_solved)
		// 		{
		// 			// Solve the auxiliary column generation problem:
		// 			// If feasible, resolve MP:
		// 			if (aux_column_generation(optinput, brn, mp, pp))
		// 			{
		// 				mp_solved = mp.solve();
		// 				if (!mp_solved)
		// 					throw LogicError("eva::Algorithms::column_generation", "Resolving MP after AUX-CG. Must be feasible. Somthing went wrong.");
		// 			}
		// 			else
		// 				return false;
		// 		}
		// 		if (mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER
		// 				&& Helper::compare_floats_smaller(mp.get_currentObjective(), solution.objective))
		// 		{
		// 			solution = mp.get_currentSolution();
		// 		}
		// 		// B. Solve pricing problem for all vehicles that are not fixed:
		// 		pricingProblemResult = pp.find_neg_reduced_cost_schedule(mp.get_currentDuals(), greedyBranch, true, true, true,timeOutClock);
		// 		// C. Determine the lowest reduced cost:
		// 		min_rc = Constants::BIG_DOUBLE;
		// 		vec_min_schedule.clear();
		// 		for (const auto &vecSchedules : pricingProblemResult.resSchedule)
		// 		{
		// 			for (const auto &schedule : vecSchedules)
		// 			{
		// 				if (Helper::compare_floats_smaller(schedule.reducedCost, min_rc))
		// 				{
		// 					min_rc = schedule.reducedCost;
		// 					if(!vec_min_schedule.empty()) vec_min_schedule.pop_back();
		// 					vec_min_schedule.push_back(std::vector<SubVehicleSchedule>({schedule}));
		// 				}
		// 			}
		// 		}
		// 		// D. If min-rc is smaller than max, a min-schedule is found:
		// 		if (Helper::compare_floats_smaller(min_rc, Constants::BIG_DOUBLE))
		// 		{
		// 			// Only add the min_schedule to the mp, and fix everything on the schedule:
		// 			mpAddingVarsResult = mp.addVarsSchedule(vec_min_schedule, true);
		// 			min_schedule = vec_min_schedule[0][0];
		// 			// Fix the vehicle, and all components:
		// 			// 1. Vehicle Rotation:
		// 			greedyBranch = BranchNode(++size, greedyBranch, Branch(1.0, 1.0, BranchVehicleRotation(optinput.get_vehicle(min_schedule.indexVehicle))),optinput);
		// 			// 2. All trips:
		// 			for(const auto& idx_trip : min_schedule.vecTripNodeIndexes)
		// 				greedyBranch = BranchNode(++size, greedyBranch, Branch(1.0, 1.0, BranchVehicleTrip(optinput.get_vehicle(min_schedule.indexVehicle),optinput.get_trip(idx_trip))),optinput);
		// 			// 3. All maintenances:
		// 			for(const auto& idx_maintenance : min_schedule.vecMaintenanceNodesIndexes)
		// 				greedyBranch = BranchNode(++size, greedyBranch, Branch(1.0, 1.0, BranchVehicleMaintenance(optinput.get_vehicle(min_schedule.indexVehicle),optinput.get_maintenance(idx_maintenance))),optinput);
		// 		}
		// 		else
		// 		{
		// 			return true;
		// 		}
		// 	}
		// 	// Must solve pricing problem to optimality. Pick one vehicle at a time, and fix every trip, every hub-support.
		// 	return true;
		// };

		static ColumnGenerationResults column_generation(OptimisationInput& optinput, const BranchNode& brn, MasterProblem& mp, PricingProblem& pp, Solution& solution, const double& lb_integer, bool is_root, const double& convergence_acceptance,const std::chrono::high_resolution_clock::time_point& timeOutClockPH)
		{
			// 0. Initialisation:
			bool keep_iterating = false;
			bool mp_solved = false, mp_charger_capacity_feasible = false;;
			bool solve_all_pp_vehicles = false;
			ColumnGenerationResults res;
			
			double cur_lb = brn.get_lb();
			res.lb = cur_lb;

			std::chrono::high_resolution_clock::time_point startClock;
			std::chrono::high_resolution_clock::time_point timeOutClock = std::min(std::chrono::high_resolution_clock::now() + std::chrono::seconds(optinput.get_config().get_const_column_generation_timelimit()), timeOutClockPH);
			eva::PricingProblemResult pricingProblemResult;
			eva::StatusVarSchedulesAdded mpAddingVarsResult;

			// 0. Stats initialisation:
			Stats::PerformanceDetail stats_pd;
			stats_pd.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
			stats_pd.indexBranchingNode = brn.get_index();
			stats_pd.lb_integer = lb_integer;
			stats_pd.time_mpSolver = mp.get_totalRuntimeSolver();
			stats_pd.time_ppSolver = pp.get_totalRuntimeSolver();
			stats_pd.mp_size_constraints = mp.get_sizeConstraints();
			stats_pd.mp_size_variables = mp.get_sizeVariables();
			stats_pd.pp_network_construction_ms = pp.get_network_construction_ms();
			stats_pd.columnsAdded = 0;
			stats_pd.iteration = 0;

			// 1. Solve the MP to initialise the duals:
			// If the mp is infeasible, solve an auxiliary column generation to recover feasibility, or determine that this node is infeasible:
			mp_solved = mp.solve();
			if (!mp_solved)
			{
				// Solve the auxiliary column generation problem:
				// If feasible, resolve MP:
				if (aux_column_generation(optinput, brn, mp, pp, timeOutClock))
				{
					mp_solved = mp.solve();
					if (!mp_solved)
						throw LogicError("eva::Algorithms::column_generation", "Resolving MP after AUX-CG. Must be feasible. Somthing went wrong.");
				}
				else
					return res;
			}

			stats_pd.lb_relaxed = brn.get_lb();
			stats_pd.ub_relaxed = mp.get_currentObjective();
			stats_pd.time_mpSolver = mp.get_totalRuntimeSolver() - stats_pd.time_mpSolver;
			stats_pd.time_ppSolver = pp.get_totalRuntimeSolver() - stats_pd.time_ppSolver;
			stats_pd.pp_network_construction_ms = pp.get_network_construction_ms() - stats_pd.pp_network_construction_ms;
			stats_pd.pp_network_size_nodes = pp.get_network_size_nodes();
			stats_pd.pp_network_size_arcs = pp.get_network_size_arcs();
			stats_pd.ub_integer = solution.objective;

			if (mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER 
				&& Helper::compare_floats_smaller(mp.get_currentObjective(), solution.objective))
			{
				// Just checking if the charger capacity is feasible in the current solution
				if (mp.check_and_update_charger_capacity(false))
				{
					solution = mp.get_currentSolution();
					stats_pd.integerFound = true;
					stats_pd.ub_integer = solution.objective;

					std::cout << "*I: " << stats_pd.lb_integer << " / " << stats_pd.ub_integer << " (" << stats_pd.gap_integer() * 100.0 << "%)" << std::endl;
				}
			}
			else
			{
				stats_pd.integerFound = false;
			}

			optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

			// 2. Start the column generation algorithm:
			do
			{
				++stats_pd.iteration;
				stats_pd.time_mpSolver = mp.get_totalRuntimeSolver(); // Init the runtime, to later adjust.
				stats_pd.time_ppSolver = pp.get_totalRuntimeSolver(); // Init the runtime, to later adjust.
				stats_pd.pp_network_construction_ms = pp.get_network_construction_ms();

				
				// Check for some flags in this iteration:
				// OR if the restricted master problem is currently bigger than the best solution objective
				// because, if the rmp has a weaker lower bound, it can be immediately pruned. 
				// but this only works if the entire pricing problem is solved to optimality.
				solve_all_pp_vehicles = (optinput.get_config().get_flag_interim_solve_all_vehicles() && (stats_pd.iteration % optinput.get_config().get_const_nth_iter_solve_all()) == 0)
										|| (Helper::compare_floats_smaller(solution.objective, mp.get_currentObjective()));

				// a. First, investigate, if any columns from the deleted pool can be added:
				mpAddingVarsResult = mp.addPoolVarsSchedule(mp.get_currentDuals(), brn, true);

				// b. If no variables have already been added to the RMP, then run the pricing problem:
				if (mpAddingVarsResult.columnsAdded <= 0)
				{
					// 1. Solve the pricing problem:
					pricingProblemResult = pp.find_neg_reduced_cost_schedule(mp.get_currentDuals(), brn, true, false, solve_all_pp_vehicles,timeOutClock);
					mpAddingVarsResult = mp.addVarsSchedule(pricingProblemResult.resSchedule, true);
#ifdef DEBUG_BUILD
					// Check the added schedules:
					for(const auto& vecSchedules: pricingProblemResult.resSchedule)
					{
						for(const auto& schedule : vecSchedules)
						{
							if(!schedule.isFeasibleInBranchNode(brn))
							{
								throw LogicError("eva:cg", "non-feasible schedule in branch node being added!!!");

							}
						}
					}
#endif
					
					if(pricingProblemResult.isOptimal)
						cur_lb = mpAddingVarsResult.lb;
				}

				// c. Resolve the master problem.
				mp_solved = mp.solve(); // Then, resolve the master problem.
				if (!mp_solved)
				{
					// Solve the auxiliary column generation problem:
					// If feasible, resolve MP:
					if (aux_column_generation(optinput, brn, mp, pp, timeOutClock))
					{
						mp_solved = mp.solve();
						if (!mp_solved)
							throw LogicError("eva::Algorithms::column_generation", "Resolving MP after AUX-CG. Must be feasible. Somthing went wrong.");
					}
					else
						return res;
				}
						
				// d. If the solution was integer, then quickly check if the charger capacity was as well, and update the solution:
				if (mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER
					&& Helper::compare_floats_smaller(mp.get_currentObjective(), solution.objective))
				{
					 // Just checking if the charger capacity is feasible in the current solution
					if (mp.check_and_update_charger_capacity(false))
					{
						solution = mp.get_currentSolution();
						stats_pd.integerFound = true;
						stats_pd.ub_integer = solution.objective;

						std::cout << "*I: " << stats_pd.lb_integer << " / " << stats_pd.ub_integer << " (" << stats_pd.gap_integer() * 100.0 << "%)" << std::endl;
					}
					else
					{
						stats_pd.integerFound = false;
					}
				}
				else {
					stats_pd.integerFound = false;
				}

				stats_pd.columnsAdded = mpAddingVarsResult.columnsAdded;
				stats_pd.lb_integer = is_root ? std::max(cur_lb, stats_pd.lb_integer) : stats_pd.lb_integer;
				stats_pd.lb_relaxed = std::max(cur_lb, stats_pd.lb_relaxed);
				stats_pd.ub_relaxed = mp.get_currentObjective();

				res.lb = std::max(cur_lb, res.lb);

				// Important to note, the lb_relaxed may be higher than ub_relaxed.
				// For instance, the parent node was solved to a feasible mp solution
				// This node may be infeasible and there could be charger capacity violations
				// Then, the ub_relaxed is smaller than lb_relaxed
				// Regardless, the boolean will be false, and capacity constraints are subsequently added
				keep_iterating = stats_pd.columnsAdded > 0
					&& Helper::compare_floats_smaller(stats_pd.lb_relaxed, stats_pd.ub_relaxed)
					&& Helper::compare_floats_smaller(convergence_acceptance, stats_pd.gap_relaxed())
					&& Helper::compare_floats_smaller(stats_pd.lb_relaxed, solution.objective);
#ifndef DEBUG_BUILD
				keep_iterating = keep_iterating && std::chrono::high_resolution_clock::now() < timeOutClock;
#endif

				// e. Check if convergence-criteria is reached::
				if (!keep_iterating)
				{
					// If yes, check the charger capacity constraints:
					mp_charger_capacity_feasible = mp.check_and_update_charger_capacity(true);

					if (!mp_charger_capacity_feasible)
					{
						// Store in the performance details that charger capacity constraints have been added.
						stats_pd.lazy_constraint_added = "CSTRS_CHARGER_CAPACITY_ADDED";

						// Have to resolve the RMP with the new charger capacity constraints:
						// c. Resolve the master problem.
						mp_solved = mp.solve(); // Then, resolve the master problem.
						if (!mp_solved)
						{
							// Solve the auxiliary column generation problem:
							// If feasible, resolve MP:
							if (aux_column_generation(optinput, brn, mp, pp, timeOutClock))
							{
								mp_solved = mp.solve();
								if (!mp_solved)
									throw LogicError("eva::Algorithms::column_generation", "Resolving MP after AUX-CG. Must be feasible. Somthing went wrong.");
							}
							else
								return res;
						}
					}
				}
				
				// d. Update the bounds and stats about the model:
				stats_pd.ub_relaxed = mp.get_currentObjective(); // Update again in case the ub has changed.
				stats_pd.time_mpSolver = mp.get_totalRuntimeSolver() - stats_pd.time_mpSolver;
				stats_pd.time_ppSolver = pp.get_totalRuntimeSolver() - stats_pd.time_ppSolver;
				stats_pd.pp_network_construction_ms = pp.get_network_construction_ms() - stats_pd.pp_network_construction_ms;
				stats_pd.pp_network_size_nodes = pp.get_network_size_nodes();
				stats_pd.pp_network_size_arcs = pp.get_network_size_arcs();
				stats_pd.mp_size_constraints = mp.get_sizeConstraints();
				stats_pd.mp_size_variables = mp.get_sizeVariables();
				
				// Store the stats:
				optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

#ifdef DEBUG_BUILD
				std::cout << stats_pd.lb_relaxed << " / " << stats_pd.ub_relaxed << " / " << stats_pd.ub_integer << ", columns: " << stats_pd.columnsAdded << std::endl;
#endif

			} while (keep_iterating || !mp_charger_capacity_feasible); // g. Run the column generation until stopping criterium is reached & charger capacity constraints are all feasible.

			// Store lower bound:
			// only interested in the final lower bound.
			res.isFeasible = true;

			return res;
		};

		static void price_and_branch(OptimisationInput& optinput, Solution& solution, Stats::PlanningHorizon& stats_ph, MasterProblem& mp)
		{
			// Finally, solve MIP:
			mp.solveAsMIP();

			if (mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER)
			{
				solution = mp.get_currentSolution();
#ifdef DEBUG_BUILD
				std::cout << "BnB-solution: " << mp.get_currentObjective() << " ***Integer." << std::endl;
#endif // DEBUG_BUILD
			}

			// Update all stats:
			stats_ph.ub_integer = solution.objective;
		};

		static void truncated_column_generation(OptimisationInput& optinput, const std::chrono::high_resolution_clock::time_point& timeOutBnP, MasterProblem& mp, PricingProblem& pp, const BranchNode& parentBranchNode, Solution& sol, Stats::PlanningHorizon& stats_ph)
		{
			// Fix variables and perform a deep-dive until an integer solution is found:
			bool stoppingCriteriaReached = false;
			double fractional_value = 0.0;

			// Building one big branchNode that we keep filling up with fixings:
			BranchNode truncBranchNode = parentBranchNode;
			Branch addedBranch, lastBranch;
			std::vector<Branch> vecFixedBranches;

			// 1) Get parent node
			if (!parentBranchNode.get_vecSortedBranchOptions().empty())
			{
				while (!stoppingCriteriaReached)
				{
					stats_ph.branchingTree_size++;
					stats_ph.branchingTree_depth++;

					// Store the branch details:
					Stats::PerformanceDetail stats_pd_base;
					stats_pd_base.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
					stats_pd_base.indexBranchingNode = stats_ph.branchingTree_size;
					stats_pd_base.indexParentBranchingNode = truncBranchNode.get_index();

					// Load up the branch node with fixings:
					vecFixedBranches.clear();
					for (const auto &bo : truncBranchNode.get_vecSortedBranchOptions())
					{
						// Check if fraction is bigger than threshold:
						fractional_value = bo.get_fractionalValue() - std::floor(bo.get_fractionalValue());
						if (Helper::compare_floats_smaller_equal(optinput.get_config().get_const_frac_threshold_trunc_cg(), fractional_value))
						{
							addedBranch = bo;
							addedBranch.set_branchValue(std::ceil(addedBranch.get_fractionalValue()));
							vecFixedBranches.push_back(addedBranch);
						}
					}

					// If no branch was added, then pick the next one in the line:
					if (vecFixedBranches.empty())
					{
						addedBranch = truncBranchNode.get_nextBranch();
						addedBranch.set_branchValue(std::ceil(addedBranch.get_fractionalValue()));
						truncBranchNode = BranchNode(stats_ph.branchingTree_size, truncBranchNode, addedBranch, optinput);

						// Create the stats:
						Stats::PerformanceDetail stats_pd = stats_pd_base;
						stats_pd.fractionalValue = addedBranch.get_fractionalValue();
						stats_pd.branchValue = addedBranch.get_branchValue();
						stats_pd.branchType = addedBranch.get_branchTypeName();
						optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);
					}
					else
					{
						for (const auto &fixedBranch : vecFixedBranches)
						{
							truncBranchNode = BranchNode(stats_ph.branchingTree_size, truncBranchNode, fixedBranch, optinput);

							// Create the stats:
							Stats::PerformanceDetail stats_pd = stats_pd_base;
							stats_pd.fractionalValue = fixedBranch.get_fractionalValue();
							stats_pd.branchValue = fixedBranch.get_branchValue();
							stats_pd.branchType = fixedBranch.get_branchTypeName();
							optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);
						}
					}

					while (true)
					{
						Stats::PerformanceDetail stats_pd = stats_pd_base;
						stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars();
						stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes();

						// 6. Filter the variables:
						mp.filterVars(truncBranchNode);
						pp.updateNodeAccess(truncBranchNode);

						stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars() - stats_pd.time_mpFilterVars;
						stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes() - stats_pd.time_ppFilterNodes;

						optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

						// Solve the branching node:
						ColumnGenerationResults cg_res = column_generation(optinput, truncBranchNode, mp, pp, sol, stats_ph.lb_integer, false, optinput.get_config().get_const_linear_optimality_gap(),timeOutBnP);

						if (cg_res.isFeasible)
						{
							truncBranchNode.store_branchOptionsTruncColumnGeneration(mp.get_vecBranchOptions());
							truncBranchNode.update_lb(cg_res.lb);

							stats_ph.ub_integer = sol.objective;

							// Determine if the stopping criteria has been reached:
							if (truncBranchNode.get_vecSortedBranchOptions().empty() 
							|| mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER 
							|| Helper::compare_floats_smaller_equal(sol.objective, mp.get_currentObjective()) 
							|| Helper::compare_floats_smaller(stats_ph.gap_integer(), optinput.get_config().get_const_integer_optimality_gap()))
							{
								// Stop infinite loop.
								stoppingCriteriaReached = true;
							}

							// Break the loop, because a feasible cg was found, continue with the updated branching node:
							break;
						}
						else
						{
#ifdef DEBUG_BUILD
							std::cout << "Rollback needed." << std::endl;
#endif // DEBUG_BUILD
	   //  In case the column generation was not solved and is infeasible, rollback the last added branch, and keep iterating:
	   //  Two cases, if last branch is ceil, flip to floor. If it was floor, remove and flip the next one to floor:
							while (true)
							{
								lastBranch = truncBranchNode.get_vecBranches().back();
								truncBranchNode.pop_back_vecBranches();

								// Check if the last branch was ceil or floor, and only add it when it was ceil. Otherwise it remains removed:
								if (Helper::compare_floats_smaller(0.0, lastBranch.get_branchValue() - lastBranch.get_fractionalValue()))
								{
									// Last branch was ceil, switch it to floor:
									lastBranch.set_branchValue(std::floor(lastBranch.get_fractionalValue()));
									truncBranchNode = BranchNode(stats_ph.branchingTree_size, truncBranchNode, lastBranch, optinput);

									// Create the stats:
									Stats::PerformanceDetail stats_pd = stats_pd_base;
									stats_pd.fractionalValue = lastBranch.get_fractionalValue();
									stats_pd.branchValue = lastBranch.get_branchValue();
									stats_pd.branchType = lastBranch.get_branchTypeName();
									optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

									break;
								}
								else
								{
									// Last branch was removed:
									// Create the stats:
									Stats::PerformanceDetail stats_pd = stats_pd_base;
									stats_pd.branchType = lastBranch.get_branchTypeName();
									optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);
								}
							}
						}
					}

					// Check if there is still time left:
					if (!stoppingCriteriaReached)
						stoppingCriteriaReached = std::chrono::high_resolution_clock::now() > timeOutBnP;
				}
			}
		}


		static Branch get_strong_branch(OptimisationInput& optinput, BranchEvaluator& branchEval, MasterProblem& mp, PricingProblem& pp, Solution& sol, Stats::PlanningHorizon& stats_ph, const BranchNode& parentBranchNode, const std::chrono::high_resolution_clock::time_point& timeOutClockPH)
		{
			std::chrono::high_resolution_clock::time_point startClock = std::chrono::high_resolution_clock::now();
			
			std::priority_queue<Branch, std::vector<Branch>, CompareStrongBranchScore> pQ;
			Branch leftBranch, rightBranch, tmpBranch;
			
			// Determine the branching option with the most promising bounds:
			// Check all branching options:
			// Defined based on maxmin-rule:
			std::vector<Branch> vecBranchOptions = parentBranchNode.get_vecSortedBranchOptions();
			std::vector<Branch> vecCandidateBranches;
			
			// Check if too many branch options exist:
			if(vecBranchOptions.size() > optinput.get_config().get_const_max_number_first_tier_eval_strong_branching())
			{
				// 1) First, sort based on historic performance:
				std::sort(vecBranchOptions.begin(), vecBranchOptions.end(), [&](const Branch& l, const Branch& r)
					  {
						  return Helper::compare_floats_smaller(branchEval.get_mean_score(l), branchEval.get_mean_score(r)); 
					  });

				uint32_t max_nr_first_eval_history = optinput.get_config().get_const_max_number_first_tier_eval_strong_branching() / 2;
				for(int added = 0; added < max_nr_first_eval_history; ++added)
				{
					tmpBranch = vecBranchOptions.back();
					if(Helper::compare_floats_smaller(0.0, branchEval.get_mean_score(tmpBranch)))
					{
						vecCandidateBranches.push_back(tmpBranch);
						vecBranchOptions.pop_back();
					}
					else
					{
						break;
					}
				}

				// 2) Fill the rest, with fractional sorted branches:
				std::sort(vecBranchOptions.begin(), vecBranchOptions.end(), &Branch::compareMostFractional);
				vecCandidateBranches.insert(vecCandidateBranches.end(), vecBranchOptions.end() - (optinput.get_config().get_const_max_number_first_tier_eval_strong_branching() - vecCandidateBranches.size()) ,vecBranchOptions.end());
			}
			else
			{
				// Less branches exist as options than the limit. So, keep all.
				vecCandidateBranches = vecBranchOptions;
			}
			
			// Init:
			double cur_strong_branch_score = 0.0;

			// -------------
			// First evaluation of branches:
			for(auto& candidateBranch : vecCandidateBranches)
			{
				leftBranch = candidateBranch;
				leftBranch.set_branchValue(std::floor(leftBranch.get_fractionalValue()));

				rightBranch = candidateBranch;
				rightBranch.set_branchValue(std::ceil(rightBranch.get_fractionalValue()));
				cur_strong_branch_score = Constants::BIG_DOUBLE;

				for (const Branch& newBranch : { leftBranch, rightBranch })
				{
					BranchNode childBranchNode(stats_ph.branchingTree_size, parentBranchNode, newBranch, optinput);
					mp.filterVars(childBranchNode);
					mp.solve();

					if(mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_FRACTIONAL
					|| mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER)
						cur_strong_branch_score = std::min(cur_strong_branch_score, mp.get_currentObjective() - parentBranchNode.get_lb());
				}

				// The strong branch_score has been determined
				candidateBranch.set_strong_branching_score(cur_strong_branch_score);
				pQ.push(candidateBranch);
			}
		
			// -------------
			// Second round of evaluation, take top x-branches, and solve CG heuristically:
			// strongest_branch, candidateBranch
			Branch candidateBranch;
			Branch strongest_branch = pQ.top(); // Default top branch from first evaluation, in case no stronger branch exists.
			double strongest_branch_score = 0.0; // maximise this score in the second evaluation:
			int32_t ctr_second_evaluations = 0;
			while(ctr_second_evaluations < optinput.get_config().get_const_max_number_second_tier_eval_strong_branching() && !pQ.empty())
			{
				candidateBranch = pQ.top();
				pQ.pop();

				// Check if the candidateBranch can beat the score of the currently strongest branch:
				if (Helper::compare_floats_smaller(strongest_branch_score, candidateBranch.get_strong_branching_score()))
				{
					leftBranch = candidateBranch;
					leftBranch.set_branchValue(std::floor(leftBranch.get_fractionalValue()));

					rightBranch = candidateBranch;
					rightBranch.set_branchValue(std::ceil(rightBranch.get_fractionalValue()));

					cur_strong_branch_score = Constants::BIG_DOUBLE;
					for (const Branch &newBranch : {leftBranch, rightBranch})
					{
						BranchNode childBranchNode(stats_ph.branchingTree_size, parentBranchNode, newBranch, optinput);
						mp.filterVars(childBranchNode);
						pp.updateNodeAccess(childBranchNode);

						// Heuristic solve column generation, terminates early.
						ColumnGenerationResults res_cg = column_generation(optinput, childBranchNode, mp, pp, sol, childBranchNode.get_lb(), false, optinput.get_config().get_const_linear_optimality_gap(),timeOutClockPH);

						if (res_cg.isFeasible)
						{
							cur_strong_branch_score = std::min(cur_strong_branch_score, mp.get_currentObjective() - parentBranchNode.get_lb());
						}
					}

					// Store the branch_score:
					if(!Helper::compare_floats_equal(cur_strong_branch_score, Constants::BIG_DOUBLE))
					{
						branchEval.update_branch_mean_score(candidateBranch);
					}

					// Compare with current strongest_branch_score, if score is bigger, pick the branch instead:
					if (Helper::compare_floats_smaller(strongest_branch_score, cur_strong_branch_score))
					{
						strongest_branch_score = cur_strong_branch_score;
						strongest_branch = candidateBranch;
					}
				}
				else
				{
					break; // No need to search longer, because the pQ is sorted. So all following branches won't be able to be better either.
				}

				// Increase the counter of evaluations:
				++ctr_second_evaluations;
			}

			Stats::PerformanceDetail stats_pd;
			stats_pd.indexPlanningHorizon =optinput.get_indexPlanningHorizon();
			stats_pd.indexBranchingNode = parentBranchNode.get_index();
			stats_pd.time_strong_branch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count() ;
			optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

			return strongest_branch;
		};

		static void bestf_bnp(OptimisationInput& optinput, BranchEvaluator& branchEval, const std::chrono::high_resolution_clock::time_point& timeOutBnP, MasterProblem& mp, PricingProblem& pp, BranchNode& root_brn, Solution& sol, Stats::PlanningHorizon& stats_ph)
		{
			std::priority_queue<BranchNode, std::vector<BranchNode>, BranchNode::CompareLb> pQ;

			// 2. Initialise priority queue with results on root node:
			pQ.push(root_brn);

			// 3. Start branching:		
			BranchNode parentBranchNode;
			Branch leftBranch, rightBranch, strongBranch;
			while (!pQ.empty()
				&& std::chrono::high_resolution_clock::now() <= timeOutBnP)
			{
				// Retrieve the currently most promising branching node:
				parentBranchNode = pQ.top();
				pQ.pop();

				if (!parentBranchNode.get_vecSortedBranchOptions().empty())
				{
					// Pick strong branch:
					strongBranch = get_strong_branch(optinput,branchEval,mp,pp,sol,stats_ph,parentBranchNode,timeOutBnP);

					//leftBranch = parentBranchNode.get_nextBranch();
					leftBranch = strongBranch;
					leftBranch.set_branchValue(std::floor(leftBranch.get_fractionalValue()));

					//rightBranch = parentBranchNode.get_nextBranch();
					rightBranch = strongBranch;
					rightBranch.set_branchValue(std::ceil(rightBranch.get_fractionalValue()));

					for (const Branch& newBranch : { leftBranch, rightBranch })
					{
						BranchNode childBranchNode(++stats_ph.branchingTree_size, parentBranchNode, newBranch, optinput);

						// Store the branch details:
						Stats::PerformanceDetail stats_pd;
						stats_pd.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
						stats_pd.indexBranchingNode = childBranchNode.get_index();
						stats_pd.indexParentBranchingNode = parentBranchNode.get_index();
						stats_pd.fractionalValue = newBranch.get_fractionalValue();
						stats_pd.branchValue = newBranch.get_branchValue();
						stats_pd.branchType = newBranch.get_branchTypeName();

						int indexFromScheduleNode = -1;
						int indexToScheduleNode = -1;
						int indexCharger = -1;
						switch (newBranch.get_type())
						{
						case BranchType::TRIP_UNASSIGNED:
							indexFromScheduleNode = newBranch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index();
							indexToScheduleNode = newBranch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index();
							indexCharger = -1;
							break;
						case BranchType::VEHICLE_CHARGING_AFTER:
							stats_pd.VehicleId = newBranch.castBranchVehicleChargingAfter()->get_vehicle().get_id();
							indexFromScheduleNode = newBranch.castBranchVehicleChargingAfter()->get_indexFromScheduleNode();
							indexCharger = newBranch.castBranchVehicleChargingAfter()->get_charger().get_index();
							break;
						case BranchType::VEHICLE_CHARGING_BEFORE:
							stats_pd.VehicleId = newBranch.castBranchVehicleChargingBefore()->get_vehicle().get_id();
							indexToScheduleNode = newBranch.castBranchVehicleChargingBefore()->get_indexToScheduleNode();
							indexCharger = newBranch.castBranchVehicleChargingBefore()->get_charger().get_index();
							break;
						case BranchType::VEHICLE_ROTATION:
							stats_pd.VehicleId = newBranch.castBranchVehicleRotation()->get_vehicle().get_id();
							break;
						case BranchType::VEHICLE_TRIP:
							stats_pd.TripId = newBranch.castBranchVehicleTrip()->get_subTripNodeData().get_ptrTripNodeData()->get_trip().get_id();
							indexFromScheduleNode = newBranch.castBranchVehicleTrip()->get_subTripNodeData().get_scheduleNodeData().get_index();
							indexToScheduleNode = newBranch.castBranchVehicleTrip()->get_subTripNodeData().get_scheduleNodeData().get_index();
							indexCharger = -1;
							stats_pd.VehicleId = newBranch.castBranchVehicleTrip()->get_vehicle().get_id();
							break;
						case BranchType::VEHICLE_MAINTENANCE:
							stats_pd.MaintenanceId = newBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_ptrMaintenanceNodeData()->get_maintenance().get_id();
							indexFromScheduleNode = newBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index();
							indexToScheduleNode = newBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index();
							indexCharger = 0;
							stats_pd.VehicleId = newBranch.castBranchVehicleMaintenance()->get_vehicle().get_id();
							break;
						default:
							break;
						}

						// Solve the branching node:
						stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars();
						stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes();

						// 6. Filter the variables:
						mp.filterVars(childBranchNode);
						pp.updateNodeAccess(childBranchNode);

						stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars() - stats_pd.time_mpFilterVars;
						stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes() - stats_pd.time_ppFilterNodes;

						optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

#ifdef DEBUG_BUILD
						// Summarise the output:
						auto tid = stats_pd.TripId != Constants::BIG_UINTEGER ? std::to_string(stats_pd.TripId) : "";
						auto mid = stats_pd.MaintenanceId != Constants::BIG_UINTEGER ? std::to_string(stats_pd.MaintenanceId) : "";
						auto vid = stats_pd.VehicleId != Constants::BIG_UINTEGER ? std::to_string(stats_pd.VehicleId) : "";
						std::cout << stats_pd.branchType << ": " << stats_pd.fractionalValue << " -> " << stats_pd.branchValue << ", Vehicle: " << vid
						 << ", Charger: " << indexCharger << ", From:  " << indexFromScheduleNode << ", To: " << indexToScheduleNode << std::endl;
#endif

						// 7. Solve the branching node:
						ColumnGenerationResults cg_res = column_generation(optinput, childBranchNode, mp, pp, sol, childBranchNode.get_lb(), false, 0.0,timeOutBnP);

						// Update the results of the branch node:
						// If the branch node is feasible. Otherwise, prune it by not considering it in the branching tree (pQ) anymore:
						if (cg_res.isFeasible)
						{
							childBranchNode.store_branchOptionsBranchAndPrice(mp.get_vecBranchOptions());
							childBranchNode.update_lb(cg_res.lb);

							stats_ph.ub_integer = sol.objective;
							stats_ph.branchingTree_depth = std::max(stats_ph.branchingTree_depth, 1 + (static_cast<uint32_t>(parentBranchNode.get_vecBranches().size() - root_brn.get_vecBranches().size())));
							
							// Check if the branch node is pruned by bounds:
							if (Helper::compare_floats_smaller(childBranchNode.get_lb(), sol.objective))
							{
								// If branching tree size == mod X, perform the diving heuristic to explore the depths of this branch:
								if(childBranchNode.get_index() % optinput.get_config().get_const_nth_branching_node_dive() == 0)
								{
									auto cur_stats_ph_size = stats_ph.branchingTree_size;
									auto cur_stats_ph_depth = stats_ph.branchingTree_depth;

									std::cout << "Quick dive: " << std::endl;

									eva::Stats::PerformanceDetail quick_pd_pre;
									quick_pd_pre.branchType = "Start Quick Dive";
									optinput.get_dataHandler().storeStatsPerformanceDetail(quick_pd_pre);
									truncated_column_generation(optinput, timeOutBnP, mp, pp, childBranchNode, sol, stats_ph);
									eva::Stats::PerformanceDetail quick_pd_post;
									quick_pd_post.branchType = "End Quick Dive";
									optinput.get_dataHandler().storeStatsPerformanceDetail(quick_pd_post);
									
									stats_ph.branchingTree_size = cur_stats_ph_size;
									stats_ph.branchingTree_depth = cur_stats_ph_depth;
								}

								pQ.push(childBranchNode);
#ifdef DEBUG_BUILD
								if (mp.get_currentSolutionStatus() == MasterProblemSolutionStatus::MP_INTEGER)
								{
									std::cout << "Pruned by optimality. Integer solution found at this node." << std::endl;
								}
#endif // DEBUG_BUILD
							}
							else
							{
								// Pruned by bounds
#ifdef DEBUG_BUILD
								std::cout << "Pruned by bounds:" << childBranchNode.get_lb() << " >= " << sol.objective << std::endl;
#endif // DEBUG_BUILD
							}

							// Check for an early exit from the BnP, if the integer gap is already closed:
							if (Helper::compare_floats_smaller(stats_ph.gap_integer(), optinput.get_config().get_const_integer_optimality_gap()))
							{
								return;
							}
						}
						else
						{
							// Pruned by Infeasibility
#ifdef DEBUG_BUILD
							std::cout << "Pruned by Infeasibility." << std::endl;
#endif // DEBUG_BUILD
						}
					}
				}

				// Update the lb integer after exploring both new branching nodes:
				// Or if the only node left, or the root node already is optimal, skip right here to verify it is now optimal.
				// Maybe the best lb has improved without the ub has improved. So, an integer node might have already been discovered somewhere.
				stats_ph.lb_integer = pQ.empty() ? stats_ph.lb_integer : pQ.top().get_lb();

				if (pQ.empty())
					stats_ph.lb_integer = sol.objective; // Optimal solution found because there are not more branching nodes to explore. Entire search tree explored.
				else
					stats_ph.lb_integer = pQ.top().get_lb(); // Else, the current top node stores the global lower bound in the search tree.

#ifdef DEBUG_BUILD
				std::cout << "LB: " << stats_ph.lb_integer << " UB:" << sol.objective << std::endl;
#endif // DEBUG_BUILD

				// Check if a sufficient integer gap has been achieved:
				if (Helper::compare_floats_smaller(stats_ph.gap_integer(), optinput.get_config().get_const_integer_optimality_gap()))
				{
					return;
				}
			}
		};

		static void depthf_bnp(OptimisationInput& optinput, BranchEvaluator& branchEval, const std::chrono::high_resolution_clock::time_point& timeOutBnP, MasterProblem& mp, PricingProblem& pp, BranchNode& parentBranchNode, Solution& sol, Stats::PlanningHorizon& stats_ph)
		{
			// Only solve, if there is still time:
			if (std::chrono::high_resolution_clock::now() > timeOutBnP)
			{
				// Termination criteria of depth-first bnp:
				return;
			}

			// Branch Left and Right:
			double subtree_lb = Constants::BIG_DOUBLE;

			if (!parentBranchNode.get_vecSortedBranchOptions().empty())
			{
				std::vector<double> vecBranchValue;
				Branch childBranch = parentBranchNode.get_nextBranch();
				//Branch childBranch = get_strong_branch(optinput,branchEval,mp,pp,sol,stats_ph,parentBranchNode);

				switch(childBranch.get_type())
				{
					case BranchType::TOTAL_VEHICLES:
					case BranchType::TOTAL_TRIPS_UNASSIGNED:
					case BranchType::TRIP_UNASSIGNED:
					case BranchType::VEHICLE_ROTATION:
						// Regardless the branch value, always try left branch first, with less vehicles and less trips unassigned!
						// Same for trip unassigned, always try first to keep it assigned.
						// Same for the vehicle rotation, always try first without the vehicle.
						vecBranchValue.push_back(std::floor(childBranch.get_fractionalValue()));
						vecBranchValue.push_back(std::ceil(childBranch.get_fractionalValue()));
						break;
					case BranchType::VEHICLE_CHARGING_AFTER:
					case BranchType::VEHICLE_CHARGING_BEFORE:
					case BranchType::VEHICLE_MAINTENANCE:
					case BranchType::VEHICLE_TRIP:
					default:
						vecBranchValue.push_back(std::ceil(childBranch.get_fractionalValue()));
						vecBranchValue.push_back(std::floor(childBranch.get_fractionalValue()));
						break;
				}
								
				for (const double& curBranchValue : vecBranchValue)
				{
					// Only solve, if there is still time:
					if (std::chrono::high_resolution_clock::now() > timeOutBnP)
					{
						// Termination criteria of depth-first bnp:
						break;
					}

					// 0. init:
					childBranch.set_branchValue(curBranchValue);
					BranchNode childBranchNode(++stats_ph.branchingTree_size, parentBranchNode, childBranch,optinput);

					// Store the branch details:
					Stats::PerformanceDetail stats_pd;
					stats_pd.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
					stats_pd.indexBranchingNode = childBranchNode.get_index();
					stats_pd.indexParentBranchingNode = parentBranchNode.get_index();
					stats_pd.fractionalValue = childBranch.get_fractionalValue();
					stats_pd.branchValue = childBranch.get_branchValue();
					stats_pd.branchType = childBranch.get_branchTypeName();

					int indexFromScheduleNode = -1;
					int indexToScheduleNode = -1;
					int indexCharger = -1;
					switch (childBranch.get_type())
					{
					case BranchType::TRIP_UNASSIGNED:
						indexFromScheduleNode = childBranch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index();
						indexToScheduleNode = childBranch.castBranchTripUnassigned()->get_subTripNodeData().get_scheduleNodeData().get_index();
						indexCharger = -1;
						break;
					case BranchType::VEHICLE_CHARGING_AFTER:
						stats_pd.VehicleId = childBranch.castBranchVehicleChargingAfter()->get_vehicle().get_id();
						indexFromScheduleNode = childBranch.castBranchVehicleChargingAfter()->get_indexFromScheduleNode();
						indexCharger = childBranch.castBranchVehicleChargingAfter()->get_charger().get_index();
						break;
					case BranchType::VEHICLE_CHARGING_BEFORE:
						stats_pd.VehicleId = childBranch.castBranchVehicleChargingBefore()->get_vehicle().get_id();
						indexToScheduleNode = childBranch.castBranchVehicleChargingBefore()->get_indexToScheduleNode();
						indexCharger = childBranch.castBranchVehicleChargingBefore()->get_charger().get_index();
						break;
					case BranchType::VEHICLE_ROTATION:
						stats_pd.VehicleId = childBranch.castBranchVehicleRotation()->get_vehicle().get_id();
						break;
					case BranchType::VEHICLE_TRIP:
						stats_pd.TripId = childBranch.castBranchVehicleTrip()->get_subTripNodeData().get_ptrTripNodeData()->get_trip().get_id();
						indexFromScheduleNode = childBranch.castBranchVehicleTrip()->get_subTripNodeData().get_scheduleNodeData().get_index();
						indexToScheduleNode = childBranch.castBranchVehicleTrip()->get_subTripNodeData().get_scheduleNodeData().get_index();
						indexCharger = -1;
						stats_pd.VehicleId = childBranch.castBranchVehicleTrip()->get_vehicle().get_id();
						break;
					case BranchType::VEHICLE_MAINTENANCE:
						stats_pd.MaintenanceId = childBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_ptrMaintenanceNodeData()->get_maintenance().get_id();
						indexFromScheduleNode = childBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index();
						indexToScheduleNode = childBranch.castBranchVehicleMaintenance()->get_subMaintenanceNodeData().get_scheduleNodeData().get_index();
						indexCharger = 0;
						stats_pd.VehicleId = childBranch.castBranchVehicleMaintenance()->get_vehicle().get_id();
						break;
					default:
						break;
					}

					stats_ph.branchingTree_depth = std::max(static_cast<uint32_t>(childBranchNode.get_vecBranches().size()), stats_ph.branchingTree_depth);

					// Solve the branching node:
					stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars();
					stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes();

					// 6. Filter the variables:
					mp.filterVars(childBranchNode);
					pp.updateNodeAccess(childBranchNode);

					stats_pd.time_mpFilterVars = mp.get_totalRuntimeFilterVars() - stats_pd.time_mpFilterVars;
					stats_pd.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes() - stats_pd.time_ppFilterNodes;
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					// a. Solve the branching node node:
					ColumnGenerationResults cg_res = column_generation(optinput, childBranchNode, mp, pp, sol, childBranchNode.get_lb(), false, 0.0,timeOutBnP);

					// Prune by infeasibility, if node is infeasible:
					if (cg_res.isFeasible)
					{
						// Update branching node information:
						childBranchNode.store_branchOptionsBranchAndPrice(mp.get_vecBranchOptions());
						childBranchNode.update_lb(cg_res.lb);

						stats_ph.ub_integer = sol.objective;

						// Dive if node is not pruned:
						// a. optimality pruning:
						if (Helper::compare_floats_smaller(cg_res.lb, sol.objective)
							&& Helper::compare_floats_smaller_equal(optinput.get_config().get_const_integer_optimality_gap(), (std::abs(sol.objective - cg_res.lb) / std::abs(sol.objective)))
							&& !childBranchNode.get_vecSortedBranchOptions().empty())
						{
							depthf_bnp(optinput, branchEval, timeOutBnP, mp, pp, childBranchNode, sol, stats_ph);
						} // else: Leaf found, with satisfactory integer gap for the remaining subtree.

						// Update the lower bound:
						subtree_lb = std::min(subtree_lb, childBranchNode.get_lb());
					}
				}

				// Once both subtree's have been explored, the parent lower bound can be updated:
				if(!Helper::compare_floats_equal(subtree_lb, Constants::BIG_DOUBLE))
					parentBranchNode.update_lb(subtree_lb);
			}
		};

		static void solvePlanningHorizon(OptimisationInput& optinput, Solution& solution, Stats::PlanningHorizon& stats_ph)
		{
			// Init the stats_ph object:
			stats_ph.lb_integer = -Constants::BIG_DOUBLE;
			stats_ph.ub_integer = Constants::BIG_DOUBLE;
			stats_ph.branchingTree_depth = 0;
			stats_ph.branchingTree_size = 1;

			// 0. Initialise:
			MasterProblem mp(optinput);
			PricingProblem pp(optinput);
			BranchNode root_brn(stats_ph.branchingTree_size, optinput);
			BranchEvaluator branchEval(optinput);
			std::chrono::high_resolution_clock::time_point timeOutClock = std::chrono::high_resolution_clock::now() + std::chrono::seconds(optinput.get_config().get_const_branch_and_price_timelimit());

			// Update the pricing problem:
			mp.filterVars(root_brn);
			pp.updateNodeAccess(root_brn);

			// Display the pricing problem being used:
			switch(optinput.get_config().get_const_pricing_problem_type())
			{
				case Types::PricingProblemType::TIME_SPACE_NETWORK:
					std::cout << "Using Connection-Based Time-Space Network." << std::endl;
				break;
				case Types::PricingProblemType::CONNECTION_SEGMENT_NETWORK:
					std::cout << "Using Connection-Based Segment Network." << std::endl;
				break;
				case Types::PricingProblemType::CENTRALISED_SEGMENT_NETWORK:
					std::cout << "Using Centralised Segment Network." << std::endl;
				break;
				default:
				break;
			}

			// Solve the root node:
			// Avoid solving the root node to optimality, if only trunc. CG is used:
			ColumnGenerationResults root_cg_res;
			std::cout << "Solving Root Node." << std::endl;

			if(optinput.get_dataHandler().get_config().get_const_algorithm_type() == Types::AlgorithmType::DIVING_HEURISTIC)
				root_cg_res = column_generation(optinput, root_brn, mp, pp, solution, stats_ph.lb_integer, true, optinput.get_config().get_const_linear_optimality_gap(),timeOutClock);
			else
				root_cg_res = column_generation(optinput, root_brn, mp, pp, solution, stats_ph.lb_integer, true, 0.0,timeOutClock);

			root_brn.update_lb(root_cg_res.lb);
			auto rootBranchOptions = mp.get_vecBranchOptions();
			
			if(optinput.get_config().get_flag_terminate_after_root())
				return;

			stats_ph.lb_integer = root_brn.get_lb();
			stats_ph.ub_integer = solution.objective;
			stats_ph.algorithm = "Root-Solved";
			stats_ph.pp_nr_segments = pp.get_number_segments();

			Stats::PerformanceDetail stats_pd;
			stats_pd.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
			stats_pd.indexBranchingNode = root_brn.get_index();

			// 1. Explore the branching tree: - Depth First:
			if (root_cg_res.isFeasible
				&& std::chrono::high_resolution_clock::now() <= timeOutClock
				&& !Helper::compare_floats_equal(root_brn.get_lb(), solution.objective))
			{
				switch (optinput.get_dataHandler().get_config().get_const_algorithm_type())
				{
				case Types::AlgorithmType::PRICE_AND_BRANCH:
					std::cout << "Using Price-and-Branch Algorithm." << std::endl;
					stats_ph.algorithm = "Price-and-Branch";
					stats_pd.branchType = "Price-and-Branch";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					price_and_branch(optinput, solution, stats_ph, mp);
					
					break;
				case Types::AlgorithmType::BRANCH_AND_PRICE_BEST:
					std::cout << "Using Branch-and-Price (Best-First) Algorithm." << std::endl;
					stats_ph.algorithm = "Branch-and-Price (Best-First)";
					stats_pd.branchType = "Branch-and-Price (Best-First)";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsBranchAndPrice(rootBranchOptions);

					bestf_bnp(optinput,branchEval, timeOutClock, mp, pp, root_brn, solution, stats_ph);
					
					break;
				case Types::AlgorithmType::BRANCH_AND_PRICE_DEPTH:
					std::cout << "Using Branch-and-Price (Depth-First) Algorithm." << std::endl;
					stats_ph.algorithm = "Branch-and-Price (Depth-First)";
					stats_pd.branchType = "Branch-and-Price (Depth-First)";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsBranchAndPrice(rootBranchOptions);

					depthf_bnp(optinput,branchEval, timeOutClock, mp, pp, root_brn, solution, stats_ph);
					stats_ph.lb_integer = root_brn.get_lb();

					break;
				case Types::AlgorithmType::DIVING_HEURISTIC:
					std::cout << "Using Truncated Column Generation Algorithm." << std::endl;
					stats_ph.algorithm = "Trunc. CG";
					stats_pd.branchType = "Trunc. CG";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsTruncColumnGeneration(rootBranchOptions);

					truncated_column_generation(optinput, timeOutClock, mp, pp, root_brn, solution, stats_ph);

					break;
				case Types::AlgorithmType::DIVING_THEN_BFBNP:
					std::cout << "First Trunc. CG -> Best-First Branch-and-Price. " << std::endl;
					stats_ph.algorithm = "Trunc. CG -> Branch-and-Price (Best-First)";

					std::cout << "Trunc. CG:" << std::endl;
					stats_pd.branchType = "Trunc. CG";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsTruncColumnGeneration(rootBranchOptions);
					truncated_column_generation(optinput, timeOutClock, mp, pp, root_brn, solution, stats_ph); // Solve first to improve the upper bound.
					
					std::cout << "Branch-and-Price (Best-First):" << std::endl;
					stats_pd.branchType = "Branch-and-Price (Best-First)";
					stats_pd.indexBranchingNode = root_brn.get_index();
					stats_ph.branchingTree_depth = 1;
					stats_ph.branchingTree_size = 2;
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsBranchAndPrice(rootBranchOptions);
					bestf_bnp(optinput,branchEval, timeOutClock, mp, pp, root_brn, solution, stats_ph); // Then solve BnP Best-First search tree to improve solution (or confirm optimality).

					break;

				case Types::AlgorithmType::DIVING_THEN_DFBNP:
					std::cout << "First Trunc. CG -> Depth-First Branch-and-Price. " << std::endl;
					stats_ph.algorithm = "Trunc. CG -> Branch-and-Price (Depth-First)";

					std::cout << "Trunc. CG:" << std::endl;
					stats_pd.branchType = "Trunc. CG";
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsTruncColumnGeneration(rootBranchOptions);
					truncated_column_generation(optinput, timeOutClock, mp, pp, root_brn, solution, stats_ph); // Solve first to improve the upper bound.
					
					std::cout << "Using Branch-and-Price (Depth-First) Algorithm." << std::endl;
					stats_pd.branchType = "Branch-and-Price (Depth-First)";
					stats_pd.indexBranchingNode = root_brn.get_index();
					stats_ph.branchingTree_depth = 1;
					stats_ph.branchingTree_size = 2;
					optinput.get_dataHandler().storeStatsPerformanceDetail(stats_pd);

					root_brn.store_branchOptionsBranchAndPrice(rootBranchOptions);

					depthf_bnp(optinput, branchEval, timeOutClock, mp, pp, root_brn, solution, stats_ph);
					stats_ph.lb_integer = root_brn.get_lb();

					break;
				default:
					throw InvalidArgumentError("eva::Optimiser::run", " algorithmType is not supported.");
					break;
				}
			}

			// 2. Update all stats:
			stats_ph.size_unassignedTrips = solution.size_unassignedTrips;
			stats_ph.size_vehiclesSelected = solution.size_vehiclesSelected;
			stats_ph.time_mpSolver = mp.get_totalRuntimeSolver();
			stats_ph.time_ppSolver = pp.get_totalRuntimeSolver();
			stats_ph.time_mpFilterVars = mp.get_totalRuntimeFilterVars();
			stats_ph.time_ppFilterNodes = pp.get_totalRuntimeFilterNodes();
			stats_ph.pp_network_construction_ms = pp.get_network_construction_ms();
		};
	};

	static void trimSolutionPlanningHorizon(OptimisationInput& optinput, Solution& solution)
	{
		for (VehicleSchedule& vs : solution.vecSchedule)
		{
			// Find the end of the planning horizon:
			auto iter = vs.vecScheduleNodes.end();
			auto iterCutOff = vs.vecScheduleNodes.end();

			while (iter != vs.vecScheduleNodes.begin())
			{
				--iter;

				if (optinput.get_scheduleGraphNodeData(*iter).get_type() == ScheduleNodeType::PUT_ON_CHARGE)
					iterCutOff = iter;

				if (optinput.get_scheduleGraphNodeData(*iter).get_startTime() < optinput.get_endPlanningHorizon()
					&& optinput.get_scheduleGraphNodeData(*iter).get_type() != ScheduleNodeType::PUT_ON_CHARGE
					&& optinput.get_scheduleGraphNodeData(*iter).get_type() != ScheduleNodeType::TAKE_OFF_CHARGE)
					break;
			}

			// Update the vehicleSchedule object to account for the part that was trimmed:
			if (iterCutOff != vs.vecScheduleNodes.end())
			{
				// If the cutOff is not at the end, then:
				// a. Update the new endLocationIndex of the schedule:
				// b. Erase all nodes after the cut-off point:
				vs.indexEndLocation = optinput.get_scheduleGraphNodeData(*iterCutOff).get_startLocation().get_index();
				vs.vecScheduleNodes.erase(iterCutOff, vs.vecScheduleNodes.end());
			}
		}
	};
}

void eva::Optimiser::run()
{
	// 1. Initialise the optimisation input:
	OptimisationInput optinput(_dataHandler);

	std::chrono::high_resolution_clock::time_point startClock;

	// 2. Loop until the end of the planning horizon is reached:
	do
	{
		// Start the clock:
		startClock = std::chrono::high_resolution_clock::now();

		// a. Output the current planning horizon:
		std::cout << std::endl;
		std::cout << "Planning Horizon: " << std::to_string(optinput.get_indexPlanningHorizon())
			<< ", Date: " << Helper::DateTimeToString(optinput.get_startPlanningHorizon())
			<< " - " << Helper::DateTimeToString(optinput.get_endPlanningHorizon())
			<< " (+" << Helper::DurationToString(Helper::diffDateTime(optinput.get_endPlanningHorizon(), optinput.get_endPlanningHorizonOverlap())) << " Overlap)"
			<< std::endl;

		// b. Depending on the algorithm type, run the optimisation algorithm:
		Solution solution;

		Stats::PlanningHorizon stats_planningHorizon;
		stats_planningHorizon.indexPlanningHorizon = optinput.get_indexPlanningHorizon();
		stats_planningHorizon.startPlanningHorizon = optinput.get_startPlanningHorizon();
		stats_planningHorizon.endPlanningHorizon = optinput.get_endPlanningHorizon();
		stats_planningHorizon.endOverlapPlanningHorizon = optinput.get_endPlanningHorizonOverlap();

		// c. Solve the planning horizon:
		Algorithms::solvePlanningHorizon(optinput, solution, stats_planningHorizon);

		// d. Update the stats and the schedule:
		// Trim the solution, and remove the nodes outside of the planning horizon:
		trimSolutionPlanningHorizon(optinput, solution);
		solution.startDecisionHorizon = optinput.get_startPlanningHorizon();
		solution.endDecisionHorizon = optinput.get_endPlanningHorizon();

		// e.Bring solution over to the datahandler, store the results and schedule in the dataHandler.
		_dataHandler.storeSolution(solution);

		// f. End the clock:
		stats_planningHorizon.time_total = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startClock).count();
		std::cout << "Total Time [s]: " << stats_planningHorizon.time_total / 1000 << std::endl;
		
		// g. Finally, store the stats:
		_dataHandler.storeStatsPlanningHorizon(stats_planningHorizon);

	} while (optinput.next()); // Until there is no next planning horizon.
}
