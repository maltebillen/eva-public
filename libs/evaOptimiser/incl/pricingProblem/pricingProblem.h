#ifndef EVA_PRICING_PROBLEM
#define EVA_PRICING_PROBLEM

#include "evaConstants.h"
#include "moderator/OptimisationInput.h"
#include "moderator/moderator.h"

#include "timeSpace/timeSpaceNetwork.h"
#include "segmentBased/connectionNetwork/segmentConnectionNetwork.h"
#include "segmentBased/centralisedNetwork/segmentCentralisedNetwork.h"

namespace eva
{
	struct PricingProblemResult
	{
		bool isOptimal = false;
		std::vector<std::vector<SubVehicleSchedule>> resSchedule;
	};

	class PricingProblem
	{
		const OptimisationInput& _optinput;
		
		tsn::TimeSpaceNetwork _tsn;
		sbn::con::ConnectionBasedSegmentNetwork _connection_sbn;
		sbn::cen::CentralisedBasedSegmentNetwork _centralised_sbn;

		int64_t _mseconds_runtimeSolver = 0;
		int64_t _mseconds_filterNodeAccess = 0;
		int64_t _network_construction_ms = 0;
		uint32_t _network_size_nodes = 0;
		uint32_t _network_size_arcs = 0;

		void _initialise();
		PricingProblemResult _tsn_find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		PricingProblemResult _connection_sbn_find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		PricingProblemResult _centralised_sbn_find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		
		void _tsn_updateNodeAccess(const BranchNode& brn);

		void _solve_tsn_pricing_problem(const std::vector<Types::Index>& vecVehicleIndexes, PricingProblemResult& result, const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, bool& isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		void _solve_connection_sbn_pricing_problem(const std::vector<Types::Index>& vecVehicleIndexes, PricingProblemResult& result, const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, bool& isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		void _solve_centralised_sbn_pricing_problem(const std::vector<Types::Index>& vecVehicleIndexes, PricingProblemResult& result, const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, bool& isSolvedOptimal, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		
		std::vector<Types::Index> _shuffleVecVehicleRotation(const std::vector<uint8_t>& vecVehicleRotation);

	public:
		PricingProblem(
			const OptimisationInput& optinput
		) :
			_optinput(optinput),
			_tsn(optinput),
			_connection_sbn(optinput),
			_centralised_sbn(optinput)
		{
			_initialise();
		};

		PricingProblemResult find_neg_reduced_cost_schedule(const Duals& duals, const BranchNode& brn, const bool include_cost,const bool solve_to_optimal, const bool solve_all_vehicles, const std::chrono::high_resolution_clock::time_point& timeOutClock);
		void updateNodeAccess(const BranchNode& brn);
		const uint32_t get_number_segments() const;
		
		inline const int64_t get_totalRuntimeSolver() const { return _mseconds_runtimeSolver; };
		inline const int64_t get_totalRuntimeFilterNodes() const { return _mseconds_filterNodeAccess; };
		inline const int64_t get_network_construction_ms() const { return _network_construction_ms; };
		inline const uint32_t get_network_size_nodes() const { return _network_size_nodes; };
		inline const uint32_t get_network_size_arcs() const { return _network_size_arcs; };
	};
}





#endif // !EVA_PRICING_PROBLEM
