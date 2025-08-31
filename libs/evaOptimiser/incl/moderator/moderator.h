#ifndef EVA_MODERATOR_H
#define EVA_MODERATOR_H

#include "evaConstants.h"
#include "OptimisationInput.h"
#include "SubScheduleNodes.h"
#include "branch.h"

#include "Highs.h"

#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <tuple>

namespace eva
{
	struct Duals {
		std::vector<double> vecDualsOneSchedulePerVehicle;
		std::vector<double> vecDualsTripCoverage;
		std::vector<double> vecDualsOneVehiclePerMaintenance;
		std::vector<std::vector<std::vector<double>>> vecCumSumDualsChargerCapacity;

		Duals() = delete;

		Duals(
			const OptimisationInput& optinput
		) :
			vecDualsOneSchedulePerVehicle(std::vector<double>(optinput.get_vehicles().get_vec().size() , 0.0)),
			vecDualsTripCoverage(std::vector<double>(optinput.get_vecTrips().size(), 0.0)),
			vecDualsOneVehiclePerMaintenance(std::vector<double>(optinput.get_vecMaintenances().size(), 0.0)),
			vecCumSumDualsChargerCapacity(std::vector < std::vector<std::vector<double>>>(optinput.get_chargers().get_vec().size()))
		{
			initialise(optinput);
		};
		void initialise(const OptimisationInput& optinput);
	};

	struct ChargingSchedule
	{
		Types::Index indexFromScheduleNode = Constants::BIG_INDEX;
		Types::Index indexToScheduleNode = Constants::BIG_INDEX;
		Types::Index indexCharger = Constants::BIG_INDEX;
		Types::Index indexPutOnCharge = Constants::BIG_INDEX;
		Types::Index indexTakeOffCharge = Constants::BIG_INDEX;

		// Constructor:
		ChargingSchedule() {};

		ChargingSchedule(
			const Types::Index& fromNode,
			const Types::Index& toNode,
			const Types::Index& indexCharger
		) :
			indexFromScheduleNode(fromNode),
			indexToScheduleNode(toNode),
			indexCharger(indexCharger)
		{};

		ChargingSchedule(
			const Types::Index& fromNode,
			const Types::Index& toNode,
			const Types::Index& indexCharger,
			const Types::Index& indexPutOnCharge,
			const Types::Index& indexTakeOffCharge
		) :
			indexFromScheduleNode(fromNode),
			indexToScheduleNode(toNode),
			indexCharger(indexCharger),
			indexPutOnCharge(indexPutOnCharge),
			indexTakeOffCharge(indexTakeOffCharge)
		{};

		// Checks if the fromNode and toNode are defined. No branching at the beginning and end of a schedule.
		inline const bool isFeasibleForBranching() const { return indexFromScheduleNode != Constants::BIG_INDEX && indexToScheduleNode != Constants::BIG_INDEX; };

		// Can only check if the charging is not identical if both, fromNode and toNode are defined. Disregard the actual time at the charger.
		bool operator==(const ChargingSchedule& other) const {
			return isFeasibleForBranching()
				&& indexFromScheduleNode == other.indexFromScheduleNode
				&& indexToScheduleNode == other.indexToScheduleNode
				&& indexCharger == other.indexCharger;
		}

		bool operator<(const ChargingSchedule& other) const {
			if (indexCharger != other.indexCharger) return indexCharger < other.indexCharger;
			if (indexFromScheduleNode != other.indexFromScheduleNode) return indexFromScheduleNode < other.indexFromScheduleNode;
			if (indexToScheduleNode != other.indexToScheduleNode) return indexToScheduleNode < other.indexToScheduleNode;
			return false;
		}
	};

	class BranchNode
	{
	
		// MEMBER ATTRIBUTES

		Types::Index _index = Constants::BIG_INDEX;
		std::vector<Branch> _vecBranches;
		std::vector<Branch> _vecSortedBranchOptions;
		std::vector<std::vector<Types::Index>> _vecVehicleFixings;
		std::vector<std::unordered_map<Types::Index, Types::DateTime>> _vecVehicleFixedEndTimeLookup;
		std::vector<std::unordered_map<Types::Index, Types::DateTime>> _vecVehicleFixedStartTimeLookup;
		double _lb = -Constants::BIG_DOUBLE;

		// MEMBER FUNCTIONS

		void _initialise_root(const OptimisationInput& optinput);
		void _initialise_child(const Branch& newBranch, const OptimisationInput& optinput);
		void _update_vehicleFixings(const Branch& newBranch);
		void _prepare_fixings(const OptimisationInput& optinput);

	public: 
		BranchNode() {};

		// Root node constructor:
		BranchNode(
			const Types::Index& index,
			const OptimisationInput& optinput
		) :
			_index(index),
			_lb(-Constants::BIG_DOUBLE)
		{
			_initialise_root(optinput);
			_prepare_fixings(optinput);
		}

		BranchNode(
			const Types::Index& index,
			const BranchNode& parentBranchNode,
			const Branch& newBranch,
			const OptimisationInput& optinput
		) :
			_index(index),
			_vecBranches(parentBranchNode.get_vecBranches()),
			_lb(parentBranchNode.get_lb()),
			_vecVehicleFixings(parentBranchNode.get_vecVehicleFixings())
		{
			_initialise_child(newBranch, optinput);
		}		

		// FUNCTIONS:

		inline void pop_back_vecBranchOptions() { _vecSortedBranchOptions.pop_back(); };
		inline void pop_back_vecBranches() { _vecBranches.pop_back(); };

		inline void update_lb(const double& lb) { _lb = lb; };

		void store_branchOptionsTruncColumnGeneration(const std::vector<Branch>& vecBranchOptions);
		void store_branchOptionsBranchAndPrice(const std::vector<Branch>& vecBranchOptions);
		void writeBranchesToConsole();

		// INLINE GETTERS:

		inline const Types::Index& get_index() const { return _index; };
		inline const std::vector<Branch>& get_vecBranches() const { return _vecBranches; };
		inline const std::vector<Branch>& get_vecSortedBranchOptions() const { return _vecSortedBranchOptions; };
		inline const Branch get_nextBranch() const { return _vecSortedBranchOptions.empty() ? Branch() : _vecSortedBranchOptions.back(); };
		inline const double& get_lb() const { return _lb; };
		inline const std::vector<std::vector<Types::Index>>& get_vecVehicleFixings() const { return _vecVehicleFixings;};
		inline const Types::DateTime& get_vehicleFixedNodeNextMaxEndTime(const Types::Index& indexVehicle, const Types::Index& indexScheduleNode) const {return _vecVehicleFixedEndTimeLookup[indexVehicle].at(indexScheduleNode);};
		inline const Types::DateTime& get_vehicleFixedNodePrevMinStartTime(const Types::Index& indexVehicle, const Types::Index& indexScheduleNode) const {return _vecVehicleFixedStartTimeLookup[indexVehicle].at(indexScheduleNode);};
	
		// CLASS FUNCITONS:

		struct CompareLb
		{
			bool operator()(const BranchNode& l, const BranchNode& r) { return Helper::compare_floats_smaller(r.get_lb(), l.get_lb()); };
		};
	};
	
	struct SubVehicleSchedule : public VehicleSchedule
	{
		double cost = Constants::BIG_DOUBLE;
		double reducedCost = Constants::BIG_DOUBLE;

		std::vector<Types::Index> vecTripNodeIndexes;
		std::vector<Types::Index> vecMaintenanceNodesIndexes;
		std::vector<ChargingSchedule> vecChargingSchedule;

		// INLINE CONST
		inline const bool hasTrip(const Types::Index& indexTrip) const { return std::find(vecTripNodeIndexes.begin(), vecTripNodeIndexes.end(), indexTrip) != vecTripNodeIndexes.end(); };
		inline const bool hasMaintenance(const Types::Index& indexMaintenance) const { return std::find(vecMaintenanceNodesIndexes.begin(), vecMaintenanceNodesIndexes.end(), indexMaintenance) != vecMaintenanceNodesIndexes.end(); };
		const bool hasChargingAfter(const Types::Index& indexCharger, const Types::Index& indexFromScheduleNode) const;
		const bool hasChargingBefore(const Types::Index& indexCharger, const Types::Index& indexToScheduleNode) const;
		

		const bool isSubsetOf(const SubVehicleSchedule& other) const;

		const bool isFeasibleInBranchNode(const BranchNode& brn) const;
		const double get_current_reducedCost(const Duals& duals) const;

		// STATIC CLASS:
		
		// Compare function to determine the minimum reduced cost:
		static bool compare_rC(const SubVehicleSchedule& lhs, const SubVehicleSchedule& rhs) { return Helper::compare_floats_smaller(lhs.reducedCost, rhs.reducedCost); }
	};

	class BranchEvaluator
	{
		std::pair<double, uint32_t> _eval_branch_total_number_vehicles; // TOTAL_VEHICLES
		std::pair<double, uint32_t> _eval_branch_total_number_unassigned_trips; // TOTAL_TRIPS_UNASSIGNED 
		std::vector<std::pair<double, uint32_t>> _vec_eval_branch_vehicle_rotation; // VEHICLE_ROTATION
		std::vector<std::pair<double, uint32_t>> _vec_eval_branch_trip_unassigned; // TRIP_UNASSIGNED
		std::vector<std::vector<std::pair<double, uint32_t>>> _vec_eval_branch_vehicle_trip; // VEHICLE_TRIP
		std::vector<std::vector<std::pair<double, uint32_t>>> _vec_eval_branch_vehicle_maintenance; // VEHICLE_MAINTENANCE

		std::unordered_map<std::tuple<Types::Index, Types::Index, Types::Index>, 
			std::pair<double, uint32_t>, boost::hash<std::tuple<Types::Index, Types::Index, Types::Index>>> _umap_eval_branch_vehicle_charging_after; // VEHICLE_CHARGING_AFTER

		std::unordered_map<std::tuple<Types::Index, Types::Index, Types::Index>, 
			std::pair<double, uint32_t>, boost::hash<std::tuple<Types::Index, Types::Index, Types::Index>>> _umap_eval_branch_vehicle_charging_before; // VEHICLE_CHARGING_BEFORE
	
		void _initialise(const OptimisationInput& optinput);
		void _update_moving_average(std::pair<double, uint32_t>& cur, const double& new_data_point);

	public:
		BranchEvaluator() = delete;
		BranchEvaluator(const OptimisationInput& optinput) {
			_initialise(optinput);
		};

		void update_branch_mean_score(const Branch& branch);
		const double get_mean_score(const Branch& branch) const;
	};

};



#endif // !EVA_MODERATOR_H