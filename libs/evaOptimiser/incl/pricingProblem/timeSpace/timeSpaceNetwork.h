#ifndef EVA_TIME_SPACE_NETWORK_H
#define EVA_TIME_SPACE_NETWORK_H

#include "evaConstants.h"
#include "moderator/SubScheduleNodes.h"
#include "moderator/OptimisationInput.h"
#include "moderator/moderator.h"
#include "moderator/chargingStrategies.h"

#include "timeSpaceNodes.h"
#include "timeSpaceArcs.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>

namespace eva
{
	namespace tsn
	{
		typedef boost::adjacency_list<
			boost::vecS,
			boost::vecS,
			boost::bidirectionalS,
			TimeSpaceNodeData,
			TimeSpaceArcData>
			BoostTimeSpaceNetwork;

		typedef boost::graph_traits<BoostTimeSpaceNetwork>::vertex_descriptor BoostTimeSpaceNode;
		typedef boost::graph_traits<BoostTimeSpaceNetwork>::edge_descriptor BoostTimeSpaceArc;

		struct TimeSpaceResourceContainer
		{
			// ATTRIBUTES:
			double cost = Constants::BIG_DOUBLE;
			double reducedCost = Constants::BIG_DOUBLE;
			Types::DateTime timestamp = Constants::MAX_TIMESTAMP;
			Types::BatteryCharge soc = Constants::BIG_BATTERYCHARGE;
			uint32_t distanceLastMaintenance = Constants::BIG_INTEGER;
			bool include_distance = false;
			bool isEndSchedule = false;
			Types::DateTime max_rc_start_time = Constants::MAX_TIMESTAMP;
			bool isExemptFromDominance = false;

			// CONSTRUCTORS:

			TimeSpaceResourceContainer() {}

			TimeSpaceResourceContainer(
				const double &cost,
				const double &reducedCost,
				const Types::DateTime &timestamp,
				const uint32_t distanceLastMaintenance,
				const Types::BatteryCharge &soc,
				const bool include_distance,
				const Types::DateTime& max_rc_start_time) : cost(cost),
											   reducedCost(reducedCost),
											   timestamp(timestamp),
											   distanceLastMaintenance(distanceLastMaintenance),
											   soc(soc),
											   include_distance(include_distance),
											   max_rc_start_time(max_rc_start_time) {};

			TimeSpaceResourceContainer(
				const TimeSpaceResourceContainer &other) : include_distance(other.include_distance),
														   cost(other.cost),
														   reducedCost(other.reducedCost),
														   timestamp(other.timestamp),
														   distanceLastMaintenance(other.distanceLastMaintenance),
														   soc(other.soc),
														   isEndSchedule(other.isEndSchedule),
														   max_rc_start_time(other.max_rc_start_time),
														   isExemptFromDominance(other.isExemptFromDominance) {};

			~TimeSpaceResourceContainer() {};

			TimeSpaceResourceContainer &operator=(const TimeSpaceResourceContainer &other);

			const bool operator==(const TimeSpaceResourceContainer &other) const;
			const bool operator<(const TimeSpaceResourceContainer &other) const;
		};

		class TimeSpaceDominanceCheck
		{
			bool _solve_optimal = false;
			bool _include_distance = false;

		public:
			TimeSpaceDominanceCheck(
				const bool solve_optimal,
				const bool include_distance) : _solve_optimal(solve_optimal),
											   _include_distance(include_distance) {};

			inline bool operator()(const TimeSpaceResourceContainer &res_cont_1,
								   const TimeSpaceResourceContainer &res_cont_2) const
			{
				// Check if res_cont_1 dominates res_cont_2!
				// must be "<=" here!!!
				// must NOT be "<"!!!

				if (res_cont_2.isEndSchedule)
				{
					if (_solve_optimal)
					{
						// Only keep the minimum reduced cost label at the end schedule.
						return Helper::compare_floats_smaller(res_cont_1.reducedCost, res_cont_2.reducedCost);
					}
					// res_cont_2 is never dominated if it is at a sink node:
					return false;
				}
				else if (res_cont_2.isExemptFromDominance)
					return false;
				else if (_include_distance)
					return Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost) && res_cont_1.soc >= res_cont_2.soc && res_cont_1.distanceLastMaintenance <= res_cont_2.distanceLastMaintenance;
				else
					return Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost) && res_cont_1.soc >= res_cont_2.soc;
			};
		};

		class TimeSpaceResourceExtensionFunction
		{
		private:
			const Duals &_duals;
			const Vehicle &_vehicle;
			const OptimisationInput &_optinput;
			const bool _include_cost;
			VariableAtEndChargingStrategy _chargingStrategy;

		public:
			TimeSpaceResourceExtensionFunction(
				const Duals &duals,
				const Vehicle &vehicle,
				const OptimisationInput &optinput,
				const bool include_cost) : _duals(duals),
										   _vehicle(vehicle),
										   _optinput(optinput),
										   _include_cost(include_cost), 
										   _chargingStrategy(VariableAtEndChargingStrategy(_optinput))
			{
			}
			bool _handleTripNode(
				TimeSpaceResourceContainer &new_cont,
				const TimeSpaceResourceContainer &old_cont,
				const TimeSpaceTripNodeData *tripNodeData) const;

			bool _handleMaintenanceNode(
				TimeSpaceResourceContainer &new_cont,
				const TimeSpaceResourceContainer &old_cont,
				const TimeSpaceMaintenanceNodeData *maintenanceNodeData) const;

			bool _handleChargingNode(
				TimeSpaceResourceContainer &new_cont,
				const TimeSpaceResourceContainer &old_cont,
				const TimeSpaceChargingNodeData *chargingNodeData) const;

			bool operator()(
				const BoostTimeSpaceNetwork &boostTimeSpaceNetwork,
				TimeSpaceResourceContainer &new_cont,
				const TimeSpaceResourceContainer &old_cont,
				const BoostTimeSpaceArc &arc) const;

			inline const bool get_include_cost() const { return _include_cost; };
		};

		struct TimeSpaceResourceConstraintPaths
		{
			std::vector<std::vector<BoostTimeSpaceArc>> pareto_optimal_solutions;
			std::vector<TimeSpaceResourceContainer> pareto_optimal_resource_containers;

			TimeSpaceResourceConstraintPaths() {};

			TimeSpaceResourceConstraintPaths(
				const TimeSpaceResourceContainer &initialBaseContainer) : pareto_optimal_resource_containers(std::vector<TimeSpaceResourceContainer>({initialBaseContainer})),
																		  pareto_optimal_solutions(std::vector<std::vector<BoostTimeSpaceArc>>({std::vector<BoostTimeSpaceArc>()})) {};
		};

		class TimeSpaceResourceExtensionVisitor : public boost::default_r_c_shortest_paths_visitor
		{
			int32_t _max_labels = 50;
			int32_t _ctr_labels_sink = 0;
			bool _solve_optimal = false;
			std::chrono::high_resolution_clock::time_point _timeOutClock;

		public:
			// Constructor
			TimeSpaceResourceExtensionVisitor() = delete;
			TimeSpaceResourceExtensionVisitor(
				const uint32_t &max_labels,
				const bool solve_optimal,
				const std::chrono::high_resolution_clock::time_point& timeOutClock) : 
					_max_labels(max_labels),
					_solve_optimal(solve_optimal),
					_timeOutClock(timeOutClock) {};

			template <class Queue, class Graph>
			bool on_enter_loop(const Queue &queue, const Graph &graph)
			{
				// Stop entering the loop because the limit has been reached.
				// It only counts processed labels. There can exist more labels at the sink node that just haven't been processed yet.
				// This makes the labelling stop when at least _max_labels have been determined.

				// If the pricing problem should be solved to optimality, always allow to enter the loop:
				return (_solve_optimal || _ctr_labels_sink < _max_labels) && std::chrono::high_resolution_clock::now() < _timeOutClock; // Stops, if the number of qualifying labels has reached the maximum.
			}

			template <class Label, class Graph>
			void on_label_feasible(Label &l, const Graph &g)
			{
				if (l.cumulated_resource_consumption.isEndSchedule)
				{
					++_ctr_labels_sink;
				}
			}
		};

		struct TimeSpaceNodeFixings
			{
				Types::DateTime fixed_start_time;
				std::vector<BoostTimeSpaceNode> vec_fixed_nodes;
			};

		class TimeSpaceNetwork
		{
			const OptimisationInput &_optinput;
			VariableAtEndChargingStrategy _chargingStrategy;
			
			BoostTimeSpaceNetwork _boostTimeSpaceNetwork;

			Types::Index _indexNode{0};
			Types::Index _indexArc{0};

			std::vector<BoostTimeSpaceNode> _vecStartNodes;
			std::vector<BoostTimeSpaceNode> _vecTripNodes;
			std::vector<BoostTimeSpaceNode> _vecMaintenanceNodes;
			std::vector<std::vector<std::vector<BoostTimeSpaceNode>>> _vecChargingNodesFrom;
			std::vector<std::vector<std::vector<BoostTimeSpaceNode>>> _vecChargingNodesTo;
			std::map<eva::BoostScheduleNode, BoostTimeSpaceNode> _mapScheduleNodeLookup;
			BoostTimeSpaceNode _endNode;

			Types::Index _getNextIndexNode() { return _indexNode++; }
			Types::Index _getNextIndexArc() { return _indexArc++; }

			// FUNCTION DEFINITIONS:

			void _addTripNodes();
			void _addMaintenanceNodes();
			void _addVehicleStartEndNodes();
			void _addDeadlegs();
			void _addCharging();
			void _initialiseNodeAccess();
			void _resetAccess();

			// GETTERS
			const BoostTimeSpaceNode &_getStartNode(const Types::Index &indexVehicle) const { return _vecStartNodes[indexVehicle]; };
			const BoostTimeSpaceNode &_getTripNode(const Types::Index &indexTrip) const { return _vecTripNodes[indexTrip]; };
			const BoostTimeSpaceNode &_getMaintenanceNode(const Types::Index &indexMaintenance) const { return _vecMaintenanceNodes[indexMaintenance]; };

			const TimeSpaceNodeData &_getNodeData(const BoostTimeSpaceNode node) const { return _boostTimeSpaceNetwork[node]; };
			TimeSpaceNodeData &_getNodeData(const BoostTimeSpaceNode node) { return _boostTimeSpaceNetwork[node]; };
			const TimeSpaceArcData &_getArcData(const BoostTimeSpaceArc arc) const { return boost::get(boost::edge_bundle, _boostTimeSpaceNetwork)[arc]; };
			TimeSpaceArcData &_getArcData(const BoostTimeSpaceArc arc) { return boost::get(boost::edge_bundle, _boostTimeSpaceNetwork)[arc]; };

		public:
			TimeSpaceNetwork() = delete;

			TimeSpaceNetwork(
				const OptimisationInput &optinput) : _optinput(optinput), _chargingStrategy(VariableAtEndChargingStrategy(_optinput)) {};

			void initialise();
			void updateAccess(const BranchNode &brn);

			inline const uint32_t &get_number_nodes() const { return _indexNode; }
			inline const uint32_t &get_number_arcs() const { return _indexArc; }

			std::vector<SubVehicleSchedule> find_neg_reduced_cost_schedule_vehicle(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock);
		};
	}
}

#endif // !EVA_TIME_SPACE_NETWORK_H
