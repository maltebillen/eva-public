#ifndef EVA_SEGMENT_CONNECTION_NETWORK_H
#define EVA_SEGMENT_CONNECTION_NETWORK_H

#include "moderator/OptimisationInput.h"
#include "../segments/segments.h"

#include "segmentConnectionNetworkNodes.h"
#include "segmentConnectionNetworkArcs.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>

namespace eva
{
	namespace sbn
	{
		namespace con
		{
			typedef boost::adjacency_list<
				boost::vecS,
				boost::vecS,
				boost::directedS,
				ConNodeData,
				ConArcData>
				BoostConnectionBasedNetwork;

			typedef boost::graph_traits<BoostConnectionBasedNetwork>::vertex_descriptor BoostConnectionBasedNode;
			typedef boost::graph_traits<BoostConnectionBasedNetwork>::edge_descriptor BoostConnectionBasedArc;

			struct ConnectionBasedResourceContainer
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

				// CONSTRUCTORS:

				ConnectionBasedResourceContainer() {}

				ConnectionBasedResourceContainer(
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
												   max_rc_start_time(max_rc_start_time){};

				ConnectionBasedResourceContainer(
					const ConnectionBasedResourceContainer &other) : include_distance(other.include_distance),
																	 cost(other.cost),
																	 reducedCost(other.reducedCost),
																	 timestamp(other.timestamp),
																	 distanceLastMaintenance(other.distanceLastMaintenance),
																	 soc(other.soc),
																	 isEndSchedule(other.isEndSchedule),
																	 max_rc_start_time(other.max_rc_start_time){};

				~ConnectionBasedResourceContainer() {};

				ConnectionBasedResourceContainer &operator=(const ConnectionBasedResourceContainer &other);

				const bool operator==(const ConnectionBasedResourceContainer &other) const;
				const bool operator<(const ConnectionBasedResourceContainer &other) const;
			};

			class ConnectionBasedDominanceCheck
			{
				bool _solve_optimal = false;
				bool _include_distance = false;

			public:
				ConnectionBasedDominanceCheck(
					const bool solve_optimal,
					const bool include_distance) : _solve_optimal(solve_optimal),
												   _include_distance(include_distance) {};
												   
				
				inline bool operator()(const ConnectionBasedResourceContainer &res_cont_1,
									   const ConnectionBasedResourceContainer &res_cont_2) const
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
					else if (_include_distance)
						return Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost) 
						&& res_cont_1.distanceLastMaintenance <= res_cont_2.distanceLastMaintenance;
					else
						return Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost);
				};
			};

			class ConnectionBasedResourceExtensionFunction
			{
			private:
				const Duals &_duals;
				const Vehicle &_vehicle;
				const OptimisationInput &_optinput;
				FixAtEndChargingStrategy _chargingStrategy;
				const bool _include_cost;

			public:
				ConnectionBasedResourceExtensionFunction(
					const Duals &duals,
					const Vehicle &vehicle,
					const OptimisationInput &optinput,
					const bool include_cost) : _duals(duals),
											   _vehicle(vehicle),
											   _optinput(optinput),
											   _include_cost(include_cost),
											   _chargingStrategy(FixAtEndChargingStrategy(_optinput))
				{
				}

				bool operator()(
					const BoostConnectionBasedNetwork &boostSegmentNetwork,
					ConnectionBasedResourceContainer &new_cont,
					const ConnectionBasedResourceContainer &old_cont,
					const BoostConnectionBasedArc &arc) const;

				inline const bool get_include_cost() const { return _include_cost; };
			};

			struct ConnectionBasedResourceConstraintPaths
			{
				std::vector<std::vector<BoostConnectionBasedArc>> pareto_optimal_solutions;
				std::vector<ConnectionBasedResourceContainer> pareto_optimal_resource_containers;

				ConnectionBasedResourceConstraintPaths() {};

				ConnectionBasedResourceConstraintPaths(
					const ConnectionBasedResourceContainer &initialBaseContainer) : pareto_optimal_resource_containers(std::vector<ConnectionBasedResourceContainer>({initialBaseContainer})),
																		  pareto_optimal_solutions(std::vector<std::vector<BoostConnectionBasedArc>>({std::vector<BoostConnectionBasedArc>()})) {};
			};

			class ConnectionBasedResourceExtensionVisitor : public boost::default_r_c_shortest_paths_visitor
			{
				int32_t _max_labels = 50;
				int32_t _ctr_labels_sink = 0;
				bool _solve_optimal = false;
				std::chrono::high_resolution_clock::time_point _timeOutClock;

			public:
				// Constructor
				ConnectionBasedResourceExtensionVisitor() = delete;
				ConnectionBasedResourceExtensionVisitor(
					const uint32_t &max_labels,
					const bool solve_optimal,
					const std::chrono::high_resolution_clock::time_point &timeOutClock) : _max_labels(max_labels),
												_solve_optimal(solve_optimal), _timeOutClock(timeOutClock)  {};

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

			struct ConnectionBasedNodeFixings
			{
				Types::DateTime fixed_start_time;
				std::vector<BoostConnectionBasedNode> vec_fixed_nodes;
			};

			class ConnectionBasedSegmentNetwork
			{
				// MEMBERS

				const OptimisationInput &_optinput;
				FixAtEndChargingStrategy _chargingStrategy;
				Segments _segments;

				BoostConnectionBasedNetwork _boostSegmentNetwork;
				std::vector<BoostConnectionBasedNode> _vecVehicleStartNodes;
				std::vector<std::vector<BoostConnectionBasedNode>> _vecIncludesTripNodes;
				std::vector<std::vector<BoostConnectionBasedNode>> _vecIncludesMaintenanceNodes;
				std::map<Types::Index, std::vector<BoostConnectionBasedNode>> _mapIncludesScheduleNode;
				std::vector<std::vector<std::vector<std::vector<BoostConnectionBasedNode>>>> _vecSortedNodesChargerToCharger;

				BoostConnectionBasedNode _endNode;

				Types::Index _indexNode{0};
				Types::Index _indexArc{0};

				// PRIVATE FUNCTIONS

				void _clearNetwork();
				void _addNodes(const BranchNode &brn, const Duals &duals);
				void _addArcs(const BranchNode &brn, const Duals &duals);
				void _addBranches(const BranchNode &brn, const Duals &duals);
				
				BoostConnectionBasedNode _addNode(ConNodeData &nodeData);
				BoostConnectionBasedArc _addArc(ConArcData &arcData);
				BoostConnectionBasedArc _addArc(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode);

				FullConArcData _createArcData(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode, const Duals &duals);

				const bool _isDominated(std::vector<FullConArcData> &vecDominatingArcs, const FullConArcData &candidateArc);
				const bool _isFeasible(const BoostConnectionBasedNode fromNode, const BoostConnectionBasedNode toNode);

				Types::Index _getNextIndexNode() { return _indexNode++; }
				Types::Index _getNextIndexArc() { return _indexArc++; }

				const ConNodeData &_getNodeData(const BoostConnectionBasedNode node) const { return _boostSegmentNetwork[node]; };
				ConNodeData &_getNodeData(const BoostConnectionBasedNode node) { return _boostSegmentNetwork[node]; };
				const ConArcData &_getArcData(const BoostConnectionBasedArc arc) const { return boost::get(boost::edge_bundle, _boostSegmentNetwork)[arc]; };
				ConArcData &_getArcData(const BoostConnectionBasedArc arc) { return boost::get(boost::edge_bundle, _boostSegmentNetwork)[arc]; };

				inline const BoostConnectionBasedNode &_getStartScheduleNode(const Vehicle &vehicle) const { return _vecVehicleStartNodes[vehicle.get_index()]; };

				//ConnectionBasedResourceConstraintPaths _runLabelAlgorithm(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock);

			public:
				ConnectionBasedSegmentNetwork() = delete;

				ConnectionBasedSegmentNetwork(
					const OptimisationInput &optinput) : _optinput(optinput), _chargingStrategy(FixAtEndChargingStrategy(optinput)) {};

				// PUBLIC FUNCTIONS:

				void initialise();
				void create_reduced_graph(const Duals &duals, const BranchNode &brn);
				std::vector<SubVehicleSchedule> find_neg_reduced_cost_schedule_vehicle(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock);

				void update_branch_node_fixings(const BranchNode &brn);

				inline const uint32_t get_number_segments() const { return _segments.get_vec().size(); };
				inline const uint32_t &get_number_nodes() const { return _indexNode; };
				inline const uint32_t &get_number_arcs() const { return _indexArc; };
			};
		}

	}
}

#endif // !EVA_SEGMENT_CONNECTION_NETWORK_H
