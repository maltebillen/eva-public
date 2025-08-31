#ifndef EVA_SEGMENT_CENTRALISED_NETWORK_H
#define EVA_SEGMENT_CENTRALISED_NETWORK_H

#include "moderator/OptimisationInput.h"
#include "../segments/segments.h"

#include "segmentCentralisedNetworkNodes.h"
#include "segmentCentralisedNetworkArcs.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>

namespace eva
{
    namespace sbn
    {
        namespace cen
        {
            typedef boost::adjacency_list<
                boost::vecS,
                boost::vecS,
                boost::directedS,
                CenNodeData,
                CenArcData>
                BoostCentralisedBasedNetwork;

            typedef boost::graph_traits<BoostCentralisedBasedNetwork>::vertex_descriptor BoostCentralisedBasedNode;
            typedef boost::graph_traits<BoostCentralisedBasedNetwork>::edge_descriptor BoostCentralisedBasedArc;

            struct CentralisedBasedResourceContainer
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

				CentralisedBasedResourceContainer() {}

				CentralisedBasedResourceContainer(
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

				CentralisedBasedResourceContainer(
					const CentralisedBasedResourceContainer &other) : include_distance(other.include_distance),
																	 cost(other.cost),
																	 reducedCost(other.reducedCost),
																	 timestamp(other.timestamp),
																	 distanceLastMaintenance(other.distanceLastMaintenance),
																	 soc(other.soc),
																	 isEndSchedule(other.isEndSchedule),
																	 max_rc_start_time(other.max_rc_start_time) {};

				~CentralisedBasedResourceContainer() {};

				CentralisedBasedResourceContainer &operator=(const CentralisedBasedResourceContainer &other);

				const bool operator==(const CentralisedBasedResourceContainer &other) const;
				const bool operator<(const CentralisedBasedResourceContainer &other) const;
			};
        
            class CentralisedBasedDominanceCheck
			{
				bool _solve_optimal = false;
				bool _include_distance = false;

			public:
				CentralisedBasedDominanceCheck(
					const bool solve_optimal,
					const bool include_distance) : _solve_optimal(solve_optimal),
												   _include_distance(include_distance) {};
				
				inline bool operator()(const CentralisedBasedResourceContainer &res_cont_1,
									   const CentralisedBasedResourceContainer &res_cont_2) const
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

					if (_include_distance)
						return res_cont_1.timestamp <= res_cont_2.timestamp 
							&& res_cont_1.max_rc_start_time >= res_cont_2.max_rc_start_time
							&& Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost)
							&& res_cont_1.distanceLastMaintenance <= res_cont_2.distanceLastMaintenance;
					else
						return res_cont_1.timestamp <= res_cont_2.timestamp 
							&& res_cont_1.max_rc_start_time >= res_cont_2.max_rc_start_time
							&& Helper::compare_floats_smaller_equal(res_cont_1.reducedCost, res_cont_2.reducedCost);
				};
			};
            
            class CentralisedBasedResourceExtensionFunction
			{
			private:
				const Duals &_duals;
				const Vehicle &_vehicle;
				const OptimisationInput &_optinput;
				FixAtEndChargingStrategy _chargingStrategy;
				const bool _include_cost;

			public:
				CentralisedBasedResourceExtensionFunction(
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
					const BoostCentralisedBasedNetwork &boostSegmentNetwork,
					CentralisedBasedResourceContainer &new_cont,
					const CentralisedBasedResourceContainer &old_cont,
					const BoostCentralisedBasedArc &arc) const;

				inline const bool get_include_cost() const { return _include_cost; };
			};

            struct CentralisedBasedResourceConstraintPaths
			{
				std::vector<std::vector<BoostCentralisedBasedArc>> pareto_optimal_solutions;
				std::vector<CentralisedBasedResourceContainer> pareto_optimal_resource_containers;

				CentralisedBasedResourceConstraintPaths() {};

				CentralisedBasedResourceConstraintPaths(
					const CentralisedBasedResourceContainer &initialBaseContainer) : pareto_optimal_resource_containers(std::vector<CentralisedBasedResourceContainer>({initialBaseContainer})),
																		  pareto_optimal_solutions(std::vector<std::vector<BoostCentralisedBasedArc>>({std::vector<BoostCentralisedBasedArc>()})) {};
			};

            class CentralisedBasedResourceExtensionVisitor : public boost::default_r_c_shortest_paths_visitor
			{
				int32_t _max_labels = 50;
				int32_t _ctr_labels_sink = 0;
				bool _solve_optimal = false;
				std::chrono::high_resolution_clock::time_point _timeOutClock;

			public:
				// Constructor
				CentralisedBasedResourceExtensionVisitor() = delete;
				CentralisedBasedResourceExtensionVisitor(
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

			struct CentralisedBasedNodeFixings
			{
				Types::DateTime fixed_start_time;
				std::vector<BoostCentralisedBasedNode> vec_fixed_nodes;
			};

            class CentralisedBasedSegmentNetwork
			{
				// MEMBERS

				const OptimisationInput &_optinput;
				FixAtEndChargingStrategy _chargingStrategy;
				Segments _segments;

				BoostCentralisedBasedNetwork _boostSegmentNetwork;
				std::vector<BoostCentralisedBasedNode> _vecVehicleStartNodes;
				std::vector<BoostCentralisedBasedNode> _vecCentralChargingNodes;
				std::vector<std::vector<BoostCentralisedBasedNode>> _vecIncludesTripNodes;
				std::vector<std::vector<BoostCentralisedBasedNode>> _vecIncludesMaintenanceNodes;

				std::map<Types::Index, std::vector<BoostCentralisedBasedNode>> _mapIncludesScheduleNode;
				std::vector<std::vector<std::vector<std::vector<BoostCentralisedBasedNode>>>> _vecSortedNodesChargerToCharger;

				BoostCentralisedBasedNode _endNode;

				Types::Index _indexNode{0};
				Types::Index _indexArc{0};

				// PRIVATE FUNCTIONS

				void _clearNetwork();
				void _addNodes(const BranchNode &brn, const Duals &duals);
				void _addArcs(const BranchNode &brn, const Duals &duals);
				void _addBranches(const BranchNode &brn, const Duals &duals);

				BoostCentralisedBasedNode _addNode(CenNodeData &nodeData);
				BoostCentralisedBasedArc _addArc(CenArcData &arcData);

				//CentralisedBasedResourceConstraintPaths _runLabelAlgorithm(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock);

				// INLINE METHODS

				Types::Index _getNextIndexNode() { return _indexNode++; }
				Types::Index _getNextIndexArc() { return _indexArc++; }

				const CenNodeData &_getNodeData(const BoostCentralisedBasedNode node) const { return _boostSegmentNetwork[node]; };
				CenNodeData &_getNodeData(const BoostCentralisedBasedNode node) { return _boostSegmentNetwork[node]; };
				const CenArcData &_getArcData(const BoostCentralisedBasedArc arc) const { return boost::get(boost::edge_bundle, _boostSegmentNetwork)[arc]; };
				CenArcData &_getArcData(const BoostCentralisedBasedArc arc) { return boost::get(boost::edge_bundle, _boostSegmentNetwork)[arc]; };

				inline const BoostCentralisedBasedNode &_getStartScheduleNode(const Vehicle &vehicle) const { return _vecVehicleStartNodes[vehicle.get_index()]; };
				
			public:
				CentralisedBasedSegmentNetwork() = delete;

				CentralisedBasedSegmentNetwork(
					const OptimisationInput &optinput) : _optinput(optinput), _chargingStrategy(FixAtEndChargingStrategy(optinput)) {};

				// PUBLIC FUNCTIONS:

				void initialise();
				void create_reduced_graph(const Duals &duals, const BranchNode &brn);
				std::vector<SubVehicleSchedule> find_neg_reduced_cost_schedule_vehicle(const Duals &duals, const Vehicle &vehicle, const BranchNode &brn, const bool include_cost, const bool solve_to_optimal, bool &isSolvedOptimal,const std::chrono::high_resolution_clock::time_point& timeOutClock);

				void update_branch_node_fixings(const BranchNode &brn);

				// INLINE 

				inline const uint32_t get_number_segments() const { return _segments.get_vec().size(); };
				inline const uint32_t &get_number_nodes() const { return _indexNode; };
				inline const uint32_t &get_number_arcs() const { return _indexArc; };
			};
        }
    }
}

#endif // ! EVA_SEGMENT_CENTRALISED_NETWORK_H