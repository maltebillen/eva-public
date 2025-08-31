#ifndef EVA_SEGMENTS_SUB_GRAPH_H
#define EVA_SEGMENTS_SUB_GRAPH_H

#include "subGraphNodes.h"
#include "subGraphArcs.h"

#include "moderator/OptimisationInput.h"
#include "moderator/moderator.h"
#include "moderator/chargingStrategies.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace eva
{
	namespace sbn
	{
		namespace subgraph
		{
			typedef boost::adjacency_list<
				boost::vecS,
				boost::vecS,
				boost::directedS,
				eva::sbn::subgraph::NodeData,
				eva::sbn::subgraph::ArcData>
				BoostSubGraph;

			typedef boost::graph_traits<BoostSubGraph>::vertex_descriptor BoostSubGraphNode;
			typedef boost::graph_traits<BoostSubGraph>::edge_descriptor BoostSubGraphArc;

			class NonDominatedSchedulePiece
			{
				double _accDuals = Constants::BIG_DOUBLE;
				double _minChargingDuals = Constants::BIG_DOUBLE;
				double _maxChargingDuals = Constants::BIG_DOUBLE;
				Types::DateTime _startTime = Constants::MAX_TIMESTAMP;
				Types::DateTime _endTime = Constants::MAX_TIMESTAMP;
				Types::Index _indexFixedVehicle = Constants::BIG_INDEX;

				std::vector<Types::AccessType> _vecVehicleAccess;
				std::vector<Types::Index> _vecTripIndexes;
				std::vector<Types::Index> _vecMaintenanceIndexes;
				std::vector<Types::Index> _vecScheduleNodeIndexes;

			public:
				// CONSTRUCTORS:

				NonDominatedSchedulePiece() = delete;

				NonDominatedSchedulePiece(
					const double& accDuals,
					const double& minChargingDuals,
					const double& maxChargingDuals,
					const Types::DateTime& startTime,
					const Types::DateTime& endTime,
					const std::vector<Types::AccessType>& vecVehicleAccess,
					const std::vector<Types::Index>& vecTripIndexes,
					const std::vector<Types::Index>& vecMaintenanceIndexes,
					const std::vector<Types::Index>& vecScheduleNodeIndexes,
					const Types::Index indexFixedVehicle
				) :
					_accDuals(accDuals),
					_minChargingDuals(minChargingDuals),
					_maxChargingDuals(maxChargingDuals),
					_startTime(startTime),
					_endTime(endTime),
					_vecTripIndexes(vecTripIndexes),
					_vecVehicleAccess(vecVehicleAccess),
					_vecMaintenanceIndexes(vecMaintenanceIndexes),
					_vecScheduleNodeIndexes(vecScheduleNodeIndexes),
					_indexFixedVehicle(indexFixedVehicle)
				{}

				inline const double& get_accDuals() const { return _accDuals; };
				inline const double get_minChargingDuals() const { return _minChargingDuals; };
				inline const double get_maxChargingDuals() const { return _maxChargingDuals; };

				inline const Types::DateTime& get_startTime() const { return _startTime; };
				inline const Types::DateTime& get_endTime() const { return _endTime; };

				inline const std::vector<Types::AccessType>& get_vecVehicleAccess() const { return _vecVehicleAccess; };
				inline const std::vector<Types::Index>& get_vecTripIndexes() const { return _vecTripIndexes; };
				inline const std::vector<Types::Index>& get_vecMaintenanceIndexes() const { return _vecMaintenanceIndexes; };
				inline const std::vector<Types::Index>& get_vecScheduleNodeIndexes() const { return _vecScheduleNodeIndexes; };
				inline const Types::Index& get_indexFixedVehicle() const { return _indexFixedVehicle;};
			};

			class SegmentSubGraph
			{
				// MEMBERS 
				const OptimisationInput& _optinput;
				BoostSubGraph _boostSubGraph;
				FixAtEndChargingStrategy _chargingStrategy;

				Types::Index _indexNode{ 0 };
				Types::Index _indexArc{ 0 };

				std::vector<std::vector<BoostSubGraphNode>> _vecLayers;
				std::vector<NonDominatedSchedulePiece> _vecCurrentNonDominatedSchedulePieces;
				std::vector<Types::AccessType> _vecDefaultVehicleAccess;

				BoostSubGraphNode _sourceNode;
				BoostSubGraphNode _sinkNode;

				uint32_t _minlb_rechargeDuration = Constants::BIG_UINTEGER;
				uint32_t _maxub_rechargeDuration = Constants::BIG_UINTEGER;

				std::unordered_map<Types::Index, std::vector<BoostSubGraphNode>> _mapScheduleNodeLookUp;
				std::unordered_map<BoostSubGraphNode, Types::Index> _mapLayerLookUp;

				// FUNCTIONS
				void _updateDuals(const Duals& duals);

				Types::Index _getNextIndexNode() { return _indexNode++; }
				Types::Index _getNextIndexArc() { return _indexArc++; }

				const eva::sbn::subgraph::NodeData& _getNodeData(const BoostSubGraphNode& node) const { return _boostSubGraph[node]; };
				eva::sbn::subgraph::NodeData& _getNodeData(const BoostSubGraphNode& node) { return _boostSubGraph[node]; };
				const eva::sbn::subgraph::ArcData& _getArcData(const BoostSubGraphArc& arc) const { return boost::get(boost::edge_bundle, _boostSubGraph)[arc]; };

			public:
				SegmentSubGraph(
					const OptimisationInput& optinput
				) :
					_optinput(optinput),
					_chargingStrategy(FixAtEndChargingStrategy(optinput))
				{};

				// FUNCTION DEFINITIONS:

				void initialise(const uint32_t& nrLayers);
				void initialiseChargingDurationBounds(const uint32_t& lb, const uint32_t& ub) { _minlb_rechargeDuration = lb;  _maxub_rechargeDuration = ub; };
				void initialiseInitialVehicleAccess(const std::vector<Types::AccessType>& vecInitialVehicleAccess) { _vecDefaultVehicleAccess = vecInitialVehicleAccess;};

				void addTripNode(const SubScheduleTripNodeData& trip, const uint32_t& layer);
				void addMaintenanceNode(const SubScheduleMaintenanceNodeData& maintenance, const uint32_t& layer);
				void addAuxiliaryNodes(const Charger& startCharger, const Charger& endCharger);
				void addConnections();

				void updateCurrentNonDominatedSchedulePieces(const BranchNode& brn, const Duals& duals);
				void updateFixings(const BranchNode& brn);

				// GETTERS:

				inline const std::vector<NonDominatedSchedulePiece>& get_vecCurrentNonDominatedSchedulePieces() const { return _vecCurrentNonDominatedSchedulePieces; };
			};

			struct SubGraphResourceContainer
			{
				// ATTRIBUTES:

				double accDuals = 0.0;
				double minChargingDuals = 0.0;
				double maxChargingDuals = 0.0;
				Types::DateTime timestampEnd = Constants::MAX_TIMESTAMP;
				Types::DateTime timestampStart = Constants::MAX_TIMESTAMP;
				std::vector<Types::AccessType> vecAccess;
				Types::Index indexFixedVehicle = Constants::BIG_INDEX;
				Types::DateTime fixedAllowedEndTime = Constants::MAX_TIMESTAMP;
				bool isFeasible = true;

				// CONSTRUCTORS:

				SubGraphResourceContainer() {}

				SubGraphResourceContainer(
					const double& accDuals,
					const double& minChargingDuals,
					const double& maxChargingDuals,
					const Types::DateTime& timestampEnd,
					const Types::DateTime& timestampStart,
					const std::vector<Types::AccessType>& vecAccess,
					const Types::Index indexFixedVehicle,
					const Types::DateTime& fixedAllowedEndTime
				) :
					accDuals(accDuals),
					minChargingDuals(minChargingDuals),
					maxChargingDuals(maxChargingDuals),
					timestampEnd(timestampEnd),
					timestampStart(timestampStart),
					vecAccess(vecAccess),
					indexFixedVehicle(indexFixedVehicle),
					fixedAllowedEndTime(fixedAllowedEndTime)
				{}

				~SubGraphResourceContainer() {}

				SubGraphResourceContainer& operator=(const SubGraphResourceContainer& other);
			};

			bool operator==(const SubGraphResourceContainer& res_cont_1, const SubGraphResourceContainer& res_cont_2);
			bool operator<(const SubGraphResourceContainer& res_cont_1, const SubGraphResourceContainer& res_cont_2);

			class SubGraphDominanceCheck
			{	

			public:
				inline bool operator()(const SubGraphResourceContainer& res_cont_1,
					const SubGraphResourceContainer& res_cont_2) const
				{
					// Check if res_cont_1 dominates res_cont_2!
					// must be "<=" here!!!
					// must NOT be "<"!!!

					if (Helper::compare_is_subset(res_cont_1.vecAccess, res_cont_2.vecAccess))
					{
						if (res_cont_1.timestampStart == res_cont_2.timestampStart)
							return Helper::compare_floats_smaller_equal(res_cont_2.accDuals, res_cont_1.accDuals) && res_cont_1.timestampEnd <= res_cont_2.timestampEnd;
						else
							return Helper::compare_floats_smaller_equal(res_cont_2.accDuals + res_cont_2.maxChargingDuals, res_cont_1.accDuals + res_cont_1.minChargingDuals) && res_cont_1.timestampEnd <= res_cont_2.timestampEnd && res_cont_1.timestampStart >= res_cont_2.timestampStart;
					}
					else
						return false; // If the labels represent different access of vehicles, then the label can never be dominated.
				};
			};

			struct SubGraphResourceExtensionFunction
			{
				bool operator()(
					const BoostSubGraph& subGraph,
					SubGraphResourceContainer& new_cont,
					const SubGraphResourceContainer& old_cont,
					const BoostSubGraphArc& arc
					) const;
			};
		}
	}
}

#endif // EVA_SEGMENTS_SUB_GRAPH_H