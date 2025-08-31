#ifndef EVA_SCHEDULE_GRAPH_H
#define EVA_SCHEDULE_GRAPH_H

#include "evaConstants.h"
#include "../dataInput/dataInput.h"

#include "scheduleNode.h"
#include "scheduleArc.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace eva
{
	typedef boost::adjacency_list<
		boost::vecS,
		boost::vecS,
		boost::directedS,
		ScheduleNodeData,
		ScheduleArcData
	> BoostScheduleGraph;

	typedef boost::graph_traits<BoostScheduleGraph>::vertex_descriptor BoostScheduleNode;
	typedef boost::graph_traits<BoostScheduleGraph>::edge_descriptor BoostScheduleArc;

	struct VehiclePosition
	{
		Types::BatteryCharge soc = Constants::BIG_BATTERYCHARGE;
		BoostScheduleNode lastScheduleNode = Constants::BIG_INDEX;
		uint32_t odometerReading = Constants::BIG_UINTEGER;
		uint32_t odometerLastMaintenance = Constants::BIG_UINTEGER;
	};

	struct NodesInDateInterval
	{
		std::vector<BoostScheduleNode> vecSortedTrips;
		std::vector<BoostScheduleNode> vecSortedMaintenances;
		std::vector<BoostScheduleNode> vecSortedPutOnCharges;
		std::vector<BoostScheduleNode> vecSortedTakeOffCharges;
		std::vector<BoostScheduleNode> vecSortedVehicleStarts;
	};

	struct ScheduleResourceContainer
	{
		double cost_deadlegs = 0.0;
		double cost_maintenance = 0.0;
		double cost_vehicle = 0.0;

		uint32_t dist_deadlegs = 0;
		uint32_t odometerReading = 0;
		uint32_t odometerLastMaintenance = 0;
		std::vector<uint32_t> vecMaintenanceDistances;
		Types::BatteryCharge soc = 0;
		Types::BatteryCharge lb_soc = Constants::BIG_BATTERYCHARGE;
		Types::BatteryCharge ub_soc = -Constants::BIG_BATTERYCHARGE;

		int64_t seconds_charging = 0;
		int64_t seconds_idle = 0;
		int64_t seconds_maintenance = 0;
		int64_t seconds_productive = 0;
		BoostScheduleNode lastScheduleNode = Constants::BIG_INDEX;

		ScheduleResourceContainer() {};
		ScheduleResourceContainer(
			const uint32_t& odometerReading,
			const uint32_t& odometerLastMaintenance,
			const Types::BatteryCharge& initialSOC,
			const BoostScheduleNode& initScheduleNode
		) :
			odometerReading(odometerReading),
			odometerLastMaintenance(odometerLastMaintenance),
			soc(initialSOC),
			lastScheduleNode(initScheduleNode)
		{};

		ScheduleResourceContainer(
			const uint32_t& odometerReading,
			const uint32_t& odometerLastMaintenance,
			const Types::BatteryCharge& initialSOC
		) :
			odometerReading(odometerReading),
			odometerLastMaintenance(odometerLastMaintenance),
			soc(initialSOC)
		{};

		~ScheduleResourceContainer() {};

		inline const uint32_t distanceLastMaintenance() const { return odometerReading - odometerLastMaintenance; };
		inline const double cost_total() const { return cost_deadlegs + cost_vehicle + cost_maintenance; };
		const double avg_distance_maintenance() const;
		const double std_distance_maintenance() const;
	};

	class ScheduleGraph
	{
		// ATTRIBUTES

		BoostScheduleGraph _boostScheduleGraph;
		
		std::vector<BoostScheduleNode> _vecSortedTrips;
		std::vector<BoostScheduleNode> _vecSortedMaintenances;
		std::vector<BoostScheduleNode> _vecSortedPutOnCharges;
		std::vector<BoostScheduleNode> _vecSortedTakeOffCharges;

		std::vector<BoostScheduleNode> _vecStartNodes;
		std::vector<std::vector<BoostScheduleArc>> _vecSchedulePaths;
		std::vector<ScheduleResourceContainer> _vecCurrentVehiclePositions;

		Types::Index _indexNode{ 0 };
		Types::Index _indexArc{ 0 };

		Types::DateTime _earliestVehicleStartTime = Constants::MAX_TIMESTAMP;

		// PRIVATE FUNCTION DEFINITIONS:
		void _add_trips(const std::vector<Trip>& vecTrips);
		void _add_chargers(const std::vector<Charger>& vecChargers, const Config& config);
		void _add_maintenances(const std::vector<Maintenance>& vecMaintenances);
		void _add_scheduleStartNodes(const std::vector<Vehicle>& vecVehicles);
		BoostScheduleNode _addNode(const ScheduleNodeData& nodeData, std::vector<BoostScheduleNode>& vecSorted);
		BoostScheduleNode _addNode(const ScheduleNodeData& nodeData);
		void _storeNodeSorted(const BoostScheduleNode& node, std::vector<BoostScheduleNode>& vecSorted);

		const bool _checkNodeCoverage(const BoostScheduleNode& node) const;

		bool _checkScheduleTimeSpaceContinuity();

		// PRIVATE GETTERS:

		Types::Index _getNextIndexNode() { return _indexNode++; }
		Types::Index _getNextIndexArc() { return _indexArc++; }

		const std::pair<std::vector<BoostScheduleNode>::const_iterator, std::vector<BoostScheduleNode>::const_iterator> _getBoundsTimeSortedVector(const std::vector<BoostScheduleNode>& vec, const Types::DateTime& lb, const Types::DateTime& ub) const;
		const std::vector<BoostScheduleNode> _getNodesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate, const std::vector<BoostScheduleNode>& vecSortedNodes) const;

	public:
		// CONSTRUCTOR

		ScheduleGraph() {};

		// FUNCTION DEFINITIONS:

		const std::vector<BoostScheduleNode> getTripsInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const;
		const std::vector<BoostScheduleNode> getMaintenancesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const;
		const std::vector<BoostScheduleNode> getPutOnChargesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const;
		const std::vector<BoostScheduleNode> getTakeOffChargesInDateInterval(const Types::DateTime& startDate, const Types::DateTime& endDate) const;

		bool add_out_of_rotation(const DataInput& input, const BoostScheduleNode& fromNode, const Vehicle& vehicle, const Types::DateTime& startHorizon, const Types::DateTime& endHorizon);
		bool add_deadleg(const BoostScheduleNode& fromNode, const BoostScheduleNode& toNode, const Vehicle& vehicle);
		bool add_deadleg(const BoostScheduleNode& fromNode, const Location& toLocation, const Vehicle& vehicle);
		bool add_charging(const BoostScheduleNode& fromNode, const BoostScheduleNode& toNode, const Vehicle& vehicle);

		void initialise(const DataInput& input);
		
		void clear();
		void updateVehiclePositions(const DataInput& input);

		const ScheduleResourceContainer& getVehiclePosition(const Vehicle& vehicle) const { return _vecCurrentVehiclePositions[vehicle.get_index()]; };
		const std::vector<BoostScheduleNode> get_unassignedTripNodes() const;
		const std::vector<BoostScheduleNode> get_unassignedMaintenanceNodes() const;

		void processArc(const DataInput& input, const Vehicle& vehicle, ScheduleResourceContainer& new_cont, ScheduleResourceContainer& old_cont,	const BoostScheduleArc& arc) const;

		const ScheduleNodeData& operator[](const BoostScheduleNode& node) const { return _boostScheduleGraph[node]; }
		const ScheduleNodeData& get_nodeData(const BoostScheduleNode& node) const { return _boostScheduleGraph[node]; };
		const ScheduleNodeData& get_targetNodeData(const BoostScheduleArc& arc) const { return get_nodeData(boost::target(arc,_boostScheduleGraph)); };
		const ScheduleNodeData& get_sourceNodeData(const BoostScheduleArc& arc) const { return get_nodeData(boost::source(arc,_boostScheduleGraph)); };
		const ScheduleArcData& get_arcData(const BoostScheduleArc& arc) const { return boost::get(boost::edge_bundle, _boostScheduleGraph)[arc]; };
		const uint32_t get_numberOutgoingArcs(const BoostScheduleNode& node) const { return boost::out_degree(node, _boostScheduleGraph); };

		inline const std::vector<BoostScheduleArc>& get_vecSchedulePath(const Types::Index& indexVehicle) const { return _vecSchedulePaths[indexVehicle]; };

		inline const std::vector<BoostScheduleNode>& get_vecSortedTrips() const { return _vecSortedTrips; };
		inline const std::vector<BoostScheduleNode>& get_vecSortedMaintenances() const { return _vecSortedMaintenances; };
		inline const std::vector<BoostScheduleNode>& get_vecSortedPutOnCharges() const { return _vecSortedPutOnCharges; };
		inline const std::vector<BoostScheduleNode>& get_vecSortedTakeOffCharges() const { return _vecSortedTakeOffCharges; };
	

	};
}


#endif // ! EVA_SCHEDULE_GRAPH_H