#ifndef EVA_OPTIMISATION_INPUT_H
#define EVA_OPTIMISATION_INPUT_H

#include "evaConstants.h"
#include "evaDataHandler.h"

#include "SubScheduleNodes.h"

namespace eva
{
	class OptimisationInput
	{
		DataHandler& _dataHandler;

		Types::Index _indexPlanningHorizon = 0;
		Types::DateTime _startPlanningHorizon = Constants::MAX_TIMESTAMP;
		Types::DateTime _endPlanningHorizon = Constants::MAX_TIMESTAMP;
		Types::DateTime _endPlanningHorizonOverlap = Constants::MAX_TIMESTAMP;
		bool _flag_has_unassigned_maintenance = false;

		std::vector<SubScheduleTripNodeData> _vecTrips;
		std::vector<SubScheduleMaintenanceNodeData> _vecMaintenances;
		std::vector<std::vector<SubSchedulePutOnChargeNodeData>> _vecPutOnChargeNodes;
		std::vector<std::vector<SubScheduleTakeOffChargeNodeData>> _vecTakeOffChargeNodes;

		Types::DateTime _earliestVehicleTime = Constants::MAX_TIMESTAMP;

		// FUNCTION DEFINITIONS

		void _initialise();
		void _reset();
		void _loadScheduleNodes();

		void _updateEarliestVehicleTime();

	public:	
		OptimisationInput() = delete;
		OptimisationInput(
			DataHandler& dataHandler
		) :
			_dataHandler(dataHandler)
		{
			_initialise();
		};

		// FUNCTION DEFINITIONS

		bool next();

		// GETTERS

		inline const Types::Index get_indexPlanningHorizon() const { return _indexPlanningHorizon; };
		inline const Types::DateTime get_startPlanningHorizon() const { return _startPlanningHorizon; };
		inline const Types::DateTime get_endPlanningHorizon() const { return _endPlanningHorizon; };
		inline const Types::DateTime get_endPlanningHorizonOverlap() const { return _endPlanningHorizonOverlap; };

		inline const Config& get_config() const { return _dataHandler.get_config(); };
		inline const Vehicles& get_vehicles() const { return _dataHandler.get_vehicles(); };
		inline const Vehicle& get_vehicle(const Types::Index& index) const { return _dataHandler.get_vehicles().get_vec()[index]; };
		inline const Chargers& get_chargers() const { return _dataHandler.get_chargers(); };
		inline const Charger& get_charger(const Types::Index& index) const { return _dataHandler.get_chargers().get_vec()[index]; };
		inline const Location& get_location(const Types::Index& index) const { return _dataHandler.get_locations().get_vec()[index]; };

		inline const std::vector<SubScheduleTripNodeData>& get_vecTrips() const { return _vecTrips; };
		inline const SubScheduleTripNodeData& get_trip(const Types::Index& index) const { return _vecTrips[index]; };
		inline const std::vector<SubScheduleMaintenanceNodeData>& get_vecMaintenances() const { return _vecMaintenances; };
		inline const SubScheduleMaintenanceNodeData& get_maintenance(const Types::Index& index) const { return _vecMaintenances[index]; };
		inline const std::vector<SubSchedulePutOnChargeNodeData>& get_vecPutOnChargeNodes(const Types::Index& indexCharger) const { return _vecPutOnChargeNodes[indexCharger]; };
		inline const SubSchedulePutOnChargeNodeData& get_putOnCharge(const Types::Index& indexCharger, const Types::Index& indexPutOnCharge) const { return _vecPutOnChargeNodes[indexCharger][indexPutOnCharge]; };
		inline const std::vector<SubScheduleTakeOffChargeNodeData>& get_vecTakeOffChargeNodes(const Types::Index& indexCharger) const { return _vecTakeOffChargeNodes[indexCharger]; };
		inline const SubScheduleTakeOffChargeNodeData& get_takeOffCharge(const Types::Index& indexCharger, const Types::Index& indexTakeOffCharge) const { return _vecTakeOffChargeNodes[indexCharger][indexTakeOffCharge]; };
		const Types::Index get_nextIdxPutOnChargeAfterStartTime(const Types::Index& indexCharger, const Types::DateTime& starttime) const;
		const Types::Index get_nextIdxTakeOffChargeAfterStartTime(const Types::Index& indexCharger, const Types::DateTime& starttime) const;
		const Types::Index get_nextIdxPutOnChargeBeforeEndTime(const Types::Index& indexCharger, const Types::DateTime& endttime) const;
		const Types::Index get_nextIdxTakeOffChargeBeforeEndTime(const Types::Index& indexCharger, const Types::DateTime& endttime) const;
	
		const ScheduleResourceContainer& get_vehiclePosition(const Vehicle& vehicle) const { return _dataHandler.get_vehiclePosition(vehicle); };
		inline const ScheduleNodeData& get_scheduleGraphNodeData(const BoostScheduleNode& scheduleNode) const { return _dataHandler.get_scheduleGraph().get_nodeData(scheduleNode); };
		inline const ScheduleGraph& get_scheduleGraph() const { return _dataHandler.get_scheduleGraph(); };

		inline const bool get_flag_has_unassigned_maintenance() const { return _flag_has_unassigned_maintenance;};
		
		DataHandler& get_dataHandler() { return _dataHandler; };
	};


}


#endif // ! EVA_OPTIMISATION_INPUT_H