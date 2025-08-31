#include "incl/pricingProblem/segmentBased/segments/segment.h"

void eva::sbn::Segment::_initSegmentMetrics(const OptimisationInput &optinput, const std::vector<SegmentActivity> &vecActivities)
{
	// Iterate over all segment activities, to determine all 
	Types::Index prevLocationIndex = _startCharger.get_location().get_index();
	for (const SegmentActivity &curSegmentActivity : vecActivities)
	{
		// Update the member variables:
		_deadlegFixCost += optinput.get_config().get_cost_deadleg_fix() + optinput.get_config().get_cost_deadleg_per_km() * optinput.get_location(prevLocationIndex).get_distanceToLocation(curSegmentActivity.get_startLocationIndex());

		_distanceBeforeMaintenance = _hasMaintenance ? _distanceBeforeMaintenance : _distanceBeforeMaintenance + optinput.get_location(prevLocationIndex).get_distanceToLocation(curSegmentActivity.get_startLocationIndex()) + optinput.get_location(curSegmentActivity.get_startLocationIndex()).get_distanceToLocation(curSegmentActivity.get_endLocationIndex());
		_distanceAfterMaintenance = !_hasMaintenance ? _distanceAfterMaintenance : _distanceAfterMaintenance + optinput.get_location(prevLocationIndex).get_distanceToLocation(curSegmentActivity.get_startLocationIndex()) + optinput.get_location(curSegmentActivity.get_startLocationIndex()).get_distanceToLocation(curSegmentActivity.get_endLocationIndex());

		_hasMaintenance = curSegmentActivity.get_type() == SegmentActivityType::MAINTENANCE ? true : _hasMaintenance;

		// Store the previous location:
		prevLocationIndex = curSegmentActivity.get_endLocationIndex();
	}

	// Final member variable updates:
	_distanceBeforeMaintenance = _hasMaintenance ? _distanceBeforeMaintenance : _distanceBeforeMaintenance + optinput.get_location(prevLocationIndex).get_distanceToLocation(_endCharger.get_location());
	_distanceAfterMaintenance = !_hasMaintenance ? _distanceAfterMaintenance : _distanceAfterMaintenance + optinput.get_location(prevLocationIndex).get_distanceToLocation(_endCharger.get_location());
	_deadlegFixCost += optinput.get_config().get_cost_deadleg_fix() + optinput.get_config().get_cost_deadleg_per_km() * optinput.get_location(prevLocationIndex).get_distanceToLocation(_endCharger.get_location());

	if (optinput.get_flag_has_unassigned_maintenance())
	{
		_maintenanceFixCost = 0.5 * optinput.get_config().get_cost_coefficient_penalty_maintenance() * (std::pow(_distanceBeforeMaintenance, 2.0) + std::pow(_distanceAfterMaintenance, 2.0));
		_maintenanceVariableCostCoeff = optinput.get_config().get_cost_coefficient_penalty_maintenance() * _distanceBeforeMaintenance;
	}
	else
	{
		_maintenanceFixCost = 0.0;
		_maintenanceVariableCostCoeff = 0.0;
	}
}

void eva::sbn::Segment::_initFeasibileVehicles(const OptimisationInput &optinput)
{
	// Step 2: Determine all feasible vehicles:
	_vecFeasibleVehicle.resize(optinput.get_vehicles().get_vec().size(), Types::AccessType::NOT_ALLOWED);

	for (const Vehicle &vehicle : optinput.get_vehicles().get_vec())
	{
		// If the segment has assigned maintenance activities, and no unassigned activities, then only the assigned vehicles are feasible on this segment.
		if (vehicle.get_distanceRange() >= (_distanceBeforeMaintenance + _distanceAfterMaintenance) 
			&& !Helper::compare_floats_equal(vehicle.get_chargingSpeedKwS(_startCharger), 0.0))
		{
			_vecFeasibleVehicle[vehicle.get_index()] = Types::AccessType::ALLOWED;
		}
	}
}

void eva::sbn::Segment::_initChargingBounds(const OptimisationInput &optinput)
{
	// Determine the lb, ub recharge duration before entering the segment:
	double chargingSpeed;
	uint32_t lb, ub;
	for (const Vehicle &vehicle : optinput.get_vehicles().get_vec())
	{
		if (_vecFeasibleVehicle[vehicle.get_index()] == Types::AccessType::ALLOWED)
		{
			// Always charging at the startCharger:
			chargingSpeed = vehicle.get_chargingSpeedKwS(_startCharger);
			lb = ub = std::ceil(vehicle.get_batteryDischarge(_distanceBeforeMaintenance + _distanceAfterMaintenance) / chargingSpeed);

			// Update the minimum lb and max ub:
			_minlb_rechargeDuration = _minlb_rechargeDuration > lb ? lb : _minlb_rechargeDuration;
			_maxub_rechargeDuration = _maxub_rechargeDuration < ub ? ub : _maxub_rechargeDuration;
		}
	}

	// Step 5: Pass the min and max charging duration to the subgraph to use for computing charging duals:
	_minlb_fullPresenceAtCharger = _minlb_rechargeDuration + (optinput.get_config().get_const_charger_capacity_check() - (_minlb_rechargeDuration % optinput.get_config().get_const_charger_capacity_check())) + optinput.get_config().get_const_put_vehicle_on_charge() + optinput.get_config().get_const_take_vehicle_off_charge();
	_maxub_fullPresenceAtCharger = _maxub_rechargeDuration + (optinput.get_config().get_const_charger_capacity_check() - (_maxub_rechargeDuration % optinput.get_config().get_const_charger_capacity_check())) + optinput.get_config().get_const_put_vehicle_on_charge() + optinput.get_config().get_const_take_vehicle_off_charge();
}

void eva::sbn::Segment::_initSubgraph(const OptimisationInput &optinput, const std::vector<SegmentActivity> &vecActivities)
{
	// Step 1: Initialise the subGraph, and member variables:
	_subgraph.initialise(vecActivities.size() + 2);
	_subgraph.initialiseInitialVehicleAccess(_vecFeasibleVehicle);
	_subgraph.initialiseChargingDurationBounds(_minlb_rechargeDuration, _maxub_rechargeDuration);

	// Step 2: Iterate over all mid-layers:
	uint32_t layer = 1;
	for (const SegmentActivity &curSegmentActivity : vecActivities)
	{
		for (const auto &refIndex : curSegmentActivity.get_vecSegmentReferenceIndexes())
		{
			switch (curSegmentActivity.get_type())
			{
			case SegmentActivityType::TRIP:
				_subgraph.addTripNode(optinput.get_trip(refIndex), layer);
				break;
			case SegmentActivityType::MAINTENANCE:
				_subgraph.addMaintenanceNode(optinput.get_maintenance(refIndex), layer);
			break;
			default:
				break;
			};
		}

		++layer;
	}

	// Step 3: Create the start/end-layer and add all connections:
	_subgraph.addAuxiliaryNodes(_startCharger, _endCharger);
	_subgraph.addConnections();
}

void eva::sbn::Segment::_initialise(const OptimisationInput &optinput, const std::vector<SegmentActivity> &vecActivities) 
{
    // Initialise the segment:
	if (vecActivities.size() > 0)
	{
		// This order must remain!!!
		_initSegmentMetrics(optinput,vecActivities);
		_initFeasibileVehicles(optinput);
		_initChargingBounds(optinput);
		_initSubgraph(optinput,vecActivities);
	};
}
