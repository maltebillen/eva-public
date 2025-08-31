#include "incl/pricingProblem//segmentBased/segments/segments.h"

void eva::sbn::Segments::_createSegments(const OptimisationInput& optinput)
{
	// Step 0: Initialise:
	std::vector<SegmentActivity> vecSegmentActivities;
	std::vector<SegmentActivity>::iterator iterSegmentActivities;

	// Step 1: Find all segment Activities:
	// Maintenance:
	for (const SubScheduleMaintenanceNodeData& submaintenance : optinput.get_vecMaintenances())
	{
		// Create an initial segment activity:
		SegmentActivity temp(
			submaintenance.get_scheduleNodeData().get_startLocation().get_index(),
			submaintenance.get_scheduleNodeData().get_endLocation().get_index(),
			submaintenance.get_scheduleNodeData().get_startLocation().get_distanceToLocation(submaintenance.get_scheduleNodeData().get_endLocation()),
			SegmentActivityType::MAINTENANCE);

		// Check if a maintenance segment exists in the respective start-end combination:
		iterSegmentActivities = std::find(vecSegmentActivities.begin(), vecSegmentActivities.end(), temp);
		if (iterSegmentActivities != vecSegmentActivities.end())
		{
			iterSegmentActivities->appendReferenceIndex(submaintenance.get_index());
		}
		else
		{
			temp.appendReferenceIndex(submaintenance.get_index());
			vecSegmentActivities.push_back(temp);
		}
	}	

	// Trips:
	for (const SubScheduleTripNodeData& subtrip : optinput.get_vecTrips())
	{
		// Create an initial segment activity:
		SegmentActivity temp(
			subtrip.get_scheduleNodeData().get_startLocation().get_index(),
			subtrip.get_scheduleNodeData().get_endLocation().get_index(),
			subtrip.get_scheduleNodeData().get_startLocation().get_distanceToLocation(subtrip.get_scheduleNodeData().get_endLocation()),
			SegmentActivityType::TRIP
		);

		// Check if a trip segment exists in the respective start-end combination:
		iterSegmentActivities = std::find(vecSegmentActivities.begin(), vecSegmentActivities.end(), temp);
		if (iterSegmentActivities != vecSegmentActivities.end())
		{
			iterSegmentActivities->appendReferenceIndex(subtrip.get_index());
		}
		else
		{
			temp.appendReferenceIndex(subtrip.get_index());
			vecSegmentActivities.push_back(temp);
		}
	}

	// Step 2: Determine the vehicle with the longest range:
	uint32_t maxDistanceRange = std::max_element(optinput.get_vehicles().get_vec().begin(), optinput.get_vehicles().get_vec().end(), [&](const Vehicle& a, const Vehicle& b) {
		return a.get_distanceRange() < b.get_distanceRange();
	})->get_distanceRange();

	// Step 3: Determine the feasible segments:
	for (const Charger& startCharger : optinput.get_chargers().get_vec())
	{
		for (const Charger& endCharger : optinput.get_chargers().get_vec())
		{
			// Create all segments:
			_recursionGenerateSegments(
				optinput,
				vecSegmentActivities,
				startCharger,
				endCharger,
				maxDistanceRange,
				startCharger.get_location(),
				0,
				std::vector<SegmentActivity>()
			);
		}
	}
}

void eva::sbn::Segments::_recursionGenerateSegments(const OptimisationInput& optinput, const std::vector<SegmentActivity>& vecAllActivities, const Charger& startCharger, const Charger& endCharger, const uint32_t& range, const Location curLocation, const uint32_t distance, std::vector<SegmentActivity> vecThisSegmentActivities)
{
	// Iterate over every activity and check if the SOC is sufficient to perform the following activity as well as the return to the endCharger:
	for (const SegmentActivity& segmentActivity : vecAllActivities)
	{
		// 1. Rule: no maintenance twice at the same location within the same segment:
		if (segmentActivity.get_type() == SegmentActivityType::MAINTENANCE)
		{
			bool foundMaintenance = false;
			for(const auto& act : vecThisSegmentActivities )
			{
				if(act.get_type() == SegmentActivityType::MAINTENANCE)
				{
					foundMaintenance = true;
					break;
				}
			};

			if(foundMaintenance)
			{
				continue;
			}
		}
		
		// 2. Rule: deadleg to the new location must be feasible:
		uint32_t distanceDischarge = curLocation.get_distanceToLocation(segmentActivity.get_startLocationIndex());
		if (distanceDischarge == Constants::BIG_UINTEGER
			|| (distanceDischarge > 0 && !optinput.get_config().get_flag_allow_deadlegs())) 
			continue;

		// 3. Rule: extension to the segment activity must be feasible and within range::
		uint32_t newDistance = distance
			+ distanceDischarge
			+ segmentActivity.get_distance();

		if (newDistance <= range)
		{
			// Extension to the segment activity is feasible:
			std::vector<SegmentActivity> vecNewSegmentActivities = vecThisSegmentActivities;
			vecNewSegmentActivities.push_back(segmentActivity);

			// Final: Check if the vehicle could make a return to the endCharger:
			uint32_t deadlegDistanceToEndCharger = optinput.get_location(segmentActivity.get_endLocationIndex()).get_distanceToLocation(endCharger.get_location());
			if (deadlegDistanceToEndCharger != Constants::BIG_UINTEGER
				&& (deadlegDistanceToEndCharger == 0 || optinput.get_config().get_flag_allow_deadlegs())
				&& newDistance + deadlegDistanceToEndCharger <= range)
			{
				_vecSegments.push_back(
					Segment(
						_getNextIndex(),
						optinput,
						startCharger,
						endCharger,
						vecNewSegmentActivities
						)
				);
			}

			// Regardless, continue with the recursion:
			_recursionGenerateSegments(
				optinput,
				vecAllActivities,
				startCharger,
				endCharger,
				range,
				optinput.get_location(segmentActivity.get_endLocationIndex()),
				newDistance,
				vecNewSegmentActivities
			);
		}

	}
}

void eva::sbn::Segments::initialise(const OptimisationInput& optinput)
{
	_createSegments(optinput);
}