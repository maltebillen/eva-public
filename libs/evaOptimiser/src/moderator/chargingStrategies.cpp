#include "incl/moderator/chargingStrategies.h"

#include "evaExceptions.h"

const eva::ChargingStrategy::Session eva::FixAtEndChargingStrategy::get_chargingSession(const Types::DateTime& lb, const Types::DateTime& ub, const Vehicle& vehicle, const Charger& charger, const Types::BatteryCharge& prev_soc, const Types::BatteryCharge& fixed_discharge) const {
	eva::ChargingStrategy::Session session;

	// Check compatability:
	if (Helper::diffDateTime(lb, ub) >= (_optinput.get_config().get_const_put_vehicle_on_charge() + _optinput.get_config().get_const_take_vehicle_off_charge()))
	{
		session.charging_speed = vehicle.get_chargingSpeedKwS(charger);
		if (Helper::compare_floats_equal(session.charging_speed, 0.0))
			return session;

		session.index_takeOffCharge = _optinput.get_nextIdxTakeOffChargeBeforeEndTime(charger.get_index(), ub);

		Types::Index indexMaxTime = _optinput.get_nextIdxPutOnChargeAfterStartTime(charger.get_index(), lb);
		Types::Index indexFixedCharge = _optinput.get_nextIdxPutOnChargeBeforeEndTime(charger.get_index(),
																				  _optinput.get_takeOffCharge(charger.get_index(), session.index_takeOffCharge).get_scheduleNodeData().get_startTime() - static_cast<int64_t>(std::ceil(static_cast<double>(fixed_discharge) / session.charging_speed)));
		
		// For a charging connection to be feasible on a fixed charging strategy, there must be at least enough time to perform the necessary charging.
		if (indexMaxTime != Constants::BIG_INDEX)
		{
			if (indexFixedCharge != Constants::BIG_INDEX && indexFixedCharge >= indexMaxTime)
			{
				session.index_putOnCharge = indexFixedCharge;
			}
		}

		session.is_feasible = session.index_takeOffCharge != Constants::BIG_INDEX 
							&& session.index_putOnCharge != Constants::BIG_INDEX;
		session.is_charging = session.is_feasible ? session.index_takeOffCharge > session.index_putOnCharge : false; // If the vehice has at least one time slot between the put-on and take-off charge, it is charging.
	}
	else
	{
		session.is_charging = false; // Indicates that no charging is taking place here; however, the connection is feasible via the charger.
		session.is_feasible = (fixed_discharge == 0) && (Helper::diffDateTime(lb, ub) >= 0); // Only feasible, if the fixed discharge in the following node is zero, and the vehicle remains at the same location.
	}
	return session;
}

const eva::ChargingStrategy::Session eva::FixAtEndChargingStrategy::get_chargingSession(const Types::DateTime& lb, const Types::DateTime& ub, const Vehicle& vehicle, const Types::Index& indexCharger, const Types::BatteryCharge& prev_soc, const Types::BatteryCharge& fixed_discharge) const
{
	return get_chargingSession(lb, ub, vehicle, _optinput.get_charger(indexCharger), prev_soc, fixed_discharge);
}

const eva::ChargingStrategy::Session eva::VariableAtEndChargingStrategy::get_chargingSession(const Types::DateTime& lb, const Types::DateTime& ub, const Vehicle& vehicle, const Charger& charger, const Types::BatteryCharge& prev_soc, const Types::BatteryCharge& fixed_discharge) const
{
	eva::ChargingStrategy::Session session;

	if(Helper::diffDateTime(lb, ub) >= (_optinput.get_config().get_const_put_vehicle_on_charge() + _optinput.get_config().get_const_take_vehicle_off_charge()))
	{
		// Check compatability:
		session.charging_speed = vehicle.get_chargingSpeedKwS(charger);
		if (Helper::compare_floats_equal(session.charging_speed, 0.0))
			return session;

		// Set the bounds:
		session.index_takeOffCharge = _optinput.get_nextIdxTakeOffChargeBeforeEndTime(charger.get_index(), ub);

		Types::Index indexMaxTime = _optinput.get_nextIdxPutOnChargeAfterStartTime(charger.get_index(), lb);
		Types::Index indexFullCharge = _optinput.get_nextIdxPutOnChargeBeforeEndTime(charger.get_index(),
																					 _optinput.get_takeOffCharge(charger.get_index(), session.index_takeOffCharge).get_scheduleNodeData().get_startTime() - static_cast<int64_t>(std::ceil(static_cast<double>(vehicle.get_batteryMaxKWh() - prev_soc) / session.charging_speed)));

		session.index_putOnCharge = indexFullCharge != Constants::BIG_INDEX ? std::max(indexMaxTime, indexFullCharge) : indexMaxTime;
		session.is_feasible = session.index_takeOffCharge != Constants::BIG_INDEX && session.index_putOnCharge != Constants::BIG_INDEX;
		session.is_charging = session.is_feasible ? session.index_takeOffCharge > session.index_putOnCharge : false; // If the vehice has at least one time slot between the put-on and take-off charge, it is charging.
	}
	else
	{
		session.is_charging = false; // Indicates that no charging is taking place here; however, the connection is feasible via the charger.
		session.is_feasible = (Helper::diffDateTime(lb, ub) >= 0); // However, it remains a feasible connection if the time-space continuity is preserved. 
	}

	return session;
}

const eva::ChargingStrategy::Session eva::VariableAtEndChargingStrategy::get_chargingSession(const Types::DateTime& lb, const Types::DateTime& ub, const Vehicle& vehicle, const Types::Index& indexCharger, const Types::BatteryCharge& prev_soc, const Types::BatteryCharge& fixed_discharge) const
{
	return get_chargingSession(lb, ub, vehicle, _optinput.get_charger(indexCharger), prev_soc, fixed_discharge);
}

