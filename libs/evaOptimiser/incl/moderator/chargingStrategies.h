#ifndef EVA_CHARGING_STRATEGIES_H
#define EVA_CHARGING_STRATEGIES_H

#include "evaConstants.h"
#include "OptimisationInput.h"
#include "moderator.h"

namespace eva 
{
	struct ChargingStrategy
	{
		virtual ~ChargingStrategy() {};

		struct Session {
			Types::Index index_putOnCharge = Constants::BIG_INDEX;
			Types::Index index_takeOffCharge = Constants::BIG_INDEX;
			double charging_speed = 0.0;
			bool is_charging = false;
			bool is_feasible = false;

			inline const Types::BatteryCharge get_charge(const OptimisationInput& optinput, const Types::Index indexCharger) const {
				return Helper::diffDateTime(optinput.get_putOnCharge(indexCharger, index_putOnCharge).get_scheduleNodeData().get_endTime(),
					optinput.get_takeOffCharge(indexCharger, index_takeOffCharge).get_scheduleNodeData().get_startTime()) * charging_speed;
			}
		};

		// Function returns the putOnChargeIndex, and the takeOffCharge Index of the charging session:
		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Charger& charger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge = 0) const = 0;

		// Function returns the putOnChargeIndex, and the takeOffCharge Index of the charging session:
		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Types::Index& indexCharger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge = 0) const = 0;
	};

	class FixAtEndChargingStrategy : public ChargingStrategy
	{
		const OptimisationInput& _optinput;

	public:

		FixAtEndChargingStrategy(
			const OptimisationInput& optinput
		) :
			_optinput(optinput)
		{};

		// OVERRIDE GETTERS:

		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Charger& charger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge) const override;

		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Types::Index& indexCharger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge = 0) const override;
	};

	class VariableAtEndChargingStrategy : public ChargingStrategy
	{
		const OptimisationInput& _optinput;

	public:

		VariableAtEndChargingStrategy(
			const OptimisationInput& optinput
		) :
			_optinput(optinput)
		{};

		// OVERRIDE GETTERS:

		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Charger& charger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge = 0) const override;

		virtual const ChargingStrategy::Session get_chargingSession(
			const Types::DateTime& lb,
			const Types::DateTime& ub,
			const Vehicle& vehicle,
			const Types::Index& indexCharger,
			const Types::BatteryCharge& prev_soc,
			const Types::BatteryCharge& fixed_discharge = 0) const override;
	};

};



#endif // !EVA_CHARGING_STRATEGIES_H



