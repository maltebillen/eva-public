#ifndef EVA_DATA_CHARGER_H
#define EVA_DATA_CHARGER_H

#include "evaConstants.h"
#include "config.h"
#include "location.h"

namespace eva
{
	class Charger
	{
		// ATTRIBUTES:
		Types::Index _index;
		uint32_t _id;
		const Location& _location;
		uint32_t _capacity;
		Types::BatteryCharge _chargingSpeedVolts;
		Types::BatteryCharge _chargingSpeedAmps;

	public:		
		// CONSTRUCTORS:

		// @brief Default Constructor
		Charger() = delete;

		// @brief Single-argument constructor:
		Charger(
			const Types::Index& idx,
			const uint32_t& id,
			const Location& location,
			const uint32_t& capacity,
			const Types::BatteryCharge& chargingSpeedVolts,
			const Types::BatteryCharge& chargingSpeedAmps
		) :
			_index(idx),
			_id(id),
			_location(location),
			_capacity(capacity),
			_chargingSpeedVolts(chargingSpeedVolts),
			_chargingSpeedAmps(chargingSpeedAmps)
		{};

		// GETTERS:
		const Types::Index& get_index() const { return _index; };
		const uint32_t& get_id() const { return _id; };
		const Location& get_location() const { return _location; };
		const uint32_t& get_capacity() const { return _capacity; };
		const Types::BatteryCharge& get_chargingSpeedVolts() const { return _chargingSpeedVolts; };
		const Types::BatteryCharge& get_chargingSpeedAmps() const { return _chargingSpeedAmps; };
	};
}

#endif /* EVA_DATA_CHARGER_H */
