#ifndef EVA_VEHICLE_H
#define EVA_VEHICLE_H

#include "evaConstants.h"
#include "charger.h"

#include <vector>
#include <cmath>


namespace eva
{
	class Vehicle
	{
	private:
		// ATTRIBUTES

		Types::Index _index;
		uint32_t _id;
		Types::BatteryCharge _batteryMinKWh;
		Types::BatteryCharge _batteryMaxKWh;
		const Charger& _initialCharger;
		Types::DateTime _initialStartTime;
		Types::BatteryCharge _initialSOC;
		Types::BatteryCharge _chargingSpeedVolts;
		Types::BatteryCharge _chargingSpeedAmps;
		std::string _numberPlate;
		uint32_t _odometerReading;
		uint32_t _odometerLastMaintenance;
		bool _inRotation;
		double _cost;
		Types::BatteryCharge _kwh_per_thousand_km; // Only precise to three decimal points.

		// FUNCTION DEFINITIONS

	public:
		// CONSTRUCTORS

		// @brief Default Contructor
		Vehicle() = delete;

		// @brief Single-argument constructor
		Vehicle(
			const Types::Index& idx,
			const uint32_t& id,
			const Types::BatteryCharge& batteryMinKWh,
			const Types::BatteryCharge& batteryMaxKWh,
			const Charger& initialCharger,
			const Types::DateTime& initialStartTime,
			const Types::BatteryCharge& initialSOC,
			const Types::BatteryCharge& chargingSpeedVolts,
			const Types::BatteryCharge& chargingSpeedAmps,
			const std::string& numberPlate,
			const uint32_t& odometerReading,
			const uint32_t& odometerLastMaintenance,
			const bool inRotation,
			const double& cost,
			const Types::BatteryCharge& kwh_per_thousand_km
		) :
			_index(idx),
			_id(id),
			_batteryMinKWh(batteryMinKWh),
			_batteryMaxKWh(batteryMaxKWh),
			_initialCharger(initialCharger),
			_initialStartTime(initialStartTime),
			_initialSOC(initialSOC),
			_chargingSpeedVolts(chargingSpeedVolts),
			_chargingSpeedAmps(chargingSpeedAmps),
			_numberPlate(numberPlate),
			_odometerReading(odometerReading),
			_odometerLastMaintenance(odometerLastMaintenance),
			_inRotation(inRotation),
			_cost(cost),
			_kwh_per_thousand_km(kwh_per_thousand_km)
		{};

		// DESTRUCTOR

		// FUNCTION DEFINITIONS		

		// INLINE
		// GETTERS

		inline const Types::Index& get_index() const { return _index; };
		inline const uint32_t& get_id() const { return _id; };
		inline const Types::BatteryCharge& get_batteryMinKWh() const { return _batteryMinKWh; };
		inline const Types::BatteryCharge& get_batteryMaxKWh() const { return _batteryMaxKWh; };
		inline const Charger& get_initialCharger() const { return _initialCharger; };
		inline const Types::DateTime& get_initialStartTime() const { return _initialStartTime; };
		inline const Types::BatteryCharge& get_initialSOC() const { return _initialSOC; };
		inline const Types::BatteryCharge& get_chargingSpeedVolts() const { return _chargingSpeedVolts; };
		inline const Types::BatteryCharge& get_chargingSpeedAmps() const { return _chargingSpeedAmps; };
		inline const std::string& get_numberPlate() const { return _numberPlate; };
		inline const uint32_t& get_odometerReading() const { return _odometerReading; };
		inline const uint32_t& get_odometerLastMaintenance() const { return _odometerLastMaintenance; };
		inline const bool get_inRotation() const { return _inRotation; };
		inline const double& get_cost() const { return _cost; };

		inline const uint32_t get_distanceRange() const { return ((_batteryMaxKWh - _batteryMinKWh) * 1000) / _kwh_per_thousand_km; };
		inline const Types::BatteryCharge get_batteryRange() const { return _batteryMaxKWh - _batteryMinKWh; };
		inline const Types::BatteryCharge get_batteryDischarge(const uint32_t& distance) const { return (distance * _kwh_per_thousand_km) / 1000; };
		inline const double get_chargingSpeedKwS(const Charger& charger) const { return Helper::compare_floats_smaller_equal(this->get_chargingSpeedVolts(), charger.get_chargingSpeedVolts()) ? (this->get_chargingSpeedVolts() * std::min(this->get_chargingSpeedAmps(), charger.get_chargingSpeedAmps())) / 3600000.00 : 0.0; };
	};
}

#endif // EVA_VEHICLE_H