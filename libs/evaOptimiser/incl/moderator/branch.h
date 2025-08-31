#ifndef EVA_MODERATOR_BRANCH_H
#define EVA_MODERATOR_BRANCH_H

#include "evaConstants.h"
#include "evaDataHandler.h"
#include "SubScheduleNodes.h"
#include <memory>

namespace eva
{
	// Important: The branch type value also provides the ordering of the types.
	// i.e. branching on the vehicle rotation is always preferred over a vehicle/trip branch.
	// when sorting branching options this order is considered.
	enum class BranchType : uint8_t
	{
		VEHICLE_ROTATION = 1,
		VEHICLE_TRIP = 2,
		VEHICLE_MAINTENANCE = 3,
		VEHICLE_CHARGING_BEFORE = 4,
		VEHICLE_CHARGING_AFTER = 5,
		TOTAL_VEHICLES = 6,
		TOTAL_TRIPS_UNASSIGNED = 7,
		TRIP_UNASSIGNED = 8,
		UNDEFINED
	};


	const std::map<BranchType, const char*> BranchTypeMap
	{
		{BranchType::VEHICLE_ROTATION, "VEHICLE_ROTATION"},
		{BranchType::VEHICLE_TRIP, "VEHICLE_TRIP"},
		{BranchType::VEHICLE_MAINTENANCE, "VEHICLE_MAINTENANCE"},
		{BranchType::VEHICLE_CHARGING_BEFORE, "VEHICLE_CHARGING_BEFORE"},
		{BranchType::VEHICLE_CHARGING_AFTER, "VEHICLE_CHARGING_AFTER"},
		{BranchType::TOTAL_VEHICLES, "TOTAL_VEHICLES"},
		{BranchType::TOTAL_TRIPS_UNASSIGNED, "TOTAL_TRIPS_UNASSIGNED"},
		{BranchType::TRIP_UNASSIGNED, "TRIP_UNASSIGNED"},
		{BranchType::UNDEFINED, "UNDEFINED"}
	};

	enum class BranchPriority : uint8_t
	{
		FIRST = 1,
		SECOND = 2,
		THIRD = 3,
		UNDEFINED = 100
	};

	class BranchBaseData
	{
	public:
		// INLINE FUNCTIONS 
		inline std::unique_ptr<BranchBaseData> clone() const { return std::unique_ptr<BranchBaseData>(deepClone()); };

		virtual ~BranchBaseData() {};

	protected:
		virtual inline BranchBaseData* deepClone() const = 0;
	};

	struct BranchTotalVehicles : public BranchBaseData
	{
		// ATTRIBUTES
		BranchTotalVehicles() {};

		~BranchTotalVehicles() {};

		// GETTERS

	protected:
		virtual BranchTotalVehicles* deepClone() const override { return new BranchTotalVehicles(*this); };
	};

	struct BranchTotalTripsUnassigned : public BranchBaseData
	{
		// ATTRIBUTES
		BranchTotalTripsUnassigned() {};

		~BranchTotalTripsUnassigned() {};

		// GETTERS

	protected:
		virtual BranchTotalTripsUnassigned* deepClone() const override { return new BranchTotalTripsUnassigned(*this); };
	};

	class BranchVehicleRotation : public BranchBaseData
	{
		// ATTRIBUTES

		const Vehicle& _vehicle;

	public:
		// CONSTRUCTORS

		BranchVehicleRotation(
			const Vehicle& vehicle
		) :
			_vehicle(vehicle)
		{}

		~BranchVehicleRotation() {};

		// GETTERS

		inline const Vehicle& get_vehicle() const { return _vehicle; };

	protected:
		virtual BranchVehicleRotation* deepClone() const override { return new BranchVehicleRotation(*this); };
	};

	class BranchVehicleTrip : public BranchBaseData
	{
		// ATTRIBUTES

		const Vehicle& _vehicle;
		const SubScheduleTripNodeData& _subTripNodeData;

	public:
		// CONSTRUCTORS

		BranchVehicleTrip(
			const Vehicle& vehicle,
			const SubScheduleTripNodeData& subTripNodeData
		) :
			_vehicle(vehicle),
			_subTripNodeData(subTripNodeData)
		{}

		~BranchVehicleTrip() {};

		// GETTERS

		inline const Vehicle& get_vehicle() const { return _vehicle; };
		inline const SubScheduleTripNodeData& get_subTripNodeData() const { return _subTripNodeData; };

	protected:
		virtual BranchVehicleTrip* deepClone() const override { return new BranchVehicleTrip(*this); };
	};

	class BranchTripUnassigned : public BranchBaseData
	{
		// ATTRIBUTES

		const SubScheduleTripNodeData& _subTripNodeData;

	public:
		// CONSTRUCTORS

		BranchTripUnassigned(
			const SubScheduleTripNodeData& subTripNodeData
		) :
			_subTripNodeData(subTripNodeData)
		{
		}

		~BranchTripUnassigned() {};

		// GETTERS

		inline const SubScheduleTripNodeData& get_subTripNodeData() const { return _subTripNodeData; };

	protected:
		virtual BranchTripUnassigned* deepClone() const override { return new BranchTripUnassigned(*this); };
	};

	class BranchVehicleMaintenance : public BranchBaseData
	{
		// ATTRIBUTES

		const Vehicle& _vehicle;
		const SubScheduleMaintenanceNodeData& _subMaintenanceNodeData;

	public:
		// CONSTRUCTORS

		BranchVehicleMaintenance(
			const Vehicle& vehicle,
			const SubScheduleMaintenanceNodeData& subMaintenanceNodeData
		) :
			_vehicle(vehicle),
			_subMaintenanceNodeData(subMaintenanceNodeData)
		{}

		~BranchVehicleMaintenance() {};

		// GETTERS

		inline const Vehicle& get_vehicle() const { return _vehicle; };
		inline const SubScheduleMaintenanceNodeData& get_subMaintenanceNodeData() const { return _subMaintenanceNodeData; };

	protected:
		virtual BranchVehicleMaintenance* deepClone() const override { return new BranchVehicleMaintenance(*this); };
	};

	class BranchVehicleChargingBefore: public BranchBaseData
	{
		// ATTRIBUTES

		const Vehicle& _vehicle;
		const Charger& _charger;
		Types::Index _indexToScheduleNode;

	public:
		// CONSTRUCTORS

		BranchVehicleChargingBefore(
			const Vehicle& vehicle,
			const Charger& charger,
			const Types::Index& indexToScheduleNodeData
		) :
			_vehicle(vehicle),
			_charger(charger),
			_indexToScheduleNode(indexToScheduleNodeData)
		{}

		~BranchVehicleChargingBefore() {};
			
		// GETTERS

		inline const Vehicle& get_vehicle() const { return _vehicle; };
		inline const Charger& get_charger() const { return _charger; };
		inline const Types::Index& get_indexToScheduleNode() const { return _indexToScheduleNode; };

	protected:
		virtual BranchVehicleChargingBefore* deepClone() const override { return new BranchVehicleChargingBefore(*this); };
	};

	class BranchVehicleChargingAfter: public BranchBaseData
	{
		// ATTRIBUTES

		const Vehicle& _vehicle;
		const Charger& _charger;
		Types::Index _indexFromScheduleNode;

	public:
		// CONSTRUCTORS

		BranchVehicleChargingAfter(
			const Vehicle& vehicle,
			const Charger& charger,
			const Types::Index& indexFromScheduleNode
		) :
			_vehicle(vehicle),
			_charger(charger),
			_indexFromScheduleNode(indexFromScheduleNode)
		{}

		~BranchVehicleChargingAfter() {};
			
		// GETTERS

		inline const Vehicle& get_vehicle() const { return _vehicle; };
		inline const Charger& get_charger() const { return _charger; };
		inline const Types::Index& get_indexFromScheduleNode() const { return _indexFromScheduleNode; };

	protected:
		virtual BranchVehicleChargingAfter* deepClone() const override { return new BranchVehicleChargingAfter(*this); };
	};

	class Branch
	{
		std::unique_ptr<BranchBaseData> _ptrBranchBaseData;
		double _fractionalValue;
		double _branchValue;
		BranchType _type;
		BranchPriority _priority;

		double _strong_branching_score;

	public:

		// CONSTRUCTORS

		Branch() :
			_fractionalValue(0.0),
			_branchValue(0.0),
			_type(BranchType::UNDEFINED),
			_priority(BranchPriority::UNDEFINED)
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchTotalVehicles& btv
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::TOTAL_VEHICLES),
			_priority(BranchPriority::FIRST),
			_ptrBranchBaseData(std::make_unique<BranchTotalVehicles>(btv))
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchTotalTripsUnassigned& btu
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::TOTAL_TRIPS_UNASSIGNED),
			_priority(BranchPriority::FIRST),
			_ptrBranchBaseData(std::make_unique<BranchTotalTripsUnassigned>(btu))
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchTripUnassigned& bvc
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::TRIP_UNASSIGNED),
			_priority(BranchPriority::SECOND),
			_ptrBranchBaseData(std::make_unique<BranchTripUnassigned>(bvc))
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchVehicleRotation& bvr
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::VEHICLE_ROTATION),
			_priority(BranchPriority::SECOND),
			_ptrBranchBaseData(std::make_unique<BranchVehicleRotation>(bvr))
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchVehicleChargingBefore & bvc
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::VEHICLE_CHARGING_BEFORE),
			_priority(BranchPriority::THIRD),
			_ptrBranchBaseData(std::make_unique<BranchVehicleChargingBefore>(bvc))
		{
		};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchVehicleChargingAfter & bvc
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::VEHICLE_CHARGING_AFTER),
			_priority(BranchPriority::THIRD),
			_ptrBranchBaseData(std::make_unique<BranchVehicleChargingAfter>(bvc))
		{
		};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchVehicleMaintenance& bvm
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::VEHICLE_MAINTENANCE),
			_priority(BranchPriority::THIRD),
			_ptrBranchBaseData(std::make_unique<BranchVehicleMaintenance>(bvm))
		{};

		Branch(
			const double& branchValue,
			const double& fractionalValue,
			const BranchVehicleTrip& bvt
		) :
			_fractionalValue(fractionalValue),
			_branchValue(branchValue),
			_type(BranchType::VEHICLE_TRIP),
			_priority(BranchPriority::THIRD),
			_ptrBranchBaseData(std::make_unique<BranchVehicleTrip>(bvt))
		{};

		// COPY CONSTRUCTORS:

		Branch(Branch const& other) :
			_fractionalValue(other._fractionalValue),
			_branchValue(other._branchValue),
			_type(other._type),
			_priority(other._priority),
			_ptrBranchBaseData(other._ptrBranchBaseData->clone()),
			_strong_branching_score(other._strong_branching_score)
		{}
		Branch(Branch&& other) = default;
		Branch& operator=(Branch const& other) {
			_ptrBranchBaseData = other._ptrBranchBaseData->clone();
			_fractionalValue = other._fractionalValue;
			_branchValue = other._branchValue;
			_type = other._type;
			_priority = other._priority;
			_strong_branching_score = other._strong_branching_score;
			return *this;
		}
		Branch& operator=(Branch&& other) = default;

		// DESTRUCTOR:

		~Branch() = default;

		inline void set_branchValue(const double& val) { _branchValue = val; };
		inline void set_strong_branching_score(const double& val) { _strong_branching_score = val; };
		inline const double& get_strong_branching_score() const { return _strong_branching_score;};

		inline const double get_fractionalValue() const { return _fractionalValue; };
		inline const double get_branchValue() const { return _branchValue; };
		inline const bool get_branchValueBool() const { return Helper::compare_floats_smaller(0.0, _branchValue); };
		inline const BranchType get_type() const { return _type; }
		inline const BranchPriority get_priority() const { return _priority; }
		inline const char* get_branchTypeName() const { return BranchTypeMap.find(_type)->second; };

		inline const BranchTotalVehicles* castBranchTotalVehicles() const { return static_cast<const BranchTotalVehicles*>(_ptrBranchBaseData.get()); };
		inline const BranchTotalTripsUnassigned* castBranchTotalTripsUnassigned() const { return static_cast<const BranchTotalTripsUnassigned*>(_ptrBranchBaseData.get()); };
		inline const BranchVehicleRotation* castBranchVehicleRotation() const { return static_cast<const BranchVehicleRotation*>(_ptrBranchBaseData.get()); };
		inline const BranchVehicleTrip* castBranchVehicleTrip() const { return static_cast<const BranchVehicleTrip*>(_ptrBranchBaseData.get()); };
		inline const BranchVehicleMaintenance* castBranchVehicleMaintenance() const { return static_cast<const BranchVehicleMaintenance*>(_ptrBranchBaseData.get()); };
		inline const BranchVehicleChargingBefore* castBranchVehicleChargingBefore() const { return static_cast<const BranchVehicleChargingBefore*>(_ptrBranchBaseData.get()); };
		inline const BranchVehicleChargingAfter* castBranchVehicleChargingAfter() const { return static_cast<const BranchVehicleChargingAfter*>(_ptrBranchBaseData.get()); };
		inline const BranchTripUnassigned* castBranchTripUnassigned() const { return static_cast<const BranchTripUnassigned*>(_ptrBranchBaseData.get()); };
		
		static bool compareMostFractional(const Branch& l, const Branch& r) {
			if (l.get_priority() == r.get_priority()) return Helper::compare_floats_smaller(std::abs(0.5 - (r.get_fractionalValue() - std::floor(r.get_fractionalValue()))), std::abs(0.5 - (l.get_fractionalValue() - std::floor(l.get_fractionalValue()))));
			else return l.get_priority() > r.get_priority();
		};

		static bool compareLeastFractional(const Branch& l, const Branch& r) {
			if (l.get_priority() == r.get_priority()) return Helper::compare_floats_smaller(std::abs(0.5 - (l.get_fractionalValue() - std::floor(l.get_fractionalValue()))), std::abs(0.5 - (r.get_fractionalValue() - std::floor(r.get_fractionalValue()))));
			else return l.get_priority() > r.get_priority();
		};

		static bool compareAscending(const Branch& l, const Branch& r) {
			if (l.get_priority() == r.get_priority()) return Helper::compare_floats_smaller((l.get_fractionalValue() - std::floor(l.get_fractionalValue())), (r.get_fractionalValue() - std::floor(r.get_fractionalValue())));
			else return l.get_priority() > r.get_priority();
		};
	};

	struct CompareStrongBranchScore {
		bool operator()(const Branch& l, const Branch& r) const {
			return Helper::compare_floats_smaller(l.get_strong_branching_score(), r.get_strong_branching_score()); // min-heap: lower score = lower priority
    }
};

}
#endif // !EVA_MODERATOR_BRANCH_H
