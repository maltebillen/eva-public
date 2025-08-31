#ifndef EVA_MASTER_PROBLEM_H
#define EVA_MASTER_PROBLEM_H

#include "evaConstants.h"
#include "incl/moderator/OptimisationInput.h"
#include "incl/moderator/moderator.h"

#include "Highs.h"

#include <memory>
#include <deque>

namespace eva
{
	template <typename T>
	class Variable {
		HighsInt _var;
		std::shared_ptr<T> _ptr;
		HighsBasisStatus _basisStatus = HighsBasisStatus::kNonbasic;

	public:
		bool flagDelete = false;

		Variable() {};

		Variable(
			const HighsInt& var,
			const T& ref
		) :
			_var(var),
			_ptr(std::make_shared<T>(ref))
		{};

		inline void set_var(const HighsInt& var) { _var = var; };
		inline void set_basisStatus(const HighsBasisStatus& basisStatus) { _basisStatus = basisStatus; };

		inline const HighsInt& get_var() const { return _var; };
		inline const HighsBasisStatus& get_basisStatus() const { return _basisStatus; };
		inline const std::shared_ptr<T>& get_ptr() const { return _ptr; };
	};

	template <typename T>
	class Constraint {
		HighsInt _constr = -1;
		std::shared_ptr<T> _ptr;
		double _lb = Constants::BIG_DOUBLE; 
		double _ub = Constants::BIG_DOUBLE;

	public:
		Constraint() {};

		Constraint(
			const T& ref,
			const double& lb,
			const double& ub
		) :
			_ptr(std::make_shared<T>(ref)),
			_lb(lb),
			_ub(ub)
		{};

		Constraint(
			const HighsInt& constr,
			const T& ref,
			const double& lb,
			const double& ub
		) :
			_constr(constr),
			_ptr(std::make_shared<T>(ref)),
			_lb(lb),
			_ub(ub)
		{};

		inline void set_constr(const HighsInt& highs_constr) { _constr = highs_constr;};

		inline const bool is_in_RMP() const { return _constr != -1; };
		inline const HighsInt& get_constr() const { return _constr; };
		inline const std::shared_ptr<T>& get_ptr() const { return _ptr; };
		inline const double& get_lb() const { return _lb;};
		inline const double& get_ub() const { return _ub;};
	};

	enum class MasterProblemSolutionStatus : uint8_t 
	{
		MP_FRACTIONAL = 0,
		MP_INTEGER,
		MP_INVALID,
		MP_INFEASIBLE,
		MP_UNDEFINED
	};

	struct StatusVarSchedulesAdded
	{
		double lb;
		uint32_t columnsAdded;
	};

	class MasterProblem
	{
		const OptimisationInput& _optinput;

		Highs _model;
		HighsSolution _currentHighsSolution;

		Duals _currentDuals;
		MasterProblemSolutionStatus _currentSolutionStatus;

		std::pair<Variable<Types::Index>, Variable<Types::Index>> _varsTotalVehiclesSlacks; // first: pos. +, second: neg. -
		std::pair<Variable<Types::Index>, Variable<Types::Index>> _varsTotalTripsUnassignedSlacks; // first: pos. +, second: neg. -
		std::vector<Variable<SubScheduleTripNodeData>> _vecVarUnallocatedTrips;
		std::vector<Variable<Vehicle>> _vecVarVehicleSelected;
		std::vector<std::vector<Variable<SubVehicleSchedule>>> _vecVarVehicleSchedules;
		std::deque<SubVehicleSchedule> _vecPoolVehicleSchedule;

		Constraint<Types::Index> _constrTotalNumberVehicles;
		Constraint<Types::Index> _constrTotalNumberTripsUnassigned;
		std::vector<Constraint<Vehicle>> _vecConstrOneSchedulePerVehicle;
		std::vector<Constraint<SubScheduleTripNodeData>> _vecConstrTripCoverage;
		std::vector<Constraint<SubScheduleMaintenanceNodeData>> _vecConstrOneVehiclePerMaintenance;
		std::vector<std::vector<Constraint<SubSchedulePutOnChargeNodeData>>> _vecConstrChargerCapacity;
		
		HighsInt _addColumn(const double& cost, const double& lb, const double& ub, const HighsInt& number_nz_coeff, const HighsInt* indices, const double* values);
		HighsInt _addRow(const double& lb, const double& ub, const HighsInt& number_nz_coeff, const HighsInt* indices, const double* values);
		HighsInt _addRow(const double& lb, const double& ub);
		HighsInt _addRow(const double& val);

		int64_t _mseconds_runtimeSolver = 0;
		int64_t _mseconds_filterVars = 0;
		HighsInt _VAR_SCHEDULES_START = 0;

		void _addVars();
		void _addConstrs();

		void _addConstrTotalNumberVehicles();
		void _addConstrTotalNumberTripsUnassigned();
		void _addConstrOneRoutingPerVehicle();
		void _addConstrTripCoverage();
		void _addConstrOneVehiclePerMaintenance();
		void _addTempConstrChargerCapacity();
		
		void _addVarTotalVehiclesSlack();
		void _addVarTotalNumberTripsUnassignedSlack();
		void _addVarsUnallocatedTrips();
		void _addVarsVehicleSelected();

		void _initialise();

		void _updateCurrentDuals();
		void _updateSolutionStatus();
		void _clean_up(const uint32_t& numberDelete);

		const double& _getDual(const HighsInt& constr);

		std::vector<Branch> _get_vecBranchOptionsTotalVehicles();
		std::vector<Branch> _get_vecBranchOptionsTotalNumberTripsUnassigned();
		std::vector<Branch> _get_vecBranchOptionsUnassignedTrips();
		std::vector<Branch> _get_vecBranchOptionsVehicleRotation();
		std::vector<Branch> _get_vecBranchOptionsVehicleTrip();
		std::vector<Branch> _get_vecBranchOptionsVehicleMaintenance();
		std::vector<Branch> _get_vecBranchOptionsVehicleChargingBefore();
		std::vector<Branch> _get_vecBranchOptionsVehicleChargingAfter();

		inline const Constraint<Types::Index>& _getConstrTotalNumberVehicles() const { return _constrTotalNumberVehicles; };
		inline const Constraint<Types::Index>& _getConstrTotalNumberTripsUnassigned() const { return _constrTotalNumberTripsUnassigned; };
		inline const Constraint<Vehicle>& _getConstrOneSchedulePerVehicle(const Types::Index indexVehicle) const { return _vecConstrOneSchedulePerVehicle[indexVehicle]; };
		inline const Constraint<SubScheduleTripNodeData>& _getConstrTripCoverage(const Types::Index indexTrip) const { return _vecConstrTripCoverage[indexTrip]; };
		inline const Constraint<SubScheduleMaintenanceNodeData>& _getConstrOneVehiclePerMaintenance(const Types::Index indexMaintenance) const { return _vecConstrOneVehiclePerMaintenance[indexMaintenance]; };
		inline const Constraint<SubSchedulePutOnChargeNodeData>& _getConstrChargerCapacity(const Types::Index indexCharger, const Types::Index indexTime) const { return _vecConstrChargerCapacity[indexCharger][indexTime]; }
		
	public: 
		MasterProblem() = delete;

		MasterProblem(
			const OptimisationInput& optinput
		) :
			_optinput(optinput),
			_currentDuals(optinput)
		{
			_initialise();
		};

		void filterVars(const BranchNode& brn);
		void writeModel();
		bool solve();
		void solveAsMIP();
		void set_aux_variable_bounds();
		void set_auxiliary_objective();
		void reset_objective();
		void reset_variable_bounds(const std::vector<std::pair<double, double>>& vecBounds);
		bool check_and_update_charger_capacity(const bool add_rmp_rows);

		StatusVarSchedulesAdded addVarsSchedule(std::vector<std::vector<SubVehicleSchedule>>& vecSchedules, const bool include_cost);
		StatusVarSchedulesAdded addPoolVarsSchedule(const Duals& duals, const BranchNode& brn, const bool include_cost);

		void store_schedule_in_pool(const SubVehicleSchedule& schedule);
		
		// GETTERS

		std::vector<Branch> get_vecBranchOptions();
		Solution get_currentSolution();
		const std::vector<std::pair<double,double>> get_vecLookupColumnBounds() const;
		const bool check_aux_variables_feasible(const std::vector<std::pair<double, double>>& vecBounds);
		
		// INLINE GETTERS:
		inline const Duals& get_currentDuals() const { return _currentDuals; };
		inline const double get_currentObjective() const { return _model.getInfo().objective_function_value; };
		inline const HighsBasis& get_currentBasis() const { return _model.getBasis(); };
		inline const MasterProblemSolutionStatus get_currentSolutionStatus() const { return _currentSolutionStatus; };

		inline const int64_t get_totalRuntimeSolver() const { return _mseconds_runtimeSolver; };
		inline const int64_t get_totalRuntimeFilterVars() const { return _mseconds_filterVars; };
		inline const uint32_t get_sizeConstraints() const { return _model.getLp().num_row_; }
		inline const uint32_t get_sizeVariables() const { return _model.getLp().num_col_; }
	};
}

#endif // ! EVA_MASTER_PROBLEM_H