#ifndef EVA_DATA_CONFIG_H
#define EVA_DATA_CONFIG_H

#include "evaConstants.h"

#include <map>

namespace eva
{
    /// <summary>
    ///  The Config class stores all parameters, bounds and cost-coefficients for the optimisation.
    ///  All member variables have been initialised and are stored with an initial value.
    ///  The config-variables can be overwritten by reading a csv file with columns "Parameter Key, Value".
    ///  NOTE: The config-variables are NOT read case-sensitive. All .csv parameter names are converted to lower-case letters!
    /// 
    ///  New parameters must be added in the following way:
    ///  1) Add member variable (with default value).
    ///  2) Add getter-function.
    ///  3) Update private method "initialise()" to override the member variable.
    /// </summary>
    class Config
    {
        enum class DataType : uint16_t {
            UINT,
            DOUBLE,
            DATETIME,
            STRING,
            BOOL
        };

        const std::map<std::string, DataType> MapDataType
        {
            {"uint", DataType::UINT},
            {"double", DataType::DOUBLE},
            {"datetime", DataType::DATETIME},
            {"string", DataType::STRING},
            {"bool", DataType::BOOL}
        };

        // ATTRIBUTES
        // Paths:
        std::string _path_to_data;
        std::string _path_to_config;
        std::string _path_to_output;

        // Date
        Types::DateTime _date_start = Constants::MAX_TIMESTAMP;
        Types::DateTime _date_end = Constants::MAX_TIMESTAMP;

        // Bounds

        // Constants
        double _const_linear_optimality_gap = 0.0001;
        double _const_integer_optimality_gap = 0.001;
        double _const_frac_threshold_trunc_cg = 0.9;

        uint32_t _const_put_vehicle_on_charge = 1 * 5 * 60; //!< A constant duration that must be reserved to put the vehicle on charge.
        uint32_t _const_take_vehicle_off_charge = 1 * 5 * 60; //!< A constant duration that must be reserved to take the vehicle off charge.
        uint32_t _const_planning_horizon_length = 24 * 60 * 60; //!< One day as default.
        uint32_t _const_planning_horizon_overlap = 0;
        uint32_t _const_charger_capacity_check = 300;
        uint32_t _const_nr_threads = 4;
        uint32_t _const_column_generation_timelimit = 900; 
        uint32_t _const_branch_and_price_timelimit = 5400; 
        uint32_t _const_nr_cols_per_vehicle_iter = 40;
        uint32_t _const_nth_iter_solve_all = 10;
        uint32_t _const_nth_branching_node_dive = 10;
        uint32_t _const_max_number_cols_mp = 5000;
        uint32_t _const_max_number_cols_mp_pool = 10000;
        uint32_t _const_max_number_first_tier_eval_strong_branching = 50;
        uint32_t _const_max_number_second_tier_eval_strong_branching = 25;

        uint8_t _const_code_algorithm_type = 4;
        uint8_t _const_code_pricing_problem_type = 2;

        // Bools
        bool _flag_minimise_number_vehicles = true;
        bool _flag_use_model_cleanup = true;
        bool _flag_interim_solve_all_vehicles = true;           
        bool _flag_allow_deadlegs = true;         
        bool _flag_terminate_after_root = false;  

        // Cost Coefficients:
        double _cost_deadleg_fix = 5.0;
        double _cost_deadleg_per_km = 2.0;
        double _cost_coefficient_penalty_maintenance = 0.05;
        double _cost_uncovered_trip = 2000.0;
        double _cost_exceeding_charger_capacity = 20000.0;

        // FUNCTION DEFINITIONS
        void _initialise(
            const std::map<std::string, double>& mapDoubleParams,
            const std::map<std::string, uint32_t>& mapUIntParams,
            const std::map<std::string, Types::DateTime>& mapDateTimeParams,
            const std::map<std::string, std::string>& mapStringParams,
            const std::map<std::string, bool>& mapBoolParams
        );

    public:
        // CONSTRUCTOR
        // @brief Default Contructor
        Config() {};

        // DESTRUCTOR
        // @brief Destructor
        ~Config() {}


        // FUNCTIONS DEFINITIONS
        void read_override(const std::string& fileName);
        void set_path_to_data(const std::string& path);
        void set_path_to_config(const std::string& path);
        void set_path_to_output(const std::string& path);

        // GETTERS
        const Types::AlgorithmType get_const_algorithm_type() const;
        const Types::PricingProblemType get_const_pricing_problem_type() const;

        // INLINE
        // GETTERS
        inline const std::string& get_path_to_data() const { return _path_to_data; };
        inline const std::string& get_path_to_config() const { return _path_to_config; };
        inline const std::string& get_path_to_output() const { return _path_to_output; };

        inline const Types::DateTime& get_date_start() const { return _date_start; };
        inline const Types::DateTime& get_date_end() const { return _date_end; };

        inline const double& get_const_linear_optimality_gap() const { return _const_linear_optimality_gap; };
        inline const double& get_const_integer_optimality_gap() const { return _const_integer_optimality_gap; };
        inline const double& get_const_frac_threshold_trunc_cg() const { return _const_frac_threshold_trunc_cg; };

        inline const uint32_t& get_const_put_vehicle_on_charge() const { return _const_put_vehicle_on_charge; };
        inline const uint32_t& get_const_take_vehicle_off_charge() const { return _const_take_vehicle_off_charge; };

        inline const uint32_t& get_const_planning_horizon_length() const { return _const_planning_horizon_length; };
        inline const uint32_t& get_const_planning_horizon_overlap() const { return _const_planning_horizon_overlap; };
        inline const uint32_t& get_const_charger_capacity_check() const { return _const_charger_capacity_check; };
        inline const uint32_t& get_const_nr_threads() const { return _const_nr_threads; };
        inline const uint32_t& get_const_column_generation_timelimit() const { return _const_column_generation_timelimit; };
        inline const uint32_t& get_const_branch_and_price_timelimit() const { return _const_branch_and_price_timelimit; };
        inline const uint32_t& get_const_nr_cols_per_vehicle_iter() const { return _const_nr_cols_per_vehicle_iter; };
        inline const uint32_t& get_const_nth_iter_solve_all() const { return _const_nth_iter_solve_all; };
        inline const uint32_t& get_const_nth_branching_node_dive() const { return _const_nth_branching_node_dive; };
        inline const uint32_t& get_const_max_number_cols_mp() const { return _const_max_number_cols_mp; };
        inline const uint32_t& get_const_max_number_cols_mp_pool() const { return _const_max_number_cols_mp_pool; };
        inline const uint32_t& get_const_max_number_first_tier_eval_strong_branching() const { return _const_max_number_first_tier_eval_strong_branching; };
        inline const uint32_t& get_const_max_number_second_tier_eval_strong_branching() const { return _const_max_number_second_tier_eval_strong_branching; };

        inline const bool get_flag_minimise_number_vehicles() const { return _flag_minimise_number_vehicles; };        
        inline const bool get_flag_use_model_cleanup() const { return _flag_use_model_cleanup; };
        inline const bool get_flag_interim_solve_all_vehicles() const { return _flag_interim_solve_all_vehicles; };
        inline const bool get_flag_allow_deadlegs() const { return _flag_allow_deadlegs; };
        inline const bool get_flag_terminate_after_root() const { return _flag_terminate_after_root; };

        inline const double& get_cost_deadleg_fix() const { return _cost_deadleg_fix; };
        inline const double& get_cost_deadleg_per_km() const { return _cost_deadleg_per_km; };
        inline const double& get_cost_coefficient_penalty_maintenance() const { return _cost_coefficient_penalty_maintenance; };
        inline const double& get_cost_uncovered_trip() const { return _cost_uncovered_trip; };
        inline const double& get_cost_exceeding_charger_capacity() const { return _cost_exceeding_charger_capacity; };
    };
};




#endif // !EVA_DATA_CONFIG_H
