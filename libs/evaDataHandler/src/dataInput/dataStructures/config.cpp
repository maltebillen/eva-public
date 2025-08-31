#include "incl/dataInput/dataStructures/config.h"

using namespace eva;

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "evaExceptions.h"

void eva::Config::_initialise(const std::map<std::string, double>& mapDoubleParams, const std::map<std::string, uint32_t>& mapUIntParams, const std::map<std::string, Types::DateTime>& mapDateTimeParams, const std::map<std::string, std::string>& mapStringParams, const std::map<std::string, bool>& mapBoolParams)
{
    try {
        this->_const_planning_horizon_length = mapUIntParams.at("const_planning_horizon_length");
        this->_const_planning_horizon_overlap = mapUIntParams.at("const_planning_horizon_overlap");

        // Only read if the start time was not yet set:
        this->_date_start = mapDateTimeParams.at("date_start");
        this->_date_end = mapDateTimeParams.at("date_end");
    }
    catch (...)
    {
        // Crucial Config Parameter not defined. 
        throw DataError("eva::Config::_initialise", "Config::initialise(): Critical Config Parameters are not defined in the config.csv file.");
    }

    // Optional Parameters:
    this->_const_linear_optimality_gap = mapDoubleParams.find("const_linear_optimality_gap") != mapDoubleParams.end() ? mapDoubleParams.at("const_linear_optimality_gap") : this->_const_linear_optimality_gap;
    this->_const_integer_optimality_gap = mapDoubleParams.find("const_integer_optimality_gap") != mapDoubleParams.end() ? mapDoubleParams.at("const_integer_optimality_gap") : this->_const_integer_optimality_gap;
    this->_const_frac_threshold_trunc_cg = mapDoubleParams.find("const_frac_threshold_trunc_cg") != mapDoubleParams.end() ? mapDoubleParams.at("const_frac_threshold_trunc_cg") : this->_const_frac_threshold_trunc_cg;
    
    this->_const_put_vehicle_on_charge = mapUIntParams.find("const_put_vehicle_on_charge") != mapUIntParams.end() ? mapUIntParams.at("const_put_vehicle_on_charge") : this->_const_put_vehicle_on_charge;
    this->_const_take_vehicle_off_charge = mapUIntParams.find("const_take_vehicle_off_charge") != mapUIntParams.end() ? mapUIntParams.at("const_take_vehicle_off_charge") : this->_const_take_vehicle_off_charge;
    this->_const_charger_capacity_check = mapUIntParams.find("const_charger_capacity_check") != mapUIntParams.end() ? mapUIntParams.at("const_charger_capacity_check") : this->_const_charger_capacity_check;
    this->_const_nr_threads = mapUIntParams.find("const_nr_threads") != mapUIntParams.end() ? mapUIntParams.at("const_nr_threads") : this->_const_nr_threads;
    this->_const_column_generation_timelimit = mapUIntParams.find("const_column_generation_timelimit") != mapUIntParams.end() ? mapUIntParams.at("const_column_generation_timelimit") : this->_const_column_generation_timelimit;
    this->_const_branch_and_price_timelimit = mapUIntParams.find("const_branch_and_price_timelimit") != mapUIntParams.end() ? mapUIntParams.at("const_branch_and_price_timelimit") : this->_const_branch_and_price_timelimit;
    this->_const_code_algorithm_type = mapUIntParams.find("const_code_algorithm_type") != mapUIntParams.end() ? mapUIntParams.at("const_code_algorithm_type") : this->_const_code_algorithm_type;
    this->_const_code_pricing_problem_type = mapUIntParams.find("const_code_pricing_problem_type") != mapUIntParams.end() ? mapUIntParams.at("const_code_pricing_problem_type") : this->_const_code_pricing_problem_type;
    this->_const_nr_cols_per_vehicle_iter = mapUIntParams.find("const_nr_cols_per_vehicle_iter") != mapUIntParams.end() ? mapUIntParams.at("const_nr_cols_per_vehicle_iter") : this->_const_nr_cols_per_vehicle_iter;
    this->_const_max_number_cols_mp = mapUIntParams.find("const_max_number_cols_mp") != mapUIntParams.end() ? mapUIntParams.at("const_max_number_cols_mp") : this->_const_max_number_cols_mp;
    this->_const_max_number_cols_mp_pool = mapUIntParams.find("const_max_number_cols_mp_pool") != mapUIntParams.end() ? mapUIntParams.at("const_max_number_cols_mp_pool") : this->_const_max_number_cols_mp_pool;
    this->_const_max_number_first_tier_eval_strong_branching = mapUIntParams.find("const_max_number_first_tier_eval_strong_branching") != mapUIntParams.end() ? mapUIntParams.at("const_max_number_first_tier_eval_strong_branching") : this->_const_max_number_second_tier_eval_strong_branching;
    this->_const_max_number_second_tier_eval_strong_branching = mapUIntParams.find("const_max_number_second_tier_eval_strong_branching") != mapUIntParams.end() ? mapUIntParams.at("const_max_number_second_tier_eval_strong_branching") : this->_const_max_number_second_tier_eval_strong_branching;
    this->_const_nth_iter_solve_all = mapUIntParams.find("const_nth_iter_solve_all") != mapUIntParams.end() ? mapUIntParams.at("const_nth_iter_solve_all") : this->_const_nth_iter_solve_all;
    this->_const_nth_branching_node_dive = mapUIntParams.find("const_nth_branching_node_dive") != mapUIntParams.end() ? mapUIntParams.at("const_nth_branching_node_dive") : this->_const_nth_branching_node_dive;
    
    this->_flag_minimise_number_vehicles = mapBoolParams.find("flag_minimise_number_vehicles") != mapBoolParams.end() ? mapBoolParams.at("flag_minimise_number_vehicles") : this->_flag_minimise_number_vehicles;
    this->_flag_interim_solve_all_vehicles = mapBoolParams.find("flag_interim_solve_all_vehicles") != mapBoolParams.end() ? mapBoolParams.at("flag_interim_solve_all_vehicles") : this->_flag_interim_solve_all_vehicles;
    this->_flag_use_model_cleanup = mapBoolParams.find("flag_use_model_cleanup") != mapBoolParams.end() ? mapBoolParams.at("flag_use_model_cleanup") : this->_flag_use_model_cleanup;
    this->_flag_allow_deadlegs = mapBoolParams.find("flag_allow_deadlegs") != mapBoolParams.end() ? mapBoolParams.at("flag_allow_deadlegs") : this->_flag_allow_deadlegs;
    this->_flag_terminate_after_root = mapBoolParams.find("flag_terminate_after_root") != mapBoolParams.end() ? mapBoolParams.at("flag_terminate_after_root") : this->_flag_terminate_after_root;

    this->_cost_deadleg_fix = mapDoubleParams.find("cost_deadleg_fix") != mapDoubleParams.end() ? mapDoubleParams.at("cost_deadleg_fix") : this->_cost_deadleg_fix;
    this->_cost_deadleg_per_km = mapDoubleParams.find("cost_deadleg_per_km") != mapDoubleParams.end() ? mapDoubleParams.at("cost_deadleg_per_km") : this->_cost_deadleg_per_km;
    this->_cost_uncovered_trip = mapDoubleParams.find("cost_uncovered_trip") != mapDoubleParams.end() ? mapDoubleParams.at("cost_uncovered_trip") : this->_cost_uncovered_trip;
    this->_cost_coefficient_penalty_maintenance = mapDoubleParams.find("cost_coefficient_penalty_maintenance") != mapDoubleParams.end() ? mapDoubleParams.at("cost_coefficient_penalty_maintenance") : this->_cost_coefficient_penalty_maintenance;
    this->_cost_exceeding_charger_capacity = mapDoubleParams.find("cost_exceeding_charger_capacity") != mapDoubleParams.end() ? mapDoubleParams.at("cost_exceeding_charger_capacity") : this->_cost_exceeding_charger_capacity;
}

void eva::Config::read_override(const std::string& fileName)
{
    // Open the file and check for success
    // ----------------------------------------------------------------------------------------------

    std::ifstream in(fileName.c_str(), std::ios::in);
    if (!in)
        throw FileError("eva::Config::read_override","Config::read_override(): File " + fileName + " does not exist!!!");

    // Definitions
    bool first_line(true);
    std::string str, line, err("Parameter Name \"");

    std::string paramKey;
    DataType type;
    std::map<std::string, double> mapDoubleParams;
    std::map<std::string, uint32_t> mapUIntParams;
    std::map<std::string, Types::DateTime> mapDateTimeParams;
    std::map<std::string, std::string> mapStringParams;
    std::map<std::string, bool> mapBoolParams;

    // ----------------------------------------------------------------------------------------------
    // Now, go over all lines in the file and read the config parameters.
    // ----------------------------------------------------------------------------------------------
    while (std::getline(in, line)) {

        // Remove carriage return symbol:
        auto pos = line.find_first_of("\r\n");
        if (pos != std::string::npos)
            line.erase(pos);

        // Stop if we have reached the end of file
        if (in.eof())
            break;

        // If this is the first line, then we may have a "sep=," command.
        // If so, then skip the line.
        if (first_line) {
            first_line = false;
            if (line.find("sep") != std::string::npos) {
                continue;
            }
        }

        // Check if this is the header. If so, then skip the line.
        if (line.find("Value") != std::string::npos)
            continue;

        // Check for comment
        if (line.find_first_of('#') != std::string::npos) {
            first_line = false; continue;
        }

        // Convert the line into a stringstream for easier access
        std::stringstream lineStream(line);


        // Extract the Parameter Name:
        // --------------------------------------------------------------------------------------------
        std::getline(lineStream, str, ',');
        if (str.length() < 1)
            throw DataError("", err + line + "\" has an empty parameter name!\n");
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        paramKey = str;

        // Extract the Parameter Datatype:
        std::getline(lineStream, str, ',');
        if (str.length() < 1)
            throw DataError("eva::Config::read_override",err + line + "\" has an empty parameter name!\n");
        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        if (MapDataType.find(str) == MapDataType.end())
            throw DataError("eva::Config::read_override", err + line + "\" has a undefined datatype!\n");

        type = MapDataType.find(str)->second;

        // Extract the Parameter Value:
        // --------------------------------------------------------------------------------------------
        std::getline(lineStream, str, ',');
        if (str.length() < 1)
            throw DataError("eva::Config::read_override", err + line + "\" has an empty parameter value!\n");

        switch (type) {
        case DataType::DOUBLE:
            mapDoubleParams.insert(std::pair<std::string, double>(paramKey, std::stof(str)));
            break;
        case DataType::UINT:
            mapUIntParams.insert(std::pair<std::string, uint32_t>(paramKey, std::stoi(str)));
            break;
        case DataType::DATETIME:
            mapDateTimeParams.insert(std::pair<std::string, Types::DateTime>(paramKey, Helper::StringToDateTime(str)));
            break;
        case DataType::STRING:
            mapStringParams.insert(std::pair<std::string, std::string>(paramKey, str));
            break;
        case DataType::BOOL:
            mapBoolParams.insert(std::pair<std::string, bool>(paramKey, Helper::stringToBoolean(str)));
            break;
        default:
            continue;
        }
    };


    // Finally, override the config memeber variables:
    this->_initialise(mapDoubleParams, mapUIntParams, mapDateTimeParams, mapStringParams, mapBoolParams);

    in.close();
}

void eva::Config::set_path_to_data(const std::string& path_to_data)
{
    _path_to_data = path_to_data;

    // Ensure that the data path ends with a slash:
    if (_path_to_data.back() != '/') _path_to_data.append("/");
}

void eva::Config::set_path_to_config(const std::string& path)
{
    _path_to_config = path;

    // Ensure that the data path ends with a slash:
    if (_path_to_config.back() != '/') _path_to_config.append("/");
}

void eva::Config::set_path_to_output(const std::string& path)
{
    _path_to_output = path;

    // Ensure that the data path ends with a slash:
    if (_path_to_output.back() != '/') _path_to_output.append("/");
}

const Types::AlgorithmType eva::Config::get_const_algorithm_type() const
{
    return static_cast<Types::AlgorithmType>(_const_code_algorithm_type);
}

const Types::PricingProblemType eva::Config::get_const_pricing_problem_type() const
{
    return static_cast<Types::PricingProblemType>(_const_code_pricing_problem_type);
}