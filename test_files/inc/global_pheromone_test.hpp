#pragma once

#include <unordered_map>
#include "../../inc/instance.hpp"
#include "../../inc/aco.hpp"
#include "../../inc/as.hpp"
#include "../../inc/common.hpp"
#include "../../inc/common_aco.hpp"
#include "../../inc/common_instance.hpp"

//**Variables**//
//Missions
//Repetitions
//Alpha
//Beta
//q0
class GlobalPheromoneTester
{
    public:
        GlobalPheromoneTester();
        ~GlobalPheromoneTester();
        void run_ab_experiment();
        void run_retention_experiment();
        void run_astar_experiment();
        void run_omega_experiment();

        void run_basic_evaluate_experiment();


        void write_output();

        void load_instance(std::string const & environment_file, std::string const & scenario_file);

        void load_missions(std::vector<int> const & missions);
        void set_repetitions(int repetitions);

        void set_alpha_values(std::vector<double> const & alpha_values);
        void set_beta_values(std::vector<double> const & beta_values);
        void set_q0(double q0);
        void set_k_values(std::vector<double> const & k_values);
        void set_omega_values(std::vector<int> const & omega_values);


        void enable_tuning(bool enable_tuning);

        void set_output_file(std::string const & output_file);

        void set_default_as_params(AS_Params const & params);
        void set_default_aco_params(ACO_Params const & params);

        Instance const & get_instance() const { return instance;}


    private:
        Instance instance;


        std::vector<int> missions;
        int repetitions;

        std::vector<double> alpha_values;
        std::vector<double> beta_values;
        std::vector<double> k_values;
        std::vector<int> omega_values;
        double q0;

        bool tuning_enabled;

        std::string output_file;

        AS_Params default_as_params;
        ACO_Params default_aco_params;

        void compute_optimal_mission_costs();

        std::unordered_map<int, int> opt_mission_costs;
        std::unordered_map<std::string, std::vector<double>> total_costs;
        std::unordered_map<std::string, std::vector< std::vector < double > > > mission_costs;

        void run_case(double alpha, double beta);
        void run_case(double k);
        void run_case_omega(int omega);

};
