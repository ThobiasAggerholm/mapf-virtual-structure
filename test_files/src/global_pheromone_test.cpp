#include "../inc/global_pheromone_test.hpp"

#include <algorithm>
#include <execution>

#include "../../inc/astar_environment.hpp"

GlobalPheromoneTester::GlobalPheromoneTester()
{
    // Constructor
}

GlobalPheromoneTester::~GlobalPheromoneTester()
{
    // Destructor
}

void GlobalPheromoneTester::run_ab_experiment()
{
    // Implementation for running the experiment
    assert(instance.instance_loaded);

    compute_optimal_mission_costs();

    std::for_each(std::execution::par, alpha_values.begin(), alpha_values.end(), [&](double alpha)
    {
        std::for_each(std::execution::par, beta_values.begin(), beta_values.end(), [&](double beta)
        {
            if(alpha == 0.0 && beta == 0.0)
                return;

            run_case(alpha, beta);
        });
    });
}

void GlobalPheromoneTester::run_retention_experiment()
{
    // Implementation for running the experiment
    assert(instance.instance_loaded);

    compute_optimal_mission_costs();

    std::for_each(std::execution::par, k_values.begin(), k_values.end(), [&](double k)
    {
        run_case(k);
    });
}

void GlobalPheromoneTester::run_omega_experiment()
{
    // Implementation for running the experiment
    assert(instance.instance_loaded);

    compute_optimal_mission_costs();

    std::for_each(std::execution::par, omega_values.begin(), omega_values.end(), [&](int omega)
    {
        run_case_omega(omega);
    });
}

//Runs the first num_agent missions regardless of settings
void GlobalPheromoneTester::run_astar_experiment()
{
    assert(instance.instance_loaded);
    //Compute A* costs
    {

        ConflictLocations conflict_data = conflict_locations(instance, missions, nullptr, 0.);
        auto bins = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_data, 1));
        
        cv::Mat map = create_map(instance);
        HeatMap heatmap = create_heatmap(map, bins);

        std::string fname_heatmap = output_file + "heatmap_astar" + ".png";
        std::string fname_heatmap_ranges = output_file + "heatmap_ranges_astar" + ".txt";
        draw_heatmap(heatmap.heatmap, fname_heatmap);
        save_heat_ranges(heatmap.heatranges, fname_heatmap_ranges);

        std::cout << "A* costs" << std::endl;
        std::cout << "Vertex conflicts: " << conflict_data.conflict_counts.vertex_conflicts << std::endl;
        std::cout << "Same direction edge conflicts: " <<  conflict_data.conflict_counts.same_direction_edge_conflicts << std::endl;
        std::cout << "Opposite direction edge conflicts: " <<  conflict_data.conflict_counts.opposite_direction_edge_conflicts << std::endl;

        std::vector<std::vector<int>> agent_paths(default_aco_params.n_agents);
        double makespan = 0;
        double sum_cost = 0;
        for(int i = 0; i < missions.size(); ++i)
        {
            int mission = missions[i];
            agent_paths[i] = agent_path(instance, instance.m_start_locations[mission], instance.m_goal_locations[mission], nullptr, 0);
            double agent_cost = agent_paths[i].size() - 1;
            sum_cost += agent_cost;
            makespan = std::max(makespan, agent_cost);
        }
        std::cout << "Makespan: " << makespan << std::endl;
        std::cout << "Sum cost: " << sum_cost << std::endl;
    }
    {
        //Compute ACO costs
        ACO aco(instance, default_aco_params, default_as_params);
        aco.run();

        EdgeMap const pheromone_map = clamp(aco.get_best_pheromone_map(), default_as_params.min_pheromone, default_as_params.max_pheromone);
        
        std::string fname_pheromone_map = output_file + "pheromone_map" + ".png";
        draw_edge_map(fname_pheromone_map, instance, pheromone_map);
        std::string fname_pheromone_map_arrow = output_file + "pheromone_map_arrow" + ".png";
        draw_arrow_edge_map(fname_pheromone_map_arrow, instance, pheromone_map, default_as_params.max_pheromone, 24);

        ConflictLocations conflict_data = conflict_locations(instance, missions, &pheromone_map, 1.);
        auto bins = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_data, 1));
        
        cv::Mat map = create_map(instance);
        HeatMap heatmap = create_heatmap(map, bins);

        std::string fname_heatmap = output_file + "heatmap" + ".png";
        std::string fname_heatmap_ranges = output_file + "heatmap_ranges" + ".txt";
        draw_heatmap(heatmap.heatmap, fname_heatmap);
        save_heat_ranges(heatmap.heatranges, fname_heatmap_ranges);

        std::cout << "ACO costs" << std::endl;
        std::cout << "Vertex conflicts: " << conflict_data.conflict_counts.vertex_conflicts << std::endl;
        std::cout << "Same direction edge conflicts: " <<  conflict_data.conflict_counts.same_direction_edge_conflicts << std::endl;
        std::cout << "Opposite direction edge conflicts: " <<  conflict_data.conflict_counts.opposite_direction_edge_conflicts << std::endl;

        std::vector<std::vector<int>> agent_paths(default_aco_params.n_agents);
        double makespan = 0;
        double sum_cost = 0;
        for(int i = 0; i < default_aco_params.n_agents; ++i)
        {
            agent_paths[i] = agent_path(instance, instance.m_start_locations[i], instance.m_goal_locations[i], &pheromone_map, 1.0);
            double agent_cost = agent_paths[i].size() - 1;
            sum_cost += agent_cost;
            makespan = std::max(makespan, agent_cost);
        }
        std::cout << "Makespan: " << makespan << std::endl;
        std::cout << "Sum cost: " << sum_cost << std::endl;
    }
}


void GlobalPheromoneTester::write_output()
{
    // Implementation for writing the output
    for(auto it = total_costs.cbegin(); it != total_costs.cend(); ++it)
    {
        std::string fname = it->first;
        std::ofstream file(fname);
        file << "total_cost" << '\n';
        for(auto it2 = it->second.cbegin(); it2 != it->second.cend(); ++it2)
        {
            file << *it2 << '\n';
        }
        file << std::flush;
    }
    for(auto it = mission_costs.cbegin(); it != mission_costs.cend(); ++it)
    {
        std::string fname = it->first;
        std::ofstream file(fname);
        file << "mission_costs" << '\n';
        for(auto it2 = it->second.cbegin(); it2 != it->second.cend(); ++it2)
        {
            for(auto it3 = it2->cbegin(); it3 != it2->cend(); ++it3)
            {
                file << *it3 << ',';
            }
            file << '\n';
        }
        file << std::flush;
    }
    std::string fname = output_file + "_opt_.csv";
    std::ofstream file(fname);
    file << "opt_mission_costs" << '\n';
    for(auto it = opt_mission_costs.cbegin(); it != opt_mission_costs.cend(); ++it)
    {
        file << it->first << ',' << it->second << '\n';
    }
    file << std::flush;
}


void GlobalPheromoneTester::load_instance(std::string const& environment_file, std::string const& scenario_file)
{
    // Implementation for loading the instance
    int mission_range = *std::max_element(missions.begin(), missions.end());
    instance = Instance(environment_file, scenario_file, mission_range + 1);
}

void GlobalPheromoneTester::load_missions(std::vector<int> const& missions)
{
    this->missions = missions;
}

void GlobalPheromoneTester::set_repetitions(int repetitions)
{
    this->repetitions = repetitions;
}

void GlobalPheromoneTester::set_alpha_values(std::vector<double> const& alpha_values)
{
    this->alpha_values = alpha_values;
}

void GlobalPheromoneTester::set_beta_values(std::vector<double> const& beta_values)
{
    this->beta_values = beta_values;
}

void GlobalPheromoneTester::set_q0(double q0)
{
    this->q0 = q0;
}

void GlobalPheromoneTester::set_k_values(std::vector<double> const & k_values)
{
    this->k_values = k_values;
}

void GlobalPheromoneTester::set_omega_values(std::vector<int> const & omega_values)
{
    this->omega_values = omega_values;
}


void GlobalPheromoneTester::enable_tuning(bool enable_tuning)
{
    this->tuning_enabled = enable_tuning;
}

void GlobalPheromoneTester::set_output_file(std::string const& output_file)
{
    this->output_file = output_file;
}

void GlobalPheromoneTester::compute_optimal_mission_costs()
{
    std::for_each(std::execution::par, missions.begin(), missions.end(), [&](int mission)
    {
        assert(instance.get_start_mission(mission) != instance.get_goal_mission(mission));
        std::vector<int> path = agent_path(instance, instance.get_start_mission(mission), instance.get_goal_mission(mission));
        opt_mission_costs[mission] = path.size() - 1;
    });
}

void GlobalPheromoneTester::set_default_as_params(AS_Params const & params)
{
    assert(instance.instance_loaded);
    default_as_params = params;
    default_as_params.graph = &instance.m_my_graph;
    default_as_params.n_vertices = instance.m_map_size;
}

void GlobalPheromoneTester::set_default_aco_params(ACO_Params const & params)
{
    assert(instance.instance_loaded);
    int num_agents = missions.size();

    default_aco_params = params;
    default_aco_params.n_agents = num_agents;
    default_aco_params.starts.resize(num_agents);
    default_aco_params.goals.resize(num_agents);

    default_aco_params.init_pheromone = (1. - 0.001)/2.;
    for(int i = 0; i < num_agents; ++i)
    {
        int mission = missions[i];
        assert(mission >= 0 && mission < instance.m_start_locations.size() && mission < instance.m_goal_locations.size());
        assert(instance.m_start_locations[mission] != instance.m_goal_locations[mission]);
        default_aco_params.goals[i] = &instance.m_my_graph[instance.m_goal_locations[mission]];
        default_aco_params.starts[i] = &instance.m_my_graph[instance.m_start_locations[mission]];
    }

}

void GlobalPheromoneTester::run_case(double alpha, double beta)
{
    // Implementation for running a case
    std::vector<int> repetitions_vector(repetitions);
    std::iota(repetitions_vector.begin(), repetitions_vector.end(), 0);
    std::vector<int> missions_vector(missions.size());
    std::iota(missions_vector.begin(), missions_vector.end(), 0);

    //Get each ants path cost
    //Get total ACO cost
    std::vector< std::vector < double > > mission_costs_vector(missions.size(), std::vector<double>(repetitions));
    std::vector<double> total_cost_vector(repetitions);
    std::for_each(std::execution::par, repetitions_vector.cbegin(), repetitions_vector.cend(), [&](int repetition)
    {
        AS_Params as_params = default_as_params;
        as_params.alpha = alpha;
        as_params.beta = beta;

        ACO_Params aco_params = default_aco_params;

        ACO aco_instance(instance, aco_params, as_params);
        aco_instance.run();

        //Get total ACO cost and add to total_cost_vector
        ACOLog const & aco_log = aco_instance.get_results_log(); 
        double min_makespan = std::numeric_limits<double>::max();
        int best_iteration = -1;
        for(int i = 0; i < aco_log.get_makespans_size(); ++i)
        {
            if(aco_log.get_makespan(i) < min_makespan)
            {
                min_makespan = aco_log.get_makespan(i);
                best_iteration = i;
            }
        }
        if(repetition == 0)
        {
            std::string fname_total = output_file + "total_converge" + "_alpha_" + std::to_string(alpha) + "_beta_" + std::to_string(beta) + ".csv";
            std::ofstream file(fname_total);
            for(int i = 0; i < aco_log.get_makespans_size(); ++i)
            {
                file << aco_log.get_makespan(i) << ',';
            }
            file << std::endl;
        }
        total_cost_vector[repetition] = min_makespan;
        //Get each AS results log and add minimum makespan to mission_costs_vector
        std::for_each(std::execution::par, missions_vector.cbegin(), missions_vector.cend(), [&aco_instance, repetition, &mission_costs_vector, &best_iteration](int mission)
        {
            AS & as_instance = aco_instance.get_as(mission);
            ASLog const & as_log = as_instance.get_log();
            double min_makespan = as_log.get_makespan(best_iteration);
            mission_costs_vector[mission][repetition] = min_makespan;
        });

    });

    //Save the results
    std::string fname_total = output_file + "total" + "_alpha_" + to_string_with_precision(alpha,1) + "_beta_" + to_string_with_precision(beta, 1) + ".csv";
    std::string fname_mission = output_file + "mission" + "_alpha_" + to_string_with_precision(alpha,1) + "_beta_" + to_string_with_precision(beta ,1) + ".csv";

    total_costs[fname_total] = total_cost_vector;
    mission_costs[fname_mission] = mission_costs_vector;
}

void GlobalPheromoneTester::run_case(double k)
{
    // Implementation for running a case
    std::vector<int> repetitions_vector(repetitions);
    std::iota(repetitions_vector.begin(), repetitions_vector.end(), 0);
    std::vector<int> missions_vector(missions.size());
    std::iota(missions_vector.begin(), missions_vector.end(), 0);

    //Get each ants path cost
    //Get total ACO cost
    std::vector< std::vector < double > > mission_costs_vector(missions.size(), std::vector<double>(repetitions));
    std::vector<double> number_conflicts(repetitions);
    std::for_each(std::execution::par, repetitions_vector.cbegin(), repetitions_vector.cend(), [&](int repetition)
    {
        AS_Params as_params = default_as_params;
        as_params.K = k;
        as_params.deposit = ((as_params.max_pheromone- as_params.min_pheromone)/(double(as_params.K*2.)));

        ACO_Params aco_params = default_aco_params;

        ACO aco_instance(instance, aco_params, as_params);
        aco_instance.run();

        /*
            Cost of each mission
                1) Go through every iteration
                2) Get the ant log
                3) get makespan from AS log for iteration
                4) Save index of iteration with best sum of costs
                5) Save the makespan for each AS for the best iteration

            Collisions for each best ant
                1) From best iteration get ant paths
                2) From ant paths get number of conflicts
        */
        //Data collection
        int best_iteration = -1;
        double min_sum_of_costs = std::numeric_limits<double>::max();
        ACOLog const & aco_log = aco_instance.get_results_log();
        for(int iter_idx = 0; iter_idx < aco_log.get_costs_size(); ++iter_idx)
        {
            double iter_sum_of_costs = 0;
            //Get each AS results log
            std::for_each(std::execution::seq, missions_vector.cbegin(), missions_vector.cend(), [&aco_instance, &iter_idx, &iter_sum_of_costs](int mission)
            {
                AS & as_instance = aco_instance.get_as(mission);
                ASLog const & as_log = as_instance.get_log();
                double min_makespan = as_log.get_makespan(iter_idx);
                iter_sum_of_costs += min_makespan;
            });
            iter_sum_of_costs += (aco_log.get_conflicts(iter_idx) * aco_params.conflict_penalty);
            if(iter_sum_of_costs < min_sum_of_costs)
            {
                min_sum_of_costs = iter_sum_of_costs;
                best_iteration = iter_idx;
            }
        }
        //Get each AS results log
        std::for_each(std::execution::par, missions_vector.cbegin(), missions_vector.cend(), [&aco_instance, repetition, &mission_costs_vector, &best_iteration](int mission)
        {
            AS & as_instance = aco_instance.get_as(mission);
            ASLog const & as_log = as_instance.get_log();
            double min_makespan = as_log.get_makespan(best_iteration);
            mission_costs_vector[mission][repetition] = min_makespan;
        });

        //Get number of conflicts
        number_conflicts[repetition] = aco_log.get_conflicts(best_iteration);

    });

    //Save the results
    std::string fname_total = output_file + "conflicts" + "_k_" + to_string_with_precision(k,1) + ".csv";
    std::string fname_mission = output_file + "SOC" + "_k_" + to_string_with_precision(k,1) + ".csv";

    total_costs[fname_total] = number_conflicts;
    mission_costs[fname_mission] = mission_costs_vector;
}

void GlobalPheromoneTester::run_case_omega(int omega)
{
    // Implementation for running a case
    std::vector<int> repetitions_vector(repetitions);
    std::iota(repetitions_vector.begin(), repetitions_vector.end(), 0);
    std::vector<int> missions_vector(missions.size());
    std::iota(missions_vector.begin(), missions_vector.end(), 0);

    //Get each ants path cost
    //Get total ACO cost
    std::vector< std::vector < double > > mission_costs_vector(missions.size(), std::vector<double>(repetitions));
    std::vector<double> number_conflicts(repetitions);
    std::for_each(std::execution::par, repetitions_vector.cbegin(), repetitions_vector.cend(), [&](int repetition)
    {
        AS_Params as_params = default_as_params;
        as_params.n_ants = omega;

        ACO_Params aco_params = default_aco_params;

        ACO aco_instance(instance, aco_params, as_params);
        aco_instance.run();

        /*
            Cost of each mission
                1) Go through every iteration
                2) Get the ant log
                3) get makespan from AS log for iteration
                4) Save index of iteration with best sum of costs
                5) Save the makespan for each AS for the best iteration

            Collisions for each best ant
                1) From best iteration get ant paths
                2) From ant paths get number of conflicts
        */
        //Data collection
        int best_iteration = -1;
        double min_sum_of_costs = std::numeric_limits<double>::max();
        ACOLog const & aco_log = aco_instance.get_results_log();
        for(int iter_idx = 0; iter_idx < aco_log.get_costs_size(); ++iter_idx)
        {
            double iter_sum_of_costs = 0;
            //Get each AS results log
            std::for_each(std::execution::seq, missions_vector.cbegin(), missions_vector.cend(), [&aco_instance, &iter_idx, &iter_sum_of_costs](int mission)
            {
                AS & as_instance = aco_instance.get_as(mission);
                ASLog const & as_log = as_instance.get_log();
                double min_makespan = as_log.get_makespan(iter_idx);
                iter_sum_of_costs += min_makespan;
            });
            iter_sum_of_costs += (aco_log.get_conflicts(iter_idx) * aco_params.conflict_penalty);
            if(iter_sum_of_costs < min_sum_of_costs)
            {
                min_sum_of_costs = iter_sum_of_costs;
                best_iteration = iter_idx;
            }
        }
        //Get each AS results log
        std::for_each(std::execution::par, missions_vector.cbegin(), missions_vector.cend(), [&aco_instance, repetition, &mission_costs_vector, &best_iteration](int mission)
        {
            AS & as_instance = aco_instance.get_as(mission);
            ASLog const & as_log = as_instance.get_log();
            double min_makespan = as_log.get_makespan(best_iteration);
            mission_costs_vector[mission][repetition] = min_makespan;
        });

        //Get number of conflicts
        number_conflicts[repetition] = aco_log.get_conflicts(best_iteration);

    });

    //Save the results
    std::string fname_total = output_file + "conflicts" + "_omega_" + std::to_string(omega) + ".csv";
    std::string fname_mission = output_file + "SOC" + "_omega_" + std::to_string(omega) + ".csv";

    total_costs[fname_total] = number_conflicts;
    mission_costs[fname_mission] = mission_costs_vector;
}

void GlobalPheromoneTester::run_basic_evaluate_experiment()
{
    assert(instance.instance_loaded);

    // Save A* path lenghts in file
    // Save A* types
    // Save A* conflict heatmap
    compute_optimal_mission_costs();
    std::ofstream f_astar_paths_before(output_file + "astar_path_costs_before" + ".csv");
    assert(f_astar_paths_before.is_open());
    for(auto astar_path_cost_before : opt_mission_costs)
    {
        f_astar_paths_before << astar_path_cost_before.first << "," << astar_path_cost_before.second << std::endl;
    }
    f_astar_paths_before.close();

    ConflictLocations conflict_data_before = conflict_locations(instance, missions, nullptr, 0.);

    auto conflict_counts_before = conflict_data_before.conflict_counts;
    std::tuple<int,int,int> astar_conflicts_before = {conflict_counts_before.vertex_conflicts,conflict_counts_before.same_direction_edge_conflicts, conflict_counts_before.opposite_direction_edge_conflicts};

    std::ofstream f_astar_conflicts_before(output_file + "astar_conflicts_before" + ".csv");
    assert(f_astar_conflicts_before.is_open());
    f_astar_conflicts_before << std::get<0>(astar_conflicts_before) << ","<<  std::get<1>(astar_conflicts_before) << "," <<  std::get<2>(astar_conflicts_before) << std::endl;
    f_astar_conflicts_before.close();

    auto bins_before = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_data_before, 1));
    cv::Mat map_before = create_map(instance);
    HeatMap heatmap_before = create_heatmap(map_before, bins_before);

    std::string fname_heatmap_before = output_file + "astar_heatmap_before" + ".png";
    std::string fname_heatmap_ranges_before = output_file + "astar_heatmap_ranges_before" + ".csv";
    draw_heatmap(heatmap_before.heatmap, fname_heatmap_before);
    save_heat_ranges(heatmap_before.heatranges, fname_heatmap_ranges_before);

    //Save ACO costs per mission
    //Save ACO conflict types
    //Add ACO Heatmap
    //Save Arrow map for first
    //Add Arrow map
    // Implementation for running a case
    std::vector<int> repetitions_vector(repetitions);
    std::iota(repetitions_vector.begin(), repetitions_vector.end(), 0);
    std::vector<int> missions_vector(missions.size());
    std::iota(missions_vector.begin(), missions_vector.end(), 0);
    
    std::vector<ConflictLocations> conflict_locations_aco(repetitions);
    std::vector<std::tuple<int, int, int>> number_conflicts(repetitions, {0,0,0});
    std::vector<EdgeMap> pheromone_maps(repetitions);

    std::vector<std::vector<int>> astar_path_lengths_after(missions.size(), std::vector<int>(repetitions, 0));
    std::vector<std::tuple<int, int, int>> astar_number_conflicts_after(repetitions, {0,0,0});
    std::vector<ConflictLocations> conflict_locations_astar(repetitions);

    std::vector<std::vector<double>> mission_costs_vector(missions.size(), std::vector<double>(repetitions, 0.));
    std::for_each(std::execution::par, repetitions_vector.cbegin(), repetitions_vector.cend(), [this, &repetitions_vector, &missions_vector, &mission_costs_vector, &number_conflicts, &conflict_locations_aco, &pheromone_maps, &astar_path_lengths_after, &conflict_locations_astar, &astar_number_conflicts_after](int repetition)
    {
        //Compute ACO costs
        ACO aco(instance, default_aco_params, default_as_params);
        aco.run();

        //Data collection
        int best_iteration = -1;
        double min_sum_of_costs = std::numeric_limits<double>::max();
        ACOLog const & aco_log = aco.get_results_log();
        std::vector<std::vector<double>> iteration_costs(aco_log.get_costs_size(), std::vector<double>(missions.size(),0.));
        for(int iter_idx = 0; iter_idx < aco_log.get_costs_size(); ++iter_idx)
        {
            double iter_sum_of_costs = 0;
            //Get each AS results log
            std::for_each(std::execution::seq, missions_vector.cbegin(), missions_vector.cend(), [&aco, &iter_idx, &iter_sum_of_costs, &repetition, &iteration_costs](int mission)
            {
                AS & as_instance = aco.get_as(mission);
                ASLog const & as_log = as_instance.get_log();
                double min_makespan = as_log.get_makespan(iter_idx);
                iter_sum_of_costs += min_makespan;
                if(repetition == 0)
                {
                    iteration_costs[iter_idx][mission] = min_makespan;
                }
            });
            iter_sum_of_costs += (aco_log.get_conflicts(iter_idx) * default_aco_params.conflict_penalty);
            if(iter_sum_of_costs < min_sum_of_costs)
            {
                min_sum_of_costs = iter_sum_of_costs;
                best_iteration = iter_idx;
            }
        }
        if(repetition == 0)
        {
            std::ofstream f_aco_iteration_costs(output_file + "aco_iteration_costs" + ".csv");
            assert(f_aco_iteration_costs.is_open());
            for(int iter_idx = 0; iter_idx < aco_log.get_costs_size(); ++iter_idx)
            {
                for(int mission = 0; mission < missions.size(); ++mission)
                {
                    f_aco_iteration_costs << iteration_costs[iter_idx][mission] << ",";
                }
                f_aco_iteration_costs << std::endl;
            }
            f_aco_iteration_costs.close();
        }
        //Get each AS results log
        std::for_each(std::execution::par, missions_vector.cbegin(), missions_vector.cend(), [&aco, repetition, &mission_costs_vector, &best_iteration](int mission)
        {
            AS & as_instance = aco.get_as(mission);
            ASLog const & as_log = as_instance.get_log();
            double min_makespan = as_log.get_makespan(best_iteration);
            mission_costs_vector[mission][repetition] = min_makespan;
        });

        //Get conflict types
        std::vector<std::vector<int>> ant_paths = aco.get_best_paths();
        ConflictLocations aco_conflict_data = conflict_locations(instance, ant_paths, nullptr, 0.);
        conflict_locations_aco[repetition] = aco_conflict_data;

        auto aco_conflict_counts = aco_conflict_data.conflict_counts;
        std::tuple<int,int,int> aco_conflicts = {aco_conflict_counts.vertex_conflicts,aco_conflict_counts.same_direction_edge_conflicts, aco_conflict_counts.opposite_direction_edge_conflicts};
        number_conflicts[repetition] = aco_conflicts;

        pheromone_maps[repetition] = aco.get_best_pheromone_map();

        EdgeMap const pheromone_map = clamp(pheromone_maps[repetition], default_as_params.min_pheromone, default_as_params.max_pheromone);

        for(int i = 0; i < missions.size(); ++i)
        {
            int mission = missions[i];
            auto agent_path_after = agent_path(instance, instance.m_start_locations[mission], instance.m_goal_locations[mission], &pheromone_map, 1.);
            astar_path_lengths_after[i][repetition] = agent_path_after.size();
        }

        ConflictLocations astar_conflict_data_after = conflict_locations(instance, missions, &pheromone_map, 1.);
        auto astar_conflict_counts_after = astar_conflict_data_after.conflict_counts;
        astar_number_conflicts_after[repetition] = {astar_conflict_counts_after.vertex_conflicts,astar_conflict_counts_after.same_direction_edge_conflicts, astar_conflict_counts_after.opposite_direction_edge_conflicts};
        conflict_locations_astar[repetition] = astar_conflict_data_after;


        //If first
        if(repetition == 0)
            draw_arrow_edge_map(output_file + "arrow_map_0" + ".png", instance, pheromone_map, default_as_params.max_pheromone - 0.001, 24);

    });
    //Save A* costs per mission
    //Save A* conflict types
    //Add A* Heatmap


    //Save avg heatmap ACO
    ConflictLocations conflict_locations_cumulated_aco = conflict_locations_aco.at(0);
    for(int repetition = 1; repetition < repetitions; ++repetition)
    {
        conflict_locations_cumulated_aco = conflict_locations_cumulated_aco + conflict_locations_aco.at(repetition);
    }
    auto aco_bins = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_locations_cumulated_aco, repetitions));
    cv::Mat map = create_map(instance);
    HeatMap heatmap_aco = create_heatmap(map, aco_bins);
    draw_heatmap(heatmap_aco.heatmap, output_file + "heatmap_aco" + ".png");
    save_heat_ranges(heatmap_aco.heatranges, output_file + "heatmap_aco_ranges" + ".csv");


    //Save avg heatmap A*
    ConflictLocations conflict_locations_cumulated_astar = conflict_locations_astar.at(0);
    for(int repetition = 1; repetition < repetitions; ++repetition)
    {
        conflict_locations_cumulated_astar = conflict_locations_cumulated_astar + conflict_locations_astar.at(repetition);
    }
    auto astar_bins = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_locations_cumulated_astar, repetitions));
    HeatMap heatmap_astar = create_heatmap(map, astar_bins);
    draw_heatmap(heatmap_astar.heatmap, output_file + "heatmap_astar_after" + ".png");
    save_heat_ranges(heatmap_astar.heatranges, output_file + "heatmap_astar_after_ranges" + ".csv");

    //Save avg arrowmap
    EdgeMap pheromone_map_cumulated = pheromone_maps.at(0);
    for(int repetition = 1; repetition < repetitions; ++repetition)
    {
        pheromone_map_cumulated = pheromone_map_cumulated + pheromone_maps.at(repetition);
    }
    pheromone_map_cumulated = pheromone_map_cumulated / repetitions;
    EdgeMap const arrow_pheromone_map = clamp(pheromone_map_cumulated, default_as_params.min_pheromone, default_as_params.max_pheromone);
    draw_arrow_edge_map(output_file + "arrow_map_avg" + ".png", instance, arrow_pheromone_map, default_as_params.max_pheromone - 0.001, 24);

    //Save A* costs per mission
    std::ofstream f_astar_paths_after(output_file + "astar_path_costs_after" + ".csv");
    for(int idx_rep = 0; idx_rep < repetitions; ++idx_rep)
    {
        for(int idx_mission = 0; idx_mission < missions.size(); ++idx_mission)
        {
            f_astar_paths_after << astar_path_lengths_after[idx_mission][idx_rep];
            if(idx_mission < missions.size() - 1)
                f_astar_paths_after << ",";
        }
        if(idx_rep < repetitions - 1)
            f_astar_paths_after << std::endl;
    }
    f_astar_paths_after.close();
    //Save A* conflict types
    std::ofstream f_astar_conflicts_after(output_file + "astar_conflicts_after" + ".csv");
    for(int idx_rep = 0; idx_rep < repetitions; ++idx_rep)
    {
        f_astar_conflicts_after << std::get<0>(astar_number_conflicts_after[idx_rep]) << "," << std::get<1>(astar_number_conflicts_after[idx_rep]) << "," << std::get<2>(astar_number_conflicts_after[idx_rep]);
        if(idx_rep < repetitions - 1)
            f_astar_conflicts_after << std::endl;
    }
    f_astar_conflicts_after.close();
    //Save ACO costs per mission
    std::ofstream f_aco_paths(output_file + "aco_path_costs" + ".csv");
    for(int idx_rep = 0; idx_rep < repetitions; ++idx_rep)
    {
        for(int idx_mission = 0; idx_mission < missions.size(); ++idx_mission)
        {
            f_aco_paths << mission_costs_vector[idx_mission][idx_rep];
            if(idx_mission < missions.size() - 1)
                f_aco_paths << ",";
        }
        if(idx_rep < repetitions - 1)
            f_aco_paths << std::endl;
    }
    f_aco_paths.close();
    //Save ACO conflict types
    std::ofstream f_aco_conflicts(output_file + "aco_conflicts" + ".csv");
    for(int idx_rep = 0; idx_rep < repetitions; ++idx_rep)
    {
        f_aco_conflicts << std::get<0>(number_conflicts[idx_rep]) << "," << std::get<1>(number_conflicts[idx_rep]) << "," << std::get<2>(number_conflicts[idx_rep]);
        if(idx_rep < repetitions - 1)
            f_aco_conflicts << std::endl;
    }
    f_aco_conflicts.close();
}