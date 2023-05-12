#include "../inc/intensification_test.hpp"

/*
    //Draw heatmap of collisions before
    auto conflict_data = conflict_locations(instance, num_agents); //TODO change to mission selection
    auto bins_before = conflict_locations_to_bins(conflict_locations_to_histogram2d(conflict_data));
    auto heatmap_before = create_heatmap(map, bins_before);
    draw_heatmap(heatmap_before.heatmap, intensification_fname + "conflict_heatmap_before" + image_extension); //TODO change file name to reflect mission selection
    save_heat_ranges(heatmap_before.heatranges, intensification_fname + "conflict_heatmap_ranges_before" + data_extension); //TODO change file name to reflect mission selection
    
*/

#include <sstream>


bool IntensificationTest::operator<(const IntensificationTest::Position &a, const IntensificationTest::Position &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

bool IntensificationTest::operator==(const IntensificationTest::Position &a, const IntensificationTest::Position &b) {
    return a.x == b.x && a.y == b.y;
}

bool IntensificationTest::operator!=(const IntensificationTest::Position &a, const IntensificationTest::Position &b) {
    return !(a == b);
}

void IntensificationTest::configure_as_params(AS_Params & as_params, Instance const & instance)
{
    as_params.graph = &instance.m_my_graph;
    as_params.n_vertices = instance.m_map_size;
    as_params.n_ants = 10;
    as_params.alpha = 1;
    as_params.beta = 1;
    as_params.min_pheromone = 0.001;
    as_params.max_pheromone = 1;
    as_params.q0 = 0.;
    as_params.K = 3;
    as_params.deposit = ((as_params.max_pheromone- as_params.min_pheromone)/(double(as_params.K*2.)));
}

void IntensificationTest::configure_aco_params(ACO_Params & aco_params, Instance const & instance, std::vector<int> missions)
{
    int num_agents = missions.size();
    aco_params.conflict_penalty = 1.0;
    aco_params.IT_NI = 100;//100;
    aco_params.IT_MAX = 1000;
    aco_params.IT_INFO = -1;
    aco_params.MAX_STEPS = 500;
    aco_params.n_agents = num_agents;
    aco_params.starts.resize(num_agents);
    aco_params.goals.resize(num_agents);
    aco_params.init_pheromone = (1. - 0.001)/2.;
    for(int i = 0; i < num_agents; ++i)
    {
        int mission = missions[i];
        assert(mission >= 0 && mission < instance.m_start_locations.size() && mission < instance.m_goal_locations.size());
        assert(instance.m_start_locations[mission] != instance.m_goal_locations[mission]);
        aco_params.goals[i] = &instance.m_my_graph[instance.m_goal_locations[mission]];
        aco_params.starts[i] = &instance.m_my_graph[instance.m_start_locations[mission]];
    }
}

void IntensificationTest::test_ab()
{
    std::string test_name = "ab";
    test_intensification(test_name, test_mission_ab);
}

void IntensificationTest::test_q0()
{
    std::string test_name = "q0";
    test_intensification(test_name, test_mission_q0);
}

void IntensificationTest::test_intensification(std::string const & test_name, std::function<void(std::string const &, Instance const &, int, int)> const & test_mission)
{
    std::string map_fname = "den312d";
    std::string map_fname_in = "../benchmark_data/mapf-map/" + map_fname + ".map"; // For debugging
    std::string agent_fname = "../benchmark_data/mapf-scen-random/"  + map_fname + "-random-1.scen"; // For debugging


    std::string intensification_fname = "../test_files/test_data/intensification/" + test_name +  "/" + map_fname + "/";
    std::vector<std::string> missons_fnames = {"mission_1", "mission_2", "mission_3", "mission_4", "mission_5"};

    int num_missions = 5; // For debugging
    int number_repetions = 20;

    Instance instance(map_fname_in, agent_fname, num_missions);

    for(int i = 0; i < num_missions; ++i)
    {
        std::vector<int> astar_path = agent_path(instance, instance.m_start_locations[i], instance.m_goal_locations[i]);
        std::cout << "Mission " << i + 1 << " A* path length: " << astar_path.size() - 1<<  "\t";
    }
    std::cout << std::endl;

    std::vector<int> i_missions(num_missions);
    std::iota(i_missions.begin(), i_missions.end(), 0);
    std::for_each(std::execution::par, i_missions.cbegin(), i_missions.cend(), [&](int i)
    {
        std::string experiment_fname = intensification_fname + missons_fnames[i] + "/";
        int mission = i;
        test_mission(experiment_fname, instance, mission, number_repetions);
    });

}

void IntensificationTest::test_mission_ab(std::string const & path_fname, Instance const & instance, int mission, int number_repetions)
{
    std::vector<std::pair<double, double>> alpha_beta_values;
    for(int i = 0; i < 11; ++i)
    {
        for(int j = 0; j < 11; ++j)
        {
            if(i == 0 && j == 0)
                continue;
            alpha_beta_values.push_back({double(i), double(j)});
        }
    }
    std::vector<int> i_alpha_beta_values(alpha_beta_values.size(), 0);
    std::iota(i_alpha_beta_values.begin(), i_alpha_beta_values.end(), 0);

    std::vector<int> i_repetitions(number_repetions, 0);
    std::iota(i_repetitions.begin(), i_repetitions.end(), 1);

    std::for_each(std::execution::par, i_alpha_beta_values.begin(), i_alpha_beta_values.end(), [&](int i)
    {
        //Create agents path and check collisions
        AS_Params as_params;
        configure_as_params(as_params, instance);

        ACO_Params aco_params;
        configure_aco_params(aco_params, instance, {mission});
        
        as_params.alpha = alpha_beta_values[i].first;
        as_params.beta = alpha_beta_values[i].second;


        std::vector<double> costs(number_repetions, 0);
        std::for_each(std::execution::par, i_repetitions.begin(), i_repetitions.end(), [&](int j)
        {
            ACO aco(instance, aco_params, as_params);
            aco.run();
            auto const & results_log = aco.get_results_log();
            //get minimum cost
            double min_cost = std::numeric_limits<double>::max();
            for(int k = 0; k < results_log.get_costs_size(); ++k)
            {
                if(results_log.get_cost(k) < min_cost)
                    min_cost = results_log.get_cost(k);
            }
            costs[j] = min_cost;
            //aco.get_results_log().write_to_file(path_fname + "_a" + to_string_with_precision(as_params.alpha, 1) + "_b" + to_string_with_precision(as_params.beta, 1) + "_rep" + std::to_string(j) + ".csv");
        });
        //Write costs to file
        std::ofstream file;
        file.open(path_fname + "_a" + to_string_with_precision(as_params.alpha, 1) + "_b" + to_string_with_precision(as_params.beta, 1) + "_costs.csv");
        for(int j = 0; j < costs.size(); ++j)
        {
            file << costs[j] << std::endl;
        }
        file.close();
        
    });

    // { //Test on tuning
    //     AS_Params as_params;
    //     configure_as_params(as_params, instance);
    //     as_params.tune_decision_weights = true;
    //     as_params.alpha = 1;
    //     as_params.beta = 1;

    //     ACO_Params aco_params;
    //     configure_aco_params(aco_params, instance, {mission});

    //     std::for_each(std::execution::par, i_repetitions.begin(), i_repetitions.end(), [&](int j)
    //     {
    //         ACO aco(instance, aco_params, as_params);
    //         aco.run();
    //         aco.get_results_log().write_to_file(path_fname + "_abtuning" + "_rep" + std::to_string(j) + ".csv");
    //     });
    // }


}

void IntensificationTest::test_mission_q0(std::string const & path_fname, Instance const & instance, int mission, int number_repetions)
{
    std::vector<double> q0_values = {0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.};

    std::vector<int> i_repetitions(number_repetions, 0);
    std::iota(i_repetitions.begin(), i_repetitions.end(), 0);

    std::for_each(std::execution::par, q0_values.begin(), q0_values.end(), [&](double q0)
    {
        //Create agents path and check collisions
        AS_Params as_params;
        configure_as_params(as_params, instance);

        ACO_Params aco_params;
        configure_aco_params(aco_params, instance, {mission});
        
        as_params.alpha = 4.;
        as_params.beta = 1.;

        as_params.q0 = q0;

        std::vector<double> costs(number_repetions, 0);
        std::for_each(std::execution::par, i_repetitions.begin(), i_repetitions.end(), [&](int j)
        {
            ACO aco(instance, aco_params, as_params);
            aco.run();
            auto const & results_log = aco.get_results_log();
            double min_cost = std::numeric_limits<double>::max();
            for(int k = 0; k < results_log.get_costs_size(); ++k)
            {
                if(results_log.get_cost(k) < min_cost)
                    min_cost = results_log.get_cost(k);
            }
            costs[j] = min_cost;
            //aco.get_results_log().write_to_file(path_fname + "_q0" + to_string_with_precision(q0, 1) + "_rep" + std::to_string(j) + ".csv");
        });
        //Write costs to file
        std::ofstream file;
        file.open(path_fname + "_q0_" + to_string_with_precision(q0, 1) + "_costs.csv");
        for(int j = 0; j < costs.size(); ++j)
        {
            file << costs[j] << std::endl;
        }
        file.close();
        
    });

    { //Test on tuning
        AS_Params as_params;
        configure_as_params(as_params, instance);
        as_params.tune_greedy_selection_prob = true;
        as_params.q0 = 0;
        as_params.alpha = 4.;
        as_params.beta = 1.;

        ACO_Params aco_params;
        configure_aco_params(aco_params, instance, {mission});
        
        std::vector<double> costs(number_repetions, 0);
        std::for_each(std::execution::par, i_repetitions.begin(), i_repetitions.end(), [&](int j)
        {
            ACO aco(instance, aco_params, as_params);
            aco.run();
            auto const & results_log = aco.get_results_log();
            double min_cost = std::numeric_limits<double>::max();
            for(int k = 0; k < results_log.get_costs_size(); ++k)
            {
                if(results_log.get_cost(k) < min_cost)
                    min_cost = results_log.get_cost(k);
            }
            costs[j] = min_cost;
            //aco.get_results_log().write_to_file(path_fname + "_q0tuning" + "_rep" + std::to_string(j) + ".csv");
        });
        //Write costs to file
        std::ofstream file;
        file.open(path_fname + "_q0tuning.csv");
        for(int j = 0; j < costs.size(); ++j)
        {
            file << costs[j] << std::endl;
        }
        file.close();
    }



}

void IntensificationTest::test_weight_tuning()
{
//     std::string map_fname_in = "../benchmark_data/mapf-map/den312d.map"; // For debugging
//     std::string map_fname_out = "../output_data/mapf-map/den312d.png"; // For debugging

//     std::string agent_fname = "../benchmark_data/mapf-scen-random/den312d-random-1.scen"; // For debugging
//     std::string intensification_fname = "../test_files/test_data/intensification/intensification_test-den312d-";
//     std::string data_extension = ".csv";
//     std::string image_extension = ".png";
//     int num_agents = 1; // For debugging

//     Instance instance(map_fname_in, agent_fname, num_agents);

//     int number_repitions = 20;
// #define TEST_INTENSIFICATION 0
// #if TEST_INTENSIFICATION
//     std::vector<std::pair<double, double>> alpha_beta_values = {{1.,1.}};
// #else
//     std::vector<std::pair<double, double>> alpha_beta_values = {{0.,1.}};
// #endif


//     cv::Mat map = create_map(instance);

//     //Draw heatmap of collisions before    
//     std::vector<int> i_alpha_beta_values(alpha_beta_values.size(), 0);
//     std::iota(i_alpha_beta_values.begin(), i_alpha_beta_values.end(), 0);

//     std::for_each(std::execution::par, i_alpha_beta_values.begin(), i_alpha_beta_values.end(), [&](int i)
//     {
//         std::cout << "Intensification test with alpha = " << alpha_beta_values[i].first << " and beta = " << alpha_beta_values[i].second << std::endl;

//         //Create agents path and check collisions
//         AS_Params as_params;
//         configure_as_params(as_params, instance);
//         as_params.tune_decision_weights = true;
//         as_params.tune_greedy_selection_prob = true;

//         ACO_Params aco_params;
//         configure_aco_params(aco_params, instance, num_agents);
        
//         as_params.alpha = alpha_beta_values[i].first;
//         as_params.beta = alpha_beta_values[i].second;
//         EdgeMap average_edge_map;
//         ConflictLocations comulated_conflict_data;
//         bool snapshot = false;
//         bool firstIt = true;
//         std::vector < std::vector<std::tuple<int, double, double, int>> > results_log;
//         for(int j = 0; j < number_repitions; ++j)
//         {
//             ACO aco(instance, aco_params, as_params);
//             aco.run();
//             results_log.push_back(aco.get_results_log());
//             auto conflict_data_it = conflict_locations(instance, num_agents);
//             comulated_conflict_data = comulated_conflict_data + conflict_data_it;
//             if(!snapshot)
//             {
//                 std::string fname_snapshot = intensification_fname + "_edge_map_snapshot_alpha_" + std::to_string((int)as_params.alpha) + "_beta_" + std::to_string((int)as_params.beta) + "_tuning_" + image_extension;
//                 draw_edge_map(fname_snapshot, instance, aco.get_best_pheromone_map());
//                 snapshot = true;
//             }
//             if(firstIt)
//             {
//                 average_edge_map = aco.get_best_pheromone_map();
//                 firstIt = false;
//             }
//             else
//             {
//                 average_edge_map = average_edge_map + aco.get_best_pheromone_map();
//             }
//         }
//         //Average of results log
//         std::vector<std::tuple<int, double, double, int>> average_results_log;
//         int num_it = 0;
//         int num_reached_timestep = 0;
//         bool done = true;
//         firstIt = true;
//         while(!done || firstIt)
//         {
//             firstIt = false;
//             done = true;
//             int iteration = 0;
//             double cost = 0.;
//             double makespan = 0.;
//             double conflicts = 0.;
//             num_reached_timestep = 0;
//             for(int j = 0; j < results_log.size(); ++j)
//             {
//                 if(results_log[j].size() > num_it)
//                 {
//                     iteration = std::get<0>(results_log[j][num_it]);
//                     cost += std::get<1>(results_log[j][num_it]);
//                     makespan += std::get<2>(results_log[j][num_it]);
//                     conflicts += std::get<3>(results_log[j][num_it]);
//                     ++num_reached_timestep;
//                     done = false;
//                 }
//             }
//             if(!done)
//             {
//                 average_results_log.push_back(std::make_tuple(num_it, cost/double(num_reached_timestep), makespan/double(num_reached_timestep), conflicts/double(num_reached_timestep)));
//             }
//             ++num_it;
//         }

//         //Save average results log
//         std::ofstream results_file;
//         results_file.open(intensification_fname + "_results_log_average_alpha_" + std::to_string((int)as_params.alpha) + "_beta_" + std::to_string((int)as_params.beta) + "_tuning_" + data_extension);
//         for(int k = 0; k < average_results_log.size(); ++k)
//         {
//             results_file << std::get<0>(average_results_log[k]) << ", " << std::get<1>(average_results_log[k]) << ", " << std::get<2>(average_results_log[k]) << ", " << std::get<3>(average_results_log[k]) << std::endl;
//         }

//         average_edge_map = average_edge_map/number_repitions;
//         std::string fname_average = intensification_fname + "_edge_map_average_alpha_" + std::to_string((int)as_params.alpha) + "_beta_" + std::to_string((int)as_params.beta) + "_tuning_" + image_extension;
//         draw_edge_map(fname_average, instance, average_edge_map);
//         auto bins_after = conflict_locations_to_bins(conflict_locations_to_histogram2d(comulated_conflict_data, number_repitions));
//         auto heatmap_after = create_heatmap(map, bins_after);
//         draw_heatmap(heatmap_after.heatmap, intensification_fname + "conflict_heatmap_after_alpha_" + std::to_string((int)as_params.alpha) + "_beta_" + std::to_string((int)as_params.beta) + "_tuning_" + image_extension);
//         save_heat_ranges(heatmap_after.heatranges, intensification_fname + "conflict_heatmap_ranges_after_alpha_"  + std::to_string((int)as_params.alpha) + "_beta_" + std::to_string((int)as_params.beta) + "_tuning_"  + data_extension);
//         std::cout << "Alpha: " << as_params.alpha << " Beta: " << as_params.beta << std::endl;
//         std::cout << "Average vertex conflicts: " << (double)comulated_conflict_data.conflict_counts.vertex_conflicts  / double(number_repitions) << std::endl;
//         std::cout << "Average same direction edge conflicts: " << (double) comulated_conflict_data.conflict_counts.same_direction_edge_conflicts  / double(number_repitions) << std::endl;
//         std::cout << "Average opposite direction edge conflicts: " << (double) comulated_conflict_data.conflict_counts.opposite_direction_edge_conflicts  / double(number_repitions) << std::endl;
//     });
}

IntensificationTest::ConflictLocations IntensificationTest::conflict_locations(Instance const & instance, int num_agents, EdgeMap * edgemap, double k)
{

    using namespace IntensificationTest;
    std::vector<std::vector<int>> agent_paths(num_agents);
    for(int i = 0; i < num_agents; ++i)
    {
        agent_paths[i] = agent_path(instance, instance.m_start_locations[i], instance.m_goal_locations[i], edgemap, k);
    }
    


    std::vector< std::vector < Position > > agent_positions(agent_paths.size());
    for(int i = 0; i < agent_paths.size(); ++i)
    {
        agent_positions[i].resize(agent_paths[i].size());
        for(int j = 0; j < agent_paths[i].size(); ++j)
        {
            int x = instance.get_col_coordinate(agent_paths[i][j]);
            int y = instance.get_row_coordinate(agent_paths[i][j]);

            agent_positions[i][j] = {x, y};
        }
    }

    std::vector<Conflict> vertex_conflict_locations;
    std::vector<Conflict> same_direction_edge_conflict_locations;
    std::vector<Conflict> opposite_direction_edge_conflict_locations;
    ConflictCounts conflict_counts = count_conflicts(agent_positions, vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations);

    ConflictLocations conflict_locations  = {vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations,  conflict_counts};
    return conflict_locations;

}

IntensificationTest::ConflictCounts IntensificationTest::count_conflicts(const std::vector<std::vector<Position>> &agent_paths,
                    std::vector<Conflict> &vertex_conflict_locations,
                    std::vector<Conflict> &same_direction_edge_conflict_locations,
                    std::vector<Conflict> &opposite_direction_edge_conflict_locations)
{
    using namespace IntensificationTest;
    int num_agents = agent_paths.size();
    ConflictCounts conflict_counts = {0, 0, 0};

    vertex_conflict_locations.reserve(num_agents * num_agents);
    same_direction_edge_conflict_locations.reserve(num_agents * num_agents);
    opposite_direction_edge_conflict_locations.reserve(num_agents * num_agents);

    for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
            const std::vector<Position> &path_i = agent_paths[i];
            const std::vector<Position> &path_j = agent_paths[j];

            int max_timestep = std::min(path_i.size(), path_j.size());
            for (int t = 0; t < max_timestep; ++t) {
                // Check for vertex conflict
                if (path_i[t] == path_j[t]) {
                    vertex_conflict_locations.push_back({path_i[t], t});
                    conflict_counts.vertex_conflicts++;
                }

                // Check for edge conflict (same direction)
                if (t > 0 && (path_i[t - 1] == path_j[t - 1]) && (path_i[t] == path_j[t])) {
                    same_direction_edge_conflict_locations.push_back({path_i[t], t});
                    conflict_counts.same_direction_edge_conflicts++;
                }

                // Check for edge conflict (opposite direction)
                if (t > 0 && (path_i[t - 1] == path_j[t]) && (path_j[t - 1] == path_i[t])) {
                    opposite_direction_edge_conflict_locations.push_back({path_i[t], t});
                    opposite_direction_edge_conflict_locations.push_back({path_j[t], t});
                    conflict_counts.opposite_direction_edge_conflicts++;
                }
            }
        }
    }

    return conflict_counts;
}

IntensificationTest::HeatMap IntensificationTest::create_heatmap(cv::Mat const & map, std::vector<IntensificationTest::Bin> const & bins)
{
    using namespace IntensificationTest;

    cv::Mat heatmap = map.clone();
    std::map<int, int> heatranges;
    int max_count = 0;
    for(int i = 0; i < bins.size(); ++i)
    {
        if(bins[i].count > max_count)
        {
            max_count = bins[i].count;
        }
    }
    //Color bins
    for(int i = 0; i < bins.size(); ++i)
    {
        int x = bins[i].x;
        int y = bins[i].y;
        int count = bins[i].count;
        int heat_value = 255 * count / max_count;
        heatranges[heat_value] = count;
        heatmap.at<cv::Vec3b>(y, x) = cv::Vec3b(255 - heat_value, 255 - heat_value, 255);
    }

    return {heatranges, heatmap};   
}

void IntensificationTest::draw_heatmap(cv::Mat const & heatmap, std::string const & fname_map)
{
    cv::imwrite(fname_map, heatmap);
}

void IntensificationTest::save_heat_ranges(std::map<int, int> const & heatranges, std::string const & fname_heatranges)
{
    std::ofstream ofs(fname_heatranges);
    for(auto it = heatranges.begin(); it != heatranges.end(); ++it)
    {
        ofs << it->first << ", " << it->second << std::endl;
    }
    ofs.close();
}

void IntensificationTest::add_grid(cv::Mat &scaled_image, int scale_factor) {
    int spacing_x = 2 * scale_factor;
    int spacing_y = 2 * scale_factor;
    int rows = scaled_image.cols / spacing_x;
    int cols = scaled_image.rows / spacing_y;

    for (int i = 0; i <= cols; i++) {
        int x = i * spacing_x;
        cv::line(scaled_image, cv::Point(x, 0), cv::Point(x, scaled_image.rows), cv::Scalar(0, 0, 0), 1);
    }
    for (int i = 0; i <= rows; i++) {
        int y = i * spacing_y;
        cv::line(scaled_image, cv::Point(0, y), cv::Point(scaled_image.cols, y), cv::Scalar(0, 0, 0), 1);
    }
}

void IntensificationTest::draw_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, int scale_factor)
{
    // Determine the width and height of the ASCII art
    int square_size = 2;
    int width = instance.m_num_of_cols * square_size;
    int height = instance.m_num_of_rows * square_size;

    // Create a black image with the specified width and height
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0)); 
    for(int i = 0; i < instance.m_my_graph.size(); ++i)
    {
        //Map graph coordinate to left_corner coordinate
        int row = instance.get_row_coordinate(i);
        int col = instance.get_col_coordinate(i);

        int new_row = row * square_size;
        int new_col = col * square_size;

        std::vector<std::pair<int,int>> square_coordinates = {
            std::make_pair(new_row, new_col),
            std::make_pair(new_row, new_col + 1),
            std::make_pair(new_row + 1, new_col),
            std::make_pair(new_row + 1, new_col + 1),
            }; // up, down, left, right coloring scheme

        if(!instance.valid_pos(i))
        {
            for(const auto & sc : square_coordinates)
            {
                if((height <= sc.first) || (width <= sc.second))
                    continue;

                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            }
            continue;
        }


        //Map direction
        std::vector<std::pair<int,int>> nns = {
            std::make_pair(row - 1, col),
            std::make_pair(row + 1, col),
            std::make_pair(row, col - 1),
            std::make_pair(row, col + 1)
        }; // up, down, left, right
        std::vector<int> nn_ids;
        for(const auto & nn : nns)
        {
            if((nn.first < 0) || (nn.second < 0))
                nn_ids.push_back(-1);
            else if(!instance.valid_move(i, instance.linearize_coordinate(nn.first, nn.second)))
                nn_ids.push_back(-1);
            else
                nn_ids.push_back(instance.linearize_coordinate(nn.first, nn.second));
        }

        //Nodes
        Node const* curr = &instance.m_my_graph[i];
        std::vector<Node const*> nn_nodes;
        for(int nn_id : nn_ids)
        {
            if(nn_id != -1)
                nn_nodes.push_back(&instance.m_my_graph[nn_id]);
            else
                nn_nodes.push_back(nullptr);
        }

        //Coloring
        for(int i_sc = 0; i_sc < square_coordinates.size(); ++i_sc)
        {
            const auto & sc = square_coordinates[i_sc];          

            if((!instance.valid_pos(nn_ids[i_sc])) || (nn_ids[i_sc] == -1))
            {
                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 100;
                color[1] = 100;
                color[2] = 100;
            }
            else
            {
                double ph =  pheromones.read(curr,nn_nodes[i_sc]);
                double ph_opp = pheromones.read(nn_nodes[i_sc], curr);
                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 0;
                color[1] = 255 * ph;
                color[2] = 255 * std::max(1 - ph, 0.);
            }
        }
    }
    //Save image
        // Scale the image
    // cv::imwrite(f_name, img);
    cv::Mat scaled_image;//(height*2, width*2, CV_8UC3, cv::Scalar(0, 0, 0)); 
    cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);

    // Call the draw_grid function to draw the grid on the scaled_image
    add_grid(scaled_image, scale_factor);

    // Save the image
    cv::imwrite(f_name, scaled_image);
}

cv::Mat IntensificationTest::create_map(Instance const & instance)
{
    cv::Mat img(instance.m_num_of_rows, instance.m_num_of_cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR
    for (int i = 0; i < instance.m_num_of_rows; i++)
    {
        for (int j = 0; j < instance.m_num_of_cols; j++)
        {
            int pos = instance.linearize_coordinate(i, j);
            bool type = instance.m_my_map[pos];

            if(type == 1)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(100, 100, 100);
                continue;
            }
            else if(type == 0)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    return img;
}

std::vector<IntensificationTest::Bin> IntensificationTest::conflict_locations_to_bins(std::unordered_map< int, std::unordered_map<int, int> > const & histogram2D)
{
    using namespace IntensificationTest;
    //Create bins from histogram2D
    std::vector<Bin> bins;
    for(auto it = histogram2D.cbegin(); it != histogram2D.cend(); ++it)
    {
        int x = it->first;
        for(auto it2 = it->second.cbegin(); it2 != it->second.cend(); ++it2)
        {
            int y = it2->first;
            int value = it2->second;
            Bin bin;
            bin.x = x;
            bin.y = y;
            bin.count = value;
            bins.push_back(bin);
        }
    }
    return bins;
}

std::unordered_map< int, std::unordered_map<int, int> > IntensificationTest::conflict_locations_to_histogram2d(IntensificationTest::ConflictLocations const & conflict_locations, int normalization)
{
    using namespace IntensificationTest;
    std::unordered_map< int, std::unordered_map<int, int> > histogram2D;
    for(int i = 0; i < conflict_locations.vertex_conflict_locations.size(); ++i)
    {
        Conflict const & conflict = conflict_locations.vertex_conflict_locations[i];
        Position const & pos = conflict.position;
        int x = pos.x;
        int y = pos.y;

        //if x and y exists in histogram2D add 1 to the value
        //else add x and y to histogram2D and set value to 1
        if(histogram2D.find(x) != histogram2D.end())
        {
            if(histogram2D[x].find(y) != histogram2D[x].end())
            {
                histogram2D[x][y] += 1;
            }
            else
            {
                histogram2D[x][y] = 1;
            }
        }
        else
        {
            histogram2D[x][y] = 1;
        }
    }
    for(int i = 0; i < conflict_locations.opposite_direction_edge_conflict_locations.size(); ++i)
    {
        Conflict const & conflict = conflict_locations.opposite_direction_edge_conflict_locations[i];
        Position const & pos = conflict.position;
        int x = pos.x;
        int y = pos.y;

        //if x and y exists in histogram2D add 1 to the value
        //else add x and y to histogram2D and set value to 1
        if(histogram2D.find(x) != histogram2D.end())
        {
            if(histogram2D[x].find(y) != histogram2D[x].end())
            {
                histogram2D[x][y] += 1;
            }
            else
            {
                histogram2D[x][y] = 1;
            }
        }
        else
        {
            histogram2D[x][y] = 1;
        }
    }
    //Normalize
    for(auto it = histogram2D.begin(); it != histogram2D.end(); ++it)
    {
        for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
        {
            it2->second /= std::round((double)it2->second/(double)normalization);
        }
    }
    return histogram2D;
}

