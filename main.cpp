#include "inc/instance.hpp"
#include "inc/as.hpp"
#include "inc/aco.hpp"
#include "inc/astar_environment.hpp"

#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <cassert>
#include <bits/stdc++.h>
#include <algorithm>
#include <execution>
#include <chrono>
#include <utility>

#include <opencv2/opencv.hpp>

#include "test_files/inc/intensification_test.hpp"
#include "test_files/inc/global_pheromone_test.hpp"

template <typename Func>
void getExecutionTime(const std::string& title, Func func)
{
    const auto sta = std::chrono::steady_clock::now();
    func();
    const std::chrono::duration<double> dur = std::chrono::steady_clock::now()-sta;
    std::cout << title << ": " << dur.count() << " sec. " << std::endl;
}



bool draw_edge_map(std::string f_name, const Instance & instance, EdgeMap & pheromones)
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
                if((width <= sc.first) || (height <= sc.second))
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
    int scale_factor = 12;
    cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);

    // Calculate the spacing between grid lines
    int spacing_x = 2*scale_factor;
    int spacing_y = 2*scale_factor;

    // Define the number of rows and columns for the grid
    int rows = scaled_image.cols/spacing_x;
    int cols = scaled_image.rows/spacing_y;

    // Loop through the rows and columns and draw lines to form the grid
    for (int i = 0; i <= cols; i++) {
        int x = i * spacing_x;
        cv::line(scaled_image, cv::Point(x, 0), cv::Point(x, scaled_image.rows), cv::Scalar(0, 0, 0), 1);
    }
    for (int i = 0; i <= rows; i++) {
        int y = i * spacing_y;
        cv::line(scaled_image, cv::Point(0, y), cv::Point(scaled_image.cols, y), cv::Scalar(0, 0, 0), 1);
    }

    return cv::imwrite(f_name, scaled_image);
}

void redraw_heatmap(std::string fname, std::string fname_new, std::vector<std::pair<cv::Vec3b, cv::Vec3b>> const & mappings)
{
    // Load the image
    cv::Mat image = cv::imread(fname);

    // Iterate over each pixel in the image
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            cv::Vec3b& pixel = image.at<cv::Vec3b>(i, j);

            // Check if the pixel color matches any of the old colors
            for(auto mapping : mappings)
            {
                if(pixel == mapping.first)
                    pixel = mapping.second;
            }
        }
    }

    // Display the modified image
    cv::imwrite(fname_new, image);
}

std::vector<std::vector<int>> generateRanges(int start, int numGroups, int interval) {
    std::vector<std::vector<int>> ranges;
    for (int i = 0; i < numGroups; ++i) {
        std::vector<int> range;
        for (int j = 0; j < interval; ++j) {
            range.push_back(start + i * interval + j);
        }
        ranges.push_back(range);
    }
    return ranges;
}

#define TERMINAL 0

//Driver function
int main(int argc, char** argv)
{
#if TERMINAL
    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " arg1 arg2" << std::endl;
        return 1;
    }

    int arg1 = std::stoi(argv[1]);
    int arg2 = std::stoi(argv[2]);

    std::string path_prefix = ""; 
#else
    int arg1 = 0;
    int arg2 = 0;
    std::string path_prefix = "../"; 
#endif
    //IntensificationTest::test_ab();
    //IntensificationTest::test_q0();
    // Define the colors to replace
    // cv::Vec3b oldColor1(255 - 85, 255 - 85, 255);   // Blue (255-0, 255-85, 255)
    // cv::Vec3b oldColor2(255 - 170, 255 - 170, 255);  // Green (255-0, 255-170, 255)
    // cv::Vec3b oldColor3(255 - 255, 255 - 255, 255);  // Cyan (255-0, 255-255, 255)

    // cv::Vec3b newColor1(255 - 63, 255 - 63, 255);   // New Blue (255-0, 255-63, 255)
    // cv::Vec3b newColor2(255 - 127, 255 - 127, 255);  // New Green (255-0, 255-127, 255)
    // cv::Vec3b newColor3(255 - 191, 255 - 191, 255);  // New Cyan (255-0, 255-191, 255)
    // std::vector<std::pair<cv::Vec3b, cv::Vec3b>> mapping = {{oldColor1, newColor1}, {oldColor2, newColor2}, {oldColor3, newColor3}};
    // redraw_heatmap("../test_files/test_data/global_pheromone_map/evaluation/den312d/mission_range5/heatmap_astar_after.png", "../test_files/test_data/global_pheromone_map/evaluation/den312d/mission_range5/heatmap_astar_after_redraw.png", mapping);
    // return 0;

    std::vector<std::string> test_extensions = {"_base", "_elite", "_local", "_dynamic_penalty"};

    std::string map_extension = ".map";
    std::string scenario_extension = ".scen";

    std::vector<std::string> maps = {"den312d", "warehouse-10-20-10-2-1", "ost003d"};
    std::vector<std::string> scenarios = {"den312d-random-1", "warehouse-10-20-10-2-1-random-1", "ost003d-random-1"};
    std::vector<std::vector<int>> mission_ranges_training = generateRanges(0, 5, 30);
    std::vector<std::vector<int>> mission_ranges_evaluation = generateRanges(150, 28, 30);
    std::vector<int> all_missions(999);
    std::iota(all_missions.begin(), all_missions.end(), 0);
    int repetitions = 2;

    bool astar_before = false;
    std::mutex mutex;

    std::vector<int> maps_indexes = {0, 1, 2};
    std::vector<int> test_extension_range = {0, 1, 2, 3};
    std::for_each(std::execution::par, test_extension_range.begin(), test_extension_range.end(), [&](int t_e)
    {
        bool elite = false;
        bool local = false;
        bool dynamic_penalty = false;
        std::string test_extension = test_extensions[t_e];
        if(t_e == 1)
            elite = true;
        else if(t_e == 2)
            local = true;
        else if(t_e == 3)
            dynamic_penalty = true;
            
        std::for_each(std::execution::par, maps_indexes.begin(), maps_indexes.end(), [&](int maps_idx)
        {
            std::string map_name = maps[maps_idx];
            std::string scenario_name = scenarios[maps_idx];

            try{
                GlobalPheromoneTester global_pheromone_tester;

                global_pheromone_tester.load_missions(all_missions);
                global_pheromone_tester.set_repetitions(repetitions);

                std::string output_file = path_prefix + "test_files/test_data/global_pheromone_map_reuse/evaluation" + test_extension + "/" + map_name + "/";
                global_pheromone_tester.set_output_file(output_file);
                global_pheromone_tester.load_instance(path_prefix + "benchmark_data/mapf-map/" + map_name + map_extension, path_prefix + "benchmark_data/mapf-scen-random/" + scenario_name + scenario_extension);



                AS_Params as_params;
                as_params.elite_selection = elite;
                as_params.n_ants = 20;
                as_params.min_pheromone = 0.001;
                as_params.max_pheromone = 1;
                as_params.K = 6;
                as_params.deposit = ((as_params.max_pheromone- as_params.min_pheromone)/(double((as_params.K + (int)as_params.elite_selection)*2.)));
                as_params.alpha = (local) ? 7. : 4.;
                as_params.beta = 1.;
                as_params.gamma = (local) ? 4. : 0.0;
                as_params.q0 = 0.0;
                as_params.tune_greedy_selection_prob = true;

                ACO_Params aco_params;
                aco_params.conflict_penalty = 1.0;
                aco_params.IT_NI = 100;//100;
                aco_params.IT_MAX = 1000;
                aco_params.IT_INFO = -1;
                aco_params.MAX_STEPS = 500;
                aco_params.increase_penalty = dynamic_penalty;
                aco_params.init_pheromone = (1. - 0.001)/2.;

                global_pheromone_tester.set_default_as_params(as_params);
                global_pheromone_tester.set_default_aco_params(aco_params, 30);

                global_pheromone_tester.enable_tuning(elite);
                
                global_pheromone_tester.run_reuse_evaluate_experiment(mission_ranges_training, mission_ranges_evaluation);

                std::lock_guard<std::mutex> lock(mutex);
                if(!astar_before)
                {
                    astar_before = true;
                    int total_conflicts = 0;
                    int swapping_conflicts = 0;
                    for(auto const & mission_range_evaluation : mission_ranges_evaluation)
                    {
                        ConflictLocations astar_conflict_data_after = conflict_locations(global_pheromone_tester.get_instance(), mission_range_evaluation, nullptr, 0.0);
                        auto conflicts = astar_conflict_data_after.conflict_counts;
                        total_conflicts += conflicts.vertex_conflicts + conflicts.same_direction_edge_conflicts + conflicts.opposite_direction_edge_conflicts;
                        swapping_conflicts += conflicts.opposite_direction_edge_conflicts;
                    }
                    //write to file
                    std::ofstream file("../test_files/test_data/global_pheromone_map_reuse/astar_conflicts.csv");
                    file << total_conflicts << "," << swapping_conflicts << std::endl;
                    file.close();
                }

            }
            catch(std::exception& e)
            {
                std::cout << "Failed " << map_name << std::endl;
                std::cout << e.what() << std::endl;
            }
        });
    });

    // std::vector<std::string> test_extensions = {"", "_dynamic_penalty"};

    // std::string map_extension = ".map";
    // std::string scenario_extension = ".scen";

    // std::vector<std::string> maps = {"den312d", "warehouse-10-20-10-2-1", "ost003d"};
    // std::vector<std::string> scenarios = {"den312d-random-1", "warehouse-10-20-10-2-1-random-1", "ost003d-random-1"};
    // std::vector<std::pair<int, int>> mission_ranges = {{0, 29}, {30, 59}, {60, 89}, {90, 119}, {120, 149}};
    // int repetitions = 20;

    // std::vector<int> maps_indexes = {0, 1, 2};
    // std::vector<int> mission_range_indexes = {0,1,2,3,4};
    // std::vector<int> test_extension_range = {1};
    // std::for_each(test_extension_range.begin(), test_extension_range.end(), [&](int t_e)
    // {
    //     bool elite = false;
    //     std::string test_extension = test_extensions[t_e];
    //     if(t_e == 1)
    //         elite = true;
            
    //     std::for_each(std::execution::par, maps_indexes.begin(), maps_indexes.end(), [&](int maps_idx)
    //     {
    //         std::string map_name = maps[maps_idx];
    //         std::string scenario_name = scenarios[maps_idx];

    //         std::for_each(std::execution::par, mission_range_indexes.begin(), mission_range_indexes.end(), [&](int mission_range_idx)
    //         {
    //             try{
    //                 GlobalPheromoneTester global_pheromone_tester;
    //                 std::vector<int> missions(30); 
    //                 auto mission_range = mission_ranges[mission_range_idx];
    //                 std::iota(missions.begin(), missions.end(), mission_range.first);
    //                 global_pheromone_tester.load_missions(missions);
    //                 global_pheromone_tester.set_repetitions(repetitions);

    //                 int file_mission_range_idx = mission_range_idx + 1;
    //                 std::string output_file = path_prefix + "test_files/test_data/global_pheromone_map/evaluation" + test_extension + "/" + map_name + "/mission_range" + std::to_string(file_mission_range_idx) + "/";
    //                 global_pheromone_tester.set_output_file(output_file);
    //                 global_pheromone_tester.load_instance(path_prefix + "benchmark_data/mapf-map/" + map_name + map_extension, path_prefix + "benchmark_data/mapf-scen-random/" + scenario_name + scenario_extension);

    //                 AS_Params as_params;
    //                 as_params.elite_selection = elite;
    //                 as_params.n_ants = 20;
    //                 as_params.min_pheromone = 0.001;
    //                 as_params.max_pheromone = 1;
    //                 as_params.K = 6;
    //                 as_params.deposit = ((as_params.max_pheromone- as_params.min_pheromone)/(double((as_params.K + (int)as_params.elite_selection)*2.)));
    //                 as_params.alpha = 7.;
    //                 as_params.beta = 1.;
    //                 as_params.gamma = 4.;
    //                 as_params.q0 = 0.0;
    //                 as_params.tune_greedy_selection_prob = true;

    //                 ACO_Params aco_params;
    //                 aco_params.conflict_penalty = 1.0;
    //                 aco_params.IT_NI = 100;//100;
    //                 aco_params.IT_MAX = 1000;
    //                 aco_params.IT_INFO = -1;
    //                 aco_params.MAX_STEPS = 500;
    //                 aco_params.increase_penalty = true;
    //                 aco_params.init_pheromone = (1. - 0.001)/2.;

    //                 global_pheromone_tester.set_default_as_params(as_params);
    //                 global_pheromone_tester.set_default_aco_params(aco_params);

    //                 global_pheromone_tester.enable_tuning(elite);
                    
    //                 global_pheromone_tester.run_basic_evaluate_experiment();

    //                 cv::Mat map = create_map(global_pheromone_tester.get_instance());
    //                 draw_missions(map, global_pheromone_tester.get_instance(), missions);
    //                 cv::imwrite(output_file + "map_missions.png", map);
    //             }
    //             catch(std::exception& e)
    //             {
    //                 std::cout << "Failed " << map_name << " with mission range " << mission_range_idx + 1 << std::endl;
    //                 std::cout << e.what() << std::endl;
    //             }
    //         });
    //     });
    // });
        // GlobalPheromoneTester global_pheromone_tester;
        // std::vector<int> missions(max_mission); 
        // std::iota(missions.begin(), missions.end(), 0);
        // global_pheromone_tester.load_missions(missions);
        // global_pheromone_tester.set_repetitions(20);

        // global_pheromone_tester.set_output_file("../test_files/test_data/global_pheromone_map/" + map_name + "/");
        // global_pheromone_tester.load_instance("../benchmark_data/mapf-map/" + map_name + map_extension, "../benchmark_data/mapf-scen-random/" + scenario_name + scenario_extension);

        // AS_Params as_params;
        // as_params.elite_selection = false;
        // as_params.n_ants = 20;
        // as_params.min_pheromone = 0.001;
        // as_params.max_pheromone = 1;
        // as_params.K = 6;
        // as_params.deposit = ((as_params.max_pheromone- as_params.min_pheromone)/(double((as_params.K + (int)as_params.elite_selection)*2.)));
        // as_params.alpha = 4.;
        // as_params.beta = 1.;
        // as_params.q0 = 0.0;
        // as_params.tune_greedy_selection_prob = true;

        // ACO_Params aco_params;
        // aco_params.conflict_penalty = 1.0;
        // aco_params.IT_NI = 100;//100;
        // aco_params.IT_MAX = 1000;
        // aco_params.IT_INFO = -1;
        // aco_params.MAX_STEPS = 500;

        // global_pheromone_tester.set_default_as_params(as_params);
        // global_pheromone_tester.set_default_aco_params(aco_params);

        // // global_pheromone_tester.set_alpha_values({2., 3., 4., 5., 6.});
        // // global_pheromone_tester.set_beta_values({1., 2., 3.});
        
        
        // // global_pheromone_tester.set_q0(0.0);
        // global_pheromone_tester.enable_tuning(true);

        // // global_pheromone_tester.set_alpha_values({ 4.});
        // global_pheromone_tester.set_beta_values({1.});

        // std::vector<int> mission_idxs(30);
        // std::iota(mission_idxs.begin(), mission_idxs.end(), 0);
        // cv::Mat map = create_map(global_pheromone_tester.get_instance());
        // draw_missions(map, global_pheromone_tester.get_instance(), mission_idxs);
        // cv::imwrite("../test_files/test_data/global_pheromone_map/den312d/omega_6/map_missions_mapf.png", map);

        //global_pheromone_tester.run_astar_experiment();
        // global_pheromone_tester.set_k_values({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
        // global_pheromone_tester.run_retention_experiment();
        // global_pheromone_tester.set_omega_values({1, 5, 10, 15, 20, 25, 30, 35, 40, 45});
        // global_pheromone_tester.run_omega_experiment();

        /*
            Evaluate full algorithm:
                1) Multiple Environments (The three mentioned)
                2) Different sets of missions
                3) Heatmap Average
                4) Costs
                5) Number of conflicts
                6) Conflict types
                7) Arrow map for path structure

        */
        // global_pheromone_tester.write_output();

        
    return 0;

//     // if(argc < 4)
//     //     std::cout) << "Expected four commandline arguements. Recieved " << argc << std::endl; return 1;
//     //std::string  map_fname = argv[1]; // For release
//     //std::string  agent_fname = argv[2]; // For release
//     //int num_agents = argv[3]; // For release

// #define MYMAP 0
// #if MYMAP
//     std::string map_fname_in = "../benchmark_data/mapf-map/my_map.map"; // For debugging
//     std::string map_fname_out = "../output_data/mapf-map/my_map.png"; // For debugging

//     std::string agent_fname = "../benchmark_data/mapf-scen-random/my_scene.scen"; // For debugging
//     int num_agents = 3; // For debugging
// #else
//     std::string map_fname_in = "../benchmark_data/mapf-map/den312d.map"; // For debugging
//     std::string map_fname_out = "../output_data/mapf-map/den312d.png"; // For debugging

//     std::string agent_fname = "../benchmark_data/mapf-scen-random/den312d-random-1.scen"; // For debugging
//     int num_agents = 25; // For debugging
// #endif

//     Instance instance(map_fname_in, agent_fname, num_agents);

//     //Create agents path and check collisions

//     AS_Params as_params;
//     as_params.graph = &instance.m_my_graph;
//     as_params.n_vertices = instance.m_map_size;
//     as_params.n_ants = 25;
//     as_params.alpha = 1;
//     as_params.beta = 1;
//     as_params.min_pheromone = 0.001;
//     as_params.max_pheromone = 1;
//     as_params.q0 = 0.2;
//     as_params.K = 3;
//     as_params.deposit = (as_params.max_pheromone/(double(as_params.K*2.)));

//     ACO_Params aco_params;
//     aco_params.conflict_penalty = 1.0;
//     aco_params.IT_NI = 100;//100;
//     aco_params.IT_MAX = 1000;
//     aco_params.IT_INFO = 10;
//     aco_params.MAX_STEPS = 500;
//     aco_params.n_agents = num_agents;
//     aco_params.starts.resize(num_agents);
//     aco_params.goals.resize(num_agents);
//     aco_params.init_pheromone = 0.5;

//     for(int i = 0; i < num_agents; ++i)
//     {
//         assert(instance.m_start_locations[i] != instance.m_goal_locations[i]);
//         aco_params.goals[i] = &instance.m_my_graph[instance.m_goal_locations[i]];
//         aco_params.starts[i] = &instance.m_my_graph[instance.m_start_locations[i]];
//     }

//     ACO aco(instance, aco_params, as_params);
//     {
//         double k = 0.;
//         std::vector<std::vector<int>> agent_paths(num_agents);
//         for(int i = 0; i < num_agents; ++i)
//         {
//             agent_paths[i] = agent_path(instance, instance.m_start_locations[i], instance.m_goal_locations[i], aco.get_best_pheromone_map(), k);
//         }

//     //Check agents path and check number of collisions
//        //Check agents path and check number of collisions
//         int num_collisions = 0;
//         int last_time_step = 0;
//         for(int i = 0; i < num_agents; ++i)
//         {
//             if(agent_paths[i].size() > last_time_step)
//                 last_time_step = agent_paths[i].size();
//         }
//         int scale_factor = 30;
//         cv::VideoWriter video_writer("/home/thob_agg/bin/mapf-virtual-structure/output_data/videos/input_001.avi", cv::VideoWriter::fourcc('M','J','P','G'), 1, cv::Size(scale_factor * instance.m_num_of_cols, scale_factor * instance.m_num_of_rows));
//         std::cout << "Recording video" << std::endl;
//         cv::Mat img(instance.m_num_of_rows, instance.m_num_of_cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR
//         for (int i = 0; i < instance.m_num_of_rows; i++)
//         {
//             for (int j = 0; j < instance.m_num_of_cols; j++)
//             {
//                 int pos = instance.linearize_coordinate(i, j);
//                 bool type = instance.m_my_map[pos];

//                 if(type == 1)
//                 {
//                     img.at<cv::Vec3b>(i, j) = cv::Vec3b(100, 100, 100);
//                     continue;
//                 }
//                 else if(type == 0)
//                 {
//                     img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
//                 }
//            }
//         }

        
//         cv::Mat scaled_image;
//         cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
//         video_writer.write(scaled_image);
//         for(int time_step = 0; time_step < last_time_step; ++time_step)
//         {
//             cv::Mat new_frame = img.clone();
//             std::unordered_map<int, int>  occupied_vertex;
//             std::unordered_map<int, std::unordered_set<int>>  occupied_edge;
//             for(int i = 0; i < num_agents; ++i)
//             {
//                 if(agent_paths[i].size() <= time_step)
//                     continue;
//                 if(occupied_vertex.find(agent_paths[i][time_step]) == occupied_vertex.end())
//                     occupied_vertex[agent_paths[i][time_step]] = 1;
//                 else
//                     occupied_vertex[agent_paths[i][time_step]] += 1;

//                 if(time_step > 0)
//                     occupied_edge[agent_paths[i][time_step - 1]].insert(agent_paths[i][time_step]);
//             }
//             std::unordered_set<int>  conflict_vertex;
//             std::unordered_map<int, std::unordered_set<int>>  conflict_edge;
//             for(int i = 0; i < num_agents; ++i)
//             {
//                 if(agent_paths[i].size() <= time_step)
//                     continue;

//                 if(occupied_vertex.at(agent_paths[i][time_step]) > 1) //Vertex collision
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);
//                     if(conflict_vertex.find(agent_paths[i][time_step]) == conflict_vertex.end())
//                     {
//                         conflict_vertex.insert(agent_paths[i][time_step]);
//                         ++num_collisions;
//                     }
//                 }
//                 else if((time_step > 0) && (occupied_edge.find(agent_paths[i][time_step]) != occupied_edge.end()) && (occupied_edge.at(agent_paths[i][time_step]).find(agent_paths[i][time_step-1]) != occupied_edge.at(agent_paths[i][time_step]).end())) //Edge collision
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);

//                     conflict_edge[agent_paths[i][time_step - 1]];
//                     conflict_edge[agent_paths[i][time_step - 1]].insert(agent_paths[i][time_step]);
//                     if(conflict_edge.find(agent_paths[i][time_step]) == conflict_edge.end()) //Check
//                         ++num_collisions;
//                     else if(conflict_edge.at(agent_paths[i][time_step]).find(agent_paths[i][time_step - 1]) == conflict_edge.at(agent_paths[i][time_step]).end())
//                         ++num_collisions;
//                 }
//                 else
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
//                 }
//             }

//             cv::Mat scaled_image;
//             cv::resize(new_frame, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
//             video_writer.write(scaled_image);
//         }
//         std::cout << "Number of collisions before ACO: " << num_collisions << std::endl;
//         video_writer.release();
//         std::cout << "Finished recording video" << std::endl;
//     }

    
//     std::cout << "Running ACO" << std::endl;
//     aco.run();
//     std::cout << "Finished  ACO" << std::endl;
//     //Save aco.m_results_log to file
//     std::ofstream results_file;
//     results_file.open("../test_files/test_data/alpha_beta_1_1.csv");
//     auto results_log = aco.get_results_log();
//     for(int i = 0; i < results_log.size(); ++i)
//     {
//         results_file << std::get<0>(results_log[i]) << ", " << std::get<1>(results_log[i]) << ", " << std::get<2>(results_log[i]) << ", " << std::get<3>(results_log[i]) << std::endl;
//     }

//     //Check agents path and check number of collisions


    

//     // std::vector<int> best_path_0(aco.m_as_paths[0].size());
//     // for(int i = 0; i < best_path_0.size(); ++i)
//     //     best_path_0[i] = aco.m_as_paths[0][i]->vertex_id;

//     // std::vector<int> best_path_1(aco.m_as_paths[1].size());
//     // for(int i = 0; i < best_path_1.size(); ++i)
//     //     best_path_1[i] = aco.m_as_paths[1][i]->vertex_id;

//     // if(!best_path_0.empty())
//     //     instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_0.png", best_path_0);

//     // if(!best_path_1.empty())
//     //     instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_1.png", best_path_1);
//     {
//         double k = 1;
//         std::vector<std::vector<int>> agent_paths(num_agents);
//         for(int i = 0; i < num_agents; ++i)
//         {
//             agent_paths[i] = agent_path(instance, aco.m_aco_params.starts[i]->get_id(), aco.m_aco_params.goals[i]->get_id(), aco.get_best_pheromone_map(), k);
//             //instance.map_route_to_image("../output_data/mapf-map/my_map_agent_path_" + std::to_string(i) + ".png", agent_paths[i]);
//         }
//         //Check agents path and check number of collisions
//         int num_collisions = 0;
//         int last_time_step = 0;
//         for(int i = 0; i < num_agents; ++i)
//         {
//             if(agent_paths[i].size() > last_time_step)
//                 last_time_step = agent_paths[i].size();
//         }
//         int scale_factor = 30;
//         cv::VideoWriter video_writer("/home/thob_agg/bin/mapf-virtual-structure/output_data/videos/output_001.avi", cv::VideoWriter::fourcc('M','J','P','G'), 1, cv::Size(scale_factor * instance.m_num_of_cols, scale_factor * instance.m_num_of_rows));
//         std::cout << "Recording video" << std::endl;
//         cv::Mat img(instance.m_num_of_rows, instance.m_num_of_cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR
//         for (int i = 0; i < instance.m_num_of_rows; i++)
//         {
//             for (int j = 0; j < instance.m_num_of_cols; j++)
//             {
//                 int pos = instance.linearize_coordinate(i, j);
//                 bool type = instance.m_my_map[pos];

//                 if(type == 1)
//                 {
//                     img.at<cv::Vec3b>(i, j) = cv::Vec3b(100, 100, 100);
//                     continue;
//                 }
//                 else if(type == 0)
//                 {
//                     img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
//                 }
//         }   }

        
//         cv::Mat scaled_image;
//         cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
//         video_writer.write(scaled_image);
//         for(int time_step = 0; time_step < last_time_step; ++time_step)
//         {
//             cv::Mat new_frame = img.clone();
//             std::unordered_map<int, int>  occupied_vertex;
//             std::unordered_map<int, std::unordered_set<int>>  occupied_edge;
//             for(int i = 0; i < num_agents; ++i)
//             {
//                 if(agent_paths[i].size() <= time_step)
//                     continue;
//                 if(occupied_vertex.find(agent_paths[i][time_step]) == occupied_vertex.end())
//                     occupied_vertex[agent_paths[i][time_step]] = 1;
//                 else
//                     occupied_vertex[agent_paths[i][time_step]] += 1;

//                 if(time_step > 0)
//                     occupied_edge[agent_paths[i][time_step - 1]].insert(agent_paths[i][time_step]);
//             }
//             std::unordered_set<int>  conflict_vertex;
//             std::unordered_map<int, std::unordered_set<int>>  conflict_edge;
//             for(int i = 0; i < num_agents; ++i)
//             {
//                 if(agent_paths[i].size() <= time_step)
//                     continue;

//                 if(occupied_vertex.at(agent_paths[i][time_step]) > 1) //Vertex collision
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);
//                     if(conflict_vertex.find(agent_paths[i][time_step]) == conflict_vertex.end())
//                     {
//                         conflict_vertex.insert(agent_paths[i][time_step]);
//                         ++num_collisions;
//                     }
//                 }
//                 else if((time_step > 0) && (occupied_edge.find(agent_paths[i][time_step]) != occupied_edge.end()) && (occupied_edge.at(agent_paths[i][time_step]).find(agent_paths[i][time_step-1]) != occupied_edge.at(agent_paths[i][time_step]).end())) //Edge collision
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);

//                     conflict_edge[agent_paths[i][time_step - 1]];
//                     conflict_edge[agent_paths[i][time_step - 1]].insert(agent_paths[i][time_step]);
//                     if(conflict_edge.find(agent_paths[i][time_step]) == conflict_edge.end()) //Check
//                         ++num_collisions;
//                     else if(conflict_edge.at(agent_paths[i][time_step]).find(agent_paths[i][time_step - 1]) == conflict_edge.at(agent_paths[i][time_step]).end())
//                         ++num_collisions;
//                 }
//                 else
//                 {
//                     int row = instance.get_row_coordinate(agent_paths[i][time_step]);
//                     int col = instance.get_col_coordinate(agent_paths[i][time_step]);
//                     new_frame.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 0, 0);
//                 }
//             }
//             cv::Mat scaled_image;
//             cv::resize(new_frame, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
//             video_writer.write(scaled_image);
//         }
//         std::cout << "Number of collisions after ACO: " << num_collisions << std::endl;
//         video_writer.release();
//         std::cout << "Finished recording video" << std::endl;
//     }

//     draw_edge_map("../output_data/mapf-map/my_map_aco_edge_map_latest.png", instance, aco.get_pheromone_map());
//     draw_edge_map("../output_data/mapf-map/my_map_aco_edge_map_best.png", instance, aco.get_best_pheromone_map());

//     return 0;
}


//TODO
    /*
        //Only compute choice info for edges that are in the path

        Paper on network flow
        Local optimization of paths
    */