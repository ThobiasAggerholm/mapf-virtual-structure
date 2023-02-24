#include "inc/instance.hpp"
#include "inc/as.hpp"
#include "inc/aco.hpp"

#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <cassert>
#include <bits/stdc++.h>
#include <algorithm>
#include <execution>
#include <chrono>

#include <opencv2/opencv.hpp>

template <typename Func>
void getExecutionTime(const std::string& title, Func func)
{
    const auto sta = std::chrono::steady_clock::now();
    func();
    const std::chrono::duration<double> dur = std::chrono::steady_clock::now()-sta;
    std::cout << title << ": " << dur.count() << " sec. " << std::endl;
}

bool save_to_heat_img(std::string f_name, int n_cols, int n_rows, const std::vector<double> & heat_map)
{
        // Determine the width and height of the ASCII art
    int width = n_cols;
    int height = n_rows;

    // Create a black image with the specified width and height
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
            int pos = width * i + j;

            int red_color = heat_map[pos] * 255;
            if(255 < red_color)
            {
                std::cout << "Color out of range " << red_color << std::endl;
                red_color = 255;
            }

            cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
            color[0] = 0;
            color[1] = 0;
            color[2] = red_color;
        }
    }
	// Save the image
    return cv::imwrite(f_name, img);
}



//Driver function
int main(int argc, char** argv)
{
    // if(argc < 4)
    //     std::cout << "Expected four commandline arguements. Recieved " << argc << std::endl; return 1;
    //std::string  map_fname = argv[1]; // For release
    //std::string  agent_fname = argv[2]; // For release
    //int num_agents = argv[3]; // For release

    std::string map_fname_in = "../benchmark_data/mapf-map/my_map.map"; // For debugging
    std::string map_fname_out = "../output_data/mapf-map/my_map.png"; // For debugging

    std::string agent_fname = "../benchmark_data/mapf-scen-random/my_scene.scen"; // For debugging
    int num_agents = 2; // For debugging

    Instance instance(map_fname_in, agent_fname, num_agents);

    AS_Params as_params;
    as_params.graph = &instance.m_my_graph;
    as_params.n_vertices = instance.m_map_size;
    as_params.n_ants = 25;
    as_params.alpha = 1;
    as_params.beta = 5;
    as_params.init_pheromone = 0.001;
    as_params.evaporation_rate = 0.8;

    ACO aco(instance, num_agents, as_params);
    std::cout << "Running ACO" << std::endl;
    aco.run(5000, 10000);
    std::cout << "Finished  ACO" << std::endl;

    std::vector<int> best_path_0(aco.m_as_paths[0].size());
    for(int i = 0; i < best_path_0.size(); ++i)
        best_path_0[i] = aco.m_as_paths[0][i]->vertex_id;

    std::vector<int> best_path_1(aco.m_as_paths[1].size());
    for(int i = 0; i < best_path_1.size(); ++i)
        best_path_1[i] = aco.m_as_paths[1][i]->vertex_id;

    if(!best_path_0.empty())
        instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_0.png", best_path_0);

    if(!best_path_1.empty())
        instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_1.png", best_path_1);

    std::vector<int> max_path_0;
    max_path_0.reserve(aco.m_as_paths[0].size());

    auto const & pheromone_map_0 = aco.m_ant_systems[0].m_pheromones;
    int pos = instance.m_start_locations[0];
    max_path_0.push_back(pos);
    while(pos != instance.m_goal_locations[0])
    {
        Node const* curr = &instance.m_my_graph[pos];
        std::vector<int> ns = instance.get_neighbors(pos);
        double max_pheromone = 0;
        int next = ns[0];
        for(int i = 0; i < ns.size(); ++i)
        {
            Node const* cand = &instance.m_my_graph[ns[i]];
            if(cand->vertex_id == instance.m_goal_locations[0])
            {
                next = cand->vertex_id;
                break;
            }
            if(max_pheromone < pheromone_map_0.at(curr).at(cand))
            {
                max_pheromone = pheromone_map_0.at(curr).at(cand);
                next = cand->vertex_id;
            }
        }
        pos = next;
        max_path_0.push_back(pos);
    }
    instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_0_max.png", max_path_0);

    std::vector<int> max_path_1;
    max_path_1.reserve(aco.m_as_paths[1].size());

    auto const & pheromone_map_1 = aco.m_ant_systems[1].m_pheromones;
    pos = instance.m_start_locations[1];
    max_path_1.push_back(pos);
    while(pos != instance.m_goal_locations[1])
    {
        Node const* curr = &instance.m_my_graph[pos];
        std::vector<int> ns = instance.get_neighbors(pos);
        double max_pheromone = 0;
        int next = ns[0];
        for(int i = 0; i < ns.size(); ++i)
        {
            Node const* cand = &instance.m_my_graph[ns[i]];
            if(cand->vertex_id == instance.m_goal_locations[1])
            {
                next = cand->vertex_id;
                break;
            }
            if(max_pheromone < pheromone_map_1.at(curr).at(cand))
            {
                max_pheromone = pheromone_map_1.at(curr).at(cand);
                next = cand->vertex_id;
            }
        }
        pos = next;
        max_path_1.push_back(pos);
    }
    instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_1_max.png", max_path_1);


    return 0;
}
