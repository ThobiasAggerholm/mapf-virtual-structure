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
                std::cout << "Color out of rannge " << red_color << std::endl;
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

    std::string map_fname_in = "../benchmark_data/mapf-map/Berlin_1_256.map"; // For debugging
    std::string map_fname_out = "../output_data/mapf-map/Berlin_1_256_map.png"; // For debugging

    std::string agent_fname = "../benchmark_data/mapf-scen-random/Berlin_1_256-random-1.scen"; // For debugging
    int num_agents = 2; // For debugging

    Instance instance(map_fname_in, agent_fname, num_agents);

    AS_Params as_params;
    as_params.graph = &instance.m_my_graph;
    as_params.n_vertices = instance.m_map_size;
    as_params.n_ants = 20;
    as_params.alpha = 0;
    as_params.beta = 0;
    as_params.init_pheromone = 0.1;
    as_params.evaporation_rate = 0.05;

    ACO aco(instance, num_agents, as_params);
    std::cout << "Running ACO" << std::endl;
    aco.run(300, 10000);
    std::cout << "Finished  ACO" << std::endl;

    // if(!best_path.empty())
    //     instance.map_route_to_image("../output_data/mapf-map/Berlin_1_256_ant_path.png", best_path);

    return 0;
}
