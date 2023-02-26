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

bool agent_path_search(std::vector<Cell> & cell_details, const Instance & instance, int start, int goal, EdgeMap & pheromones, int k)
{
	// If the source is out of range
    if( instance.valid_pos(start) == false )
    {
        return false;
    }

	// If the destination is out of range
    if( instance.valid_pos(goal) == false )
    {
        return false;
    }

	// If the destination cell is the same as source cell
	if ( start == goal )
    {
		return false;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	std::vector<bool> closedList(instance.m_map_size, false);

	// Declare a 2D array of structure to hold the details
	// of that cell
    cell_details.resize(instance.m_map_size);

	for(Cell & cell : cell_details)
    {
        cell.f = INFINITY;
        cell.g = INFINITY;
        cell.h = INFINITY;
        cell.parent_i = -1;
    }

	// Initialising the parameters of the starting node
    cell_details[start].f = 0;
    cell_details[start].g = 0;
    cell_details[start].h = 0;
    cell_details[start].parent_i = start;

	/*
	Create an open list having information as-
	<f, i>
	where f = g + h,
	and i is the index of that cell
	This open list is implemented as a set of pairs
    */
	std::set<std::pair<double, int>> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(std::make_pair(0.0, start));

	// We set this boolean value as false as initially
	// the destination is not reached.
	bool found_dest = false;

	while (!openList.empty())
    {
		auto p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the closed list
        int i = p.second;
		closedList[i] = true;

		// To store the 'g', 'h' and 'f' of the 4 successors
		double gNew, hNew, fNew;
        std::vector<int> successors = instance.get_neighbors(i);
        for(const int & successor : successors)
        {
            if(successor == goal)
            {
                cell_details[successor].parent_i = i;
                found_dest = true;
                return found_dest;
            }
            // If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
            else if (closedList[successor] == false)
            {
				gNew = cell_details[i].g + instance.get_neighbor_distance(i, successor);
                Node const* curr = &instance.m_my_graph[i];
                Node const* next = &instance.m_my_graph[successor];
				hNew = instance.get_manhattan_distance(successor, goal) - (pheromones.read(curr,next) * k) + (pheromones.read(next, curr) * k);
                // Transition cost from q to p
                // Heuristic of p
                // Penalty -> k * pheromone opposite
                // Reward -> k * pheromone in direction    
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//			 OR
				// If it is on the open list already, check
				// to see if this path to that square is
				// better, using 'f' cost as the measure.
				if ((cell_details[successor].f == FLT_MAX)
					|| (cell_details[successor].f > fNew))
                {
					openList.insert(std::make_pair(
						fNew,successor));

					// Update the details of this cell
					cell_details[successor].f = fNew;
					cell_details[successor].g = gNew;
					cell_details[successor].h = hNew;
					cell_details[successor].parent_i = i;
				}
			}
        }
    }
    return found_dest;
}

std::vector<int> agent_path(const Instance & instance, int start, int goal, EdgeMap & pheromones, int k)
{
    std::vector<Cell> cell_details;
    bool dest_found = agent_path_search(cell_details, instance, start, goal, pheromones, k);
    if(!dest_found)
        return {};
    
    int pos = goal;

	std::vector<int> path;

	while (cell_details[pos].parent_i != pos)
    {
		path.push_back(pos);
		pos = cell_details[pos].parent_i;
	}

	path.push_back(pos);

    return path;
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
    as_params.beta = 1;
    as_params.init_pheromone = 0.001;
    as_params.min_pheromone = 0.5;
    as_params.max_pheromone = 1;
    as_params.evaporation_rate = 0.8;

    ACO aco(instance, num_agents, as_params);
    std::cout << "Running ACO" << std::endl;
    aco.run(1000, 10000);
    std::cout << "Finished  ACO" << std::endl;

    

    // std::vector<int> best_path_0(aco.m_as_paths[0].size());
    // for(int i = 0; i < best_path_0.size(); ++i)
    //     best_path_0[i] = aco.m_as_paths[0][i]->vertex_id;

    // std::vector<int> best_path_1(aco.m_as_paths[1].size());
    // for(int i = 0; i < best_path_1.size(); ++i)
    //     best_path_1[i] = aco.m_as_paths[1][i]->vertex_id;

    // if(!best_path_0.empty())
    //     instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_0.png", best_path_0);

    // if(!best_path_1.empty())
    //     instance.map_route_to_image("../output_data/mapf-map/my_map_ant_path_1.png", best_path_1);

    int k = 1;
    for(int i = 0; i < num_agents; ++i)
    {
        std::vector<int> path = agent_path(instance, instance.m_start_locations[i], instance.m_goal_locations[i], aco.m_best_pheromones, k);
        instance.map_route_to_image("../output_data/mapf-map/my_map_agent_path_" + std::to_string(i) + ".png", path);
    }
    draw_edge_map("../output_data/mapf-map/my_map_aco_edge_map_latest.png", instance, aco.m_pheromones);
    draw_edge_map("../output_data/mapf-map/my_map_aco_edge_map_best.png", instance, aco.m_best_pheromones);
    //Draw EdgeMap
        //Go through map and make black if not valid
        // Else draw intensity of efach edge in four blocks

    return 0;
}
