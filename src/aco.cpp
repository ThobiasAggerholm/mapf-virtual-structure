#include "../inc/aco.hpp"

#include <cassert>
#include <algorithm>
#include <execution>
#include <limits>
#include <iostream>
#include <opencv2/opencv.hpp>

void log_pheromones(std::vector<double> & map, EdgeMap & pheromones)
{
    for(auto it_vertex = pheromones.begin(); it_vertex != pheromones.end(); ++it_vertex)
    {
        for(auto it_edge = it_vertex->second.begin();  it_edge != it_vertex->second.end(); ++it_edge)
        {
            map.at(it_edge->first->vertex_id) += it_edge->second;
        }
    }
}

bool save_to_pheromones_img(std::string f_name, int n_cols, int n_rows, const std::vector<double> & pheromones_map)
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

            int red_color = pheromones_map[pos] * 255;
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

ACO::ACO(const Instance & instance, int n_ant_systems, const AS_Params & ap)
 : m_instance{instance}, m_a_star(instance)
{
    assert(ap.graph != nullptr);
    m_ant_systems.reserve(n_ant_systems);
    for(int i = 0; i < n_ant_systems; ++i)
        m_ant_systems.push_back(AS(*ap.graph, ap.n_vertices, ap.n_ants, ap.alpha, ap.beta, ap.evaporation_rate, ap.init_pheromone, ap.init_choice_info));
    
    for(int i = 0; i < ap.graph->size(); ++i)
    {
        Node const* node = &(*ap.graph)[i];
        m_vertex_locks[node];
        for(auto cit = node->edges.cbegin(); cit != node->edges.cend(); ++cit)
            m_edge_locks[node][cit->first];
    }


    assert(instance.m_start_locations.size() == instance.m_goal_locations.size());
    int n_searches = (instance.m_goal_locations.size() < n_ant_systems) ? instance.m_goal_locations.size() : n_ant_systems;
    m_i_goals.resize(n_searches);
    m_i_starts.resize(n_searches);
    for(int i = 0; i < m_i_goals.size(); ++i)
    {
        assert(instance.m_start_locations[i] != instance.m_goal_locations[i]);
        m_i_goals[i] = instance.m_goal_locations[i];
        m_i_starts[i] = instance.m_start_locations[i];
    }

    m_active_as_indices.resize(n_searches);
    std::iota(m_active_as_indices.begin(), m_active_as_indices.end(), 0);

    for(int i = 0; i < ap.graph->size(); ++i)
    {
        Node const* node = &(*ap.graph)[i];
        m_node_occupation[node] = std::make_pair(-1, 0);
    }

    // For solution logging
    m_as_scores.resize(m_ant_systems.size(), std::numeric_limits<double>::max());
    m_as_paths.resize(m_ant_systems.size());

}


void ACO::run(int IT_NI, int IT_MAX)
{
    // std::vector< std::vector < double > > pheromone_maps;
    // pheromone_maps.push_back(std::vector<double>(m_instance.m_map_size, 0));
    // log_pheromones(pheromone_maps.back(), m_ant_systems[0].m_pheromones);

    int it_ni = 0;
    int it = 0;
    std::vector<double> last_best_scores(m_ant_systems.size(), std::numeric_limits<double>::max());
    while((it_ni <= IT_NI) && (it < IT_MAX))
    {
        ++it;
        ++it_ni;
        construct_solutions();
        update_pheromones();

        // if((it % 50) == 0)
        // {
        //     pheromone_maps.push_back(std::vector<double>(m_instance.m_map_size, 0));
        //     log_pheromones(pheromone_maps.back(), m_ant_systems[0].m_pheromones);
        // }

        log_best_solutions();
        std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this, &last_best_scores, &it_ni](int i_as){
            if(m_as_scores[i_as] < last_best_scores[i_as])
            {
                std::cout << "New best score, as " << i_as << ": " << m_as_scores[i_as] << " - Compared to last best score: " << last_best_scores[i_as] << std::endl;
                last_best_scores[i_as] = m_as_scores[i_as];
                it_ni = 0;
            }
        });
        if((it % (IT_MAX/10)) == 0)
            std::cout << "Iteration: "<<  it << std::endl;
    }

    // double highest_pheromone_val = 0;
    // for(int i = 0; i < pheromone_maps.size(); ++i)
    // {
    //     for(int j = 0; j < pheromone_maps[i].size(); ++j)
    //     {
    //         if(highest_pheromone_val < pheromone_maps[i][j])
    //             highest_pheromone_val = pheromone_maps[i][j];
    //     }
    // }
    // for(int i = 0; i < pheromone_maps.size(); ++i)
    // {
    //     for(int j = 0; j < pheromone_maps[i].size(); ++j)
    //     {
    //         pheromone_maps[i][j] = pheromone_maps[i][j] / highest_pheromone_val;
    //     }
    //     std::string fname = "../output_data/pheromone_maps/p_map_" + std::to_string(i) + ".png";
    //     save_to_pheromones_img(fname, m_instance.m_num_of_cols, m_instance.m_num_of_rows, pheromone_maps[i]);

    // }

}


void ACO::construct_solutions()
{
    //Initialize
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int & i)
    {
            // All ants in ant system
        std::vector<Ant> & ants = m_ant_systems[i].m_ants; 

        //Start and goal node
        int i_start = m_i_starts[i];
        int i_goal = m_i_goals[i];
        Node const* start_node = &m_instance.m_my_graph[i_start];

        assert(m_vertex_locks.at(start_node).try_lock());
        m_node_occupation[start_node].first = i;
        m_node_occupation[start_node].second = ants.size();
        m_vertex_locks.at(start_node).unlock();
        std::for_each(std::execution::par, std::begin(ants), std::end(ants), [start_node](Ant & ant)
        {
            ant.reset();
            ant.move(start_node, 0);
        });
    });

    //Search
    int n_max_steps = 500;
    int n_steps = 0;
    bool are_goals_reached = false;
    while((n_steps < n_max_steps) && !are_goals_reached)
    {
        ++n_steps;
        are_goals_reached = take_step();
    }


    //Finalize
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int  i)
    {
        AS & as = m_ant_systems[i];
        std::for_each(std::execution::par, std::begin(as.m_ant_indices), std::end(as.m_ant_indices), [this, &as, &i](int i_ant)
        {
            //Compute A* path for each ant position to goal with no regard for collisions
            // std::vector<Cell> cell_details;
            // bool path_found = m_a_star.search(cell_details, as.m_ants[i_ant].m_tour.back()->vertex_id, m_i_goals[i]);
            // if(path_found)
            // {
            //     std::vector<int> path = m_a_star.trace_path(cell_details, m_i_goals[i]);
            //     assert(path.front() == m_i_goals[i]);
            //     for(int i_path = path.size()-2; i_path >= 0; --i_path)
            //     {
            //         Node const* curr = as.m_ants[i_ant].m_tour.back();
            //         Node const* next = &m_instance.m_my_graph[path[i_path]];
            //         double transition_cost = m_instance.m_my_graph.at(curr->vertex_id).edges.at(next);
            //         as.m_ants[i_ant].move(next, transition_cost);
            //     }
            //     as.m_ants[i_ant].m_found_gold = true;
            // }
            as.m_ants[i_ant].return_home();
        });

    });

    //Clean up vertex mutexes
    std::for_each(std::execution::par, std::begin(m_vertex_locks), std::end(m_vertex_locks),  [](auto &it){
        it.second.unlock();
    });

    //Clean up occupation
    std::for_each(std::execution::par, std::begin(m_node_occupation), std::end(m_node_occupation),  [](auto &it)
    {
        it.second.first = -1;
        it.second.second = 0;
    });
}

void ACO::update_pheromones()
{
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int  i)
    {
        AS & as = m_ant_systems[i];
        as.pheromone_update();
    });
}

bool ACO::take_step()
{
    std::mutex mtx;
    bool are_goals_reached = true;
    // Request move
    
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this, &mtx, &are_goals_reached](int & i)
    {
        AS & as = m_ant_systems[i];
        std::mutex mtx_ant;
        bool is_as_goal_reached = true;
        std::for_each(std::execution::par, std::begin(as.m_ant_indices), std::end(as.m_ant_indices), [this, &as, &mtx_ant, &is_as_goal_reached, i](int & i_ant)
        {
            Ant & ant = as.m_ants[i_ant];
            int start = m_instance.m_start_locations[i];
            int goal = m_instance.m_goal_locations[i];

            if(!ant.m_found_gold)
            {
                Node const* curr_node =ant.m_tour.back();

                std::vector<int> neighbors = m_instance.get_neighbors(curr_node->vertex_id);
                std::vector<Node const*> neighbors_nodes;
                neighbors_nodes.reserve(neighbors.size());
                for(auto i_neighbor : neighbors)
                    neighbors_nodes.push_back(&m_instance.m_my_graph[i_neighbor]);

                // // N obstacles around J
                // std::vector<double> stimulated_probability(neighbors.size());
                // for(int i_neighbor = 0; i_neighbor < neighbors.size(); ++i_neighbor)
                // {
                //     int n_obs = 4 - m_instance.get_neighbors(neighbors[i_neighbor]).size();
                //     stimulated_probability[i_neighbor] = double(get_combinations(4 - n_obs - 1, 1))/double(get_combinations(4, n_obs));
                // }
                
                //Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes, &heuristics, &stimulated_probability);
                Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes);

                //Check if move is possible
                bool is_moved = false;
                m_vertex_locks[next_node].lock();
                if((m_node_occupation[next_node].first == i) || (m_node_occupation[next_node].first == -1))
                {
                    double transition_cost = m_instance.m_my_graph.at(curr_node->vertex_id).edges.at(next_node);
                    bool not_revisited = ant.move(next_node, transition_cost);
                    is_moved = true;

                    // Take ownership of occupation
                    m_node_occupation[next_node].first = i;
                    m_node_occupation[next_node].second += 1;


                    // Remove ownership from previous occupation
                    m_node_occupation[curr_node].second -= 1;
                    if(m_node_occupation[curr_node].second == 0)
                        m_node_occupation[curr_node].first = -1;
                }
                m_vertex_locks[next_node].unlock();
                if(!is_moved)
                    ant.wait();


                if(ant.m_tour.back()->vertex_id == goal)
                {
                    ant.m_found_gold = true;
                }
                else
                {
                    mtx_ant.lock();
                    is_as_goal_reached = false;
                    mtx_ant.unlock();
                }
            }
        });
        mtx.lock();
        are_goals_reached = are_goals_reached && is_as_goal_reached;
        mtx.unlock();
    });

    // Resolve moves
    return are_goals_reached;
}

void ACO::log_best_solutions()
{
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int i_as)
    {
        AS & as = m_ant_systems[i_as];
        double new_best_score = std::numeric_limits<double>::max();
        int i_best_ant = 0;
        for(int i_ant = 0; i_ant < as.m_ants.size(); ++i_ant)
        {
            if(!as.m_ants[i_ant].m_found_gold)
                continue;

            if(as.m_ants[i_ant].m_tour_length < new_best_score)
                new_best_score = as.m_ants[i_ant].m_tour_length;
        }
        if(new_best_score < m_as_scores[i_as])
        {
            m_as_scores[i_as] = new_best_score;
            m_as_paths[i_as] = as.m_ants[i_best_ant].m_tour;
        }
    });
}
