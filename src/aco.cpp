#include "../inc/aco.hpp"

#include <cassert>
#include <algorithm>
#include <execution>
#include <limits>
#include <iostream>
#include <opencv2/opencv.hpp>

#define OCCUPATION 1

// void log_pheromones(std::vector<double> & map, EdgeMap & pheromones)
// {
//     for(auto it_vertex = pheromones.begin(); it_vertex != pheromones.end(); ++it_vertex)
//     {
//         for(auto it_edge = it_vertex->second.begin();  it_edge != it_vertex->second.end(); ++it_edge)
//         {
//             map.at(it_edge->first->vertex_id) += it_edge->second.second;
//         }
//     }
// }

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
 : m_instance{instance}, m_a_star(instance), m_as_params{ap}
{
    assert(ap.graph != nullptr);

    // Global pheromone map
    for(int i = 0; i < ap.graph->size(); ++i)
    {
        Node const* node = &(*ap.graph)[i];
        for(auto cit = node->edges.cbegin(); cit != node->edges.cend(); ++cit)
            m_pheromones.write(node, cit->first, ap.init_pheromone);
    }

    m_ant_systems.reserve(n_ant_systems);
    for(int i = 0; i < n_ant_systems; ++i)
        m_ant_systems.push_back(AS(*ap.graph, ap.n_vertices, ap.n_ants, ap.alpha, ap.beta, ap.init_pheromone, ap.max_pheromone, ap.q0, ap.K, ap.deposit, m_pheromones, ap.init_choice_info));
    
#if OCCUPATION
    for(int i = 0; i < ap.graph->size(); ++i)
    {
        Node const* node = &(*ap.graph)[i];
        m_vertex_locks[node];
        for(auto cit = node->edges.cbegin(); cit != node->edges.cend(); ++cit)
            m_edge_locks[node][cit->first];
    }
#endif

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

#if OCCUPATION
    for(int i = 0; i < ap.graph->size(); ++i)
    {
        Node const* node = &(*ap.graph)[i];
        m_node_occupation[node] = std::make_pair(-1, 0);
    }
#endif

    // For solution logging
    m_as_scores.resize(m_ant_systems.size(), m_instance.m_my_map.size());
    m_as_paths.resize(m_ant_systems.size());

}


void ACO::run(int IT_NI, int IT_MAX)
{
    // std::vector< std::vector < double > > pheromone_maps;
    // pheromone_maps.push_back(std::vector<double>(m_instance.m_map_size, 0));
    // log_pheromones(pheromone_maps.back(), m_ant_systems[0].m_pheromones);

    m_best_pheromones = static_cast<const EdgeMap&>(m_pheromones);

    int it_ni = 0;
    int it = 0;
    //std::vector<double> last_best_scores(m_ant_systems.size(), std::numeric_limits<double>::max());
    double best_score = std::numeric_limits<double>::max();
    while((it_ni <= IT_NI) && (it < IT_MAX))
    {
        ++it;
        ++it_ni;
        construct_solutions();
        update_pheromones();
        log_best_solutions();

        // Show progress
        std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this, &best_score, &it_ni](int i_as){
            double score = std::accumulate(m_as_scores.begin(), m_as_scores.end(), 0);
            if(score < 0)
                bool test = true;
            if(score < best_score)
            {
                std::cout << "New best score: " << score << " compared to last best score: " << best_score << std::endl;
                best_score = score;
                m_best_pheromones =  static_cast<const EdgeMap&>(m_pheromones);
                it_ni = 0;
            }
        });
        if((it % (IT_MAX/10)) == 0)
            std::cout << "Iteration: "<<  it << std::endl;
    }

}

void ACO::repair()
{
    std::for_each(std::execution::par, m_pheromones.cbegin(), m_pheromones.cend(), [this](auto it)
    {
        for(auto cit = it.second.cbegin(); cit != it.second.cend(); ++cit)
        {
            double ph_val = m_pheromones.read(it.first, cit->first);
            ph_val = (ph_val < m_as_params.min_pheromone) ? m_as_params.min_pheromone : ph_val;
            ph_val = (m_as_params.max_pheromone < ph_val) ? m_as_params.max_pheromone : ph_val;
            m_pheromones.write(it.first, cit->first, ph_val);
        }
    });
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

#if OCCUPATION
        assert(m_vertex_locks.at(start_node).try_lock());
        m_node_occupation[start_node].first = i;
        m_node_occupation[start_node].second = ants.size();
        m_vertex_locks.at(start_node).unlock();
#endif
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
            as.m_ants[i_ant].return_home();
        });

    });

#if OCCUPATION
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
#endif
}

void ACO::update_pheromones()
{
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int  i)
    {
        AS & as = m_ant_systems[i];
        as.pheromone_update();
    });
    repair();
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int  i)
    {
        AS & as = m_ant_systems[i];
        as.compute_choice_information();
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
                Node const* curr_node = ant.m_tour.back();
                std::vector<int> neighbors = m_instance.get_neighbors(curr_node->vertex_id);

                std::vector<Node const*> neighbors_nodes;
                neighbors_nodes.reserve(neighbors.size());
                
                for(auto i_neighbor : neighbors)
                    neighbors_nodes.push_back(&m_instance.m_my_graph[i_neighbor]);

                // N obstacles around J
                // std::vector<double> stimulated_probability(neighbors.size());
                // for(int i_neighbor = 0; i_neighbor < neighbors.size(); ++i_neighbor)
                // {
                //     int n_obs = 4 - m_instance.get_neighbors(neighbors[i_neighbor]).size();
                //     stimulated_probability[i_neighbor] = double(get_combinations(4 - n_obs - 1, 1))/double(get_combinations(4, n_obs));
                // }
                
                //Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes, &heuristics, &stimulated_probability);
                Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes);

                //Check if move is possible
#if OCCUPATION
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
#else
                double transition_cost = m_instance.m_my_graph.at(curr_node->vertex_id).edges.at(next_node);
                bool not_revisited = ant.move(next_node, transition_cost);
#endif


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
