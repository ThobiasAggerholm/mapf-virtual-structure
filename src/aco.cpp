#include "../inc/aco.hpp"

#include <cassert>
#include <algorithm>
#include <execution>
#include <limits>
#include <iostream>

ACO::ACO(const Instance & instance, int n_ant_systems, const AS_Params & ap)
 : m_instance{instance}
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
    int it_ni = 0;
    int it = 0;
    double last_best_score = std::numeric_limits<double>::max();
    while((it_ni <= IT_NI) && (it < IT_MAX))
    {
        ++it;
        ++it_ni;
        construct_solutions();
        update_pheromones();
        log_best_solutions();
        double it_best_score = *std::min_element(m_as_scores.begin(), m_as_scores.end());
        if(it_best_score < last_best_score)
        {
            std::cout << "New best score: " << it_best_score << " - COmpared to last best score: " << last_best_score << std::endl;
            last_best_score = it_best_score;
            it_ni = 0;
        }
        std::cout << "Iteration: "<<  it << std::endl;
    }
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
    int n_max_steps = 1000;
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
        std::for_each(std::execution::par, std::begin(as.m_ant_indices), std::end(as.m_ant_indices), [&as](int i_ant){
            as.m_ants[i_ant].return_home();
        });

    });

    //Clean up vertex mutexes
    std::for_each(std::execution::par, std::begin(m_vertex_locks), std::end(m_vertex_locks),  [](auto &it){
        it.second.unlock();
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
    std::for_each(std::execution::par, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this, &mtx, &are_goals_reached](int & i)
    {
        AS & as = m_ant_systems[i];
        std::mutex mtx_ant;
        bool is_as_goal_reached = true;
        std::for_each(std::execution::par, std::begin(as.m_ant_indices), std::end(as.m_ant_indices), [this, &as, &mtx_ant, &is_as_goal_reached, i](int & i_ant)
        {
            Ant & ant = as.m_ants[i_ant];
            int goal = m_instance.m_goal_locations[i];
            if(!ant.m_found_gold)
            {
                Node const* curr_node =ant.m_tour.back();

                std::vector<int> neighbors = m_instance.get_neighbors(curr_node->vertex_id);
                std::vector<Node const*> neighbors_nodes;
                neighbors_nodes.reserve(neighbors.size());
                for(auto i_neighbor : neighbors)
                    neighbors_nodes.push_back(&m_instance.m_my_graph[i_neighbor]);

                Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes);

                //If vertex is free
                    //If edge is free
                        //Move
                //Else 
                    //Wait

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

    //Clean up edge mutexes
    // std::for_each(std::execution::par, m_edge_locks.begin(), m_edge_locks.end(), [](auto & edges)
    // {
    //     for(auto it_dst = edges.second.begin(); it_dst != edges.second.end();  ++it_dst)
    //     {
    //         it_dst->second.unlock();
    //     }
    // });
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
