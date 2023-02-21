#include "../inc/as.hpp"
#include <cassert>
#include <bits/stdc++.h>

#include <algorithm>
#include <execution>

bool Ant::move(Node const* next, double cost)
{
    bool not_visited = true;
    if(m_visited.find(next) != m_visited.end())
        not_visited = false;
    else
        m_visited[next] = true;

    m_tour.push_back(next);
    m_tour_length += cost;

    return not_visited;
}

void Ant::wait()
{
    m_tour.push_back(m_tour.back());
    m_tour_length += 1;
}

int Ant::return_home()
{
    m_tour.push_back(m_tour[0]);
    m_is_returned_home = true;
    m_tour_length *= 2;
    return m_tour_length;
}

void Ant::reset()
{
    int previous_size = m_tour.size();
    m_visited.clear();
    m_tour.clear();

    // Memory heuristic
    m_visited.reserve(previous_size);
    m_tour.reserve(previous_size);

    m_tour_length = 0;
    m_found_gold = false;
    m_is_returned_home = false;
}

void Ant::deposit_pheromone(EdgeMap & pheromone_map)
{
    if(m_found_gold == false)
        return;
    double deposit = 1./m_tour_length;
    //Check if first is last
    int i_last_edge = (m_is_returned_home) ? m_tour.size() - 1 : m_tour.size(); // From first to goal without cost of returning home
    for(int i = 1; i < i_last_edge; ++i)
    {   
        Node const* src = m_tour[i-1];
        Node const* dst = m_tour[i]; 
        if(src == dst)
            continue; // Wait move
        pheromone_map.at(src).at(dst) += deposit;
    }
}


AS::AS(const std::vector<Node> & graph, int n_vertices, int n_ants, double alpha, double beta,
         double evaporation_rate, double init_pheromone, EdgeMap * init_choice_info)
: m_alpha{alpha}, m_beta{beta}, m_evaporation_rate{evaporation_rate}, m_init_pheromone{init_pheromone}, m_ants(n_ants)
{
    m_dim_indices.resize(n_vertices);
    std::iota(m_dim_indices.begin(), m_dim_indices.end(), 0);

    m_ant_indices.resize(m_ants.size());
    std::iota(m_ant_indices.begin(), m_ant_indices.end(), 0);
    
    // Build pheromone map from graph
    for(int i = 0; i < graph.size(); ++i)
    {
        Node const* node = &graph[i];


        std::unordered_map<Node const*, double> edges;
        edges.reserve(node->edges.size());
        for(auto cit = node->edges.cbegin(); cit != node->edges.cend(); ++cit)
            edges[cit->first] = init_pheromone;
        
        m_pheromones[node] = std::move(edges);
    }

    if(init_choice_info != nullptr)
    {
        m_init_choice_info = *init_choice_info;
    }
    else
    {
        // Build m_init_choice_info from graph
        for(int i = 0; i < graph.size(); ++i)
        {
            Node const* node = &graph[i];


            std::unordered_map<Node const*, double> edges;
            edges.reserve(node->edges.size());
            for(auto cit = node->edges.cbegin(); cit != node->edges.cend(); ++cit)
                edges[cit->first] = cit->second;
            
            m_init_choice_info[node] = std::move(edges);
        }
    }

    compute_choice_information();
}

Node const* AS::decision_rule(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors)
{
    double sum_probs = 0;
    std::vector<double> selection_props(neighbors.size());
    
    Ant & ant = m_ants[k_ant];

    int i_neighbor = 0;
    for(auto & neighbor : neighbors)
    {
        if(ant.m_visited.find(neighbor) != ant.m_visited.end())
            selection_props[i_neighbor] = 0;
        else
        {
            selection_props[i_neighbor] = m_choice_info[curr][neighbor];
            sum_probs += selection_props[i_neighbor];
        }
        ++i_neighbor;
    }
    
    int j = 0;
    if(sum_probs == 0)
        j = choose_best_next(k_ant, curr, neighbors);
    else
    {
        srand(time(NULL));
        double r = (double(rand()) / double(RAND_MAX) ) * sum_probs;
        
        double p = selection_props[j];
        while(p < r)
        {
            j = j + 1;
            p += selection_props[j];
        }
    }

    return neighbors[j];
}

int AS::choose_best_next(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors)
{
    Ant & ant = m_ants[k_ant];
    double v = 0;
    int nn = 0; // Next neighbor
    int i_neighbor = 0;
    for(auto neighbor : neighbors)
    {
        if(ant.m_visited.find(neighbor) == ant.m_visited.end())
        {
            if(m_choice_info[curr][neighbor] > v)
            {
                nn = i_neighbor;
                v = m_choice_info[curr][neighbor];
            }
        }

        ++i_neighbor;
    }

    return nn;
}

void AS::pheromone_update()
{
    evaporate();
    for(Ant & ant : m_ants)
    {
        ant.deposit_pheromone(m_pheromones); // Before multi threading mutex is needed to avoid data race
    }
    compute_choice_information();
}

void AS::compute_choice_information()
{
    std::for_each(std::execution::par, std::begin(m_init_choice_info), std::end(m_init_choice_info), [this](auto it)
    {
        for(auto cit = m_init_choice_info[it.first].cbegin(); cit != m_init_choice_info[it.first].cend(); ++cit)
        {
            m_choice_info[it.first][cit->first] =  m_init_choice_info[it.first][cit->first] + m_pheromones[it.first][cit->first];
        }
    });
}

void AS::evaporate()
{
    std::for_each(std::execution::par, std::begin(m_pheromones), std::end(m_pheromones), [this](auto it)
    {
        for(auto cit = m_pheromones[it.first].cbegin(); cit != m_pheromones[it.first].cend(); ++cit)
        {
            m_pheromones[it.first][cit->first] *= (1 - m_evaporation_rate);
        }
    });
}




