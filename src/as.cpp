#include "../inc/as.hpp"
#include <cassert>
#include <bits/stdc++.h>

#include <algorithm>
#include <execution>
#include <cmath>
#include <chrono>

#define CONFIDENCE 0
#define EXECUTION_POLICY_AS std::execution::seq


bool Ant::move(Node const& next, double edge_cost)
{
    assert(m_goal != nullptr);

    bool is_previously_visited = false;
    auto target_id = get_target().get_id();

    if(target_id == m_goal->get_id())
    {
        is_previously_visited = m_visited_to_goal.find(next.get_id()) != m_visited_to_goal.end();
        m_visited_to_goal[next.get_id()] = true;
    }
    else if(target_id == m_start->get_id())
    {
        is_previously_visited = m_visited_to_start.find(next.get_id()) != m_visited_to_start.end();
        m_visited_to_start[next.get_id()] = true;
    }
    else
    {
        assert(false);
        return false; // Early return in case of error
    }

    m_tour.push_back(&next);
    m_tour_length += edge_cost;

    if(next.get_id() == m_goal->get_id())
        m_reached_goal = true;
    
    if(m_reached_goal && next.get_id() == m_start->get_id())
        m_reached_start = true;
    
    return is_previously_visited;
}


void Ant::reset()
{
    size_t previous_size = m_tour.size();

    m_visited_to_start.clear();
    m_visited_to_goal.clear();
    m_tour.clear();
    m_pheromone_trails.clear();
    m_tour_length = 0;

    m_reached_start = false;
    m_reached_goal = false; // if the ant found gold then target is m_start

    // Memory heuristic
    m_visited_to_start.reserve(previous_size);
    m_visited_to_goal.reserve(previous_size);
    m_tour.reserve(previous_size);
}

void Ant::remove_trail(EdgeMap &  pheromones)
{
    for(auto & trail : m_pheromone_trails)
    {
        pheromones.add(trail.get_src(), trail.get_dst(), -trail.get_deposit());
        pheromones.add(trail.get_dst(), trail.get_src(), trail.get_deposit());
    }
}

void Ant::set_goal(Node const* goal)
{
    assert(goal != nullptr);
    m_goal = goal;
}

void Ant::set_start(Node const* start)
{
    assert(start != nullptr);
    m_start = start;
}

Node const* Ant::get_goal() const
{
    return m_goal;
}

Node const* Ant::get_start() const
{
    return m_start;
}

Node const& Ant::get_target() const
{
    Node const* target = m_goal;
    assert(target != nullptr);
    return *target;
}
 
Node const* Ant::get_current_location() const
{
    if(m_tour.size() == 0)
        return nullptr;
    else
        return m_tour.back();
}


void Ant::put_trail(EdgeMap &  pheromones, double deposit, double max_pheromone, bool add_trail)
{
    for(size_t i = 0; i < m_tour.size()-2; ++i)  // Last node is home
    {
        Node const* curr = m_tour[i];
        Node const* next = m_tour[i+ 1];

#if CONFIDENCE
        double confidence = get_confidence(pheromones, curr, next, max_pheromone);
        confidence = confidence;

        deposit = deposit * confidence;
#endif
        if(add_trail)
            m_pheromone_trails.push_back(PheromoneTrail(curr, next, deposit));

        pheromones.add(curr, next, deposit); // Positive deposit
        pheromones.add(next, curr, -deposit); // Negative deposit
    }
}

bool Ant::is_visited(Node const & node) const
{
    //auto & m_visited = (!m_reached_goal) ? m_visited_to_goal : m_visited_to_start;
    auto & m_visited = m_visited_to_goal;
    return (m_visited.find(node.get_id()) != m_visited.end());
}

bool Ant::is_completed() const
{
    return (m_reached_goal);
}


double Ant::get_tour_length() const
{
    return m_tour_length;
}

std::unordered_map<int, bool> const & Ant::get_visited_to_goal() const
{
    return m_visited_to_goal;
}

std::unordered_map<int, bool> const & Ant::get_visited_to_start() const
{
    return m_visited_to_start;
}

std::vector<Node const*> const & Ant::get_tour() const
{
    return m_tour;
}

void Ant::set_tour(std::vector<Node const*> const & tour)
{
    m_tour = tour;
}


AS::AS(AS_Params const& params, EdgeMap & pheromone_map, unsigned int seed)
: m_params{params}, m_ants(params.n_ants), m_pheromones_global{pheromone_map}, m_pheromones_local{static_cast<const EdgeMap &>(pheromone_map)},  generator(seed)
{
    m_ant_indices.resize(m_ants.size());
    std::iota(m_ant_indices.begin(), m_ant_indices.end(), 0);

    double equal_prob = (m_params.max_pheromone-m_params.min_pheromone)/2.;
    entropy_max = -(equal_prob*log(equal_prob) *4);
}

void AS::reset_ants()
{
    for(auto & ant : m_ants)
    {
        ant.reset();
    }
}

void AS::set_missions(Node const* start, Node const* goal)
{
    assert(start != nullptr);
    assert(goal != nullptr);
    for(auto & ant : m_ants)
    {
        ant.set_start(start);
        ant.set_goal(goal);
    }
    if(m_params.elite_selection)
    {
        elite_ant.set_start(start);
        elite_ant.set_goal(goal);
    }
}

Ant & AS::get_ant(int k_ant)
{
    assert(k_ant < m_ants.size());
    return m_ants[k_ant];
}

Ant & AS::get_elite_ant()
{
    return elite_ant;
}

Ant const & AS::get_elite_ant() const
{
    return elite_ant;
}

std::vector<int> const & AS::get_dim_indices() const
{
    return m_dim_indices;
}
std::vector<int> const & AS::get_ant_indices() const
{
    return m_ant_indices;
}


Node const* AS::decision_rule(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors, std::vector<double> const & hp)
{
    assert(k_ant < m_ants.size());

    double sum_probs = 0;
    std::vector<double> selection_probs(neighbors.size());
    
    Ant const & ant = ((k_ant == m_ants.size()) && m_params.elite_selection) ? elite_ant : m_ants[k_ant];

    int i_neighbor = 0;
    for(auto & neighbor : neighbors)
    {
        if(ant.is_visited(*neighbor))
            selection_probs[i_neighbor] = 0;
        else
        {
            double hp_ij = (i_neighbor < hp.size()) ? hp[i_neighbor] : 1;
            double pheromone_level_global = std::clamp(m_pheromones_global.read(curr, neighbor), m_params.min_pheromone, m_params.max_pheromone);
            double pheromone_level_local = std::clamp(m_pheromones_local.read(curr, neighbor), m_params.min_pheromone, m_params.max_pheromone);
        
            selection_probs[i_neighbor] = std::pow(pheromone_level_global, m_params.alpha) * std::pow(hp_ij, m_params.beta) * std::pow(pheromone_level_local, m_params.gamma);
            sum_probs += selection_probs[i_neighbor];
        }
        ++i_neighbor;
    }

    // Find the maximum probability
    int max_j = std::distance(selection_probs.begin(), std::max_element(selection_probs.begin(), selection_probs.end()));
    double max_prob = selection_probs[max_j];

    std::uniform_real_distribution<double> distribution(0.0,1.0);
    double q = distribution(generator);

    if(q <= m_params.q0) //If greedy decision 
    {  
        if((max_j == -1) || (sum_probs == 0.))// but equal distribution
        {
            std::uniform_int_distribution<> distribution_int(0, neighbors.size()-1);
            max_j = distribution_int(generator);
        }
            
        return neighbors[max_j];
    }
    
    int j = 0;
    if(sum_probs == 0.)
    {
        j = choose_random_next(k_ant, curr, neighbors);
    }
    else
    {
        double r = distribution(generator) * sum_probs;
        
        double p = selection_probs[j];
        while(p < r)
        {
            j = j + 1;
            p += selection_probs[j];
        }
    }
    return neighbors[j];
}

int AS::choose_random_next(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors)
{
    std::uniform_int_distribution<> distr(0, neighbors.size()-1); // define the range
    int nn = distr(generator);
    return nn;
}

void AS::add_population()
{
    Ant const * best_ant = nullptr;
    for(int i = 0; i < m_ants.size(); ++i)
    {
        if(m_ants[i].is_completed() && (best_ant == nullptr))
            best_ant = &m_ants[i];
        else if(m_ants[i].is_completed() && ( m_ants[i].get_tour_length() < best_ant->get_tour_length()))
            best_ant = &m_ants[i];
    }

    if(best_ant == nullptr)
        return;

    //If ant length is less then elite ant length then replace, else add population
    if( m_params.elite_selection && ( (elite_ant.get_tour_length() > best_ant->get_tour_length()) || !elite_ant.is_completed() ))
    {
        if(elite_ant.is_completed())
        {
            elite_ant.remove_trail(m_pheromones_global);
            elite_ant.remove_trail(m_pheromones_local);
        }

        elite_ant = *best_ant;
        remove_loops(elite_ant);
        elite_ant.put_trail(m_pheromones_global, m_params.deposit, m_params.max_pheromone);
        elite_ant.put_trail(m_pheromones_local, m_params.deposit, m_params.max_pheromone, false);
    }
    else
    {
        // Add new  best ant
        Ant population_ant = *best_ant;
        remove_loops(population_ant);
        m_population.push_back(population_ant);
        m_population.back().put_trail(m_pheromones_global, m_params.deposit, m_params.max_pheromone);
        m_population.back().put_trail(m_pheromones_local, m_params.deposit, m_params.max_pheromone, false);

        //Remove the oldest ant
        if(m_params.K < m_population.size())
        {
            m_population.front().remove_trail(m_pheromones_global);
            m_population.front().remove_trail(m_pheromones_local);
            m_population.pop_front();
        }
    }
}


void AS::pheromone_update()
{
    double last_best = (!m_population.empty()) ? m_population.back().get_tour_length() : std::numeric_limits<double>::max();

    add_population();
    
    ++iter;
    if(!m_population.empty() && (m_population.back().get_tour_length() > last_best))
        iter_ni = 0;
    else
        ++iter_ni;
}


void AS::remove_loops(Ant & ant) const
{
    Node const* start = ant.get_start();
    Node const* goal = ant.get_goal();
    auto const & ant_tour = ant.get_tour();
    if(ant_tour.empty())
        return;

    std::unordered_map<Node const*, int> visited_to_goal;
    std::unordered_map<Node const*, int> visited_to_start;
    populate_visited_maps(visited_to_goal, visited_to_start, ant_tour, goal);

    std::vector<Node const*> new_tour;
    new_tour.reserve(ant_tour.size());
    create_new_tour_without_loops(new_tour, ant_tour, visited_to_goal, visited_to_start, goal);

    ant.set_tour(new_tour);
}
//Fills [start : goal] and ]goal : start] maps
void AS::populate_visited_maps(std::unordered_map<Node const*, int> & visited_to_goal, std::unordered_map<Node const*, int> & visited_to_start, const std::vector<Node const*> & ant_tour, Node const* goal) const
{
    bool is_goal_reached = false;
    std::unordered_map<Node const*, int> * visited = &visited_to_goal;
    for(int i = 0; i < ant_tour.size(); ++i)
    {
        Node const* curr = ant_tour[i];
        (*visited)[curr] = i;

        if(curr == goal)
            is_goal_reached = true;

        visited = (is_goal_reached) ? &visited_to_start : &visited_to_goal;
    }
}

void AS::create_new_tour_without_loops(std::vector<Node const*> & new_tour, std::vector<Node const*> const & ant_tour, std::unordered_map<Node const*, int> const & visited_to_goal, std::unordered_map<Node const*, int> const & visited_to_start, Node const* goal) const
{
    bool is_goal_reached = false;
    for(int i = 0; i < ant_tour.size();)
    {
        Node const* curr = ant_tour.at(i);
        new_tour.push_back(curr);

        if(curr == goal)
            is_goal_reached = true;

        if(i == ant_tour.size()-1)
            break;

        auto const & visited = (is_goal_reached) ? visited_to_start : visited_to_goal;
        for(auto it = curr->cbegin(); it != curr->cend(); ++it) // Loop over neighbors
        {
            if(visited.find(it->first) != visited.end()) // If visited later fast forward
            {
                if(i < visited.at(it->first))
                    i = visited.at(it->first);
            }
        }
    }
}

void AS::tune_parameters()
{
    if(m_params.tune_greedy_selection_prob)
        m_params.q0 = 0.0 + 1./((double)iter_ni + 1.0);
}




