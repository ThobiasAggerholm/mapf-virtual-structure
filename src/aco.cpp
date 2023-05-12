#include "../inc/aco.hpp"

#include "../inc/astar.hpp"

#include <cassert>
#include <algorithm>
#include <execution>
#include <limits>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_set>

#define OCCUPATION 0
#define REPLAY_STEP 0
#define ONLINE_PARAMETER_TUNING 1
#define EXECUTION_POLICY_ACO std::execution::seq



void Occupation::add_occupation(int as_index)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_occupations.size() <= as_index)
        m_occupations.resize(as_index+1, 0);

    if(m_occupations[as_index] == 0)
        m_density++;

    m_occupations[as_index]++;
}

bool Occupation::remove_occupation(int as_index)
{
    bool is_removed = false;
    std::lock_guard<std::mutex> lock(m_mutex);
    if((as_index < m_occupations.size()) && (m_occupations[as_index] > 0))
    {
        m_occupations[as_index]--;
        if(m_occupations[as_index] == 0)
            m_density--;
        is_removed = true;
    }
    return is_removed;
}

int Occupation::read_density()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    int density = m_density;
    return density;
}

int Occupation::read_occupation(int as_index)
{
    int occupation = 0;
    std::lock_guard<std::mutex> lock(m_mutex);
    if(as_index < m_occupations.size())
        occupation = m_occupations[as_index];
    return occupation;
}

void Occupation::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::for_each(EXECUTION_POLICY_ACO, std::begin(m_occupations), std::end(m_occupations), [this](int & val)
    {
        val = 0;
    });
    m_density = 0;
}

void ACO_Construction_State::occupy(Node const * node, int group_index, int unit_index)
{
    std::unique_lock<std::mutex> lock(vertex_mutex);
    vertex_occupations[node]; //Ensure exists in table
    lock.unlock();

    vertex_occupations[node].add_occupation(group_index);
}

bool ACO_Construction_State::move(Node const * src, Node const * dst, int group_index, int unit_index)
{
    std::unique_lock<std::mutex> lock1(vertex_mutex);
    auto it_check = vertex_occupations.find(src);
    auto it_end = vertex_occupations.end();
    lock1.unlock();

    if(it_check == it_end)
        return false;

    bool  is_removed = vertex_occupations[src].remove_occupation(group_index);
    if(!is_removed)
        return false;
    
    occupy(dst, group_index, unit_index);
    assert(vertex_occupations.find(dst) != vertex_occupations.end());

    std::unique_lock<std::mutex> lock2(edge_mutex);
    edge_occupations[src][dst]; //Ensure exists in table
    lock2.unlock();

    edge_occupations[src][dst].add_occupation(group_index);

    std::unique_lock<std::mutex> lock3(transition_mutex);
    transition_history[group_index][unit_index] = dst;
    lock3.unlock();

    return true;
}

Occupation* ACO_Construction_State::read_vertex(Node const * node)
{
    std::lock_guard<std::mutex> lock(vertex_mutex);
    auto it_check = vertex_occupations.find(node);
    auto it_end = vertex_occupations.end();

    if(it_check == it_end)
        return nullptr;

    return &(vertex_occupations.at(node));
}

Occupation* ACO_Construction_State::read_edge(Node const * src, Node const * dst)
{
    std::unique_lock<std::mutex> lock1(edge_mutex);
    auto it_check_src = edge_occupations.find(src);
    auto it_end_src = edge_occupations.end();
    lock1.unlock();

    if(it_check_src == it_end_src)
        return nullptr;
    
    std::unique_lock<std::mutex> lock2(edge_mutex);
    auto it_check_dst = edge_occupations[src].find(dst);
    auto it_end_dst = edge_occupations[src].end();
    lock2.unlock();

    if(it_check_dst == it_end_dst)
        return nullptr;

    return &(edge_occupations.at(src).at(dst));
}


void ACO_Construction_State::reset_transitions()
{
    std::lock_guard<std::mutex> lock1(edge_mutex);
    edge_occupations.clear();

    std::lock_guard<std::mutex> lock2(transition_mutex);
    transition_history.clear();
}

void ACO_Construction_State::reset()
{
    std::lock_guard<std::mutex> lock1(vertex_mutex);
    vertex_occupations.clear();

    std::lock_guard<std::mutex> lock2(transition_mutex);
    transition_history.clear();

    std::lock_guard<std::mutex> lock3(edge_mutex);
    edge_occupations.clear();
}


ACO::ACO(const Instance & instance, const ACO_Params & aco_params, AS_Params & ap)
 : m_instance{instance}, m_aco_params{aco_params}, m_as_params{ap}
{
    assert(m_as_params.graph != nullptr);

    // Global pheromone map
    for(int i = 0; i < m_instance.m_my_graph.size(); ++i)
    {
        Node const* node = &m_instance.m_my_graph[i];
        for(auto cit = node->cbegin(); cit != node->cend(); ++cit)
            m_pheromones.write(node, cit->first, m_aco_params.init_pheromone);
    }

    m_best_pheromones = static_cast<const EdgeMap&>(m_pheromones);
    m_ant_systems.reserve(m_aco_params.n_agents);
    std::random_device rd;
    for(int i = 0; i < m_aco_params.n_agents; ++i)
    {
        m_ant_systems.push_back(AS(ap, m_pheromones, rd()));

        Node const* start_node = m_aco_params.starts[i];
        Node const* goal_node = m_aco_params.goals[i];
        m_ant_systems.back().set_missions(start_node, goal_node);
    }

    m_active_as_indices.resize(m_aco_params.n_agents);
    std::iota(m_active_as_indices.begin(), m_active_as_indices.end(), 0);
}


void ACO::run()
{
    m_best_pheromones = static_cast<const EdgeMap&>(m_pheromones);

    int it_ni = 0;
    int it = 0;

    double best_cost = std::numeric_limits<double>::max();
    m_results_log.reserve(m_aco_params.IT_MAX);

    while((it_ni < m_aco_params.IT_NI) && (it < m_aco_params.IT_MAX))
    {
        ++it;
        ++it_ni;

        construct_solutions();
        update_pheromones();
        double new_cost = log_best_solutions(it, it_ni);

        // Show progress
        if(new_cost < best_cost)
        {
            if(m_aco_params.IT_INFO != -1)
                std::cout << "New best score: " << new_cost << " compared to last best score: " << best_cost << std::endl;
            best_cost = new_cost;
            m_best_pheromones = static_cast<const EdgeMap&>(m_pheromones);
            it_ni = 0;

            best_paths.clear();
            for(AS const & as: m_ant_systems)
            {
                Ant const & ant = (as.get_params().elite_selection) ? as.get_elite_ant() : as.get_last_ant_from_population();
                std::vector<Node const*> const & tour = ant.get_tour();
                std::vector<int> tour_indices;
                tour_indices.reserve(tour.size());
                for(Node const * node: tour)
                    tour_indices.push_back(node->get_id());
                best_paths.push_back(std::move(tour_indices));
            }
        }

        if(m_aco_params.IT_INFO != -1)
            if((it % m_aco_params.IT_INFO) == 0)
                std::cout << "Iteration: "<<  it << std::endl;

#if ONLINE_PARAMETER_TUNING
        update_parmeters();
#endif
    }

}



void ACO::construct_solutions()
{
    ACO_Construction_State state;
    std::unordered_map< int, std::pair<double, std::unordered_set < int > > > searching_ants;
    searching_ants.reserve(m_active_as_indices.size());
    //Initialize by setting all ants to their start positions
    std::for_each(EXECUTION_POLICY_ACO, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this, &state, &searching_ants](int & i)
    {
        // Ant system to home
        AS & as = m_ant_systems[i];
        //Searching ants
        auto & ant_indices = as.get_ant_indices();
        searching_ants[i] = std::pair<double, std::unordered_set<int>>(std::numeric_limits<double>::max(), std::unordered_set<int> (ant_indices.begin(), ant_indices.end()));
        //Start and goal node
        Node const* start_node = m_aco_params.starts[i];
        //Reset ants
        //Reset and  set start and  goal node
        as.reset_ants();
        std::for_each(EXECUTION_POLICY_ACO, std::cbegin(as.get_ant_indices()), std::cend(as.get_ant_indices()), [&state, &as, &start_node, &i](int const & i_ant)
        {
            as.get_ant(i_ant).move(*start_node, 0.0);
            state.occupy(start_node, i, i_ant);
        });

        //If elite ant is completed, occupy its start position
        if(as.get_params().elite_selection)
        {
            Ant & elite_ant = as.get_elite_ant();
            if(elite_ant.is_completed())
            {
                elite_ant.set_tour_length(0.);
                if(elite_ant.get_tour().size() > 0)
                {
                    //Occupy elite ant start position
                    state.occupy(elite_ant.get_start(), i, as.get_ant_indices().back() + 1);
                }

            }
        }
    });

    // Construct solutions
    bool are_goals_reached = false;
    //Searching ants
    while(!are_goals_reached)
    {
        state.increment_time_step();
        request_step(state, searching_ants);
        take_step(state, searching_ants);

        state.reset_transitions();
        are_goals_reached = searching_ants.size() == 0;
    }
}


void ACO::update_pheromones()
{
    std::for_each(EXECUTION_POLICY_ACO, std::begin(m_active_as_indices), std::end(m_active_as_indices), [this](int  i)
    {
        AS & as = m_ant_systems[i];
        as.pheromone_update();
    });
    //repair();
}

void ACO::request_step(ACO_Construction_State & state, std::unordered_map< int, std::pair<double, std::unordered_set < int > > > const & searching_ants)
{
    std::mutex mtx;
    // Request move
    std::for_each(EXECUTION_POLICY_ACO, std::cbegin(searching_ants), std::cend(searching_ants), [&](auto & it_as)
    {
        //If all ants are completed, skip
        int i_as = it_as.first;
        AS & as = m_ant_systems.at(i_as);
        std::for_each(EXECUTION_POLICY_ACO, std::cbegin(it_as.second.second), std::cend(it_as.second.second), [&](auto & it_ant)
        {
            int i_ant = it_ant;
            Ant & ant = as.get_ant(i_ant);

            //Get neighbors to current position
            Node const* curr_node = ant.get_current_location();
            std::vector<int> neighbors = m_instance.get_neighbors(curr_node->get_id());

            std::vector<Node const*> neighbors_nodes;
            neighbors_nodes.reserve(neighbors.size());
            
            for(auto i_neighbor : neighbors)
                neighbors_nodes.push_back(&m_instance.m_my_graph[i_neighbor]);

            //Increase probability if distance to goal is smaller
            std::vector<double> heuristic_probability(neighbors.size());
            for(int i_neighbor = 0; i_neighbor < neighbors.size(); ++i_neighbor)
            {
                Node const target_node = ant.get_target();
                Node  const* neighbor_node = neighbors_nodes[i_neighbor];
                double distance_curr_to_target = m_instance.get_manhattan_distance(curr_node->get_id(), target_node.get_id());
                double distance_neighbor_to_target = m_instance.get_manhattan_distance(neighbor_node->get_id(), target_node.get_id());
                heuristic_probability[i_neighbor] = 1.0 / (2.0 - (distance_curr_to_target - distance_neighbor_to_target));
            }
            
            //Make decision on next node
            Node const* next_node = as.decision_rule(i_ant, curr_node, neighbors_nodes, heuristic_probability);
            //Update state with next move
            bool is_moved = state.move(curr_node, next_node, i_as, i_ant);
            assert(is_moved); //Error in code if not true
        });
        if(as.get_params().elite_selection)
        {
            if(as.get_elite_ant().is_completed())
            {
                Ant & elite_ant = as.get_elite_ant();
                int time_step = state.get_time_step();
                auto const & tour = elite_ant.get_tour();
                if((time_step < tour.size()) && (time_step > 0))
                {
                    bool is_moved = state.move(tour.at(time_step-1), tour.at(time_step), i_as, as.get_ant_indices().back() + 1);
                    assert(is_moved); //Error in code if not true
                }
            }
        }
    });
    

    // Resolve moves
    return;
}

void ACO::take_step(ACO_Construction_State & state, std::unordered_map< int, std::pair<double, std::unordered_set < int > > > & searching_ants)
{    
    //Execute ant tranisitions
    std::for_each(EXECUTION_POLICY_ACO, state.begin_history(), state.end_history(), [&](auto it_as)
    {
        int i_as = it_as.first; //Ant system index
        AS & as = m_ant_systems[i_as];
        
        std::mutex mtx;
        std::for_each(EXECUTION_POLICY_ACO, std::begin(it_as.second), std::end(it_as.second), [&](auto it_ant)
        {
            int i_ant = it_ant.first; //Ant index
            if(i_ant == as.get_n_ants()) return; //Elite ant
            Ant & ant = as.get_ant(i_ant); //Ant

            //Get transition
            Node const* curr_node = ant.get_current_location();
            Node const* next_node = it_ant.second;

            //Load traffic of transition
            Occupation* vertex_occupation = state.read_vertex(next_node);
            Occupation* edge_occupation = state.read_edge(curr_node, next_node); // in opposite direction
            Occupation* opposite_edge_occupation = state.read_edge(next_node, curr_node);

            int vertex_conflicts = vertex_occupation->read_density() - 1;
            int edge_conflicts = edge_occupation->read_density() - 1;
            bool opposite_edge_self_occupy = (opposite_edge_occupation != nullptr && 0 < opposite_edge_occupation->read_occupation(i_as)) ? true : false; 
            int opposite_edge_conflicts = (opposite_edge_occupation != nullptr) ? opposite_edge_occupation->read_density() - (int)opposite_edge_self_occupy : 0;

            int number_of_conflicts = vertex_conflicts + edge_conflicts + opposite_edge_conflicts;
        
            //Update cost based on traffic denisty
            double transition_cost = curr_node->at(next_node) + ((double)number_of_conflicts * m_aco_params.conflict_penalty);
            bool not_revisited = ant.move(*next_node, transition_cost);
            std::lock_guard<std::mutex> lck(mtx);
            //Exit if ant is completed
            if(ant.is_completed())
            {
                if(ant.get_tour_length() < searching_ants.at(i_as).first)
                {
                    searching_ants.at(i_as).first = ant.get_tour_length();
                }
                searching_ants.at(i_as).second.erase(i_ant);
            }
            //Exit if ant is worse or equalt to the best solution
            else if(ant.get_tour_length() >= searching_ants.at(i_as).first)
            {
                searching_ants.at(i_as).second.erase(i_ant);
            }
        });

        if(as.get_params().elite_selection)
        {
            if(as.get_elite_ant().is_completed())
            {
                Ant & elite_ant = as.get_elite_ant();
                int time_step = state.get_time_step();
                auto const & tour = elite_ant.get_tour();
                if((time_step < tour.size()) && (time_step > 0))
                {
                    //Get transition
                    Node const* curr_node = tour.at(time_step-1);
                    Node const* next_node = tour.at(time_step);

                    //Load traffic of transition
                    Occupation* vertex_occupation = state.read_vertex(next_node);
                    Occupation* edge_occupation = state.read_edge(curr_node, next_node); // in opposite direction
                    Occupation* opposite_edge_occupation = state.read_edge(next_node, curr_node);

                    int vertex_conflicts = vertex_occupation->read_density() - 1;
                    int edge_conflicts = edge_occupation->read_density() - 1;
                    bool opposite_edge_self_occupy = (opposite_edge_occupation != nullptr && 0 < opposite_edge_occupation->read_occupation(i_as)) ? true : false; 
                    int opposite_edge_conflicts = (opposite_edge_occupation != nullptr) ? opposite_edge_occupation->read_density() - (int)opposite_edge_self_occupy : 0;

                    int number_of_conflicts = vertex_conflicts + edge_conflicts + opposite_edge_conflicts;
                
                    //Update cost based on traffic denisty
                    double transition_cost = curr_node->at(next_node) + ((double)number_of_conflicts * m_aco_params.conflict_penalty);
                    double new_tour_length = elite_ant.get_tour_length() + transition_cost;
                    elite_ant.set_tour_length(new_tour_length);
                }
            }
        }

        std::lock_guard<std::mutex> lck(mtx);
        if(searching_ants.at(i_as).second.empty())
        {
            if(as.get_params().elite_selection)
            {
                if(as.get_elite_ant().is_completed())
                {
                    Ant & elite_ant = as.get_elite_ant();
                    int time_step = state.get_time_step();
                    auto const & tour = elite_ant.get_tour();
                    if(time_step < tour.size() - 1) 
                    {
                        double new_tour_length = elite_ant.get_tour_length() + (tour.size() - 1 - time_step);
                        elite_ant.set_tour_length(new_tour_length);
                    }
                }
            }
            searching_ants.erase(i_as);
        }
    });
}

int ACO::count_conflicts(std::vector<Ant const*> const & ants) const
{
    int num_ants = ants.size();
    int num_conflicts = 0;
    std::vector<int> ants_indices(num_ants);
    std::iota(ants_indices.begin(), ants_indices.end(), 0);
    //Fix  diagonal symmetry
    std::for_each(std::execution::par, ants_indices.cbegin(),  ants_indices.cend(),[&](auto const & ant_indice)
    {
        std::vector<Node const*> const &path_i = ants[ant_indice]->get_tour();
        std::mutex mtx_inner;
        std::for_each(std::execution::par, ants_indices.cbegin() +  ant_indice +  1,  ants_indices.cend(),[&](auto const & ant_indice_inner)
        {
            std::vector<Node const*> const &path_j = ants[ant_indice_inner]->get_tour();
            int max_timestep = std::min(path_i.size(), path_j.size());
            for (int t = 0; t < max_timestep; ++t)
            {
                // Check for vertex conflict
                if (path_i[t]->get_id() == path_j[t]->get_id())
                {
                    std::lock_guard<std::mutex> lck(mtx_inner);
                    ++num_conflicts;
                }
                // Check for edge conflict (opposite direction)
                if (t > 0 && (path_i[t - 1] == path_j[t]) && (path_j[t - 1] == path_i[t]))
                {
                    std::lock_guard<std::mutex> lck(mtx_inner);
                    ++num_conflicts;
                }
            }
        });
    });

    return num_conflicts;
}


double ACO::log_best_solutions(int it, int it_ni)
{
    //Log iter number
    //Compute cost
    //Compute makespan
    //Compute number of conflicts
    bool is_infinite = false;
    double makespan = 0.0;
    int number_of_conflicts = 0;
    double average_entropy = 0;
    double penalized_cost = 0.0;

    //Compute entropy and makespan
    double SOC = 0.0;
    std::vector<Ant const*> latest_best_ants(m_ant_systems.size());
    std::for_each(std::execution::seq, m_active_as_indices.begin(), m_active_as_indices.end(), [&](auto & i_as)
    {
        //Compute cost
        int population_size = m_ant_systems[i_as].get_population_size();
        if(!m_ant_systems[i_as].get_params().elite_selection)
            assert(population_size > 0);
        latest_best_ants[i_as] = ( m_ant_systems[i_as].get_params().elite_selection) ? &m_ant_systems[i_as].get_elite_ant() : &m_ant_systems[i_as].get_last_ant_from_population();

        //Compute makespan
        makespan = std::max(latest_best_ants[i_as]->get_tour_length(), makespan);


        auto & as_log = m_ant_systems[i_as].get_log();
        double last_ant_tour_length = latest_best_ants[i_as]->get_tour_length();
        as_log.add_makespan(last_ant_tour_length);
        SOC += last_ant_tour_length;
    });

    average_entropy /= (double)m_active_as_indices.size();

    //compute number of conflicts
    number_of_conflicts = count_conflicts(latest_best_ants);

    //compute penalized_cost
    penalized_cost = SOC + m_aco_params.conflict_penalty * number_of_conflicts;

    //Log
    m_results_log.add_iteration(it);
    m_results_log.add_makespan(makespan);
    m_results_log.add_conflicts(number_of_conflicts);
    m_results_log.add_entropy(average_entropy);
    m_results_log.add_cost(penalized_cost);

    return penalized_cost;
}

void ACO::update_parmeters()
{
    //Update parameters
    for(int i = 0; i < m_ant_systems.size(); ++i)
    {
        m_ant_systems[i].tune_parameters();
    }
}
