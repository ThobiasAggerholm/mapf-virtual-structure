#pragma once
#include <vector>
#include "as.hpp"
#include <mutex>
#include <unordered_map>
#include <utility>
#include "instance.hpp"
#include "astar.hpp"
#include <unordered_set>
#include "aco_log.hpp"

class Occupation
{
public:
    Occupation() {}
    Occupation(int n_as) : m_occupations(n_as, 0) {}
    Occupation(const Occupation &src) : m_occupations(src.m_occupations), m_density(src.m_density) {}
    Occupation(Occupation &&src) : m_occupations(std::move(src.m_occupations)), m_density(src.m_density) {}
    Occupation &operator=(const Occupation &rhs)
    {
        m_occupations = rhs.m_occupations;
        m_density = rhs.m_density;
        return *this;
    }
    Occupation &operator=(Occupation &&rhs)
    {
        m_occupations = std::move(rhs.m_occupations);
        m_density = rhs.m_density;
        return *this;
    }

    void add_occupation(int as_index);
    bool remove_occupation(int as_index);
    int read_density();
    int read_occupation(int as_index);

    void reset();

    private:
        std::vector<int> m_occupations;
        int m_density = 0;
        std::mutex m_mutex;
};

class ACO_Construction_State
{
    public:
        void occupy(Node const * node, int group_index, int unit_index = 0);
        bool move(Node const * src, Node const * dst, int group_index, int unit_index = 0);

        Occupation* read_vertex(Node const * node);
        Occupation* read_edge(Node const * src, Node const * dst);

        void reset_transitions();
        void reset();

        auto begin_history() {return std::begin(transition_history);}
        auto end_history() {return std::end(transition_history);}

        int get_time_step() const {return time_step;}
        void increment_time_step() {time_step++;}
        void reset_time_step() {time_step = 0;}

    private:
        std::unordered_map<Node const *, std::unordered_map<Node const *, Occupation>> edge_occupations;
        std::unordered_map<Node const *, Occupation> vertex_occupations;
        std::unordered_map<int, std::unordered_map<int, Node const *>> transition_history; // group_index, unit_index, node
        std::mutex edge_mutex;
        std::mutex vertex_mutex;
        std::mutex transition_mutex;

        int time_step = 0;
};


class ACO_Params
{
    public:
        double conflict_penalty = 1.0;
        int IT_NI = 100;
        int IT_MAX = 1000;
        int IT_INFO = 10;

        int MAX_STEPS = 500;

        int n_agents = 0;
        std::vector<Node const*> starts;
        std::vector<Node const*> goals;

        double init_pheromone = 0.5;

        std::vector< std::vector <Node const *>> const * dynamic_obstacles = nullptr;
        std::vector< Node const *> const * static_obstacles = nullptr;
};


class ACO
{
    public:
        ACO(const Instance &instance, const ACO_Params & aco_params, AS_Params &as_params);
        void run();

        ACOLog const &  get_results_log() const {return m_results_log;}
        EdgeMap const &  get_best_pheromone_map() const {return m_best_pheromones;}
        EdgeMap  const &  get_pheromone_map() const {return m_pheromones;}

        AS & get_as(int index) {return m_ant_systems.at(index);}


        std::vector<std::vector<int>> get_best_paths() const {return best_paths;}
    private:
        const Instance &m_instance;
        ACO_Params m_aco_params;
        AS_Params &m_as_params;

        std::vector<AS> m_ant_systems;
        EdgeMap m_pheromones;

        std::vector<int> m_active_as_indices; // Indices for as needed in search

        EdgeMap m_best_pheromones;

        ACOLog m_results_log; // Iteration, cost, makespan, conflicts, average_entropy
        
        std::vector<std::vector<int>> best_paths;

        void construct_solutions();
        void update_pheromones();
        double log_best_solutions(int it, int it_ni);
        void update_parmeters();

        void request_step(ACO_Construction_State & state, std::unordered_map< int, std::pair<double, std::unordered_set < int > > > const & searching_ants);
        void take_step(ACO_Construction_State & state, std::unordered_map< int, std::pair<double, std::unordered_set < int > > > & searching_ants);

        int count_conflicts(std::vector<Ant const*> const & ants) const;
};