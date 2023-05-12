#pragma once

#include <list>
#include <vector>
#include <unordered_map>
#include <random>
#include <utility>
#include "common.hpp"
#include <mutex>
#include <deque>

#include "edgemap.hpp"
#include "as_log.hpp"

class PheromoneTrail
{
    public:
            PheromoneTrail(Node const* src, Node const* dst, double deposit) : src(src), dst(dst), deposit(deposit) {}

            Node const* get_src() const { return src; }
            Node const* get_dst() const { return dst; }
            double get_deposit() const { return deposit; }

private:
    Node const* src = nullptr;
    Node const* dst = nullptr;
    double deposit = 0;
};


class Ant
{
    public:
        bool move(Node const & next, double edge_cost);
        void reset();

        void remove_trail(EdgeMap & pheromones);
        void put_trail(EdgeMap & pheromones, double deposit, double max_pheromone);

        bool is_visited(Node const & node) const;
        bool is_completed() const;

        //double get_confidence(EdgeMap &  pheromones, Node const* curr, Node const* next, double max_pheromone);

        void set_goal(Node const* goal);
        void set_start(Node const* start);
        Node const* get_goal() const;
        Node const* get_start() const;

        double get_tour_length() const;

        std::unordered_map<int, bool> const & get_visited_to_goal() const;
        std::unordered_map<int, bool> const & get_visited_to_start() const;

        std::vector<Node const*> const & get_tour() const;
        void set_tour(std::vector<Node const*> const & tour);
        size_t get_tour_size() const { return m_tour.size();}

        Node const* get_current_location() const; //Returns the last visited node
        Node const& get_target() const;  //Returns goal or start node

        void set_tour_length(double tour_length) {m_tour_length = tour_length;}
    
    private:
        std::unordered_map<int, bool> m_visited_to_goal; // <vertex_id, visited>
        std::unordered_map<int, bool> m_visited_to_start; // <vertex_id, visited>

        std::vector<Node const*> m_tour;
        std::list<PheromoneTrail> m_pheromone_trails;
        double m_tour_length = 0;

        bool m_reached_start = false;
        bool m_reached_goal = false; // if the ant found gold then target is m_start
        Node const* m_start = nullptr;
        Node const* m_goal = nullptr;

};

struct AS_Params
{ 
    std::vector<Node> const* graph = nullptr;
    int n_vertices = 0;
    int n_ants = 0;
    double alpha = 1;
    double beta = 1;
    
    double min_pheromone = 0.5;
    double max_pheromone = 1;
    double q0 = 0.1;
    int K = 3;
    double deposit = 1./3.;
    EdgeMap * init_choice_info = nullptr;

    bool tune_decision_weights = false;
    bool tune_greedy_selection_prob = false;
    bool elite_selection = false;
};

// TODO pheromones on directed edges
class AS
{
    public:
        AS(AS_Params const& params, EdgeMap & pheromone_map, unsigned int seed);
        // AS(AS const& src);
        // AS(AS && src);
        // ~AS();
        
        void reset_ants();
        void set_missions(Node const* start, Node const* goal);

        Node const* decision_rule(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors, std::vector<double> const & hp);
        void pheromone_update();
        void tune_parameters();

        Ant & get_ant(int k_ant);
        Ant & get_elite_ant();
        Ant const & get_elite_ant() const;

        std::vector<int> const & get_dim_indices() const;
        std::vector<int> const & get_ant_indices() const;

        size_t get_population_size() const  {return m_population.size();}
        Ant const & get_last_ant_from_population() const {return m_population.back();}

        ASLog & get_log() {return m_log;}

        AS_Params const & get_params() const {return m_params;}
        int get_n_ants() const {return m_params.n_ants;}


    private:
        int choose_random_next(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors);
        void add_population();

        void remove_loops(Ant & ant) const;
        void populate_visited_maps(std::unordered_map<Node const*, int> & visited_to_goal, std::unordered_map<Node const*, int> & visited_to_start, const std::vector<Node const*> & ant_tour, Node const* goal) const;
        void create_new_tour_without_loops(std::vector<Node const*> & new_tour, std::vector<Node const*> const & ant_tour, std::unordered_map<Node const*, int> const & visited_to_goal, std::unordered_map<Node const*, int> const & visited_to_start, Node const* goal) const;



        std::vector<Ant> m_ants;
        Ant elite_ant;

        ASLog m_log;


        EdgeMap & m_pheromones;
        EdgeMap m_choice_info;

        std::deque<Ant> m_population;

        AS_Params m_params;

        std::vector<int> m_dim_indices; // Helper structures for parallization
        std::vector<int> m_ant_indices;

        int iter = 0; // Number of iterations
        int iter_ni = 0; // Number of iterations without improvement


        double entropy_max = 1.;
        double entropy_min = 0;

        double m_replay_step = 1;

        std::mt19937 generator;
        std::uniform_real_distribution<double> distribution;

};