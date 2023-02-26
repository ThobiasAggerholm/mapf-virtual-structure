#pragma once

#include <vector>
#include <unordered_map>
#include <random>
#include <utility>
#include "common.hpp"
#include <mutex>
#include <queue>

class EdgeMap
{
    public:
        EdgeMap();
        EdgeMap(EdgeMap &src);
        EdgeMap(const EdgeMap &src);
        ~EdgeMap();

        EdgeMap& operator=(EdgeMap &rhs);
        EdgeMap& operator=(const EdgeMap &rhs);

        auto cbegin() const {return m_edge_map.cbegin();}
        auto cend() const {return m_edge_map.cend();}

        double read(Node const* src, Node const* dst);
        void  write(Node const* src, Node const* dst, double val);
        void add(Node const* src, Node const* dst, double val);

    private:
        std::unordered_map<Node const*, std::unordered_map<Node const*, double>> m_edge_map;
        std::unordered_map<Node const*, std::unordered_map<Node const*, std::mutex>> m_mutex_map;
};

class Ant
{
    public:
        bool move(Node const* next, double cost);
        void wait();
        int return_home();
        void reset();

        void remove_trail(EdgeMap & pheromones, double deposit);
        void put_trail(EdgeMap & pheromones, double deposit);

        bool m_is_returned_home = false;
        std::unordered_map<Node const*, bool> m_visited;
        std::vector<Node const*> m_tour;
        double m_tour_length = 0;
        bool m_found_gold = false;

};

struct AS_Params
{ 
    std::vector<Node> const* graph = nullptr;
    int n_vertices = 0;
    int n_ants = 0;
    double alpha = 1;
    double beta = 1;
    
    double init_pheromone = 0.001;
    double min_pheromone = 0.5;
    double max_pheromone = 1;
    double q0 = 0.1;
    double K = 3;
    double deposit = 1./3.;
    EdgeMap * init_choice_info = nullptr;
};

// TODO pheromones on directed edges
class AS
{
    public:
        AS(const std::vector<Node> & graph, int n_vertices, int n_ants, double alpha, double beta,
         double init_pheromone, double max_pheromone, double q0, double K, double deposit, EdgeMap & pheromone_map, EdgeMap * init_choice_info = nullptr);

        Node const* decision_rule(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors, std::vector<double> const* heuristics = nullptr, std::vector<double> const* sp = nullptr);
        int choose_random_next(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors);
        void add_pop();
        void pheromone_update();
        void compute_choice_information();

        std::vector<Ant> m_ants;

        std::vector<int> m_dim_indices; // Helper structures for parallization
        std::vector<int> m_ant_indices;
        EdgeMap & m_pheromones;
        EdgeMap m_choice_info;
        EdgeMap m_init_choice_info;

        std::queue<Ant> m_population;

        double m_q0;
        double m_K;
        double m_init_pheromone;
        double m_max_pheromone;
        double m_deposit;

        // TODO use of values?
        double m_alpha, m_beta;
        

    private:

};