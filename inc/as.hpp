#pragma once

#include <vector>
#include <unordered_map>
#include <random>
#include <utility>
#include "common.hpp"

typedef std::unordered_map<Node const*, std::unordered_map<Node const*, double>> EdgeMap;

class Ant
{
    public:
        bool move(Node const* next, double cost);
        void wait();
        int return_home();
        void reset();

        void deposit_pheromone(EdgeMap & pheromone_map);

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
    
    double evaporation_rate = 0;
    double init_pheromone = 0;
    EdgeMap * init_choice_info = nullptr;
};

// TODO pheromones on directed edges
class AS
{
    public:
        AS(const std::vector<Node> & graph, int n_vertices, int n_ants, double alpha, double beta,
         double evaporation_rate, double init_pheromone, EdgeMap * init_choice_info = nullptr);

        Node const* decision_rule(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors, std::vector<double> const* heuristics = nullptr, std::vector<double> const* sp = nullptr);
        int choose_random_next(int k_ant, Node const* curr, const std::vector<Node const*> & neighbors);
        void pheromone_update();
        void compute_choice_information();
        void evaporate();

        std::vector<Ant> m_ants;

        std::vector<int> m_dim_indices; // Helper structures for parallization
        std::vector<int> m_ant_indices;
        EdgeMap m_pheromones;
        EdgeMap m_choice_info;
        EdgeMap m_init_choice_info;

        double m_evaporation_rate;
        double m_init_pheromone;

        // TODO use of values?
        double m_alpha, m_beta;
        

    private:

};