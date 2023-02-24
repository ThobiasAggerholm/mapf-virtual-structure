#pragma once
#include <vector>
#include "as.hpp"
#include <mutex>
#include <unordered_map>
#include <utility>
#include "instance.hpp"
#include "astar.hpp"


class  ACO
{
    public:
        ACO(const Instance & instance, int n_ant_systems, const AS_Params & as_params);
        void run(int IT_NI, int IT_MAX);
        
        std::vector< std::vector < Node const* > > m_as_paths;
    
        const Instance & m_instance;
        AStar m_a_star;
        std::vector<AS> m_ant_systems;

        std::unordered_map<Node const*, std::unordered_map<Node  const*, std::mutex>> m_edge_locks;
        std::unordered_map<Node const*, std::mutex> m_vertex_locks;

        std::vector<int> m_i_starts;
        std::vector<int> m_i_goals;

        std::vector<int> m_active_as_indices; // Indices for as needed in search
        std::unordered_map<Node const*, std::pair<int, int>> m_node_occupation; // Which AS has how many ants on node pair<AS, n_ants>

        std::vector<double> m_as_scores;

        void construct_solutions();
        void update_pheromones();
        void log_best_solutions();

        bool take_step();
};