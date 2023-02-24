#pragma once
//Based of https://github.com/Jiaoyang-Li/CBSH2-RTC/blob/main/inc/Instance.h
#include <string>
#include <vector>
#include <unordered_map>
#include "common.hpp"
#include <cassert>

class Instance
{
    public:


        Instance(const std::string& map_fname, const std::string& agent_fname,
                 int num_of_agents = 0, const std::string& agent_indices = "");
        
        inline int linearize_coordinate(int row, int col) const { return ( this->m_num_of_cols * row + col); }
	    inline int linearize_coordinate(const std::pair<int, int>& cell) const { return linearize_coordinate(cell.first, cell.second); }
        
        inline int get_row_coordinate(int id) const { return id / this->m_num_of_cols; }
		inline int get_col_coordinate(int id) const { return id % this->m_num_of_cols; }
		inline std::pair<int, int> get_coordinate(int id) const { return std::make_pair(id / this->m_num_of_cols, id % this->m_num_of_cols); }

        
		inline int get_manhattan_distance(int loc1, int loc2) const
		{
			int loc1_y = get_row_coordinate(loc1);
			int loc1_x = get_col_coordinate(loc1);
			int loc2_y = get_row_coordinate(loc2);
			int loc2_x = get_col_coordinate(loc2);
			return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
		}
        inline double get_neighbor_distance(int curr, int nb) const
        {
            assert(get_manhattan_distance(curr, nb) == 1);
            assert(valid_move(curr, nb));
            Node const* p_nb = &m_my_graph[nb];

            return m_my_graph[curr].edges.at(p_nb);
        }

        bool valid_pos(int pos) const;
        bool valid_move(int curr,int next) const;
        inline bool isObstacle(int loc) const { return m_my_map[loc]; }

        std::vector<int> get_neighbors(int curr) const;

        bool save_map(std::string out_fname) const;
        bool map_route_to_image(std::string out_fname, const std::vector<int> & path) const;



        int m_num_of_cols = 0;
        int m_num_of_rows = 0;
        
        int m_map_size = 0;

        std::vector<bool> m_my_map;
        std::vector<Node> m_my_graph;
        std::string m_map_fname;
        std::string m_agent_fname;
        int m_num_of_agents = 0;
        std::string m_agent_indices;
        std::vector<int> m_start_locations;
	    std::vector<int> m_goal_locations;
        std::vector<double> m_optimal_lengths;

        bool load_map();
        bool load_agents();
        bool load_graph();
};