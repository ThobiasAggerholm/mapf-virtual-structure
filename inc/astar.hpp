#pragma once
#include "instance.hpp"

// A structure to hold the necessary parameters
struct Cell {
	// Index of its parent
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	int parent_i;
	// f = g + h
	double f, g, h;
};

class AStar
{
    public:
        //Configure distance setting
        AStar(const Instance & instance);
        bool search(std::vector<Cell> & cell_details, const int & src, const int & goal);

        double calculate_h_value(int pos, int goal);
        std::vector<int> trace_path(const std::vector<Cell> & cell_details, int goal);



        const Instance & m_instance;
};