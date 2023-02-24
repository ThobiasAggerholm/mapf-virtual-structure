#include "../inc/astar.hpp"

#include <iostream>
#include <cmath>
#include <bits/stdc++.h>

AStar::AStar(const Instance & instance)
 : m_instance{instance}
{

}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
bool AStar::search(std::vector<Cell> & cell_details, const int & src, const int & goal)
{
	// If the source is out of range
    if( m_instance.valid_pos(src) == false )
    {
        return false;
    }

	// If the destination is out of range
    if( m_instance.valid_pos(goal) == false )
    {
        return false;
    }

	// If the destination cell is the same as source cell
	if ( src == goal )
    {
		return false;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	std::vector<bool> closedList(m_instance.m_map_size, false);

	// Declare a 2D array of structure to hold the details
	// of that cell
    cell_details.resize(m_instance.m_map_size);

	for(Cell & cell : cell_details)
    {
        cell.f = INFINITY;
        cell.g = INFINITY;
        cell.h = INFINITY;
        cell.parent_i = -1;
    }

	// Initialising the parameters of the starting node
    cell_details[src].f = 0;
    cell_details[src].g = 0;
    cell_details[src].h = 0;
    cell_details[src].parent_i = src;

	/*
	Create an open list having information as-
	<f, i>
	where f = g + h,
	and i is the index of that cell
	This open list is implemented as a set of pairs
    */
	std::set<std::pair<double, int>> openList;

	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.insert(std::make_pair(0.0, src));

	// We set this boolean value as false as initially
	// the destination is not reached.
	bool found_dest = false;

	while (!openList.empty())
    {
		auto p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the closed list
        int i = p.second;
		closedList[i] = true;

		/*
		Generating 4 successor of this cell
                  N
				  |
			W----Cell----E
				  | 
                  S

		Cell-->Popped Cell (i, j)
		N --> North	 (i-1, j)
		S --> South	 (i+1, j)
		E --> East	 (i, j+1)
		W --> West		 (i, j-1)*/

		// To store the 'g', 'h' and 'f' of the 4 successors
		double gNew, hNew, fNew;
        std::vector<int> successors = m_instance.get_neighbors(i);
        for(const int & successor : successors)
        {
            if(successor == goal)
            {
                cell_details[successor].parent_i = i;
                found_dest = true;
                return true;
            }
            // If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
            else if (closedList[successor] == false)
            {
				gNew = cell_details[i].g + m_instance.get_neighbor_distance(i, successor);
				hNew = calculate_h_value(successor, goal);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to
				// the open list. Make the current square
				// the parent of this square. Record the
				// f, g, and h costs of the square cell
				//			 OR
				// If it is on the open list already, check
				// to see if this path to that square is
				// better, using 'f' cost as the measure.
				if ((cell_details[successor].f == FLT_MAX)
					|| (cell_details[successor].f > fNew))
                {
					openList.insert(std::make_pair(
						fNew,successor));

					// Update the details of this cell
					cell_details[successor].f = fNew;
					cell_details[successor].g = gNew;
					cell_details[successor].h = hNew;
					cell_details[successor].parent_i = i;
				}
			}
        }
    }
	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destination cell. This may happen when the
	// there is no way to destination cell (due to
	// blockages)
	if (found_dest == false)
		std::cout << "Failed to find the Destination Cell" << std::endl;

	return found_dest;
}

// A Utility Function to calculate the 'h' heuristics.
double AStar::calculate_h_value(int pos, int goal)
{
	// Return using the distance formula
	return m_instance.get_manhattan_distance(pos, goal);
}

// A Utility Function to trace the path from the source
// to destination
std::vector<int> AStar::trace_path(const std::vector<Cell> & cell_details, int goal)
{
    int pos = goal;

	std::vector<int> path;

	while (cell_details[pos].parent_i != pos)
    {
		path.push_back(pos);
		pos = cell_details[pos].parent_i;
	}

	path.push_back(pos);

	return path;
}
