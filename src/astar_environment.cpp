#include "../inc/astar_environment.hpp"

bool agent_path_search(std::vector<Cell> & cell_details, const Instance & instance, int start, int goal, EdgeMap  const * pheromones, double k)
{
	// If the source is out of range
    if( instance.valid_pos(start) == false )
    {
        return false;
    }

	// If the destination is out of range
    if( instance.valid_pos(goal) == false )
    {
        return false;
    }

	// If the destination cell is the same as source cell
	if ( start == goal )
    {
		return false;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	std::vector<bool> closedList(instance.m_map_size, false);

	// Declare a 2D array of structure to hold the details
	// of that cell
    cell_details.resize(instance.m_map_size);

	for(Cell & cell : cell_details)
    {
        cell.f = INFINITY;
        cell.g = INFINITY;
        cell.h = INFINITY;
        cell.parent_i = -1;
    }

	// Initialising the parameters of the starting node
    cell_details[start].f = 0;
    cell_details[start].g = 0;
    cell_details[start].h = 0;
    cell_details[start].parent_i = start;

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
	openList.insert(std::make_pair(0.0, start));

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

		// To store the 'g', 'h' and 'f' of the 4 successors
		double gNew, hNew, fNew;
        std::vector<int> successors = instance.get_neighbors(i);
        for(const int & successor : successors)
        {
            if(successor == goal)
            {
                cell_details[successor].parent_i = i;
                found_dest = true;
                return found_dest;
            }
            // If the successor is already on the closed
			// list or if it is blocked, then ignore it.
			// Else do the following
            else if (closedList[successor] == false)
            {
				gNew = cell_details[i].g + instance.get_neighbor_distance(i, successor);
                Node const* curr = &instance.m_my_graph[i];
                Node const* next = &instance.m_my_graph[successor];
				hNew = instance.get_manhattan_distance(successor, goal);
                if(pheromones != nullptr)
				{
					double pheromone_diff = pheromones->read(next, curr) - pheromones->read(curr,next);
                    hNew += pheromone_diff * k;
				}
                // Transition cost from q to p
                // Heuristic of p
                // Penalty -> k * pheromone opposite
                // Reward -> k * pheromone in direction    
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
    return found_dest;
}

std::vector<int> agent_path(const Instance & instance, int start, int goal, EdgeMap  const * pheromones, double k)
{
    std::vector<Cell> cell_details;
    bool dest_found = agent_path_search(cell_details, instance, start, goal, pheromones, k);
    if(!dest_found)
        return {};
    
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