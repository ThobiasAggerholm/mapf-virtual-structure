#pragma once

#include "astar.hpp"
#include "edgemap.hpp"
#include <set>
#include <cmath>
#include <bits/stdc++.h>

bool agent_path_search(std::vector<Cell> & cell_details, const Instance & instance, int start, int goal, EdgeMap const * pheromones = nullptr, double k = 0.);


std::vector<int> agent_path(const Instance & instance, int start, int goal, EdgeMap const * pheromones = nullptr, double k = 0.0);