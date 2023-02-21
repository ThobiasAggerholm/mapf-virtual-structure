#pragma once
#include <unordered_map>

//Node for weighted graph
struct Node
{
    int vertex_id;
    std::unordered_map<Node const*, double> edges; // Next vertex, cost
};
