#pragma once
#include <unordered_map>

//Node for weighted graph
struct Node
{
    int vertex_id;
    std::unordered_map<Node const*, double> edges; // Next vertex, cost
};

int factorial(int n);

int get_combinations(int a, int b);