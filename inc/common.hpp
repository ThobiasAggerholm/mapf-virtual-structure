#pragma once
#include <unordered_map>
#include <string>
#include <sstream>
#include <ios>
//Node for weighted graph
struct Node
{
    //Make getters
    int get_id() const {return vertex_id;}
    void set_id(int id) {vertex_id = id;}
    std::unordered_map<Node const*, double> const & get_edges() const {return edges;}
    auto cbegin() const {return edges.cbegin();}
    auto cend() const {return edges.cend();}
    double at(Node const* node) const {return edges.at(node);}
    void insert_edge(Node const* node, double edge_weight) {edges[node] = edge_weight;}
    private:
        int vertex_id;
        std::unordered_map<Node const*, double> edges; // Next vertex, cost
};

int factorial(int n);

int get_combinations(int a, int b);

int get_direction(int a, int b);


template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}