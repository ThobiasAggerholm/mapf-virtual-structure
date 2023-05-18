#pragma once

#include <vector>
#include <unordered_map>
#include <random>
#include <utility>
#include "common.hpp"
#include <mutex>
#include <deque>
#include <execution>

#define EXECUTION_POLICY_EDGEMAP std::execution::seq

class EdgeMap
{
    public:
        EdgeMap();
        EdgeMap(EdgeMap &src);
        EdgeMap(const EdgeMap &src);
        ~EdgeMap();

        EdgeMap& operator=(EdgeMap &rhs);
        EdgeMap& operator=(const EdgeMap &rhs);
        EdgeMap operator+(EdgeMap const & rhs) const;
        EdgeMap operator/(double const & rhs) const;

        auto cbegin() const {return m_edge_map.cbegin();}
        auto cend() const {return m_edge_map.cend();}

        auto begin() {return m_edge_map.begin();}
        auto end() {return m_edge_map.end();}

        double read(Node const* src, Node const* dst);
        double read(Node const* src, Node const* dst) const;
        void  write(Node const* src, Node const* dst, double val);
        void add(Node const* src, Node const* dst, double val);

        void export_geometric(std::string fname, int map_width, int map_height) const;

        double reduce();
        size_t size() const;

    private:
        std::unordered_map<Node const*, std::unordered_map<Node const*, double>> m_edge_map;
        std::unordered_map<Node const*, std::unordered_map<Node const*, std::mutex>> m_mutex_map;
};