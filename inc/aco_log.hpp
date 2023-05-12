#pragma once
#include <vector>
#include <fstream> // Don't forget to include the <fstream> header for file operations
#include <iostream>
#include <limits>
#include <iomanip>


class ACOLog
{
    public:
        void add_iteration(int iteration) {iterations.push_back(iteration);}
        void add_makespan(double makespan) {makespans.push_back(makespan);}
        void add_cost(double cost)  {costs.push_back(cost);}
        void add_entropy(double entropy) {entropies.push_back(entropy);}
        void add_conflicts(double ant_tour_length) {conflicts.push_back(ant_tour_length);}

        int get_iteration(int index) const {return iterations.at(index);}
        double get_makespan(int index) const {return makespans.at(index);}
        double get_cost(int index) const {return costs.at(index);}
        double get_entropy(int index) const {return entropies.at(index);}
        double get_conflicts(int index) const {return conflicts.at(index);}

        int get_iterations_size() const {return iterations.size();}
        int get_makespans_size() const  {return makespans.size();}
        int get_costs_size() const {return costs.size();}
        int get_entropies_size() const {return entropies.size();}
        int get_conflicts_size() const {return conflicts.size();}

        void reserve(int size) {
            iterations.reserve(size);
            makespans.reserve(size);
            costs.reserve(size);
            entropies.reserve(size);
            conflicts.reserve(size);
        }
        void reset() {
            iterations.clear();
            makespans.clear();
            costs.clear();
            entropies.clear();
            conflicts.clear();
        }

        void print() const;
        void write_to_file(const std::string& filename, std::string const & delimiter = ",") const;

    private:
        std::vector<int> iterations;
        std::vector<double> makespans;
        std::vector<double> conflicts;
        std::vector<double> costs;
        std::vector<double> entropies;
};