#include "../inc/aco_log.hpp"

void ACOLog::print() const
{
    size_t max_size = std::max({iterations.size(), makespans.size(), costs.size(), entropies.size(), conflicts.size()});
    std::cout << std::setw(5) << "Index"
              << std::setw(12) << "Iteration"
              << std::setw(12) << "Makespan"
              << std::setw(12) << "Cost"
              << std::setw(12) << "Entropy"
              << std::setw(12) << "Conflicts" << std::endl;

    for (size_t i = 0; i < max_size; ++i) {
        std::cout << std::setw(5) << i;
        std::cout << std::setw(12) << (i < iterations.size() ? std::to_string(iterations[i]) : "INF");
        std::cout << std::setw(12) << (i < makespans.size() ? std::to_string(makespans[i]) : "INF");
        std::cout << std::setw(12) << (i < costs.size() ? std::to_string(costs[i]) : "INF");
        std::cout << std::setw(12) << (i < entropies.size() ? std::to_string(entropies[i]) : "INF");
        std::cout << std::setw(12) << (i < conflicts.size() ? std::to_string(conflicts[i]) : "INF");
        std::cout << std::endl;
    }

}

void ACOLog::write_to_file(const std::string& filename, std::string const & delimiter) const
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << "\n";
        return;
    }

    size_t max_size = std::max({iterations.size(), makespans.size(), costs.size(), entropies.size(), conflicts.size()});


    file << "Index" + delimiter + "Iteration" + delimiter + "Makespan" + delimiter + "Cost" + delimiter + "Entropy" + delimiter + "Conflicts\n";
    for (std::size_t i = 0; i < max_size; ++i) {
        file <<  i;
        file << delimiter << (i < iterations.size() ? std::to_string(iterations[i]) : "INF");
        file << delimiter << (i < makespans.size() ? std::to_string(makespans[i]) : "INF");
        file << delimiter << (i < costs.size() ? std::to_string(costs[i]) : "INF");
        file << delimiter << (i < entropies.size() ? std::to_string(entropies[i]) : "INF");
        file << delimiter << (i < conflicts.size() ? std::to_string(conflicts[i]) : "INF");
        file << "\n";
    }
    file << std::flush;
    file.close();
}