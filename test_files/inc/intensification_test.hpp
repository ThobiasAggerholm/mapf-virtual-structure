#pragma once

#include "../../inc/common.hpp"
#include "../../inc/instance.hpp"
#include "../../inc/edgemap.hpp"
#include "../../inc/as.hpp"
#include "../../inc/aco.hpp"
#include "../../inc/astar_environment.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <fstream>
#include <unordered_map>


namespace IntensificationTest
{

struct Position {
    int x, y;
};

bool operator<(const Position &a, const Position &b);

bool operator==(const Position &a, const Position &b);

bool operator!=(const Position &a, const Position &b);

struct Conflict {
    Position position;
    int timestep;
};


struct ConflictCounts {
    int vertex_conflicts = 0;
    int same_direction_edge_conflicts = 0;
    int opposite_direction_edge_conflicts = 0;
};

struct ConflictLocations
{
    ConflictLocations operator+(ConflictLocations const & rhs) const
    {
        ConflictLocations result;
        result.vertex_conflict_locations = vertex_conflict_locations;
        result.vertex_conflict_locations.insert(result.vertex_conflict_locations.end(), rhs.vertex_conflict_locations.begin(), rhs.vertex_conflict_locations.end());
        result.same_direction_edge_conflict_locations = same_direction_edge_conflict_locations;
        result.same_direction_edge_conflict_locations.insert(result.same_direction_edge_conflict_locations.end(), rhs.same_direction_edge_conflict_locations.begin(), rhs.same_direction_edge_conflict_locations.end());
        result.opposite_direction_edge_conflict_locations = opposite_direction_edge_conflict_locations;
        result.opposite_direction_edge_conflict_locations.insert(result.opposite_direction_edge_conflict_locations.end(), rhs.opposite_direction_edge_conflict_locations.begin(), rhs.opposite_direction_edge_conflict_locations.end());
        result.conflict_counts.vertex_conflicts = conflict_counts.vertex_conflicts + rhs.conflict_counts.vertex_conflicts;
        result.conflict_counts.same_direction_edge_conflicts = conflict_counts.same_direction_edge_conflicts + rhs.conflict_counts.same_direction_edge_conflicts;
        result.conflict_counts.opposite_direction_edge_conflicts = conflict_counts.opposite_direction_edge_conflicts + rhs.conflict_counts.opposite_direction_edge_conflicts;
        return result;
    }

    std::vector<Conflict> vertex_conflict_locations;
    std::vector<Conflict> same_direction_edge_conflict_locations;
    std::vector<Conflict> opposite_direction_edge_conflict_locations;
    ConflictCounts conflict_counts;
};

struct Bin
{
    int x, y;
    int count;
};

struct HeatMap
{
    std::map<int, int> heatranges;
    cv::Mat heatmap;
};

void configure_as_params(AS_Params & as_params, Instance const & instance);

void configure_aco_params(ACO_Params & aco_params, Instance const & instance, std::vector<int> missions);

void test_intensification(std::string const & test_name, std::function<void(std::string const &, Instance const &, int, int)> const & test_mission);

void test_ab();
void test_mission_ab(std::string const & path_fname, Instance const & instance, int mission, int number_repetions);

void test_q0();
void test_mission_q0(std::string const & path_fname, Instance const & instance, int mission, int number_repetions);


void test_weight_tuning();

//Save number and position of following edge conflicts
//Save number and position of vertex conflicts not caused by following edge conflicts
//Save number amd position edge conflicts caused by opposite direction movement
ConflictLocations conflict_locations(Instance const & instance, int num_agents, EdgeMap * edgemap = nullptr, double k = 0.);




ConflictCounts count_conflicts(const std::vector<std::vector<Position>> &agent_paths,
                    std::vector<Conflict> &vertex_conflict_locations,
                    std::vector<Conflict> &same_direction_edge_conflict_locations,
                    std::vector<Conflict> &opposite_direction_edge_conflict_locations);



HeatMap create_heatmap(cv::Mat const & map, std::vector<Bin> const & bins);

void draw_heatmap(cv::Mat const & heatmap, std::string const & fname_map);

void save_heat_ranges(std::map<int, int> const & heatranges, std::string const & fname_heatranges);

void add_grid(cv::Mat &scaled_image, int scale_factor);

void draw_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, int scale_factor = 12);

cv::Mat create_map(Instance const & instance);

std::vector<Bin> conflict_locations_to_bins(std::unordered_map< int, std::unordered_map<int, int> > const & histogram2D);

std::unordered_map< int, std::unordered_map<int, int> > conflict_locations_to_histogram2d(ConflictLocations const & conflict_locations, int normalization = 1);

}//End of namespace