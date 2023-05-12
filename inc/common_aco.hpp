#pragma once

#include <opencv2/opencv.hpp>
#include "aco.hpp"
#include "instance.hpp"
#include "astar_environment.hpp"

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
    ConflictLocations operator+(ConflictLocations const & rhs) const;

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

ConflictLocations conflict_locations(Instance const & instance, std::vector<int> missions, EdgeMap const * edgemap, double k);
ConflictLocations conflict_locations(Instance const & instance, std::vector<std::vector<int>> agent_paths, EdgeMap const * edgemap, double k);


ConflictCounts count_conflicts(const std::vector<std::vector<Position>> &agent_paths,
                    std::vector<Conflict> &vertex_conflict_locations,
                    std::vector<Conflict> &same_direction_edge_conflict_locations,
                    std::vector<Conflict> &opposite_direction_edge_conflict_locations);


HeatMap create_heatmap(cv::Mat const & map, std::vector<Bin> const & bins);

void draw_heatmap(cv::Mat const & heatmap, std::string const & fname_map);

void save_heat_ranges(std::map<int, int> const & heatranges, std::string const & fname_heatranges);

void add_grid(cv::Mat &scaled_image, int scale_factor);

void add_grid_scaled(cv::Mat &scaled_image, int scale_factor);

void draw_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, int scale_factor = 12);

void draw_arrow_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, double threshold = 1., int scale_factor = 12);

std::vector<Bin> conflict_locations_to_bins(std::unordered_map< int, std::unordered_map<int, int> > const & histogram2D);

std::unordered_map< int, std::unordered_map<int, int> > conflict_locations_to_histogram2d(ConflictLocations const & conflict_locations, int normalization);

EdgeMap clamp(EdgeMap const & edgemap, double min, double max);

void draw_pheromone_arrows(cv::Mat& img, const Instance& instance, const EdgeMap& pheromones, double threshold, int scale_factor);

cv::Mat create_scaled_map(const cv::Mat& map, int scale_factor);