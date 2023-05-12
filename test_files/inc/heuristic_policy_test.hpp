#pragma once

#include "../../inc/common.hpp"
#include ".././inc/instance.hpp"
#include <vector>
#include <random>
#include <unordered_set>
#include <functional>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <execution>

#include <opencv2/opencv.hpp>

namespace HeuristicPolicyTest
{

struct Location
{
    Location(int x, int y) : x(x), y(y) {}
    int x;
    int y;
};

int manhattan_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

int direction_rule(Location const & current, Location const & next, Location const & goal)
{
    return 1.0 / (2.0 - double(manhattan_distance(current.x, current.y, goal.x, goal.y) - manhattan_distance(next.x, next.y, goal.x, goal.y)));
}

int distance_rule(Location const & current, Location const & next, Location const & goal)
{
    return 1.0 / double(manhattan_distance(next.x, next.y, goal.x, goal.y));
}

int uniform_rule(Location const & current, Location const & next, Location const & goal)
{
    return 0.0;
}



// Select a random element from a vector of weights where probability of each of the n possible numbers to be produced being their corresponding weight divided by the total of all weights.
int roulette_wheel_selection(std::vector<double> const & weights)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    return d(gen);
}

class DecisionRuleTester
{
    public:
        DecisionRuleTester(Instance const& instance) : m_instance_data(instance) {}
        ~DecisionRuleTester() {}

        std::vector< std::vector < int > > run(const std::function<double(Location const &, Location const &, Location const &)>& user_decision_rule, std::vector<int> missions, int repititions);
        std::unordered_map<int, bool> get_visited_nodes() { return m_visited_nodes; }
    

    private:
        void reset() { m_visited_nodes.clear(); }
        std::vector<Node const*> get_neighbors(Node const* current_node);
        Location get_location(Node const* node) { return Location(m_instance_data.get_col_coordinate(node->vertex_id), m_instance_data.get_row_coordinate(node->vertex_id)); }

        int test_decision_rule(Node const* start, Node const* goal_node, std::function<double(Location const &, Location const &, Location const &)> heuristic_func);
        Node const* decision_rule(Node const* current_node, std::vector<Node const*> const & neighbors, Node const* goal_node, std::function<double(Location const &, Location const &, Location const &)> heuristic_func);
        
        std::unordered_map<int, bool> m_visited_nodes;
        Instance const& m_instance_data;
};

//Takes a user defined decision rule, a vector of missions, and a number of repititions and returnes the number of iterations for the decision rule for each mission for each repitition in a 2D vector
std::vector< std::vector < int > > DecisionRuleTester::run(const std::function<double(Location const &, Location const &, Location const &)>& user_decision_rule, std::vector<int> missions, int repititions)
{

    std::vector< std::vector < int > > results_for_decision_rule;
    results_for_decision_rule.resize(missions.size());
    for(int j = 0; j < missions.size(); ++j)
    {
        std::cout << "Mission " << j << std::endl;
        if(m_instance_data.m_start_locations.size() < j || m_instance_data.m_goal_locations.size() < j)
            throw std::runtime_error("Mission index out of bounds");

        std::vector<int> & results_for_mission = results_for_decision_rule[j];
        results_for_mission.reserve(repititions);
        for(int k = 0; k < repititions; ++k)
        {
            std::cout<< "Repitition " << k << std::endl;
            int mission_idx = missions[j];
            Node const* start_node = &m_instance_data.m_my_graph.at(m_instance_data.m_start_locations[mission_idx]);
            Node const* goal_node = &m_instance_data.m_my_graph.at(m_instance_data.m_goal_locations[mission_idx]);
            int result = test_decision_rule(start_node, goal_node, user_decision_rule);
            results_for_mission.push_back(result);
        }
    }

    return results_for_decision_rule;
}

std::vector<Node const*> DecisionRuleTester::get_neighbors(Node const* current_node)
{
    std::vector<Node const*> neighbors;
    std::vector<int> neighbor_ids = m_instance_data.get_neighbors(current_node->vertex_id);
    neighbors.reserve(neighbor_ids.size());
    for (auto const& neighbor_id : neighbor_ids)
    neighbors.push_back(&m_instance_data.m_my_graph.at(neighbor_id));
    return neighbors;
}


// Test decision rule using the given heuristic function
int DecisionRuleTester::test_decision_rule(Node const* start, Node const* goal_node, std::function<double(Location const &, Location const &, Location const &)> heuristic_func) {
    reset();

    Node const* current_node = start;
    m_visited_nodes.insert({current_node->vertex_id, true});
    int steps = 0;
    
    while (current_node != goal_node) {

        current_node = decision_rule(current_node, get_neighbors(current_node), goal_node, heuristic_func);
        m_visited_nodes.insert({current_node->vertex_id, true});
        ++steps;
    }
    return steps;
}

// Decision rule with given heuristic function
Node const* DecisionRuleTester::decision_rule(Node const* current_node, std::vector<Node const*> const & neighbors, Node const* goal_node, std::function<double(Location const &, Location const &, Location const &)> heuristic_func) {
    std::vector<double> probabilities(neighbors.size(), 0.);
    double sum_probabilities = 0.;

    Location start_location = get_location(current_node);
    Location goal_location = get_location(goal_node);
    
    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (m_visited_nodes.find(neighbors[i]->vertex_id) != m_visited_nodes.end())
            continue;

        Location neighbor_location = get_location(neighbors[i]);

        probabilities[i] = heuristic_func(start_location, neighbor_location, goal_location);
        sum_probabilities += probabilities[i];
    }
    
    Node const* next = nullptr;
    if (sum_probabilities == 0.) {
        next = neighbors[roulette_wheel_selection(std::vector<double>(neighbors.size(), 1.0))];
    } else {
        next = neighbors[roulette_wheel_selection(probabilities)];
    }
    return next;
}

#define FILE_RESULTS 0
#define VISUALIZE_RESULTS 1

cv::Mat create_map(Instance const & instance)
{
    cv::Mat img(instance.m_num_of_rows, instance.m_num_of_cols, CV_8UC3, cv::Scalar(0, 0, 0)); // BGR
    for (int i = 0; i < instance.m_num_of_rows; i++)
    {
        for (int j = 0; j < instance.m_num_of_cols; j++)
        {
            int pos = instance.linearize_coordinate(i, j);
            bool type = instance.m_my_map[pos];

            if(type == 1)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(100, 100, 100);
                continue;
            }
            else if(type == 0)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    return img;  
}

cv::Mat add_point(cv::Mat img, Instance const & instance, int pos, cv::Vec3b color = cv::Vec3b(255, 0, 0))
{
    int row = instance.get_row_coordinate(pos);
    int col = instance.get_col_coordinate(pos);
    img.at<cv::Vec3b>(row, col) = color;
    return img;
}

cv::Mat add_path(cv::Mat img, Instance const & instance, std::vector<int> positions)
{
    for (int i = 0; i < positions.size(); i++)
    {
        int pos = positions[i];
        add_point(img, instance, pos);
    }
    return img;
}

void draw_map(std::string fname, cv::Mat img, int scale_factor = 1)
{
    cv::Mat scaled_image;//BGR
    cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
    cv::imwrite(fname, img);
}

//TODO visualize the missions
void test_heuristic_policy()
{
    std::string map_fname_in = "../benchmark_data/mapf-map/maze-128-128-10.map";
    std::string agent_fname = "../benchmark_data/mapf-scen-random/maze-128-128-10-random-1.scen"; 
    std::string decision_rule_fname = "../test_files/test_data/heuristic_policy/decision_rule_test-maze-128-128-10-";
    std::string extension = ".csv";

    int num_agents = 1000;
    Instance instance(map_fname_in, agent_fname, num_agents);

    std::vector<int> order_agents(num_agents);
    std::iota(order_agents.begin(), order_agents.end(), 0);

    std::vector<std::function<double(Location const &, Location const &, Location const &)>> decision_rules = {uniform_rule, distance_rule, direction_rule};
    std::vector<std::string> decision_rule_names = {"uniform", "distance", "direction"};

    if(decision_rules.size() != decision_rule_names.size())
        throw std::runtime_error("Decision rule and name vectors are not the same size");

    std::vector<int> decision_rule_indices(decision_rules.size());
    std::iota(decision_rule_indices.begin(), decision_rule_indices.end(), 0);

#if FILE_RESULTS
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(order_agents.begin(), order_agents.end(), gen);

    int number_of_missions = 10;
    std::vector<int> missions = std::vector(order_agents.begin(), order_agents.begin() + number_of_missions);
    int number_of_repititions = 100;
    std::vector< std::vector < std::vector < int > > > results(decision_rules.size());
    std::for_each(std::execution::par, decision_rule_indices.cbegin(), decision_rule_indices.cend(), [&instance, &decision_rules, &decision_rule_names, &missions, &number_of_repititions, &results](int i)
    {
        std::cout << "Running decision rule: " << decision_rule_names[i] << std::endl;
        DecisionRuleTester tester(instance);
        results[i] = tester.run(decision_rules[i], missions, number_of_repititions);
        std::cout << "Finished running decision rule: " << decision_rule_names[i] << std::endl;
    });



    for (int i = 0; i < decision_rules.size(); i++)
    {
        std::ofstream file;
        std::string file_name = decision_rule_fname + decision_rule_names[i] + extension;
        file.open(file_name);
        for (int j = 0; j < number_of_missions; j++)
        {
            for (int k = 0; k < number_of_repititions; k++)
            {
                file << results[i][j][k] << ",";
            }
            file << std::endl;
        }
        file.close();
    }
#endif

#if VISUALIZE_RESULTS

    std::sort(order_agents.begin(), order_agents.end(), [&instance](int a, int b)
    {
        int distance_a = instance.get_manhattan_distance(instance.m_start_locations.at(a), instance.m_goal_locations.at(a));
        int distance_b = instance.get_manhattan_distance(instance.m_start_locations.at(b), instance.m_goal_locations.at(b));
        return distance_a > distance_b;
    });

    int number_of_missions = 1;
    std::vector<int> missions = std::vector(order_agents.begin(), order_agents.begin() + number_of_missions);
    std::cout << "Running scenario: " << missions.at(0) << std::endl;
    std::cout << "Start: " << instance.m_start_locations.at(missions.at(0)) << " Goal: " << instance.m_goal_locations.at(missions.at(0)) << std::endl;
    std::cout << "Distance: " << instance.get_manhattan_distance(instance.m_start_locations.at(missions.at(0)), instance.m_goal_locations.at(missions.at(0))) << std::endl;
    int number_of_repititions = 1;
    std::vector < std::vector < int > > results(decision_rules.size());
    std::for_each(std::execution::par, decision_rule_indices.cbegin(), decision_rule_indices.cend(), [&instance, &decision_rules, &decision_rule_names, &missions, &number_of_repititions, &results](int i)
    {
        
        std::cout << "Running decision rule: " << decision_rule_names[i] << std::endl;
        DecisionRuleTester tester(instance);
        std::vector< std::vector < int > > number_iterations = tester.run(decision_rules[i], missions, number_of_repititions);
        auto visited_nodes = tester.get_visited_nodes();

        std::vector<int> path;
        path.reserve(visited_nodes.size());
        for(auto it = visited_nodes.begin(); it != visited_nodes.end(); ++it)
        {
            path.push_back(it->first);
        }
        results[i] = path;
        std::cout << "Finished running decision rule: " << decision_rule_names[i] << " " << "Took iterations: " << number_iterations.at(0).at(0) << std::endl;
    });


    for(int i = 0; i < decision_rules.size(); i++)
    {
        cv::Mat img = create_map(instance);
        add_path(img, instance, results[i]);
        add_point(img, instance, instance.m_start_locations.at(missions.at(0)), cv::Vec3b(0, 0, 255));
        add_point(img, instance, instance.m_goal_locations.at(missions.at(0)), cv::Vec3b(0, 255, 0));
        std::string fname = decision_rule_fname + decision_rule_names[i] + "-map-" + ".png";
        draw_map(fname, img, 4);
    }
#endif
    
    return;
}

} //End  of Namespace