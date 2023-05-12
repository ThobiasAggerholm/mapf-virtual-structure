#include "../inc/common_aco.hpp"
#include <cmath>
#include "../inc/common_instance.hpp"

bool operator<(const Position &a, const Position &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

bool operator==(const Position &a, const Position &b) {
    return a.x == b.x && a.y == b.y;
}

bool operator!=(const Position &a, const Position &b) {
    return !(a == b);
}

ConflictLocations ConflictLocations::operator+(ConflictLocations const & rhs) const
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

ConflictLocations conflict_locations(Instance const & instance, std::vector<std::vector<int>> agent_paths, EdgeMap const * edgemap, double k)
{  

    std::vector< std::vector < Position > > agent_positions(agent_paths.size());
    for(int i = 0; i < agent_paths.size(); ++i)
    {
        agent_positions[i].resize(agent_paths[i].size());
        for(int j = 0; j < agent_paths[i].size(); ++j)
        {
            int x = instance.get_col_coordinate(agent_paths[i][j]);
            int y = instance.get_row_coordinate(agent_paths[i][j]);

            agent_positions[i][j] = {x, y};
        }
    }

    std::vector<Conflict> vertex_conflict_locations;
    std::vector<Conflict> same_direction_edge_conflict_locations;
    std::vector<Conflict> opposite_direction_edge_conflict_locations;
    ConflictCounts conflict_counts = count_conflicts(agent_positions, vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations);

    ConflictLocations conflict_locations  = {vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations,  conflict_counts};
    return conflict_locations;

}

ConflictLocations conflict_locations(Instance const & instance, std::vector<int> missions, EdgeMap const * edgemap, double k)
{

    std::vector<std::vector<int>> agent_paths(missions.size());
    for(int i = 0; i < missions.size(); ++i)
    {
        int mission = missions[i];
        agent_paths[i] = agent_path(instance, instance.m_start_locations[mission], instance.m_goal_locations[mission], edgemap, k);
    }
    


    std::vector< std::vector < Position > > agent_positions(agent_paths.size());
    for(int i = 0; i < agent_paths.size(); ++i)
    {
        agent_positions[i].resize(agent_paths[i].size());
        for(int j = 0; j < agent_paths[i].size(); ++j)
        {
            int x = instance.get_col_coordinate(agent_paths[i][j]);
            int y = instance.get_row_coordinate(agent_paths[i][j]);

            agent_positions[i][j] = {x, y};
        }
    }

    std::vector<Conflict> vertex_conflict_locations;
    std::vector<Conflict> same_direction_edge_conflict_locations;
    std::vector<Conflict> opposite_direction_edge_conflict_locations;
    ConflictCounts conflict_counts = count_conflicts(agent_positions, vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations);

    ConflictLocations conflict_locations  = {vertex_conflict_locations, same_direction_edge_conflict_locations, opposite_direction_edge_conflict_locations,  conflict_counts};
    return conflict_locations;

}


ConflictCounts count_conflicts(const std::vector<std::vector<Position>> &agent_paths,
                    std::vector<Conflict> &vertex_conflict_locations,
                    std::vector<Conflict> &same_direction_edge_conflict_locations,
                    std::vector<Conflict> &opposite_direction_edge_conflict_locations)
{
    int num_agents = agent_paths.size();
    ConflictCounts conflict_counts = {0, 0, 0};

    vertex_conflict_locations.reserve(num_agents * num_agents);
    same_direction_edge_conflict_locations.reserve(num_agents * num_agents);
    opposite_direction_edge_conflict_locations.reserve(num_agents * num_agents);

    for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
            const std::vector<Position> &path_i = agent_paths[i];
            const std::vector<Position> &path_j = agent_paths[j];

            int max_timestep = std::min(path_i.size(), path_j.size());
            for (int t = 0; t < max_timestep; ++t) {
                // Check for vertex conflict
                if (path_i[t] == path_j[t]) {
                    vertex_conflict_locations.push_back({path_i[t], t});
                    conflict_counts.vertex_conflicts++;
                }

                // Check for edge conflict (same direction)
                if (t > 0 && (path_i[t - 1] == path_j[t - 1]) && (path_i[t] == path_j[t])) {
                    same_direction_edge_conflict_locations.push_back({path_i[t], t});
                    conflict_counts.same_direction_edge_conflicts++;
                }

                // Check for edge conflict (opposite direction)
                if (t > 0 && (path_i[t - 1] == path_j[t]) && (path_j[t - 1] == path_i[t])) {
                    opposite_direction_edge_conflict_locations.push_back({path_i[t], t});
                    opposite_direction_edge_conflict_locations.push_back({path_j[t], t});
                    conflict_counts.opposite_direction_edge_conflicts++;
                }
            }
        }
    }

    return conflict_counts;
}


HeatMap create_heatmap(cv::Mat const & map, std::vector<Bin> const & bins)
{
    cv::Mat heatmap = map.clone();
    std::map<int, int> heatranges;
    int max_count = 0;
    for(int i = 0; i < bins.size(); ++i)
    {
        if(bins[i].count > max_count)
        {
            max_count = bins[i].count;
        }
    }
    //Color bins
    if(max_count == 0)
        max_count = 1;
    for(int i = 0; i < bins.size(); ++i)
    {
        int x = bins[i].x;
        int y = bins[i].y;
        int count = bins[i].count;
        int heat_value = 255 * count / max_count;
        heatranges[heat_value] = count;
        heatmap.at<cv::Vec3b>(y, x) = cv::Vec3b(255 - heat_value, 255 - heat_value, 255);
    }

    return {heatranges, heatmap};   
}

void draw_heatmap(cv::Mat const & heatmap, std::string const & fname_map)
{
    cv::imwrite(fname_map, heatmap);
}

void save_heat_ranges(std::map<int, int> const & heatranges, std::string const & fname_heatranges)
{
    std::ofstream ofs(fname_heatranges);
    for(auto it = heatranges.begin(); it != heatranges.end(); ++it)
    {
        ofs << it->first << ", " << it->second << std::endl;
    }
    ofs.close();
}

void add_grid(cv::Mat &scaled_image, int scale_factor) {
    int spacing_x = 2 * scale_factor;
    int spacing_y = 2 * scale_factor;
    int rows = scaled_image.cols / spacing_x;
    int cols = scaled_image.rows / spacing_y;

    for (int i = 0; i <= cols; i++) {
        int x = i * spacing_x;
        cv::line(scaled_image, cv::Point(x, 0), cv::Point(x, scaled_image.rows), cv::Scalar(0, 0, 0), 1);
    }
    for (int i = 0; i <= rows; i++) {
        int y = i * spacing_y;
        cv::line(scaled_image, cv::Point(0, y), cv::Point(scaled_image.cols, y), cv::Scalar(0, 0, 0), 1);
    }
}

void add_grid_scaled(cv::Mat &scaled_image, int scale_factor) {
    int spacing_x = scale_factor;
    int spacing_y = scale_factor;
    int rows = scaled_image.rows / spacing_y;
    int cols = scaled_image.cols / spacing_x;

    for (int i = 0; i <= cols; i++) {
        int x = i * spacing_x;
        cv::line(scaled_image, cv::Point(x, 0), cv::Point(x, scaled_image.rows - 1), cv::Scalar(0, 0, 0), 1);
    }
    for (int i = 0; i <= rows; i++) {
        int y = i * spacing_y;
        cv::line(scaled_image, cv::Point(0, y), cv::Point(scaled_image.cols - 1, y), cv::Scalar(0, 0, 0), 1);
    }
}

void draw_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, int scale_factor)
{
    // Determine the width and height of the ASCII art
    int square_size = 2;
    int width = instance.m_num_of_cols * square_size;
    int height = instance.m_num_of_rows * square_size;

    // Create a black image with the specified width and height
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0)); 
    for(int i = 0; i < instance.m_my_graph.size(); ++i)
    {
        //Map graph coordinate to left_corner coordinate
        int row = instance.get_row_coordinate(i);
        int col = instance.get_col_coordinate(i);

        int new_row = row * square_size;
        int new_col = col * square_size;

        std::vector<std::pair<int,int>> square_coordinates = {
            std::make_pair(new_row, new_col),
            std::make_pair(new_row, new_col + 1),
            std::make_pair(new_row + 1, new_col),
            std::make_pair(new_row + 1, new_col + 1),
            }; // up, down, left, right coloring scheme

        if(!instance.valid_pos(i))
        {
            for(const auto & sc : square_coordinates)
            {
                if((height <= sc.first) || (width <= sc.second))
                    continue;

                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            }
            continue;
        }


        //Map direction
        std::vector<std::pair<int,int>> nns = {
            std::make_pair(row - 1, col),
            std::make_pair(row + 1, col),
            std::make_pair(row, col - 1),
            std::make_pair(row, col + 1)
        }; // up, down, left, right
        std::vector<int> nn_ids;
        for(const auto & nn : nns)
        {
            if((nn.first < 0) || (nn.second < 0))
                nn_ids.push_back(-1);
            else if(!instance.valid_move(i, instance.linearize_coordinate(nn.first, nn.second)))
                nn_ids.push_back(-1);
            else
                nn_ids.push_back(instance.linearize_coordinate(nn.first, nn.second));
        }

        //Nodes
        Node const* curr = &instance.m_my_graph[i];
        std::vector<Node const*> nn_nodes;
        for(int nn_id : nn_ids)
        {
            if(nn_id != -1)
                nn_nodes.push_back(&instance.m_my_graph[nn_id]);
            else
                nn_nodes.push_back(nullptr);
        }

        //Coloring
        for(int i_sc = 0; i_sc < square_coordinates.size(); ++i_sc)
        {
            const auto & sc = square_coordinates[i_sc];          

            if((!instance.valid_pos(nn_ids[i_sc])) || (nn_ids[i_sc] == -1))
            {
                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 100;
                color[1] = 100;
                color[2] = 100;
            }
            else
            {
                double ph =  pheromones.read(curr,nn_nodes[i_sc]);
                double ph_opp = pheromones.read(nn_nodes[i_sc], curr);
                cv::Vec3b & color = img.at<cv::Vec3b>(sc.first,sc.second);
                color[0] = 0;
                color[1] = 255 * ph;
                color[2] = 255 * std::max(1 - ph, 0.);
            }
        }
    }
    //Save image
        // Scale the image
    // cv::imwrite(f_name, img);
    cv::Mat scaled_image;//(height*2, width*2, CV_8UC3, cv::Scalar(0, 0, 0)); 
    cv::resize(img, scaled_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);

    // Call the draw_grid function to draw the grid on the scaled_image
    add_grid(scaled_image, scale_factor);

    // Save the image
    cv::imwrite(f_name, scaled_image);
}

void draw_arrow_edge_map(std::string f_name, const Instance & instance, EdgeMap const & pheromones, double threshold, int scale_factor)
{
    cv::Mat map = create_map(instance);
    cv::Mat scaled_map = create_scaled_map(map, scale_factor);

    // Call the draw_grid function to draw the grid on the scaled_image
    add_grid_scaled(scaled_map, scale_factor);

    draw_pheromone_arrows(scaled_map, instance, pheromones, threshold, scale_factor);

    // Save the image
    cv::imwrite(f_name, scaled_map);
}

void draw_pheromone_arrows(cv::Mat& img, const Instance& instance, const EdgeMap& pheromones, double threshold, int scale_factor) {
    auto is_in_bounds = [&](int row, int col) {
        return row >= 0 && row < instance.m_num_of_rows && col >= 0 && col < instance.m_num_of_cols;
    };

    for (int node_idx = 0; node_idx < instance.m_my_graph.size(); ++node_idx) {
        int row = instance.get_row_coordinate(node_idx);
        int col = instance.get_col_coordinate(node_idx);

        if (!instance.valid_pos(node_idx)) {
            continue;
        }

        std::vector<std::pair<int, int>> neighbors = {
            std::make_pair(row - 1, col),
            std::make_pair(row + 1, col),
            std::make_pair(row, col - 1),
            std::make_pair(row, col + 1)
        }; // up, down, left, right

        std::pair<int, int> arrow_direction = std::make_pair(0, 0);
        Node const* curr_node = &instance.m_my_graph[node_idx];

        for (int neighbor_idx = 0; neighbor_idx < neighbors.size(); ++neighbor_idx) {
            const auto& neighbor = neighbors[neighbor_idx];
            if (!is_in_bounds(neighbor.first, neighbor.second)) continue;

            int neighbor_id = instance.linearize_coordinate(neighbor.first, neighbor.second);
            if (!instance.valid_pos(neighbor_id) || !instance.valid_move(node_idx, neighbor_id)) {
                continue;
            }

            Node const* neighbor_node = &instance.m_my_graph[neighbor_id];
            double ph_value = pheromones.read(curr_node, neighbor_node);

            if (ph_value >= threshold) {
                arrow_direction.first += neighbor.second - col;
                arrow_direction.second += neighbor.first - row;
            }
        }

        if (arrow_direction.first != 0 || arrow_direction.second != 0) {
            cv::Point start_point(col * scale_factor + scale_factor / 2, row * scale_factor + scale_factor / 2);
            cv::Point end_point(start_point.x + arrow_direction.first * scale_factor / 2, start_point.y + arrow_direction.second * scale_factor / 2);

            cv::Scalar arrow_color(0, 0, 255); // Red
            int arrow_thickness = 1;
            double tip_length = 0.3; // Increase the size of the arrowhead (relative to the arrow length)
            cv::arrowedLine(img, start_point, end_point, arrow_color, arrow_thickness, cv::LINE_AA, 0, tip_length);
        }
    }
}


cv::Mat create_scaled_map(const cv::Mat& map, int scale_factor) {
    cv::Mat scaled_map;
    cv::resize(map, scaled_map, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST_EXACT);
    return scaled_map;
}       


std::vector<Bin> conflict_locations_to_bins(std::unordered_map< int, std::unordered_map<int, int> > const & histogram2D)
{
    //Create bins from histogram2D
    std::vector<Bin> bins;
    for(auto it = histogram2D.cbegin(); it != histogram2D.cend(); ++it)
    {
        int x = it->first;
        for(auto it2 = it->second.cbegin(); it2 != it->second.cend(); ++it2)
        {
            int y = it2->first;
            int value = it2->second;
            Bin bin;
            bin.x = x;
            bin.y = y;
            bin.count = value;
            bins.push_back(bin);
        }
    }
    return bins;
}

std::unordered_map< int, std::unordered_map<int, int> > conflict_locations_to_histogram2d(ConflictLocations const & conflict_locations, int normalization)
{
    assert(0 < normalization);
    std::unordered_map< int, std::unordered_map<int, int> > histogram2D;
    for(int i = 0; i < conflict_locations.vertex_conflict_locations.size(); ++i)
    {
        Conflict const & conflict = conflict_locations.vertex_conflict_locations[i];
        Position const & pos = conflict.position;
        int x = pos.x;
        int y = pos.y;

        //if x and y exists in histogram2D add 1 to the value
        //else add x and y to histogram2D and set value to 1
        if(histogram2D.find(x) != histogram2D.end())
        {
            if(histogram2D[x].find(y) != histogram2D[x].end())
            {
                histogram2D[x][y] += 1;
            }
            else
            {
                histogram2D[x][y] = 1;
            }
        }
        else
        {
            histogram2D[x][y] = 1;
        }
    }
    for(int i = 0; i < conflict_locations.opposite_direction_edge_conflict_locations.size(); ++i)
    {
        Conflict const & conflict = conflict_locations.opposite_direction_edge_conflict_locations[i];
        Position const & pos = conflict.position;
        int x = pos.x;
        int y = pos.y;

        //if x and y exists in histogram2D add 1 to the value
        //else add x and y to histogram2D and set value to 1
        if(histogram2D.find(x) != histogram2D.end())
        {
            if(histogram2D[x].find(y) != histogram2D[x].end())
            {
                histogram2D[x][y] += 1;
            }
            else
            {
                histogram2D[x][y] = 1;
            }
        }
        else
        {
            histogram2D[x][y] = 1;
        }
    }
    //Normalize
    for(auto it = histogram2D.begin(); it != histogram2D.end(); ++it)
    {
        for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
        {
            it2->second = std::round((double)it2->second/(double)normalization);
        }
    }
    return histogram2D;
}

EdgeMap clamp(EdgeMap const & edgemap, double min, double max)
{
    EdgeMap result = edgemap;
    for(auto it = result.begin(); it != result.end(); ++it)
    {
        for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
        {
            it2->second = std::clamp(it2->second, min, max);
        }
    }
    return result;
}