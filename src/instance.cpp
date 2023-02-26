#include "../inc/instance.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

// Tokenizing a string using stringstream
#include <bits/stdc++.h>

Instance::Instance(const std::string& map_fname, const std::string& agent_fname,
                 int num_of_agents, const std::string& agent_indices)
                 : m_map_fname{map_fname}, m_agent_fname{agent_fname},
                     m_num_of_agents{num_of_agents}, m_agent_indices{agent_indices}
{
    bool succ = load_map();
    if(!succ)
        std::cout << "Could not load map from pathname " << m_map_fname << std::endl;
    succ = load_agents();
    if(!succ)
        std::cout << "Could not load agents from pathname " << m_agent_fname << std::endl;

    succ = load_graph();
    if(!succ)
        std::cout << "Could not load graph from map" << std::endl;
}

bool Instance::load_map()
{
	using namespace std;
	ifstream myfile(m_map_fname);
	if (!myfile.is_open())
		return false;

	string line;
	getline(myfile, line);
    char sep = ' ';
    string token;


    getline(myfile, line);
    stringstream ss_height(line);

    getline(ss_height, token, sep); // Name of info (height)
    getline(ss_height, token, sep); // Height

    m_num_of_rows = stoi(token); // read number of rows

    getline(myfile, line);
    stringstream ss_width(line);

    getline(ss_width, token, sep); // Name of info (width)
    getline(ss_width, token, sep); // Width


    m_num_of_cols = stoi(token); // read number of cols

    getline(myfile, line); // skip "map"

	m_map_size = m_num_of_cols * m_num_of_rows;
	m_my_map.resize(m_map_size, false);
	// read map (and start/goal locations)
	for (int i = 0; i < m_num_of_rows; i++)
	{
		getline(myfile, line);
		for (int j = 0; j < m_num_of_cols; j++)
		{
			m_my_map[linearize_coordinate(i, j)] = (line[j] != '.');
		}
	}
	myfile.close();

	return true;
}

bool Instance::load_agents()
{
	using namespace std;
	ifstream myfile(m_agent_fname);
	if (!myfile.is_open())
		return false;
    
	string line;
	getline(myfile, line);

    if (m_num_of_agents < 1)
    {
        cout << "The number of agents should be larger than 0" << endl;
        return false;
    }
    m_start_locations.resize(m_num_of_agents);
    m_goal_locations.resize(m_num_of_agents);
    m_optimal_lengths.resize(m_num_of_agents);

    vector<int> ids(m_num_of_agents);
    for (int i = 0; i < m_num_of_agents; i++)
        ids[i] = i;

    char sep('\t');	
    int count = 0;
    int i = 0;
    while(i < m_num_of_agents)
    {
        getline(myfile, line);
        stringstream ss(line);
        std::string token;
        for(int i_skip = 0; i_skip < 4; ++i_skip)
            getline(ss, token, sep);// skip the first number// skip the map name// skip the columns// skip the rows

        getline(ss, token, sep);
        // read start [x,y] for agent i
        int col = stoi(token);
        getline(ss, token, sep);
        int row = stoi(token);
        m_start_locations[i] = linearize_coordinate(row, col);
        // read goal [row,col] for agent i
        getline(ss, token, sep);
        col = stoi(token);
        getline(ss, token, sep);
        row = stoi(token);
        m_goal_locations[i] = linearize_coordinate(row, col);
        // read optimal length
        getline(ss, token, sep);
        m_optimal_lengths[i] = stod(token);
        i++;
        count++;
    }

    myfile.close();
    return true;
}

bool Instance::load_graph()
{
    if(m_my_map.size() == 0)
        return false;
    
    // Build nodes
    m_my_graph.resize(m_my_map.size());
    for(int i = 0; i < m_my_map.size(); ++i)
    {
        m_my_graph[i].vertex_id = i;
        std::vector<int> neighbors = get_neighbors(i);
        for(int i_nb = 0; i_nb < neighbors.size(); ++i_nb)
        {
            int nb = neighbors[i_nb];
            Node* p_nb = &m_my_graph[nb];
            m_my_graph[i].edges[p_nb] = get_manhattan_distance(i, nb);
        }
    }
    
    return true;
}

bool Instance::valid_pos(int pos) const
{
	if ((pos < 0) || (pos >= m_map_size))
		return false;
	if (m_my_map[pos])
		return false;
    return true;
}


bool Instance::valid_move(int curr, int next) const
{
	if ((next < 0) || (next >= m_map_size))
		return false;
	if (m_my_map[next])
		return false;
	return get_manhattan_distance(curr, next) < 2;
}

std::vector<int> Instance::get_neighbors(int curr) const
{
    std::vector<int> neighbors;
	std::vector<int> candidates = {curr + 1, curr - 1, curr + m_num_of_cols, curr - m_num_of_cols};
	for(int next : candidates)
	{
		if (valid_move(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

bool Instance::save_map(std::string out_fname) const
{
	std::ofstream myfile;
	myfile.open(out_fname);
	if (!myfile.is_open())
	{
		std::cout << "Fail to save the map to " << out_fname << std::endl;
		return false;
	}
	myfile << m_num_of_rows << "," << m_num_of_cols << std::endl;
	for (int i = 0; i < m_num_of_rows; i++)
	{
		for (int j = 0; j < m_num_of_cols; j++)
		{
			if (m_my_map[linearize_coordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << std::endl;
	}
	myfile.close();
    return true;
}

bool Instance::map_route_to_image(std::string out_fname, const std::vector<int> & path) const
{
    std::unordered_set<int> set_path(path.begin(), path.end());
    
    // Determine the width and height of the ASCII art
    int width = m_num_of_cols;
    int height = m_num_of_rows;

    // Create a black image with the specified width and height
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

	for (int i = 0; i < m_num_of_rows; i++)
	{
		for (int j = 0; j < m_num_of_cols; j++)
		{
            int pos = linearize_coordinate(i, j);
            if(set_path.find(pos) != set_path.end())
            {
                if((*set_path.find(pos)) == path.front())
                {
                    // get pixel
                    cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
                    color[0] = 0;
                    color[1] = 0;
                    color[2] = 255;
                }
                else if((*set_path.find(pos)) == path[path.size()-1])
                {
                    cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
                    color[0] = 0;
                    color[1] = 255;
                    color[2] = 0;
                }
                else
                {
                    cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
                    color[0] = 255;
                    color[1] = 0;
                    color[2] = 0;
                }
            }
			else if (m_my_map[pos])
            {
                // get pixel
                cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
                color[0] = 105;
                color[1] = 105;
                color[2] = 105;
            }
			else
            {
                cv::Vec3b & color = img.at<cv::Vec3b>(i,j);
                color[0] = 255;
                color[1] = 255;
                color[2] = 255;
            }

		}
	}
	// Save the image
    return cv::imwrite(out_fname, img);
}
