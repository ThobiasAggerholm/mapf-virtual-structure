#include "../inc/common_instance.hpp"
#include <algorithm>

std::vector<cv::Vec3b> generate_rgb_colors(int n) {
    std::vector<cv::Vec3b> colors;
    colors.reserve(n);
    // Compute color step
    float step = 179.0f / (n);

    // Generate colors
    for (int i = 0; i < n; i++) {
        cv::Vec3b color;
        color[0] = (int)(step * i);  // H channel
        color[1] = 255;  // S channel
        color[2] = 255;  // V channel
        colors.push_back(color);
    }

    // Convert colors from HSV to RGB
    cv::Mat hsv(n, 1, CV_8UC3);
    cv::Mat bgr(n, 1, CV_8UC3);
    for (int i = 0; i < n; i++) {
        hsv.at<cv::Vec3b>(i, 0) = colors[i];
    }
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    for (int i = 0; i < n; i++) {
        colors[i] = bgr.at<cv::Vec3b>(i, 0);
    }

    return colors;
}



cv::Mat create_map(Instance const & instance)
{
    if(instance.instance_loaded == false)
        return cv::Mat();
        
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

void draw_missions(cv::Mat & mat, Instance const & instance, std::vector<int> const & mission_idxs)
{
    if(instance.instance_loaded == false)
        return;

    int max_mission_idx = *std::max_element(mission_idxs.begin(), mission_idxs.end());

    if(max_mission_idx >= instance.m_goal_locations.size() || max_mission_idx >= instance.m_start_locations.size())
        return;

    //Create unique colors for each mission
    std::vector<cv::Vec3b> colors = generate_rgb_colors(mission_idxs.size());

    
    for(int i = 0; i < mission_idxs.size(); i++)
    {
        int mission_idx = mission_idxs[i];
        int start = instance.m_start_locations[mission_idx];
        int goal = instance.m_goal_locations[mission_idx];

        int start_row = instance.get_row_coordinate(start);
        int start_col = instance.get_col_coordinate(start);

        int goal_row = instance.get_row_coordinate(goal);
        int goal_col = instance.get_col_coordinate(goal);

        cv::Point start_point(start_col, start_row);
        cv::Point goal_point(goal_col, goal_row);

        //Draw points in pixels
        cv::circle(mat, start_point, 0, colors[mission_idx], 1);
        cv::circle(mat, goal_point, 0, colors[mission_idx], 1);
    }
}
