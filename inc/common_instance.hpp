#pragma once

#include <opencv2/opencv.hpp>
#include "instance.hpp"

cv::Mat create_map(Instance const & instance);

void draw_missions(cv::Mat & mat, Instance const & instance, std::vector<int> const & mission_idxs);