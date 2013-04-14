#pragma once

#include <opencv2/core.hpp>
#include <opencv2/viz/types.hpp>
#include <vector>

namespace temp_viz
{
    CV_EXPORTS void mesh_load(std::vector<temp_viz::Vertices>& polygons, cv::Mat& cloud, cv::Mat& colors, const std::string& file);
}
