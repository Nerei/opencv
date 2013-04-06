#pragma once

#include <vector>
#include <opencv2/core/cvdef.h>

namespace pcl
{
    struct CV_EXPORTS Vertices
    {
        std::vector<unsigned int> vertices;
    };


    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };
}
