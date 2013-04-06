#pragma once

#include <vector>
#include <opencv2/core/cvdef.h>

namespace temp_viz
{



    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };
}
