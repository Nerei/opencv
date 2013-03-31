#pragma once

#include <vector>
#include <pcl/common/eigen.h>
#include <q/point_cloud.h>
#include "q/PointCloud2.h"
#include <opencv2/core/cvdef.h>

namespace pcl
{
    struct CV_EXPORTS Vertices
    {
        std::vector<unsigned int> vertices;
    };

    struct CV_EXPORTS PolygonMesh
    {
        ::sensor_msgs::PointCloud2 cloud;
        std::vector< ::pcl::Vertices>  polygons;
    };

    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };
}
