#pragma once

#include <vector>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/cvdef.h>
#include<sensor_msgs/PointCloud2.h>


namespace pcl
{

#if 1
    struct CV_EXPORTS Vertices
    {
        std::vector<unsigned int> vertices;
    };

    struct CV_EXPORTS PolygonMesh
    {
        ::sensor_msgs::PointCloud2 cloud;
        std::vector< ::pcl::Vertices>  polygons;
    };
#endif

    struct CV_EXPORTS ModelCoefficients
    {
        std::vector<float> values;
    };
}
