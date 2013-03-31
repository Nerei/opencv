#pragma once
#include <vector>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/Vertices.h>

namespace pcl
{
  struct PolygonMesh
  {
    ::sensor_msgs::PointCloud2 cloud;
    std::vector< ::pcl::Vertices>  polygons;
  };
}

