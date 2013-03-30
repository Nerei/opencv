/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2008-2013, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and / or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "test_precomp.hpp"
#include <opencv2/viz.hpp>
#include <q/visualization/window.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_load()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream ifs("d:/cloud_dragon.ply");

    std::string str;
    for(size_t i = 0; i < 11; ++i)
        std::getline(ifs, str);

    for(size_t i = 0; i < 20000; ++i)
    {
        pcl::PointXYZ p;
        ifs >> p.x >> p.y >> p.z;
        cloud->push_back(p);
    }
    return cloud;
}

TEST(Viz_viz3d, accuracy)
{
    //cv::Window wnd("wind");
    //wnd.spin();

    pcl::visualization::PCLVisualizer v;

    v.addCoordinateSystem(1.0, Eigen::Affine3f::Identity());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cloud_load();

    cv::Mat data(1, cloud->size(), CV_32FC4, (void*)&cloud->points[0]);

    std::vector<cv::Mat> channels;
    cv::split(data, channels);
    channels.resize(3);
    cv::merge(channels, data);

    cv::Mat colors(data.size(), CV_8UC3, cv::Scalar(0, 255, 0));
    v.addPointCloud(data, colors);


    cv::Mat normals(data.size(), CV_32FC3, cv::Scalar(0, 10, 0));

    v.addPointCloudNormals(data, normals, 100, 0.02, "n");

    //pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> hander;
    //v.addPointCloud<pcl::PointXYZ>(cloud, hander);
    //v.addPointCloud<pcl::PointXYZ>(cloud);

    v.spinOnce(1000, true);

    v.addText("===Abd sadfljsadlk", 100, 100, cv::Scalar(255, 0, 0), 15);


    //v.updatePointCloud<pcl::PointXYZ>(cloud, hander);

    //channels[0] *= 7;
    cv::merge(channels, data);
    colors.setTo(cv::Scalar(255, 0, 0));

    std::cout << "aaa" << std::endl;

    v.addSphere(pcl::PointXYZ(0, 0, 0), 0.3, 0, 0, 1);
    v.updatePointCloud(data, colors);
    v.spin();
}
