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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <string>


cv::Mat cvcloud_load()
{
    cv::Mat cloud(1, 20000, CV_32FC3);
        std::ifstream ifs("d:/cloud_dragon.ply");

    std::string str;
    for(size_t i = 0; i < 11; ++i)
        std::getline(ifs, str);

    cv::Point3f* data = cloud.ptr<cv::Point3f>();
    for(size_t i = 0; i < 20000; ++i)
        ifs >> data[i].x >> data[i].y >> data[i].z;

    return cloud;
}

void mesh_load(std::vector<pcl::Vertices>& polygons, cv::Mat& cloud, cv::Mat& colors, const char* file = "d:/horse.ply")
{
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(file);
    reader->Update();
    vtkSmartPointer<vtkPolyData> poly_data = reader->GetOutput ();

    typedef unsigned int uint32_t;
    polygons.clear();

    vtkSmartPointer<vtkPoints> mesh_points = poly_data->GetPoints ();
    vtkIdType nr_points = mesh_points->GetNumberOfPoints ();
    vtkIdType nr_polygons = poly_data->GetNumberOfPolys ();

    cloud.create(1, nr_points, CV_32FC3);

    double point_xyz[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
    {
        mesh_points->GetPoint (i, &point_xyz[0]);
        cloud.ptr<cv::Point3f>()[i] = cv::Point3d(point_xyz[0], point_xyz[1], point_xyz[2]);;
    }

    // Then the color information, if any
    vtkUnsignedCharArray* poly_colors = NULL;
    if (poly_data->GetPointData() != NULL)
        poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("Colors"));

    // some applications do not save the name of scalars (including PCL's native vtk_io)
    if (!poly_colors && poly_data->GetPointData () != NULL)
        poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("scalars"));

    if (!poly_colors && poly_data->GetPointData () != NULL)
        poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("RGB"));

    // TODO: currently only handles rgb values with 3 components
    if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
    {
        colors.create(1, nr_points, CV_8UC3);
        unsigned char point_color[3];

        for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); i++)
        {
            poly_colors->GetTupleValue (i, &point_color[0]);

            //RGB or BGR?????
            colors.ptr<cv::Vec3b>()[i] = cv::Vec3b(point_color[0], point_color[1], point_color[2]);
        }
    }
    else
        colors.release();

    // Now handle the polygons
    polygons.resize (nr_polygons);
    vtkIdType* cell_points;
    vtkIdType nr_cell_points;
    vtkCellArray * mesh_polygons = poly_data->GetPolys ();
    mesh_polygons->InitTraversal ();
    int id_poly = 0;
    while (mesh_polygons->GetNextCell (nr_cell_points, cell_points))
    {
        polygons[id_poly].vertices.resize (nr_cell_points);
        for (int i = 0; i < nr_cell_points; ++i)
            polygons[id_poly].vertices[i] = static_cast<int> (cell_points[i]);
        ++id_poly;
    }
    //return xyzrgb_cloud;

}

void convert_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& points, cv::Mat& cloud, cv::Mat& colors)
{
    cloud.create(points->height, points->width, CV_32FC3);
    colors.create(cloud.size(), CV_8UC3);

    std::cout << points->height << ", " << points->width << ", " << cloud.cols << std::endl;

    for(int x = 0; x < cloud.cols; ++x)
    {
        pcl::PointXYZRGB p = points->points[x];
        cloud.ptr<cv::Point3f>()[x] = cv::Point3f(p.x, p.y, p.z);
        colors.ptr<cv::Vec3b>()[x] = cv::Vec3b(p.b, p.g, p.r);
    }
}

TEST(Viz_viz3d, accuracy)
{
    pcl::visualization::PCLVisualizer v;

    v.addCoordinateSystem(1.0, Eigen::Affine3f::Identity());

    cv::Mat cloud = cvcloud_load();

    cv::Mat colors(cloud.size(), CV_8UC3, cv::Scalar(0, 255, 0));
    v.addPointCloud(cloud, colors);
    cv::Mat normals(cloud.size(), CV_32FC3, cv::Scalar(0, 10, 0));

    v.addPointCloudNormals(cloud, normals, 100, 0.02, "n");

    pcl::ModelCoefficients mc;
    mc.values.resize(4);
    mc.values[0] = mc.values[1] = mc.values[2] = mc.values[3] = 1;
    v.addPlane(mc);

    std::vector<pcl::Vertices> polygons;
    cv::Mat me_cl, me_co;
    mesh_load(polygons, me_cl, me_co, "d:/horse.ply");


    v.addPolygonMesh(me_cl, me_co, cv::Mat(), polygons, "pq");


    v.spinOnce(1000, true);


    for(int i = 0; i < me_cl.cols; ++i)
        me_cl.ptr<cv::Point3f>()[i] += cv::Point3f(1, 1, 1);


    v.updatePolygonMesh(me_cl, me_co, cv::Mat(), polygons, "pq");


    v.addText("===Abd sadfljsadlk", 100, 100, cv::Scalar(255, 0, 0), 15);
        for(int i = 0; i < cloud.cols; ++i)
        cloud.ptr<cv::Point3f>()[i].x *=2;

    colors.setTo(cv::Scalar(255, 0, 0));

    v.addSphere(pcl::PointXYZ(0, 0, 0), 0.3, 0, 0, 1);

    cv::Mat cvpoly(1, 5, CV_32FC3);
    cv::Point3f* pdata = cvpoly.ptr<cv::Point3f>();
    pdata[0] = cv::Point3f(0, 0, 0);
    pdata[1] = cv::Point3f(0, 1, 1);
    pdata[2] = cv::Point3f(3, 1, 2);
    pdata[3] = cv::Point3f(0, 2, 4);
    pdata[4] = cv::Point3f(7, 2, 3);
    v.addPolygon(cvpoly);

    v.updatePointCloud(cloud, colors);
    v.spin();
}
