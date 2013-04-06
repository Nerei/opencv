#pragma once

#include <opencv2/core/cvdef.h>
#include <opencv2/core.hpp>
#include <vtkMatrix4x4.h>
#include <Eigen/Core>

namespace temp_viz
{


    CV_EXPORTS Eigen::Matrix4d vtkToEigen (vtkMatrix4x4* vtk_matrix);
    CV_EXPORTS Eigen::Vector2i worldToView (const Eigen::Vector4d &world_pt, const Eigen::Matrix4d &view_projection_matrix, int width, int height);
    CV_EXPORTS void getViewFrustum (const Eigen::Matrix4d &view_projection_matrix, double planes[24]);

    enum FrustumCull
    {
        PCL_INSIDE_FRUSTUM,
        PCL_INTERSECT_FRUSTUM,
        PCL_OUTSIDE_FRUSTUM
    };

    CV_EXPORTS int cullFrustum (double planes[24], const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb);
    CV_EXPORTS float viewScreenArea (const Eigen::Vector3d &eye, const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, const Eigen::Matrix4d &view_projection_matrix, int width, int height);

    enum RenderingProperties
    {
        PCL_VISUALIZER_POINT_SIZE,
        PCL_VISUALIZER_OPACITY,
        PCL_VISUALIZER_LINE_WIDTH,
        PCL_VISUALIZER_FONT_SIZE,
        PCL_VISUALIZER_COLOR,
        PCL_VISUALIZER_REPRESENTATION,
        PCL_VISUALIZER_IMMEDIATE_RENDERING,
        PCL_VISUALIZER_SHADING
    };

    enum RenderingRepresentationProperties
    {
        PCL_VISUALIZER_REPRESENTATION_POINTS,
        PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        PCL_VISUALIZER_REPRESENTATION_SURFACE
    };

    enum ShadingRepresentationProperties
    {
        PCL_VISUALIZER_SHADING_FLAT,
        PCL_VISUALIZER_SHADING_GOURAUD,
        PCL_VISUALIZER_SHADING_PHONG
    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Camera class holds a set of camera parameters together with the window pos/size. */
    class CV_EXPORTS Camera
    {
    public:
        /** \brief Focal point or lookAt.
                      * \note The view direction can be obtained by (focal-pos).normalized ()
                      */
        double focal[3];

        /** \brief Position of the camera. */
        double pos[3];

        /** \brief Up vector of the camera.
                      * \note Not to be confused with the view direction, bad naming here. */
        double view[3];

        /** \brief Clipping planes depths.
                      * clip[0] is near clipping plane, and clip [1] is the far clipping plane
                      */
        double clip[2];

        /** \brief Field of view angle in y direction (radians). */
        double fovy;

        // the following variables are the actual position and size of the window on the screen and NOT the viewport!
        // except for the size, which is the same the viewport is assumed to be centered and same size as the window.
        double window_size[2];
        double window_pos[2];


        /** \brief Computes View matrix for Camera (Based on gluLookAt)
                      * \param[out] view_mat the resultant matrix
                      */
        void computeViewMatrix (Eigen::Matrix4d& view_mat) const;

        /** \brief Computes Projection Matrix for Camera
                      *  \param[out] proj the resultant matrix
                      */
        void computeProjectionMatrix (Eigen::Matrix4d& proj) const;

        /** \brief converts point to window coordiantes
                      * \param[in] pt xyz point to be converted
                      * \param[out] window_cord vector containing the pts' window X,Y, Z and 1
                      *
                      * This function computes the projection and view matrix every time.
                      * It is very inefficient to use this for every point in the point cloud!
                      */
        void cvtWindowCoordinates (const cv::Point3f& pt, Eigen::Vector4d& window_cord) const
        {
            Eigen::Matrix4d proj, view;
            this->computeViewMatrix (view);
            this->computeProjectionMatrix (proj);
            this->cvtWindowCoordinates (pt, window_cord, proj*view);
            return;
        }

        /** \brief converts point to window coordiantes
                      * \param[in] pt xyz point to be converted
                      * \param[out] window_cord vector containing the pts' window X,Y, Z and 1
                      * \param[in] composite_mat composite transformation matrix (proj*view)
                      *
                      * Use this function to compute window coordinates with a precomputed
                      * transformation function.  The typical composite matrix will be
                      * the projection matrix * the view matrix.  However, additional
                      * matrices like a camera disortion matrix can also be added.
                      */
        void cvtWindowCoordinates (const cv::Point3f& pt, Eigen::Vector4d& window_cord, const Eigen::Matrix4d& composite_mat) const
        {
            Eigen::Vector4d pte (pt.x, pt.y, pt.z, 1);
            window_cord = composite_mat * pte;
            window_cord = window_cord/window_cord (3);
            window_cord[0] = (window_cord[0]+1.0) / 2.0*window_size[0];
            window_cord[1] = (window_cord[1]+1.0) / 2.0*window_size[1];
            window_cord[2] = (window_cord[2]+1.0) / 2.0;
        }
    };

}
