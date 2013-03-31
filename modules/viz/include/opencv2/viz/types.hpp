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


    template <typename PointT>
    class CV_EXPORTS PlanarPolygon
    {
    public:
        typedef boost::shared_ptr<PlanarPolygon<PointT> > Ptr;
        typedef boost::shared_ptr<const PlanarPolygon<PointT> > ConstPtr;

        PlanarPolygon () {}

        /** \brief Constructor for PlanarPolygon
            * \param[in] contour a vector of points bounding the polygon
            * \param[in] coefficients a vector of the plane's coefficients (a,b,c,d)
            */
        PlanarPolygon (typename pcl::PointCloud<PointT>::VectorType &contour, Eigen::Vector4f& coefficients)
            : contour_ (contour), coefficients_ (coefficients) {}

        virtual ~PlanarPolygon () {}

        /** \brief Set the internal contour
            * \param[in] contour the new planar polygonal contour
            */
        void setContour (const pcl::PointCloud<PointT> &contour) { contour_ = contour.points; }

        /** \brief Getter for the contour / boundary */
        typename pcl::PointCloud<PointT>::VectorType& getContour (){ return (contour_); }

        /** \brief Getter for the contour / boundary */
        const typename pcl::PointCloud<PointT>::VectorType& getContour () const { return (contour_); }

        /** \brief Setr the internal coefficients
            * \param[in] coefficients the new coefficients to be set
            */
        void setCoefficients (const Eigen::Vector4f &coefficients) { coefficients_ = coefficients; }

        /** \brief Set the internal coefficients
            * \param[in] coefficients the new coefficients to be set
            */
        void setCoefficients (const pcl::ModelCoefficients &coefficients)
        {
            for (int i = 0; i < 4; i++)
                coefficients_[i] = coefficients.values.at (i);
        }

        /** \brief Getter for the coefficients */
        Eigen::Vector4f& getCoefficients () { return (coefficients_); }

        /** \brief Getter for the coefficients */
        const Eigen::Vector4f& getCoefficients () const { return (coefficients_); }

    protected:
        /** \brief A list of points on the boundary/contour of the planar region. */
        typename pcl::PointCloud<PointT>::VectorType contour_;

        /** \brief A list of model coefficients (a,b,c,d). */
        Eigen::Vector4f coefficients_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
