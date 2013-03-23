/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCL_POINT_CLOUD_COLOR_HANDLERS_H_
#define PCL_POINT_CLOUD_COLOR_HANDLERS_H_

#if defined __GNUC__
#pragma GCC system_header
#endif

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <q/visualization/common/common.h>
// VTK includes
#include <vtkSmartPointer.h>
#include <vtkDataArray.h>
#include <vtkFloatArray.h>
#include <vtkUnsignedCharArray.h>

namespace pcl
{
  namespace visualization
  {
    template <typename PointT>
    class PointCloudColorHandler
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<PointCloudColorHandler<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandler<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandler () :
          cloud_ (), capable_ (false), field_idx_ (-1), fields_ ()
        {}

        /** \brief Constructor. */
        PointCloudColorHandler (const PointCloudConstPtr &cloud) :
          cloud_ (cloud), capable_ (false), field_idx_ (-1), fields_ ()
        {}

        /** \brief Destructor. */
        virtual ~PointCloudColorHandler () {}

        /** \brief Check if this handler is capable of handling the input data or not. */
        inline bool
        isCapable () const { return (capable_); }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const = 0;

        /** \brief Abstract getFieldName method. */
        virtual std::string
        getFieldName () const = 0;

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const = 0;

        /** \brief Set the input cloud to be used.
          * \param[in] cloud the input cloud to be used by the handler
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud)
        {
          cloud_ = cloud;
        }

      protected:
        /** \brief A pointer to the input dataset. */
        PointCloudConstPtr cloud_;

        /** \brief True if this handler is capable of handling the input data, false
          * otherwise.
          */
        bool capable_;

        /** \brief The index of the field holding the data that represents the color. */
        int field_idx_;

        /** \brief The list of fields available for this PointCloud. */
        std::vector<sensor_msgs::PointField> fields_;
    };

    //////////////////////////////////////////////////////////////////////////////////////
    /** \brief Handler for random PointCloud colors (i.e., R, G, B will be randomly chosen)
      * \author Radu B. Rusu 
      * \ingroup visualization
      */
    template <typename PointT>
    class PointCloudColorHandlerRandom : public PointCloudColorHandler<PointT>
    {
      typedef typename PointCloudColorHandler<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:
        typedef boost::shared_ptr<PointCloudColorHandlerRandom<PointT> > Ptr;
        typedef boost::shared_ptr<const PointCloudColorHandlerRandom<PointT> > ConstPtr;

        /** \brief Constructor. */
        PointCloudColorHandlerRandom () :
          PointCloudColorHandler<PointT> ()
        {
          capable_ = true;
        }

        /** \brief Constructor. */
        PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud) :
          PointCloudColorHandler<PointT> (cloud)
        {
          capable_ = true;
        }

        /** \brief Abstract getName method. */
        virtual std::string
        getName () const { return ("PointCloudColorHandlerRandom"); }

        /** \brief Get the name of the field used. */
        virtual std::string
        getFieldName () const { return ("[random]"); }

        /** \brief Obtain the actual color for the input dataset as vtk scalars.
          * \param[out] scalars the output scalars containing the color for the dataset
          * \return true if the operation was successful (the handler is capable and 
          * the input cloud was given as a valid pointer), false otherwise
          */
        virtual bool
        getColor (vtkSmartPointer<vtkDataArray> &scalars) const;

      protected:
        // Members derived from the base class
        using PointCloudColorHandler<PointT>::cloud_;
        using PointCloudColorHandler<PointT>::capable_;
    };
  }
}

#include <q/visualization/impl/point_cloud_color_handlers.hpp>

#endif      // PCL_POINT_CLOUD_COLOR_HANDLERS_H_

