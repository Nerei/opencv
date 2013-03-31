#pragma once

#include <vector>
#include <vtkLODActor.h>
#include <vtkSmartPointer.h>
#include <opencv2/core/cvdef.h>
#include <q/visualization/3rdparty.h>

namespace pcl
{
  namespace visualization
  {
    class CV_EXPORTS CloudActor
    {
      public:
        virtual ~CloudActor () {}
        /** \brief The actor holding the data to render. */
        vtkSmartPointer<vtkLODActor> actor;

        /** \brief The active color handler. */
        int color_handler_index_;

        /** \brief The active geometry handler. */
        int geometry_handler_index_;

        /** \brief The viewpoint transformation matrix. */
        vtkSmartPointer<vtkMatrix4x4> viewpoint_transformation_;

        /** \brief Internal cell array. Used for optimizing updatePointCloud. */
        vtkSmartPointer<vtkIdTypeArray> cells;
    };

    typedef boost::unordered_map<std::string, CloudActor> CloudActorMap;
    typedef boost::shared_ptr<CloudActorMap> CloudActorMapPtr;

    typedef boost::unordered_map<std::string, vtkSmartPointer<vtkProp> > ShapeActorMap;
    typedef boost::shared_ptr<ShapeActorMap> ShapeActorMapPtr;

    typedef std::map<int, vtkSmartPointer<vtkProp> > CoordinateActorMap;
  }
}

