#pragma once

#include <vector>
#include <vtkLODActor.h>
#include <vtkSmartPointer.h>
#include <opencv2/core/cvdef.h>
#include <q/3rdparty.h>
#include <map>
#include <opencv2/core.hpp>

#include "precomp.hpp"

namespace temp_viz
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

    typedef std::map<std::string, CloudActor> CloudActorMap;
    typedef cv::Ptr<CloudActorMap> CloudActorMapPtr;

    typedef std::map<std::string, vtkSmartPointer<vtkProp> > ShapeActorMap;
    typedef cv::Ptr<ShapeActorMap> ShapeActorMapPtr;


    class RenWinInteract
    {
      public:

        RenWinInteract () : xy_plot_ (vtkSmartPointer<vtkXYPlotActor>::New ()),
                            ren_ (vtkSmartPointer<vtkRenderer>::New ()),
                            win_ (vtkSmartPointer<vtkRenderWindow>::New ()),
                            interactor_ (),
                            style_ ()
        {}

        /** \brief The XY plot actor holding the actual data. */
        vtkSmartPointer<vtkXYPlotActor> xy_plot_;

        /** \brief The renderer used. */
        vtkSmartPointer<vtkRenderer> ren_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The render window interactor. */
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
        /** \brief The render window interactor style. */
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style_;
    };

    typedef std::map<std::string, RenWinInteract> RenWinInteractMap;
}

