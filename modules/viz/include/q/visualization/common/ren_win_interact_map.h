#pragma once

#include <q/visualization/vtk.h>
#include <map>

namespace pcl
{
  namespace visualization
  {
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
}
