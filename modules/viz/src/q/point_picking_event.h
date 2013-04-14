#pragma once

#include <opencv2/core/cvdef.h>

#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

namespace temp_viz
{
    class CV_EXPORTS PointPickingCallback : public vtkCommand
    {
    public:
        static PointPickingCallback *New () { return (new PointPickingCallback); }

        PointPickingCallback () : x_ (0), y_ (0), z_ (0), idx_ (-1), pick_first_ (false) {}
        virtual ~PointPickingCallback () {}
        virtual void Execute (vtkObject *caller, unsigned long eventid, void*);
        int performSinglePick (vtkRenderWindowInteractor *iren);
        int performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z);

    private:
        float x_, y_, z_;
        int idx_;
        bool pick_first_;
    };

} //namespace temp_viz

