#include <q/visualization/point_picking_event.h>
#include <q/visualization/interactor_style.h>
#include <vtkPointPicker.h>
#include <vtkRendererCollection.h>

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PointPickingCallback::Execute (vtkObject *caller, unsigned long eventid, void*)
{
    vtkRenderWindowInteractor* iren = reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->GetInteractor ();

    if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetShiftKey () > 0))
    {
        float x = 0, y = 0, z = 0;
        int idx = performSinglePick (iren, x, y, z);
        // Create a PointPickingEvent if a point was selected
        if (idx != -1)
        {
            cv::PointPickingEvent event (idx, x, y, z);
            reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->point_picking_signal_ (event);
        }
    }
    else if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetAltKey () == 1))
    {
        pick_first_ = !pick_first_;
        float x = 0, y = 0, z = 0;
        int idx = -1;
        if (pick_first_)
            idx_ = performSinglePick (iren, x_, y_, z_);
        else
            idx = performSinglePick (iren, x, y, z);
        // Create a PointPickingEvent
        cv::PointPickingEvent event (idx_, idx, x_, y_, z_, x, y, z);
        reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->point_picking_signal_ (event);
    }
    // Call the parent's class mouse events
    reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->OnLeftButtonDown ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
int pcl::visualization::PointPickingCallback::performSinglePick (vtkRenderWindowInteractor *iren)
{
    int mouse_x, mouse_y;
    //vtkPointPicker *picker = reinterpret_cast<vtkPointPicker*> (iren->GetPicker ());
    vtkPointPicker *picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());

    if (!picker)
    {
        std::cout << "Point picker not available, not selecting any points!" << std::endl;
        return -1;
    }
    //iren->GetMousePosition (&mouse_x, &mouse_y);
    mouse_x = iren->GetEventPosition ()[0];
    mouse_y = iren->GetEventPosition ()[1];
    iren->StartPickCallback ();

    vtkRenderer *ren = iren->FindPokedRenderer (iren->GetEventPosition ()[0], iren->GetEventPosition ()[1]);
    picker->Pick (mouse_x, mouse_y, 0.0, ren);
    return (static_cast<int> (picker->GetPointId ()));
}

/////////////////////////////////////////////////////////////////////////////////////////////
int pcl::visualization::PointPickingCallback::performSinglePick (vtkRenderWindowInteractor *iren, float &x, float &y, float &z)
{
    int mouse_x, mouse_y;
    // vtkPointPicker *picker = reinterpret_cast<vtkPointPicker*> (iren->GetPicker ());
    vtkPointPicker *picker = vtkPointPicker::SafeDownCast (iren->GetPicker ());

    if (!picker)
    {
        std::cout << "Point picker not available, not selecting any points!" << std::endl;
        return -1;
    }
    //iren->GetMousePosition (&mouse_x, &mouse_y);
    mouse_x = iren->GetEventPosition ()[0];
    mouse_y = iren->GetEventPosition ()[1];
    iren->StartPickCallback ();

    vtkRenderer *ren = iren->FindPokedRenderer (iren->GetEventPosition ()[0], iren->GetEventPosition ()[1]);
    picker->Pick (mouse_x, mouse_y, 0.0, ren);

    int idx = static_cast<int> (picker->GetPointId ());
    if (picker->GetDataSet () != NULL)
    {
        double p[3];
        picker->GetDataSet ()->GetPoint (idx, p);
        x = float (p[0]); y = float (p[1]); z = float (p[2]);
    }
    return (idx);
}

