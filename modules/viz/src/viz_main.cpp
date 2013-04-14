#include "precomp.hpp"

#include <opencv2/calib3d.hpp>
#include <q/shapes.h>
#include <q/viz3d_impl.hpp>

#include <vtkRenderWindowInteractor.h>
#ifndef __APPLE__
vtkRenderWindowInteractor* vtkRenderWindowInteractorFixNew ()
{
  return (vtkRenderWindowInteractor::New ());
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
temp_viz::Viz3d::VizImpl::VizImpl (const std::string &name)
    :  style_ (vtkSmartPointer<temp_viz::InteractorStyle>::New ())
    , cloud_actor_map_ (new CloudActorMap)
    , shape_actor_map_ (new ShapeActorMap)
    , s_lastDone_(0.0)
{
    renderer_ = vtkSmartPointer<vtkRenderer>::New ();

    // Create a RendererWindow
    window_ = vtkSmartPointer<vtkRenderWindow>::New ();

    // Set the window size as 1/2 of the screen size
    cv::Vec2i window_size = cv::Vec2i(window_->GetScreenSize()) / 2;
    window_->SetSize (window_size.val);

    window_->AddRenderer (renderer_);

    // Create the interactor style
    style_->Initialize ();
    style_->setRenderer (renderer_);
    style_->setCloudActorMap (cloud_actor_map_);
    style_->UseTimersOn ();

    /////////////////////////////////////////////////
    interactor_ = vtkSmartPointer <vtkRenderWindowInteractor>::Take (vtkRenderWindowInteractorFixNew ());

    //win_->PointSmoothingOn ();
    //win_->LineSmoothingOn ();
    //win_->PolygonSmoothingOn ();
    window_->AlphaBitPlanesOff ();
    window_->PointSmoothingOff ();
    window_->LineSmoothingOff ();
    window_->PolygonSmoothingOff ();
    window_->SwapBuffersOn ();
    window_->SetStereoTypeToAnaglyph ();

    interactor_->SetRenderWindow (window_);
    interactor_->SetInteractorStyle (style_);
    //interactor_->SetStillUpdateRate (30.0);
    interactor_->SetDesiredUpdateRate (30.0);

    // Initialize and create timer, also create window
    interactor_->Initialize ();
    timer_id_ = interactor_->CreateRepeatingTimer (5000L);

    // Set a simple PointPicker
    vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
    pp->SetTolerance (pp->GetTolerance () * 2);
    interactor_->SetPicker (pp);

    exit_main_loop_timer_callback_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New ();
    exit_main_loop_timer_callback_->viz_ = this;
    exit_main_loop_timer_callback_->right_timer_id = -1;
    interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

    exit_callback_ = vtkSmartPointer<ExitCallback>::New ();
    exit_callback_->viz_ = this;
    interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);

    resetStoppedFlag ();


    //////////////////////////////

    String window_name("Viz");
    window_name = name.empty() ? window_name : window_name + " - " + name;
    window_->SetWindowName (window_name.c_str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
temp_viz::Viz3d::VizImpl::~VizImpl ()
{
    if (interactor_ != NULL)
        interactor_->DestroyTimer (timer_id_);
    // Clear the collections
    //ren_->RemoveAllItems ();
    renderer_->Clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::saveScreenshot (const std::string &file) { style_->saveScreenshot (file); }

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection temp_viz::Viz3d::VizImpl::registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> callback)
{
    return (style_->registerKeyboardCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection temp_viz::Viz3d::VizImpl::registerMouseCallback (boost::function<void (const cv::MouseEvent&)> callback)
{
    return (style_->registerMouseCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection temp_viz::Viz3d::VizImpl::registerPointPickingCallback (boost::function<void (const cv::PointPickingEvent&)> callback)
{
    return (style_->registerPointPickingCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::spin ()
{
    resetStoppedFlag ();
    // Render the window before we start the interactor
    window_->Render ();
    interactor_->Start ();
}


namespace temp_viz
{
    inline double getTime ()
    {
      boost::posix_time::ptime epoch_time (boost::gregorian::date (1970, 1, 1));
      boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time ();
      return (static_cast<double>((current_time - epoch_time).total_nanoseconds ()) * 1.0e-9);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::spinOnce (int time, bool force_redraw)
{
    resetStoppedFlag ();

    if (time <= 0)
        time = 1;

    if (force_redraw)
        interactor_->Render ();

    double s_now_ = temp_viz::getTime();
    if (s_lastDone_ > s_now_)
      s_lastDone_ = s_now_;

    if ((s_now_ - s_lastDone_) > (1.0 / interactor_->GetDesiredUpdateRate ()))
    {
        exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer (time);
        interactor_->Start ();
        interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
        s_lastDone_ = s_now_;
    }
}

int feq (double a, double b) { return fabs (a - b) < 1e-9; }

//void quat_to_angle_axis (const Eigen::Quaternionf &qx, double &theta, double axis[3])
//{
//    double q[4];
//    q[0] = qx.w();
//    q[1] = qx.x();
//    q[2] = qx.y();
//    q[3] = qx.z();

//    double halftheta = acos (q[0]);
//    theta = halftheta * 2;
//    double sinhalftheta = sin (halftheta);
//    if (feq (halftheta, 0)) {
//        axis[0] = 0;
//        axis[1] = 0;
//        axis[2] = 1;
//        theta = 0;
//    } else {
//        axis[0] = q[1] / sinhalftheta;
//        axis[1] = q[2] / sinhalftheta;
//        axis[2] = q[3] / sinhalftheta;
//    }
//}


/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::addCoordinateSystem (double scale, const cv::Affine3f& affine, const std::string &id)
{
    vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
    axes->SetOrigin (0, 0, 0);
    axes->SetScaleFactor (scale);

    vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
    axes_colors->Allocate (6);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.0);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (0.5);
    axes_colors->InsertNextValue (1.0);
    axes_colors->InsertNextValue (1.0);

    vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
    axes_data->Update ();
    axes_data->GetPointData ()->SetScalars (axes_colors);

    vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
    axes_tubes->SetInput (axes_data);
    axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
    axes_tubes->SetNumberOfSides (6);

    vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    axes_mapper->SetScalarModeToUsePointData ();
    axes_mapper->SetInput (axes_tubes->GetOutput ());

    vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
    axes_actor->SetMapper (axes_mapper);

    cv::Vec3f t = affine.translation();
    axes_actor->SetPosition (t[0], t[1], t[2]);

    cv::Matx33f m = affine.rotation();

    cv::Vec3f rvec;
    cv::Rodrigues(m, rvec);

    float r_angle = cv::norm(rvec);
    rvec *= 1.f/r_angle;

//    Eigen::Quaternionf rf;
//    rf = Eigen::Quaternionf(m);
//    double r_angle;
//    double r_axis[3];
//    quat_to_angle_axis(rf,r_angle,r_axis);
    //
    axes_actor->SetOrientation(0,0,0);
    axes_actor->RotateWXYZ(r_angle*180/CV_PI,rvec[0], rvec[1], rvec[2]);
    //WAS:  axes_actor->SetOrientation (roll, pitch, yaw);

    // Save the ID and actor pair to the global actor map
    (*shape_actor_map_)[id] = axes_actor;
    addActorToRenderer (axes_actor);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeCoordinateSystem (const std::string &id)
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    if (am_it == shape_actor_map_->end ())
        return (false);

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second))
    {
        // Remove the ID pair to the global actor map
        shape_actor_map_->erase (am_it);
        return (true);
    }
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removePointCloud (const std::string &id)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return false;

    if (removeActorFromRenderer (am_it->second.actor))
        return cloud_actor_map_->erase (am_it), true;

    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeShape (const std::string &id)
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    // Extra step: check if there is a cloud with the same ID
    CloudActorMap::iterator ca_it = cloud_actor_map_->find (id);

    bool shape = true;
    // Try to find a shape first
    if (am_it == shape_actor_map_->end ())
    {
        // There is no cloud or shape with this ID, so just exit
        if (ca_it == cloud_actor_map_->end ())
            return false;
        // Cloud found, set shape to false
        shape = false;
    }

    // Remove the pointer/ID pair to the global actor map
    if (shape)
    {
        if (removeActorFromRenderer (am_it->second))
        {
            shape_actor_map_->erase (am_it);
            return (true);
        }
    }
    else
    {
        if (removeActorFromRenderer (ca_it->second.actor))
        {
            cloud_actor_map_->erase (ca_it);
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeText3D (const std::string &id)
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it == shape_actor_map_->end ())
        return false;

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second))
        return shape_actor_map_->erase (am_it), true;

    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeAllPointClouds ()
{
    // Check to see if the given ID entry exists
    CloudActorMap::iterator am_it = cloud_actor_map_->begin ();
    while (am_it != cloud_actor_map_->end () )
    {
        if (removePointCloud (am_it->first))
            am_it = cloud_actor_map_->begin ();
        else
            ++am_it;
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeAllShapes ()
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->begin ();
    while (am_it != shape_actor_map_->end ())
    {
        if (removeShape (am_it->first))
            am_it = shape_actor_map_->begin ();
        else
            ++am_it;
    }
    return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor)
{
    vtkLODActor* actor_to_remove = vtkLODActor::SafeDownCast (actor);



    // Iterate over all actors in this renderer
    vtkPropCollection* actors = renderer_->GetViewProps ();
    actors->InitTraversal ();

    vtkProp* current_actor = NULL;
    while ((current_actor = actors->GetNextProp ()) != NULL)
    {
        if (current_actor != actor_to_remove)
            continue;
        renderer_->RemoveActor (actor);
        //        renderer->Render ();
        // Found the correct viewport and removed the actor
        return (true);
    }

    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor)
{
    vtkActor* actor_to_remove = vtkActor::SafeDownCast (actor);

    // Add it to all renderers
    //rens_->InitTraversal ();


        // Iterate over all actors in this renderer
    vtkPropCollection* actors = renderer_->GetViewProps ();
    actors->InitTraversal ();
    vtkProp* current_actor = NULL;
    while ((current_actor = actors->GetNextProp ()) != NULL)
    {
        if (current_actor != actor_to_remove)
            continue;
        renderer_->RemoveActor (actor);
        //        renderer->Render ();
        // Found the correct viewport and removed the actor
        return (true);
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::addActorToRenderer (const vtkSmartPointer<vtkProp> &actor)
{
    // Add it to all renderers
    //rens_->InitTraversal ();

    renderer_->AddActor (actor);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor)
{
    vtkProp* actor_to_remove = vtkProp::SafeDownCast (actor);

    // Initialize traversal
    //rens_->InitTraversal ();


    // Iterate over all actors in this renderer
    vtkPropCollection* actors = renderer_->GetViewProps ();
    actors->InitTraversal ();
    vtkProp* current_actor = NULL;
    while ((current_actor = actors->GetNextProp ()) != NULL)
    {
        if (current_actor != actor_to_remove)
            continue;
        renderer_->RemoveActor (actor);
        //        renderer->Render ();
        // Found the correct viewport and removed the actor
        return (true);
    }

    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkLODActor> &actor, bool use_scalars)
{
    if (!actor)
        actor = vtkSmartPointer<vtkLODActor>::New ();

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
    mapper->SetInput (data);

    if (use_scalars)
    {
        vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
        if (scalars)
        {
            cv::Vec3d minmax(scalars->GetRange());
            mapper->SetScalarRange(minmax.val);
            mapper->SetScalarModeToUsePointData ();

            // interpolation OFF, if data is a vtkPolyData that contains only vertices, ON for anything else.
            vtkPolyData* polyData = vtkPolyData::SafeDownCast (data);
            bool interpolation = (polyData && polyData->GetNumberOfCells () != polyData->GetNumberOfVerts ());

            mapper->SetInterpolateScalarsBeforeMapping (interpolation);
            mapper->ScalarVisibilityOn ();
        }
    }
    mapper->ImmediateModeRenderingOff ();

    actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
    actor->GetProperty ()->SetInterpolationToFlat ();

    /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
    /// shown when there is a vtkActor with backface culling on present in the scene
    /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
    // actor->GetProperty ()->BackfaceCullingOn ();

    actor->SetMapper (mapper);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setBackgroundColor (const Color& color)
{
    Color c = vtkcolor(color);
    renderer_->SetBackground (c.val);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setPointCloudColor (const Color& color, const std::string &id)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it != cloud_actor_map_->end ())
    {
        vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

        Color c = vtkcolor(color);
        actor->GetProperty ()->SetColor (c.val);
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->Modified ();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::getPointCloudRenderingProperties (int property, double &value, const std::string &id)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return false;

    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

    switch (property)
    {
        case PCL_VISUALIZER_POINT_SIZE:
        {
            value = actor->GetProperty ()->GetPointSize ();
            actor->Modified ();
            break;
        }
        case PCL_VISUALIZER_OPACITY:
        {
            value = actor->GetProperty ()->GetOpacity ();
            actor->Modified ();
            break;
        }
        case PCL_VISUALIZER_LINE_WIDTH:
        {
            value = actor->GetProperty ()->GetLineWidth ();
            actor->Modified ();
            break;
        }
        default:
            CV_Assert("getPointCloudRenderingProperties: Unknown property");
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::setPointCloudRenderingProperties (int property, double value, const std::string &id)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return std::cout << "[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <" << id << ">!" << std::endl, false;

    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

    switch (property)
    {
        case PCL_VISUALIZER_POINT_SIZE:
        {
            actor->GetProperty ()->SetPointSize (float (value));
            actor->Modified ();
            break;
        }
        case PCL_VISUALIZER_OPACITY:
        {
            actor->GetProperty ()->SetOpacity (value);
            actor->Modified ();
            break;
        }
            // Turn on/off flag to control whether data is rendered using immediate
            // mode or note. Immediate mode rendering tends to be slower but it can
            // handle larger datasets. The default value is immediate mode off. If you
            // are having problems rendering a large dataset you might want to consider
            // using immediate more rendering.
        case PCL_VISUALIZER_IMMEDIATE_RENDERING:
        {
            actor->GetMapper ()->SetImmediateModeRendering (int (value));
            actor->Modified ();
            break;
        }
        case PCL_VISUALIZER_LINE_WIDTH:
        {
            actor->GetProperty ()->SetLineWidth (float (value));
            actor->Modified ();
            break;
        }
        default:
            CV_Assert("setPointCloudRenderingProperties: Unknown property");
    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::setPointCloudSelected (const bool selected, const std::string &id)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return std::cout << "[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <" << id << ">!" << std::endl, false;

    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
    if (selected)
    {
        actor->GetProperty ()->EdgeVisibilityOn ();
        actor->GetProperty ()->SetEdgeColor (1.0, 0.0, 0.0);
        actor->Modified ();
    }
    else
    {
        actor->GetProperty ()->EdgeVisibilityOff ();
        actor->Modified ();
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setShapeColor (const Color& color, const std::string &id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        vtkActor* actor = vtkActor::SafeDownCast (am_it->second);

        Color c = vtkcolor(color);
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->GetProperty ()->SetColor (c.val);
        actor->GetProperty ()->SetEdgeColor (c.val);
        actor->GetProperty ()->SetAmbient (0.8);
        actor->GetProperty ()->SetDiffuse (0.8);
        actor->GetProperty ()->SetSpecular (0.8);
        actor->GetProperty ()->SetLighting (0);
        actor->Modified ();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::setShapeRenderingProperties (int property, double value, const std::string &id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it == shape_actor_map_->end ())
        return std::cout << "[setShapeRenderingProperties] Could not find any shape with id <" << id << ">!\n" << std::endl, false;

    vtkActor* actor = vtkActor::SafeDownCast (am_it->second);

    switch (property)
    {
    case PCL_VISUALIZER_POINT_SIZE:
    {
        actor->GetProperty ()->SetPointSize (float (value));
        actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_OPACITY:
    {
        actor->GetProperty ()->SetOpacity (value);
        actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_LINE_WIDTH:
    {
        actor->GetProperty ()->SetLineWidth (float (value));
        actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_FONT_SIZE:
    {
        vtkTextActor* text_actor = vtkTextActor::SafeDownCast (am_it->second);
        vtkSmartPointer<vtkTextProperty> tprop = text_actor->GetTextProperty ();
        tprop->SetFontSize (int (value));
        text_actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_REPRESENTATION:
    {
        switch (int (value))
        {
            case PCL_VISUALIZER_REPRESENTATION_POINTS:    actor->GetProperty ()->SetRepresentationToPoints (); break;
            case PCL_VISUALIZER_REPRESENTATION_WIREFRAME: actor->GetProperty ()->SetRepresentationToWireframe (); break;
            case PCL_VISUALIZER_REPRESENTATION_SURFACE:   actor->GetProperty ()->SetRepresentationToSurface ();  break;
        }
        actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_SHADING:
    {
        switch (int (value))
        {
        case PCL_VISUALIZER_SHADING_FLAT: actor->GetProperty ()->SetInterpolationToFlat (); break;
        case PCL_VISUALIZER_SHADING_GOURAUD:
        {
            if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetNormals ())
            {
                std::cout << "[temp_viz::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Gouraud shading was requested. Estimating normals...\n" << std::endl;

                vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New ();
                normals->SetInput (actor->GetMapper ()->GetInput ());
                normals->Update ();
                vtkDataSetMapper::SafeDownCast (actor->GetMapper ())->SetInput (normals->GetOutput ());
            }
            actor->GetProperty ()->SetInterpolationToGouraud ();
            break;
        }
        case PCL_VISUALIZER_SHADING_PHONG:
        {
            if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetNormals ())
            {
                std::cout << "[temp_viz::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Phong shading was requested. Estimating normals...\n" << std::endl;
                vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New ();
                normals->SetInput (actor->GetMapper ()->GetInput ());
                normals->Update ();
                vtkDataSetMapper::SafeDownCast (actor->GetMapper ())->SetInput (normals->GetOutput ());
            }
            actor->GetProperty ()->SetInterpolationToPhong ();
            break;
        }
        }
        actor->Modified ();
        break;
    }
    default:
        CV_Assert("setShapeRenderingProperties: Unknown property");

    }
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::initCameraParameters ()
{
    Camera camera_temp;
    // Set default camera parameters to something meaningful
    camera_temp.clip = Vec2d(0.01, 1000.01);

    // Look straight along the z-axis
    camera_temp.focal = Vec3d(0.0, 0.0, 1.0);

    // Position the camera at the origin
    camera_temp.pos = Vec3d(0.0, 0.0, 0.0);

    // Set the up-vector of the camera to be the y-axis
    camera_temp.view_up = Vec3d(0.0, 1.0, 0.0);

    // Set the camera field of view to about
    camera_temp.fovy = 0.8575;
    camera_temp.window_size = Vec2i(window_->GetScreenSize()) / 2;
    camera_temp.window_pos = Vec2i(0, 0);

    setCameraParameters (camera_temp);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::cameraParamsSet () const { return (camera_set_); }

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::updateCamera ()
{
    std::cout << "[temp_viz::PCLVisualizer::updateCamera()] This method was deprecated, just re-rendering all scenes now." << std::endl;
    //rens_->InitTraversal ();
    // Update the camera parameters

    renderer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::updateShapePose (const std::string &id, const cv::Affine3f& pose)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    vtkLODActor* actor;

    if (am_it == shape_actor_map_->end ())
        return (false);
    else
        actor = vtkLODActor::SafeDownCast (am_it->second);

    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();

    convertToVtkMatrix (pose.matrix, matrix);

    actor->SetUserMatrix (matrix);
    actor->Modified ();

    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::getCameras (temp_viz::Camera& camera)
{
    vtkCamera* active_camera = renderer_->GetActiveCamera ();

    camera.pos = cv::Vec3d(active_camera->GetPosition());
    camera.focal = cv::Vec3d(active_camera->GetFocalPoint());
    camera.clip = cv::Vec2d(active_camera->GetClippingRange());
    camera.view_up = cv::Vec3d(active_camera->GetViewUp());

    camera.fovy = active_camera->GetViewAngle()/ 180.0 * CV_PI;
    camera.window_size = cv::Vec2i(renderer_->GetRenderWindow()->GetSize());
    camera.window_pos = cv::Vec2d::all(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////
cv::Affine3f temp_viz::Viz3d::VizImpl::getViewerPose ()
{
    vtkCamera& camera = *renderer_->GetActiveCamera ();

    Vec3d pos(camera.GetPosition());
    Vec3d view_up(camera.GetViewUp());
    Vec3d focal(camera.GetFocalPoint());

    Vec3d y_axis = normalized(view_up);
    Vec3d z_axis = normalized(focal - pos);
    Vec3d x_axis = normalized(y_axis.cross(z_axis));

    cv::Matx33d R;
    R(0, 0) = x_axis[0];
    R(0, 1) = y_axis[0];
    R(0, 2) = z_axis[0];

    R(1, 0) = x_axis[1];
    R(1, 1) = y_axis[1];
    R(1, 2) = z_axis[1];

    R(2, 0) = x_axis[2];
    R(2, 1) = y_axis[2];
    R(2, 2) = z_axis[2];

    return cv::Affine3f(R, pos);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::resetCamera ()
{
    renderer_->ResetCamera ();
}


/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraPosition (const cv::Vec3d& pos, const cv::Vec3d& view, const cv::Vec3d& up)
{

    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetPosition (pos[0], pos[1], pos[2]);
    cam->SetFocalPoint (view[0], view[1], view[2]);
    cam->SetViewUp (up[0], up[1], up[2]);
    renderer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z)
{
    //rens_->InitTraversal ();


    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetPosition (pos_x, pos_y, pos_z);
    cam->SetViewUp (up_x, up_y, up_z);
    renderer_->Render ();

}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraParameters (const cv::Matx33f& intrinsics, const cv::Affine3f& extrinsics)
{
    // Position = extrinsic translation
    cv::Vec3f pos_vec = extrinsics.translation();


    // Rotate the view vector
    cv::Matx33f rotation = extrinsics.rotation();
    cv::Vec3f y_axis (0.f, 1.f, 0.f);
    cv::Vec3f up_vec (rotation * y_axis);

    // Compute the new focal point
    cv::Vec3f z_axis (0.f, 0.f, 1.f);
    cv::Vec3f focal_vec = pos_vec + rotation * z_axis;

    // Get the width and height of the image - assume the calibrated centers are at the center of the image
    Eigen::Vector2i window_size;
    window_size[0] = static_cast<int> (intrinsics(0, 2));
    window_size[1] = static_cast<int> (intrinsics(1, 2));

    // Compute the vertical field of view based on the focal length and image heigh
    double fovy = 2 * atan (window_size[1] / (2. * intrinsics (1, 1))) * 180.0 / M_PI;

    //rens_->InitTraversal ();


    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetPosition (pos_vec[0], pos_vec[1], pos_vec[2]);
    cam->SetFocalPoint (focal_vec[0], focal_vec[1], focal_vec[2]);
    cam->SetViewUp (up_vec[0], up_vec[1], up_vec[2]);
    cam->SetUseHorizontalViewAngle (0);
    cam->SetViewAngle (fovy);
    cam->SetClippingRange (0.01, 1000.01);
    window_->SetSize (window_size[0], window_size[1]);

    renderer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraParameters (const temp_viz::Camera &camera)
{
    //rens_->InitTraversal ();


    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetPosition (camera.pos[0], camera.pos[1], camera.pos[2]);
    cam->SetFocalPoint (camera.focal[0], camera.focal[1], camera.focal[2]);
    cam->SetViewUp (camera.view_up[0], camera.view_up[1], camera.view_up[2]);
    cam->SetClippingRange (camera.clip.val);
    cam->SetUseHorizontalViewAngle (0);
    cam->SetViewAngle (camera.fovy * 180.0 / M_PI);

    window_->SetSize (static_cast<int> (camera.window_size[0]), static_cast<int> (camera.window_size[1]));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraClipDistances (double near, double far)
{
    //rens_->InitTraversal ();

    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetClippingRange (near, far);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setCameraFieldOfView (double fovy)
{
    //rens_->InitTraversal ();

    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetUseHorizontalViewAngle (0);
    cam->SetViewAngle (fovy * 180.0 / M_PI);

}

/////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::resetCameraViewpoint (const std::string &id)
{
    vtkSmartPointer<vtkMatrix4x4> camera_pose;
    static CloudActorMap::iterator it = cloud_actor_map_->find (id);
    if (it != cloud_actor_map_->end ())
        camera_pose = it->second.viewpoint_transformation_;
    else
        return;

    // Prevent a segfault
    if (!camera_pose)
        return;

    // set all renderer to this viewpoint
    //rens_->InitTraversal ();


    vtkSmartPointer<vtkCamera> cam = renderer_->GetActiveCamera ();
    cam->SetPosition (camera_pose->GetElement (0, 3),
                      camera_pose->GetElement (1, 3),
                      camera_pose->GetElement (2, 3));

    cam->SetFocalPoint (camera_pose->GetElement (0, 3) - camera_pose->GetElement (0, 2),
                        camera_pose->GetElement (1, 3) - camera_pose->GetElement (1, 2),
                        camera_pose->GetElement (2, 3) - camera_pose->GetElement (2, 2));

    cam->SetViewUp (camera_pose->GetElement (0, 1),
                    camera_pose->GetElement (1, 1),
                    camera_pose->GetElement (2, 1));

    renderer_->SetActiveCamera (cam);
    renderer_->ResetCameraClippingRange ();
    renderer_->Render ();
}


////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addCylinder (const temp_viz::ModelCoefficients &coefficients, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addCylinder] A shape with id <"<<id <<"> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCylinder (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addCube (const temp_viz::ModelCoefficients &coefficients, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addCube] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCube (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addCube (const cv::Vec3f& translation, const cv::Vec3f quaternion, double width, double height, double depth, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addCube] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    Eigen::Vector3f t(translation[0], translation[1], translation[2]);
    Eigen::Quaternionf q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

    vtkSmartPointer<vtkDataSet> data = createCube (t, q, width, height, depth);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                                            const Color& color , const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
        return std::cout << "[addCube] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl, false;

    vtkSmartPointer<vtkDataSet> data = createCube (x_min, x_max, y_min, y_max, z_min, z_max);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);

    Color c = vtkcolor(color);
    actor->GetProperty ()->SetColor (c.val);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, const std::string & id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addModelFromPolyData] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (polydata, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string & id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addModelFromPolyData] A shape with id <"<<id<<"> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    vtkSmartPointer <vtkTransformFilter> trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    trans_filter->SetTransform (transform);
    trans_filter->SetInput (polydata);
    trans_filter->Update();

    // Create an Actor
    vtkSmartPointer <vtkLODActor> actor;
    createActorFromVTKDataSet (trans_filter->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}


////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addModelFromPLYFile (const std::string &filename, const std::string &id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addModelFromPLYFile] A shape with id <"<<id<<"> already exists! Please choose a different id and retry.." << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
    reader->SetFileName (filename.c_str ());

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (reader->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addModelFromPLYFile (const std::string &filename, vtkSmartPointer<vtkTransform> transform, const std::string &id)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
        return std::cout << "[addModelFromPLYFile] A shape with id <"<<id<<"> already exists! Please choose a different id and retry." << std::endl, false;


    vtkSmartPointer <vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
    reader->SetFileName (filename.c_str ());

    //create transformation filter
    vtkSmartPointer <vtkTransformFilter> trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    trans_filter->SetTransform (transform);
    trans_filter->SetInputConnection (reader->GetOutputPort ());

    // Create an Actor
    vtkSmartPointer <vtkLODActor> actor;
    createActorFromVTKDataSet (trans_filter->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}


////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Add a plane from a set of given model coefficients
  * \param coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
  * \param id the plane id/name (default: "plane")
  * \param viewport (optional) the id of the new viewport (default: 0)
  */
bool temp_viz::Viz3d::VizImpl::addPlane (const temp_viz::ModelCoefficients &coefficients, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addPlane] A shape with id <"<<id<<"> already exists! Please choose a different id and retry." << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createPlane (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    //  actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetRepresentationToSurface ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

bool temp_viz::Viz3d::VizImpl::addPlane (const temp_viz::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addPlane] A shape with id <" << id << "> already exists! Please choose a different id and retry.\n" << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createPlane (coefficients, x, y, z);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addCircle (const temp_viz::ModelCoefficients &coefficients, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        std::cout << "[addCircle] A shape with id <"<<id<<"> already exists! Please choose a different id and retry.\n" << std::endl;
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = create2DCircle (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//void temp_viz::Viz3d::VizImpl::createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)
//{
////    // Create a new renderer
////    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
////    ren->SetViewport (xmin, ymin, xmax, ymax);

////    if (rens_->GetNumberOfItems () > 0)
////        ren->SetActiveCamera (rens_->GetFirstRenderer ()->GetActiveCamera ());
////    ren->ResetCamera ();

////    // Add it to the list of renderers
////    rens_->AddItem (ren);

////    if (rens_->GetNumberOfItems () <= 1)          // If only one renderer
////        viewport = 0;                               // set viewport to 'all'
////    else
////        viewport = rens_->GetNumberOfItems () - 1;

////    win_->AddRenderer (ren);
////    win_->Modified ();
//}

////////////////////////////////////////////////////////////////////////////////////////////
//void temp_viz::Viz3d::VizImpl::createViewPortCamera ()
//{
//    vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New ();
//    //rens_->InitTraversal ();
//    vtkRenderer* renderer = ren_;
//    renderer->SetActiveCamera (cam);
//    renderer->ResetCamera ();
//}

/////////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::addText (const std::string &text, int xpos, int ypos, const Color& color, int fontsize, const std::string &id)
{
   std::string tid = id.empty() ? text : id;

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (tid);
    if (am_it != shape_actor_map_->end ())
        return std::cout << "[addText] A text with id <"<<id <<"> already exists! Please choose a different id and retry.\n" << std::endl, false;

    // Create an Actor
    vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New ();
    actor->SetPosition (xpos, ypos);
    actor->SetInput (text.c_str ());

    vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
    tprop->SetFontSize (fontsize);
    tprop->SetFontFamilyToArial ();
    tprop->SetJustificationToLeft ();
    tprop->BoldOn ();

    Color c = vtkcolor(color);
    tprop->SetColor (c.val);
    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[tid] = actor;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool temp_viz::Viz3d::VizImpl::updateText (const std::string &text, int xpos, int ypos, const Color& color, int fontsize, const std::string &id)
{
    std::string tid = id.empty() ? text : id;

    ShapeActorMap::iterator am_it = shape_actor_map_->find (tid);
    if (am_it == shape_actor_map_->end ())
        return false;

    // Retrieve the Actor
    vtkTextActor *actor = vtkTextActor::SafeDownCast (am_it->second);

    actor->SetPosition (xpos, ypos);
    actor->SetInput (text.c_str ());

    vtkTextProperty* tprop = actor->GetTextProperty ();
    tprop->SetFontSize (fontsize);

    Color c = vtkcolor(color);
    tprop->SetColor (c.val);

    actor->Modified ();

    return (true);
}

bool temp_viz::Viz3d::VizImpl::addPolylineFromPolygonMesh (const Mesh3d& mesh, const std::string &id)
{
    CV_Assert(mesh.cloud.rows == 1 && mesh.cloud.type() == CV_32FC3);

    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
        return std::cout << "[addPolylineFromPolygonMesh] A shape with id <"<< id << "> already exists! Please choose a different id and retry.\n" << std::endl, false;

    vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
    poly_points->SetNumberOfPoints (mesh.cloud.size().area());

    const cv::Point3f *cdata = mesh.cloud.ptr<cv::Point3f>();
    for (int i = 0; i < mesh.cloud.cols; ++i)
        poly_points->InsertPoint (i, cdata[i].x, cdata[i].y,cdata[i].z);


    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer <vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer <vtkPolyData> polyData;
    allocVtkPolyData (polyData);

    for (size_t i = 0; i < mesh.polygons.size (); i++)
    {
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
        polyLine->GetPointIds()->SetNumberOfIds(mesh.polygons[i].vertices.size());
        for(unsigned int k = 0; k < mesh.polygons[i].vertices.size(); k++)
        {
            polyLine->GetPointIds()->SetId(k,mesh. polygons[i].vertices[k]);
        }

        cells->InsertNextCell (polyLine);
    }

    // Add the points to the dataset
    polyData->SetPoints (poly_points);

    // Add the lines to the dataset
    polyData->SetLines (cells);

    // Setup actor and mapper
    vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    mapper->SetInput (polyData);

    vtkSmartPointer < vtkActor > actor = vtkSmartPointer<vtkActor>::New ();
    actor->SetMapper (mapper);


    addActorToRenderer (actor);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;

    return (true);

}


///////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setRepresentationToSurfaceForAllActors ()
{
    vtkActorCollection * actors = renderer_->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()) != NULL)
        actor->GetProperty ()->SetRepresentationToSurface ();
}

///////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setRepresentationToPointsForAllActors ()
{
    vtkActorCollection * actors = renderer_->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()) != NULL)
        actor->GetProperty ()->SetRepresentationToPoints ();
}

///////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::setRepresentationToWireframeForAllActors ()
{
    vtkActorCollection * actors = renderer_->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()) != NULL)
        actor->GetProperty ()->SetRepresentationToWireframe ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::updateCells (vtkSmartPointer<vtkIdTypeArray> &cells, vtkSmartPointer<vtkIdTypeArray> &initcells, vtkIdType nr_points)
{
    // If no init cells and cells has not been initialized...
    if (!cells)
        cells = vtkSmartPointer<vtkIdTypeArray>::New ();

    // If we have less values then we need to recreate the array
    if (cells->GetNumberOfTuples () < nr_points)
    {
        cells = vtkSmartPointer<vtkIdTypeArray>::New ();

        // If init cells is given, and there's enough data in it, use it
        if (initcells && initcells->GetNumberOfTuples () >= nr_points)
        {
            cells->DeepCopy (initcells);
            cells->SetNumberOfComponents (2);
            cells->SetNumberOfTuples (nr_points);
        }
        else
        {
            // If the number of tuples is still too small, we need to recreate the array
            cells->SetNumberOfComponents (2);
            cells->SetNumberOfTuples (nr_points);
            vtkIdType *cell = cells->GetPointer (0);
            // Fill it with 1s
            std::fill_n (cell, nr_points * 2, 1);
            cell++;
            for (vtkIdType i = 0; i < nr_points; ++i, cell += 2)
                *cell = i;
            // Save the results in initcells
            initcells = vtkSmartPointer<vtkIdTypeArray>::New ();
            initcells->DeepCopy (cells);
        }
    }
    else
    {
        // The assumption here is that the current set of cells has more data than needed
        cells->SetNumberOfComponents (2);
        cells->SetNumberOfTuples (nr_points);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::allocVtkPolyData (vtkSmartPointer<vtkAppendPolyData> &polydata)
{
    polydata = vtkSmartPointer<vtkAppendPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::allocVtkPolyData (vtkSmartPointer<vtkPolyData> &polydata)
{
    polydata = vtkSmartPointer<vtkPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::Viz3d::VizImpl::allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata)
{
    polydata = vtkSmartPointer<vtkUnstructuredGrid>::New ();
}

////////////////////////////////////////////////////////////////////////////////////////////////
//void temp_viz::Viz3d::getTransformationMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternion<float> &orientation, Eigen::Matrix4f &transformation)
//{
//    transformation.setIdentity ();
//    transformation.block<3,3>(0,0) = orientation.toRotationMatrix ();
//    transformation.block<3,1>(0,3) = origin.head (3);
//}

//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::convertToVtkMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternion<float> &orientation, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
    // set rotation
    Eigen::Matrix3f rot = orientation.toRotationMatrix ();
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            vtk_matrix->SetElement (i, k, rot (i, k));

    // set translation
    vtk_matrix->SetElement (0, 3, origin (0));
    vtk_matrix->SetElement (1, 3, origin (1));
    vtk_matrix->SetElement (2, 3, origin (2));
    vtk_matrix->SetElement (3, 3, 1.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//void temp_viz::convertToVtkMatrix (const Eigen::Matrix4f &m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
//{
//    for (int i = 0; i < 4; i++)
//        for (int k = 0; k < 4; k++)
//            vtk_matrix->SetElement (i, k, m (i, k));
//}

void temp_viz::convertToVtkMatrix (const cv::Matx44f &m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            vtk_matrix->SetElement (i, k, m (i, k));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void temp_viz::convertToEigenMatrix (const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix, Eigen::Matrix4f &m)
{
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            m (i,k) = static_cast<float> (vtk_matrix->GetElement (i, k));
}
