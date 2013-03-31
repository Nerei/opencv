/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */


#include <pcl/ros/conversions.h>
#include <pcl/common/io.h>
#include <q/visualization/common/shapes.h>
#include <pcl/common/time.h>
#include <opencv2/viz/viz3d.hpp>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>
#include <vtkPolyDataNormals.h>
#include <vtkMapper.h>
#include <vtkDataSetMapper.h>

#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>


#include <vtkSelection.h>
#include <vtkPointPicker.h>

#include <q/visualization/3rdparty.h>
//#include <q/visualization/vtk/vtkVertexBufferObjectMapper.h>
#include <q/visualization/vtk/vtkRenderWindowInteractorFix.h>

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::PCLVisualizer (const std::string &name, const bool create_interactor)
    : interactor_ ()
    , stopped_ ()
    , timer_id_ ()
    , exit_main_loop_timer_callback_ ()
    , exit_callback_ ()
    , rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
    , win_ ()
    , style_ (vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New ())
    , cloud_actor_map_ (new CloudActorMap)
    , shape_actor_map_ (new ShapeActorMap)
    , coordinate_actor_map_ ()
    , camera_set_ ()
{
    // Create a Renderer
    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
    // Add it to the list of renderers
    rens_->AddItem (ren);

    // Create a RendererWindow
    win_ = vtkSmartPointer<vtkRenderWindow>::New ();
    win_->SetWindowName (name.c_str ());

    // Get screen size
    int *scr_size = win_->GetScreenSize ();
    // Set the window size as 1/2 of the screen size
    win_->SetSize (scr_size[0] / 2, scr_size[1] / 2);


    // Add all renderers to the window
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
        win_->AddRenderer (renderer);

    // Create the interactor style
    style_->Initialize ();
    style_->setRendererCollection (rens_);
    style_->setCloudActorMap (cloud_actor_map_);
    style_->UseTimersOn ();

    if (create_interactor)
        createInteractor ();

    win_->SetWindowName (name.c_str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::createInteractor ()
{
    interactor_ = vtkSmartPointer <vtkRenderWindowInteractor>::Take (vtkRenderWindowInteractorFixNew ());

    //win_->PointSmoothingOn ();
    //win_->LineSmoothingOn ();
    //win_->PolygonSmoothingOn ();
    win_->AlphaBitPlanesOff ();
    win_->PointSmoothingOff ();
    win_->LineSmoothingOff ();
    win_->PolygonSmoothingOff ();
    win_->SwapBuffersOn ();
    win_->SetStereoTypeToAnaglyph ();

    interactor_->SetRenderWindow (win_);
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
    exit_main_loop_timer_callback_->pcl_visualizer = this;
    exit_main_loop_timer_callback_->right_timer_id = -1;
    interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

    exit_callback_ = vtkSmartPointer<ExitCallback>::New ();
    exit_callback_->pcl_visualizer = this;
    interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);

    resetStoppedFlag ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::~PCLVisualizer ()
{
    if (interactor_ != NULL)
        interactor_->DestroyTimer (timer_id_);
    // Clear the collections
    rens_->RemoveAllItems ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::saveScreenshot (const std::string &file) { style_->saveScreenshot (file); }

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection pcl::visualization::PCLVisualizer::registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> callback)
{
    return (style_->registerKeyboardCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection pcl::visualization::PCLVisualizer::registerMouseCallback (boost::function<void (const cv::MouseEvent&)> callback)
{
    return (style_->registerMouseCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection pcl::visualization::PCLVisualizer::registerPointPickingCallback (boost::function<void (const cv::PointPickingEvent&)> callback)
{
    return (style_->registerPointPickingCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::spin ()
{
    resetStoppedFlag ();
    // Render the window before we start the interactor
    win_->Render ();
    interactor_->Start ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::spinOnce (int time, bool force_redraw)
{
    resetStoppedFlag ();

    if (time <= 0)
        time = 1;

    if (force_redraw)
        interactor_->Render ();

    DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
              exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer (time);
            interactor_->Start ();
    interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
}

int feq (double a, double b) { return fabs (a - b) < 1e-9; }

void quat_to_angle_axis (const Eigen::Quaternionf &qx, double &theta, double axis[3])
{
    double q[4];
    q[0] = qx.w();
    q[1] = qx.x();
    q[2] = qx.y();
    q[3] = qx.z();

    double halftheta = acos (q[0]);
    theta = halftheta * 2;
    double sinhalftheta = sin (halftheta);
    if (feq (halftheta, 0)) {
        axis[0] = 0;
        axis[1] = 0;
        axis[2] = 1;
        theta = 0;
    } else {
        axis[0] = q[1] / sinhalftheta;
        axis[1] = q[2] / sinhalftheta;
        axis[2] = q[3] / sinhalftheta;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::addCoordinateSystem (double scale, const Eigen::Affine3f& t, int viewport)
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

    axes_actor->SetPosition (t(0, 3), t(1, 3), t(2, 3));

    Eigen::Matrix3f m;
    m =t.rotation();
    Eigen::Quaternionf rf;
    rf = Eigen::Quaternionf(m);
    double r_angle;
    double r_axis[3];
    quat_to_angle_axis(rf,r_angle,r_axis);
    //
    axes_actor->SetOrientation(0,0,0);
    axes_actor->RotateWXYZ(r_angle*180/M_PI,r_axis[0],r_axis[1],r_axis[2]);
    //WAS:  axes_actor->SetOrientation (roll, pitch, yaw);

    // Save the ID and actor pair to the global actor map
    coordinate_actor_map_[viewport] = axes_actor;
    addActorToRenderer (axes_actor, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeCoordinateSystem (int viewport)
{
    // Check to see if the given ID entry exists
    CoordinateActorMap::iterator am_it = coordinate_actor_map_.find (viewport);

    if (am_it == coordinate_actor_map_.end ())
        return (false);

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second, viewport))
    {
        // Remove the ID pair to the global actor map
        coordinate_actor_map_.erase (am_it);
        return (true);
    }
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removePointCloud (const std::string &id, int viewport)
{
    // Check to see if the given ID entry exists
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

    if (am_it == cloud_actor_map_->end ())
        return (false);

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second.actor, viewport))
    {
        // Remove the pointer/ID pair to the global actor map
        cloud_actor_map_->erase (am_it);
        return (true);
    }
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeShape (const std::string &id, int viewport)
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
            return (false);
        // Cloud found, set shape to false
        shape = false;
    }

    // Remove the pointer/ID pair to the global actor map
    if (shape)
    {
        if (removeActorFromRenderer (am_it->second, viewport))
        {
            shape_actor_map_->erase (am_it);
            return (true);
        }
    }
    else
    {
        if (removeActorFromRenderer (ca_it->second.actor, viewport))
        {
            cloud_actor_map_->erase (ca_it);
            return (true);
        }
    }
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeText3D (const std::string &id, int viewport)
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    if (am_it == shape_actor_map_->end ())
    {
        //pcl::console::print_warn (stderr, "[removeSape] Could not find any shape with id <%s>!\n", id.c_str ());
        return (false);
    }

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second, viewport))
    {
        // Remove the pointer/ID pair to the global actor map
        shape_actor_map_->erase (am_it);
        return (true);
    }
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeAllPointClouds (int viewport)
{
    // Check to see if the given ID entry exists
    CloudActorMap::iterator am_it = cloud_actor_map_->begin ();
    while (am_it != cloud_actor_map_->end () )
    {
        if (removePointCloud (am_it->first, viewport))
            am_it = cloud_actor_map_->begin ();
        else
            ++am_it;
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeAllShapes (int viewport)
{
    // Check to see if the given ID entry exists
    ShapeActorMap::iterator am_it = shape_actor_map_->begin ();
    while (am_it != shape_actor_map_->end ())
    {
        if (removeShape (am_it->first, viewport))
            am_it = shape_actor_map_->begin ();
        else
            ++am_it;
    }
    return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor, int viewport)
{
    vtkLODActor* actor_to_remove = vtkLODActor::SafeDownCast (actor);

    // Add it to all renderers
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Should we remove the actor from all renderers?
        if (viewport == 0)
        {
            renderer->RemoveActor (actor);
            //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            // Iterate over all actors in this renderer
            vtkPropCollection* actors = renderer->GetViewProps ();
            actors->InitTraversal ();
            vtkProp* current_actor = NULL;
            while ((current_actor = actors->GetNextProp ()) != NULL)
            {
                if (current_actor != actor_to_remove)
                    continue;
                renderer->RemoveActor (actor);
                //        renderer->Render ();
                // Found the correct viewport and removed the actor
                return (true);
            }
        }
        ++i;
    }
    if (viewport == 0) return (true);
    return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor, int viewport)
{
    vtkActor* actor_to_remove = vtkActor::SafeDownCast (actor);

    // Add it to all renderers
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Should we remove the actor from all renderers?
        if (viewport == 0)
        {
            renderer->RemoveActor (actor);
            //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            // Iterate over all actors in this renderer
            vtkPropCollection* actors = renderer->GetViewProps ();
            actors->InitTraversal ();
            vtkProp* current_actor = NULL;
            while ((current_actor = actors->GetNextProp ()) != NULL)
            {
                if (current_actor != actor_to_remove)
                    continue;
                renderer->RemoveActor (actor);
                //        renderer->Render ();
                // Found the correct viewport and removed the actor
                return (true);
            }
        }
        ++i;
    }
    if (viewport == 0) return (true);
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::addActorToRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
    // Add it to all renderers
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
            renderer->AddActor (actor);
            //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            renderer->AddActor (actor);
            //      renderer->Render ();
        }
        ++i;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
    vtkProp* actor_to_remove = vtkProp::SafeDownCast (actor);

    // Initialize traversal
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Should we remove the actor from all renderers?
        if (viewport == 0)
        {
            renderer->RemoveActor (actor);
            //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            // Iterate over all actors in this renderer
            vtkPropCollection* actors = renderer->GetViewProps ();
            actors->InitTraversal ();
            vtkProp* current_actor = NULL;
            while ((current_actor = actors->GetNextProp ()) != NULL)
            {
                if (current_actor != actor_to_remove)
                    continue;
                renderer->RemoveActor (actor);
                //        renderer->Render ();
                // Found the correct viewport and removed the actor
                return (true);
            }
        }
        ++i;
    }
    if (viewport == 0) return (true);
    return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
namespace
{
// Helper function called by createActorFromVTKDataSet () methods.
// This function determines the default setting of vtkMapper::InterpolateScalarsBeforeMapping.
// Return 0, interpolation off, if data is a vtkPolyData that contains only vertices.
// Return 1, interpolation on, for anything else.
int getDefaultScalarInterpolationForDataSet (vtkDataSet* data)
{
    vtkPolyData* polyData = vtkPolyData::SafeDownCast (data);
    return (polyData && polyData->GetNumberOfCells () != polyData->GetNumberOfVerts ());
}

}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkLODActor> &actor, bool use_scalars)
{
    // If actor is not initialized, initialize it here
    if (!actor)
        actor = vtkSmartPointer<vtkLODActor>::New ();


    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
    mapper->SetInput (data);

    if (use_scalars)
    {
        vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
        double minmax[2];
        if (scalars)
        {
            scalars->GetRange (minmax);
            mapper->SetScalarRange (minmax);

            mapper->SetScalarModeToUsePointData ();
            mapper->SetInterpolateScalarsBeforeMapping (getDefaultScalarInterpolationForDataSet (data));
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

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkActor> &actor, bool use_scalars)
{
    // If actor is not initialized, initialize it here
    if (!actor)
        actor = vtkSmartPointer<vtkActor>::New ();


    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
    mapper->SetInput (data);

    if (use_scalars)
    {
        vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
        double minmax[2];
        if (scalars)
        {
            scalars->GetRange (minmax);
            mapper->SetScalarRange (minmax);

            mapper->SetScalarModeToUsePointData ();
            mapper->SetInterpolateScalarsBeforeMapping (getDefaultScalarInterpolationForDataSet (data));
            mapper->ScalarVisibilityOn ();
        }
    }
    mapper->ImmediateModeRenderingOff ();

    //actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
    actor->GetProperty ()->SetInterpolationToFlat ();

    /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
    /// shown when there is a vtkActor with backface culling on present in the scene
    /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
    // actor->GetProperty ()->BackfaceCullingOn ();

    actor->SetMapper (mapper);

    //actor->SetNumberOfCloudPoints (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10));
    actor->GetProperty ()->SetInterpolationToFlat ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setBackgroundColor (const double &r, const double &g, const double &b, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
            renderer->SetBackground (r, g, b);
            //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
            renderer->SetBackground (r, g, b);
            //      renderer->Render ();
        }
        ++i;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const std::string &id, int)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

    if (am_it == cloud_actor_map_->end ())
    {
        pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
        return (false);
    }
    // Get the actor pointer
    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

    switch (property)
    {
    case PCL_VISUALIZER_COLOR:
    {
        actor->GetProperty ()->SetColor (val1, val2, val3);
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->Modified ();
        break;
    }
    default:
    {
        pcl::console::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
        return (false);
    }
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::getPointCloudRenderingProperties (int property, double &value, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

    if (am_it == cloud_actor_map_->end ())
        return (false);
    // Get the actor pointer
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
    {
        pcl::console::print_error ("[getPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
        return (false);
    }
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::setPointCloudRenderingProperties (int property, double value, const std::string &id, int)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

    if (am_it == cloud_actor_map_->end ())
    {
        pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
        return (false);
    }
    // Get the actor pointer
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
    {
        pcl::console::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
        return (false);
    }
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::setPointCloudSelected (const bool selected, const std::string &id)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

    if (am_it == cloud_actor_map_->end ())
    {
        pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
        return (false);
    }
    // Get the actor pointer
    vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

    if (selected)
    {
        actor->GetProperty ()->EdgeVisibilityOn ();
        actor->GetProperty ()->SetEdgeColor (1.0,0.0,0.0);
        actor->Modified ();
    }
    else
    {
        actor->GetProperty ()->EdgeVisibilityOff ();
        actor->Modified ();
    }

    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::setShapeRenderingProperties (int property, double val1, double val2, double val3, const std::string &id, int)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    if (am_it == shape_actor_map_->end ())
    {
        pcl::console::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
        return (false);
    }
    // Get the actor pointer
    vtkActor* actor = vtkActor::SafeDownCast (am_it->second);

    switch (property)
    {
    case PCL_VISUALIZER_COLOR:
    {
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->GetProperty ()->SetColor (val1, val2, val3);
        actor->GetProperty ()->SetEdgeColor (val1, val2, val3);
        // The following 3 are set by SetColor automatically according to the VTK docs
        //actor->GetProperty ()->SetAmbientColor  (val1, val2, val3);
        //actor->GetProperty ()->SetDiffuseColor (val1, val2, val3);
        //actor->GetProperty ()->SetSpecularColor (val1, val2, val3);
        actor->GetProperty ()->SetAmbient (0.8);
        actor->GetProperty ()->SetDiffuse (0.8);
        actor->GetProperty ()->SetSpecular (0.8);
        actor->GetProperty ()->SetLighting (0);
        actor->Modified ();
        break;
    }
    default:
    {
        pcl::console::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
        return (false);
    }
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::setShapeRenderingProperties (int property, double value, const std::string &id, int)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    if (am_it == shape_actor_map_->end ())
    {
        pcl::console::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
        return (false);
    }
    // Get the actor pointer
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
        case PCL_VISUALIZER_REPRESENTATION_POINTS:
        {
            actor->GetProperty ()->SetRepresentationToPoints ();
            break;
        }
        case PCL_VISUALIZER_REPRESENTATION_WIREFRAME:
        {
            actor->GetProperty ()->SetRepresentationToWireframe ();
            break;
        }
        case PCL_VISUALIZER_REPRESENTATION_SURFACE:
        {
            actor->GetProperty ()->SetRepresentationToSurface ();
            break;
        }
        }
        actor->Modified ();
        break;
    }
    case PCL_VISUALIZER_SHADING:
    {
        switch (int (value))
        {
        case PCL_VISUALIZER_SHADING_FLAT:
        {
            actor->GetProperty ()->SetInterpolationToFlat ();
            break;
        }
        case PCL_VISUALIZER_SHADING_GOURAUD:
        {
            if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetNormals ())
            {
                PCL_INFO ("[pcl::visualization::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Gouraud shading was requested. Estimating normals...\n");
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
                PCL_INFO ("[pcl::visualization::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Phong shading was requested. Estimating normals...\n");
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
    {
        pcl::console::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
        return (false);
    }
    }
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::initCameraParameters ()
{
    Camera camera_temp;
    // Set default camera parameters to something meaningful
    camera_temp.clip[0] = 0.01;
    camera_temp.clip[1] = 1000.01;

    // Look straight along the z-axis
    camera_temp.focal[0] = 0.;
    camera_temp.focal[1] = 0.;
    camera_temp.focal[2] = 1.;

    // Position the camera at the origin
    camera_temp.pos[0] = 0.;
    camera_temp.pos[1] = 0.;
    camera_temp.pos[2] = 0.;

    // Set the up-vector of the camera to be the y-axis
    camera_temp.view[0] = 0.;
    camera_temp.view[1] = 1.;
    camera_temp.view[2] = 0.;

    // Set the camera field of view to about
    camera_temp.fovy = 0.8575;

    int *scr_size = win_->GetScreenSize ();
    camera_temp.window_size[0] = scr_size[0] / 2;
    camera_temp.window_size[1] = scr_size[1] / 2;

    setCameraParameters (camera_temp);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::cameraParamsSet () const { return (camera_set_); }

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::updateCamera ()
{
    PCL_WARN ("[pcl::visualization::PCLVisualizer::updateCamera()] This method was deprecated, just re-rendering all scenes now.");
    rens_->InitTraversal ();
    // Update the camera parameters
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
        renderer->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::updateShapePose (const std::string &id, const Eigen::Affine3f& pose)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);

    vtkLODActor* actor;

    if (am_it == shape_actor_map_->end ())
        return (false);
    else
        actor = vtkLODActor::SafeDownCast (am_it->second);

    vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();

    convertToVtkMatrix (pose.matrix (), matrix);

    actor->SetUserMatrix (matrix);
    actor->Modified ();

    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::getCameras (std::vector<pcl::visualization::Camera>& cameras)
{
    cameras.clear ();
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        cameras.push_back(Camera());
        cameras.back ().pos[0] = renderer->GetActiveCamera ()->GetPosition ()[0];
        cameras.back ().pos[1] = renderer->GetActiveCamera ()->GetPosition ()[1];
        cameras.back ().pos[2] = renderer->GetActiveCamera ()->GetPosition ()[2];
        cameras.back ().focal[0] = renderer->GetActiveCamera ()->GetFocalPoint ()[0];
        cameras.back ().focal[1] = renderer->GetActiveCamera ()->GetFocalPoint ()[1];
        cameras.back ().focal[2] = renderer->GetActiveCamera ()->GetFocalPoint ()[2];
        cameras.back ().clip[0] = renderer->GetActiveCamera ()->GetClippingRange ()[0];
        cameras.back ().clip[1] = renderer->GetActiveCamera ()->GetClippingRange ()[1];
        cameras.back ().view[0] = renderer->GetActiveCamera ()->GetViewUp ()[0];
        cameras.back ().view[1] = renderer->GetActiveCamera ()->GetViewUp ()[1];
        cameras.back ().view[2] = renderer->GetActiveCamera ()->GetViewUp ()[2];
        cameras.back ().fovy = renderer->GetActiveCamera ()->GetViewAngle () / 180.0 * M_PI;
        cameras.back ().window_size[0] = renderer->GetRenderWindow ()->GetSize ()[0];
        cameras.back ().window_size[1] = renderer->GetRenderWindow ()->GetSize ()[1];
        cameras.back ().window_pos[0] = 0;
        cameras.back ().window_pos[1] = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f pcl::visualization::PCLVisualizer::getViewerPose (int viewport)
{
    Eigen::Affine3f ret (Eigen::Affine3f::Identity ());

    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    if (viewport == 0)
        viewport = 1;
    int viewport_i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        if (viewport_i == viewport)
        {
            vtkCamera& camera = *renderer->GetActiveCamera ();
            Eigen::Vector3d pos, x_axis, y_axis, z_axis;
            camera.GetPosition (pos[0], pos[1], pos[2]);
            camera.GetViewUp (y_axis[0], y_axis[1], y_axis[2]);
            camera.GetFocalPoint (z_axis[0], z_axis[1], z_axis[2]);

            z_axis = (z_axis - pos).normalized ();
            x_axis = y_axis.cross (z_axis).normalized ();

            /// TODO replace this ugly thing with matrix.block () = vector3f
            ret (0, 0) = static_cast<float> (x_axis[0]);
            ret (0, 1) = static_cast<float> (y_axis[0]);
            ret (0, 2) = static_cast<float> (z_axis[0]);
            ret (0, 3) = static_cast<float> (pos[0]);

            ret (1, 0) = static_cast<float> (x_axis[1]);
            ret (1, 1) = static_cast<float> (y_axis[1]);
            ret (1, 2) = static_cast<float> (z_axis[1]);
            ret (1, 3) = static_cast<float> (pos[1]);

            ret (2, 0) = static_cast<float> (x_axis[2]);
            ret (2, 1) = static_cast<float> (y_axis[2]);
            ret (2, 2) = static_cast<float> (z_axis[2]);
            ret (2, 3) = static_cast<float> (pos[2]);

            return ret;
        }
        viewport_i ++;
    }

    return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::resetCamera ()
{
    // Update the camera parameters
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
        renderer->ResetCamera ();
}


/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraPosition (double pos_x, double pos_y, double pos_z,
        double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetPosition (pos_x, pos_y, pos_z);
            cam->SetFocalPoint (view_x, view_y, view_z);
            cam->SetViewUp (up_x, up_y, up_z);
            renderer->Render ();
        }
        ++i;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetPosition (pos_x, pos_y, pos_z);
            cam->SetViewUp (up_x, up_y, up_z);
            renderer->Render ();
        }
        ++i;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraParameters (const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics, int viewport)
{
    // Position = extrinsic translation
    Eigen::Vector3f pos_vec = extrinsics.block<3, 1> (0, 3);

    // Rotate the view vector
    Eigen::Matrix3f rotation = extrinsics.block<3, 3> (0, 0);
    Eigen::Vector3f y_axis (0.f, 1.f, 0.f);
    Eigen::Vector3f up_vec (rotation * y_axis);

    // Compute the new focal point
    Eigen::Vector3f z_axis (0.f, 0.f, 1.f);
    Eigen::Vector3f focal_vec = pos_vec + rotation * z_axis;

    // Get the width and height of the image - assume the calibrated centers are at the center of the image
    Eigen::Vector2i window_size;
    window_size[0] = static_cast<int> (intrinsics (0, 2));
    window_size[1] = static_cast<int> (intrinsics (1, 2));

    // Compute the vertical field of view based on the focal length and image heigh
    double fovy = 2 * atan (window_size[1] / (2. * intrinsics (1, 1))) * 180.0 / M_PI;


    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetPosition (pos_vec[0], pos_vec[1], pos_vec[2]);
            cam->SetFocalPoint (focal_vec[0], focal_vec[1], focal_vec[2]);
            cam->SetViewUp (up_vec[0], up_vec[1], up_vec[2]);
            cam->SetUseHorizontalViewAngle (0);
            cam->SetViewAngle (fovy);
            cam->SetClippingRange (0.01, 1000.01);
            win_->SetSize (window_size[0], window_size[1]);

            renderer->Render ();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraParameters (const pcl::visualization::Camera &camera, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetPosition (camera.pos[0], camera.pos[1], camera.pos[2]);
            cam->SetFocalPoint (camera.focal[0], camera.focal[1], camera.focal[2]);
            cam->SetViewUp (camera.view[0], camera.view[1], camera.view[2]);
            cam->SetClippingRange (camera.clip);
            cam->SetUseHorizontalViewAngle (0);
            cam->SetViewAngle (camera.fovy * 180.0 / M_PI);

            win_->SetSize (static_cast<int> (camera.window_size[0]),
                    static_cast<int> (camera.window_size[1]));
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraClipDistances (double near, double far, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetClippingRange (near, far);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setCameraFieldOfView (double fovy, int viewport)
{
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i)
        {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
            cam->SetUseHorizontalViewAngle (0);
            cam->SetViewAngle (fovy * 180.0 / M_PI);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::resetCameraViewpoint (const std::string &id)
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
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
        cam->SetPosition (camera_pose->GetElement (0, 3),
                          camera_pose->GetElement (1, 3),
                          camera_pose->GetElement (2, 3));

        cam->SetFocalPoint (camera_pose->GetElement (0, 3) - camera_pose->GetElement (0, 2),
                            camera_pose->GetElement (1, 3) - camera_pose->GetElement (1, 2),
                            camera_pose->GetElement (2, 3) - camera_pose->GetElement (2, 2));

        cam->SetViewUp (camera_pose->GetElement (0, 1),
                        camera_pose->GetElement (1, 1),
                        camera_pose->GetElement (2, 1));

        renderer->SetActiveCamera (cam);
        renderer->ResetCameraClippingRange ();
        renderer->Render ();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCylinder (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCylinder] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCylinder (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCube (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCube (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
        double width, double height, double depth, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCube (translation, rotation, width, height, depth);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                                            double r, double g, double b, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCube (x_min, x_max, y_min, y_max, z_min, z_max);

    // Create an Actor
    vtkSmartPointer<vtkActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    actor->GetProperty ()->SetColor (r,g,b);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addSphere (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createSphere (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, const std::string & id, int viewport)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr,
                                  "[addModelFromPolyData] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                  id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (polydata, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string & id, int viewport)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr,
                                  "[addModelFromPolyData] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                  id.c_str ());
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
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}


////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename, const std::string &id, int viewport)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr,
                                  "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                  id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
    reader->SetFileName (filename.c_str ());

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (reader->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename, vtkSmartPointer<vtkTransform> transform, const std::string &id, int viewport)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr,
                                  "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                  id.c_str ());
        return (false);
    }

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
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addLine (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createLine (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

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
bool pcl::visualization::PCLVisualizer::addPlane (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addPlane] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createPlane (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    //  actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetRepresentationToSurface ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

bool pcl::visualization::PCLVisualizer::addPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addPlane] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createPlane (coefficients, x, y, z);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCircle (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCircle] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = create2DCircle (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addCone (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addCone] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
        return (false);
    }

    vtkSmartPointer<vtkDataSet> data = createCone (coefficients);

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetLighting (false);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)
{
    // Create a new renderer
    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
    ren->SetViewport (xmin, ymin, xmax, ymax);

    if (rens_->GetNumberOfItems () > 0)
        ren->SetActiveCamera (rens_->GetFirstRenderer ()->GetActiveCamera ());
    ren->ResetCamera ();

    // Add it to the list of renderers
    rens_->AddItem (ren);

    if (rens_->GetNumberOfItems () <= 1)          // If only one renderer
        viewport = 0;                               // set viewport to 'all'
    else
        viewport = rens_->GetNumberOfItems () - 1;

    win_->AddRenderer (ren);
    win_->Modified ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::createViewPortCamera (const int viewport)
{
    vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New ();
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        if (viewport == 0)
            continue;
        else if (viewport == i)
        {
            renderer->SetActiveCamera (cam);
            renderer->ResetCamera ();
        }
        ++i;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, const cv::Scalar& color, int fontsize, const std::string &id, int viewport)
{
   std::string tid = id.empty() ? text : id;

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (tid);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr, "[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
        return (false);
    }

    // Create an Actor
    vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New ();
    actor->SetPosition (xpos, ypos);
    actor->SetInput (text.c_str ());

    vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
    tprop->SetFontSize (fontsize);
    tprop->SetFontFamilyToArial ();
    tprop->SetJustificationToLeft ();
    tprop->BoldOn ();
    tprop->SetColor (color[2]/255, color[1]/255, color[0]/255);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[tid] = actor;
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::updateText (const std::string &text, int xpos, int ypos, const cv::Scalar& color, int fontsize, const std::string &id)
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
    tprop->SetColor (color[2]/255, color[1]/255, color[0]/255);

    actor->Modified ();

    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addPolygonMesh (const pcl::PolygonMesh &poly_mesh, const std::string &id, int viewport)
{
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it != cloud_actor_map_->end ())
        return std::cout << "[addPolygonMesh] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl, false;

    // Create points from polyMesh.cloud
    vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg (poly_mesh.cloud, *point_cloud);
    poly_points->SetNumberOfPoints (point_cloud->points.size ());

    size_t i;
    for (i = 0; i < point_cloud->points.size (); ++i)
        poly_points->InsertPoint (i, point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z);

    bool has_color = false;
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    if (pcl::getFieldIndex(poly_mesh.cloud, "rgb") != -1)
    {
        has_color = true;
        colors->SetNumberOfComponents (3);
        colors->SetName ("Colors");
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(poly_mesh.cloud, cloud);
        for (i = 0; i < cloud.points.size (); ++i)
        {
            const unsigned char color[3] = {cloud.points[i].r, cloud.points[i].g, cloud.points[i].b};
            colors->InsertNextTupleValue(color);
        }
    }
    if (pcl::getFieldIndex(poly_mesh.cloud, "rgba") != -1)
    {
        has_color = true;
        colors->SetNumberOfComponents (3);
        colors->SetName ("Colors");
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg(poly_mesh.cloud, cloud);
        for (i = 0; i < cloud.points.size (); ++i)
        {
            const unsigned char color[3] = {cloud.points[i].r, cloud.points[i].g, cloud.points[i].b};
            colors->InsertNextTupleValue(color);
        }
    }

    vtkSmartPointer<vtkLODActor> actor;
    if (poly_mesh.polygons.size() > 1)
    {
        //create polys from polyMesh.polygons
        vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();

        for (i = 0; i < poly_mesh.polygons.size (); i++)
        {
            size_t n_points (poly_mesh.polygons[i].vertices.size ());
            cell_array->InsertNextCell (int (n_points));
            for (size_t j = 0; j < n_points; j++)
                cell_array->InsertCellPoint (poly_mesh.polygons[i].vertices[j]);
        }

        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        //    polydata->SetStrips (cell_array);
        polydata->SetPolys (cell_array);
        polydata->SetPoints (poly_points);

        if (has_color)
            polydata->GetPointData()->SetScalars(colors);

        createActorFromVTKDataSet (polydata, actor);
    }
    else if (poly_mesh.polygons.size() == 1)
    {
        vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
        size_t n_points = poly_mesh.polygons[0].vertices.size ();
        polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

        for (size_t j=0; j < (n_points - 1); j++)
            polygon->GetPointIds ()->SetId (j, poly_mesh.polygons[0].vertices[j]);

        vtkSmartPointer<vtkUnstructuredGrid> poly_grid = vtkSmartPointer<vtkUnstructuredGrid>::New ();
        poly_grid->Allocate (1, 1);
        poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
        poly_grid->SetPoints (poly_points);
        poly_grid->Update ();

        createActorFromVTKDataSet (poly_grid, actor);
        actor->GetProperty ()->SetRepresentationToWireframe ();
    }
    else
    {
        PCL_ERROR("PCLVisualizer::addPolygonMesh: No polygons\n");
        return false;
    }

    actor->GetProperty ()->SetRepresentationToSurface ();
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*cloud_actor_map_)[id].actor = actor;

    // Save the viewpoint transformation matrix to the global actor map
    vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
    convertToVtkMatrix (point_cloud->sensor_origin_, point_cloud->sensor_orientation_, transformation);
    (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::updatePolygonMesh (const pcl::PolygonMesh &poly_mesh, const std::string &id)
{

    if (poly_mesh.polygons.empty())
    {
        pcl::console::print_error ("[updatePolygonMesh] No vertices given!\n");
        return (false);
    }

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return (false);

    // Create points from polyMesh.cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg (poly_mesh.cloud, *cloud);

    std::vector<pcl::Vertices> verts(poly_mesh.polygons); // copy vector

    // Get the current poly data
    vtkSmartPointer<vtkPolyData> polydata = static_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
    if (!polydata)
        return (false);
    vtkSmartPointer<vtkCellArray> cells = polydata->GetStrips ();
    if (!cells)
        return (false);
    vtkSmartPointer<vtkPoints> points   = polydata->GetPoints ();
    // Copy the new point array in
    vtkIdType nr_points = cloud->points.size ();
    points->SetNumberOfPoints (nr_points);

    // Get a pointer to the beginning of the data array
    float *data = static_cast<vtkFloatArray*> (points->GetData ())->GetPointer (0);

    int ptr = 0;
    std::vector<int> lookup;
    // If the dataset is dense (no NaNs)
    if (cloud->is_dense)
    {
        for (vtkIdType i = 0; i < nr_points; ++i, ptr += 3)
            memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
    }
    else
    {
        lookup.resize (nr_points);
        vtkIdType j = 0;    // true point index
        for (vtkIdType i = 0; i < nr_points; ++i)
        {
            // Check if the point is invalid
            if (!isFinite (cloud->points[i]))
                continue;

            lookup [i] = static_cast<int> (j);
            memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
            j++;
            ptr += 3;
        }
        nr_points = j;
        points->SetNumberOfPoints (nr_points);
    }

    // Get the maximum size of a polygon
    int max_size_of_polygon = -1;
    for (size_t i = 0; i < verts.size (); ++i)
        if (max_size_of_polygon < static_cast<int> (verts[i].vertices.size ()))
            max_size_of_polygon = static_cast<int> (verts[i].vertices.size ());

    // Update the cells
    cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkIdType *cell = cells->WritePointer (verts.size (), verts.size () * (max_size_of_polygon + 1));
    int idx = 0;
    if (lookup.size () > 0)
    {
        for (size_t i = 0; i < verts.size (); ++i, ++idx)
        {
            size_t n_points = verts[i].vertices.size ();
            *cell++ = n_points;
            for (size_t j = 0; j < n_points; j++, cell++, ++idx)
                *cell = lookup[verts[i].vertices[j]];
        }
    }
    else
    {
        for (size_t i = 0; i < verts.size (); ++i, ++idx)
        {
            size_t n_points = verts[i].vertices.size ();
            *cell++ = n_points;
            for (size_t j = 0; j < n_points; j++, cell++, ++idx)
                *cell = verts[i].vertices[j];
        }
    }
    cells->GetData ()->SetNumberOfValues (idx);
    cells->Squeeze ();
    // Set the the vertices
    polydata->SetStrips (cells);
    polydata->Update ();

    return (true);
}


///////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addPolylineFromPolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id, int viewport)
{
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        pcl::console::print_warn (stderr,
                                  "[addPolylineFromPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                  id.c_str ());
        return (false);
    }

    // Create points from polyMesh.cloud
    vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg (polymesh.cloud, point_cloud);
    poly_points->SetNumberOfPoints (point_cloud.points.size ());

    size_t i;
    for (i = 0; i < point_cloud.points.size (); ++i)
        poly_points->InsertPoint (i, point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z);

    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer <vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer <vtkPolyData> polyData;
    allocVtkPolyData (polyData);

    for (i = 0; i < polymesh.polygons.size (); i++)
    {
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
        polyLine->GetPointIds()->SetNumberOfIds(polymesh.polygons[i].vertices.size());
        for(unsigned int k = 0; k < polymesh.polygons[i].vertices.size(); k++)
        {
            polyLine->GetPointIds()->SetId(k,polymesh.polygons[i].vertices[k]);
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


    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;

    return (true);
}

///////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setRepresentationToSurfaceForAllActors ()
{
    ShapeActorMap::iterator am_it;
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        vtkActorCollection * actors = renderer->GetActors ();
        actors->InitTraversal ();
        vtkActor * actor;
        while ((actor = actors->GetNextActor ()) != NULL)
        {
            actor->GetProperty ()->SetRepresentationToSurface ();
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setRepresentationToPointsForAllActors ()
{
    ShapeActorMap::iterator am_it;
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        vtkActorCollection * actors = renderer->GetActors ();
        actors->InitTraversal ();
        vtkActor * actor;
        while ((actor = actors->GetNextActor ()) != NULL)
        {
            actor->GetProperty ()->SetRepresentationToPoints ();
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setRepresentationToWireframeForAllActors ()
{
    ShapeActorMap::iterator am_it;
    rens_->InitTraversal ();
    vtkRenderer* renderer = NULL;
    while ((renderer = rens_->GetNextItem ()) != NULL)
    {
        vtkActorCollection * actors = renderer->GetActors ();
        actors->InitTraversal ();
        vtkActor * actor;
        while ((actor = actors->GetNextActor ()) != NULL)
        {
            actor->GetProperty ()->SetRepresentationToWireframe ();
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::updateCells (vtkSmartPointer<vtkIdTypeArray> &cells, vtkSmartPointer<vtkIdTypeArray> &initcells, vtkIdType nr_points)
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
void pcl::visualization::PCLVisualizer::allocVtkPolyData (vtkSmartPointer<vtkAppendPolyData> &polydata)
{
    polydata = vtkSmartPointer<vtkAppendPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::allocVtkPolyData (vtkSmartPointer<vtkPolyData> &polydata)
{
    polydata = vtkSmartPointer<vtkPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata)
{
    polydata = vtkSmartPointer<vtkUnstructuredGrid>::New ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::getTransformationMatrix (const Eigen::Vector4f &origin,
        const Eigen::Quaternion<float> &orientation, Eigen::Matrix4f &transformation)
{
    transformation.setIdentity ();
    transformation.block<3,3>(0,0) = orientation.toRotationMatrix ();
    transformation.block<3,1>(0,3) = origin.head (3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertToVtkMatrix (const Eigen::Vector4f &origin,
        const Eigen::Quaternion<float> &orientation, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
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
void pcl::visualization::PCLVisualizer::convertToVtkMatrix (const Eigen::Matrix4f &m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            vtk_matrix->SetElement (i, k, m (i, k));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertToEigenMatrix (const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix, Eigen::Matrix4f &m)
{
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            m (i,k) = static_cast<float> (vtk_matrix->GetElement (i, k));
}