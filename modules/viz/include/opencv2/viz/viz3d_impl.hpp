#pragma once

#include <vtkCellData.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkProperty2D.h>
#include <vtkMapper2D.h>
#include <vtkLeaderActor2D.h>
#include <vtkAlgorithmOutput.h>

#include <q/visualization/common/shapes.h>
#include <pcl/common/io.h>

inline bool pcl::visualization::PCLVisualizer::addPolygon (const cv::Mat& cloud, const cv::Scalar& color, const std::string &id, int viewport)
{
    CV_Assert(cloud.type() == CV_32FC3 && cloud.rows == 1);

    vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkPolygon> polygon    = vtkSmartPointer<vtkPolygon>::New ();

    int total = cloud.size().area();
    poly_points->SetNumberOfPoints (total);
    polygon->GetPointIds ()->SetNumberOfIds (total);


    int i;
    for (i = 0; i < total; ++i)
    {
        cv::Point3f p = cloud.ptr<cv::Point3f>()[i];
        poly_points->SetPoint (i, p.x, p.y, p.z);
        polygon->GetPointIds ()->SetId (i, i);
    }

    vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
    allocVtkUnstructuredGrid (poly_grid);
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (poly_points);
    poly_grid->Update ();


    //////////////////////////////////////////////////////
    vtkSmartPointer<vtkDataSet> data = poly_grid;


    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
    if (am_it != shape_actor_map_->end ())
    {
        vtkSmartPointer<vtkAppendPolyData> all_data = vtkSmartPointer<vtkAppendPolyData>::New ();

        // Add old data
        all_data->AddInput (reinterpret_cast<vtkPolyDataMapper*> ((vtkActor::SafeDownCast (am_it->second))->GetMapper ())->GetInput ());

        // Add new data
        vtkSmartPointer<vtkDataSetSurfaceFilter> surface_filter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New ();
        surface_filter->SetInput (vtkUnstructuredGrid::SafeDownCast (data));
        vtkSmartPointer<vtkPolyData> poly_data = surface_filter->GetOutput ();
        all_data->AddInput (poly_data);

        // Create an Actor
        vtkSmartPointer<vtkActor> actor;
        createActorFromVTKDataSet (all_data->GetOutput (), actor);
        actor->GetProperty ()->SetRepresentationToWireframe ();
        actor->GetProperty ()->SetColor (color[2]/255, color[1]/255, color[0]/255);
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->GetProperty ()->BackfaceCullingOff ();

        removeActorFromRenderer (am_it->second, viewport);
        addActorToRenderer (actor, viewport);

        // Save the pointer/ID pair to the global actor map
        (*shape_actor_map_)[id] = actor;
    }
    else
    {
        // Create an Actor
        vtkSmartPointer<vtkActor> actor;
        createActorFromVTKDataSet (data, actor);
        actor->GetProperty ()->SetRepresentationToWireframe ();
        actor->GetProperty ()->SetColor (color[2]/255, color[1]/255, color[0]/255);
        actor->GetMapper ()->ScalarVisibilityOff ();
        actor->GetProperty ()->BackfaceCullingOff ();
        addActorToRenderer (actor, viewport);

        // Save the pointer/ID pair to the global actor map
        (*shape_actor_map_)[id] = actor;
    }

    return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->AutoLabelOn ();

  leader->GetProperty ()->SetColor (r, g, b);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->SetArrowPlacementToPoint1 ();
  if (display_length)
    leader->AutoLabelOn ();
  else
    leader->AutoLabelOff ();

  leader->GetProperty ()->SetColor (r, g, b);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}
////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2, double r_line, double g_line, double b_line,
                         double r_text, double g_text, double b_text, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->AutoLabelOn ();

  leader->GetLabelTextProperty()->SetColor(r_text, g_text, b_text);

  leader->GetProperty ()->SetColor (r_line, g_line, b_line);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
inline bool pcl::visualization::PCLVisualizer::addSphere (const pcl::PointXYZ& center, double radius, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  //vtkSmartPointer<vtkDataSet> data = createSphere (center.getVector4fMap (), radius);
  vtkSmartPointer<vtkSphereSource> data = vtkSmartPointer<vtkSphereSource>::New ();
  data->SetRadius (radius);
  data->SetCenter (double (center.x), double (center.y), double (center.z));
  data->SetPhiResolution (10);
  data->SetThetaResolution (10);
  data->LatLongTessellationOff ();
  data->Update ();

  // Setup actor and mapper
  vtkSmartPointer <vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputConnection (data->GetOutputPort ());

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  actor->SetMapper (mapper);
  //createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  actor->GetProperty ()->SetInterpolationToFlat ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->GetMapper ()->ImmediateModeRenderingOn ();
  actor->GetMapper ()->StaticOn ();
  actor->GetMapper ()->ScalarVisibilityOff ();
  actor->GetMapper ()->Update ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
inline bool pcl::visualization::PCLVisualizer::updateSphere (const pcl::PointXYZ &center, double radius, double r, double g, double b, const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it == shape_actor_map_->end ())
    return (false);

  //////////////////////////////////////////////////////////////////////////
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  vtkAlgorithm *algo = actor->GetMapper ()->GetInput ()->GetProducerPort ()->GetProducer ();
  vtkSphereSource *src = vtkSphereSource::SafeDownCast (algo);

  src->SetCenter (double (center.x), double (center.y), double (center.z));
  src->SetRadius (radius);
  src->Update ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////
inline bool pcl::visualization::PCLVisualizer::addText3D (const std::string &text, const PointXYZ& position,
  double textScale, double r, double g, double b, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (tid);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addText3d] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New ();
  textSource->SetText (text.c_str());
  textSource->Update ();

  vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  textMapper->SetInputConnection (textSource->GetOutputPort ());

  // Since each follower may follow a different camera, we need different followers
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers or just to i-nth renderer?
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkFollower> textActor = vtkSmartPointer<vtkFollower>::New ();
      textActor->SetMapper (textMapper);
      textActor->SetPosition (position.x, position.y, position.z);
      textActor->SetScale (textScale);
      textActor->GetProperty ()->SetColor (r, g, b);
      textActor->SetCamera (renderer->GetActiveCamera ());

      renderer->AddActor (textActor);
      renderer->Render ();

      // Save the pointer/ID pair to the global actor map. If we are saving multiple vtkFollowers
      // for multiple viewport
      std::string alternate_tid = tid;
      alternate_tid.append(i, '*');

      (*shape_actor_map_)[(viewport == 0) ? tid : alternate_tid] = textActor;
    }

    ++i;
  }

  return (true);
}
