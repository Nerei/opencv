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

inline bool pcl::visualization::PCLVisualizer::addPolygonMesh (const cv::Mat& cloud, const cv::Mat& colors, const cv::Mat& mask, const std::vector<pcl::Vertices> &vertices, const std::string &id, int viewport)
{
    CV_Assert(cloud.type() == CV_32FC3 && cloud.rows == 1 && !vertices.empty ());
    CV_Assert(colors.empty() || (!colors.empty() && colors.size() == cloud.size() && colors.type() == CV_8UC3));
    CV_Assert(mask.empty() || (!mask.empty() && mask.size() == cloud.size() && mask.type() == CV_8U));

    if (cloud_actor_map_->find (id) != cloud_actor_map_->end ())
      return std::cout << "[addPolygonMesh] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl, false;

//    int rgb_idx = -1;
//    std::vector<sensor_msgs::PointField> fields;


//    rgb_idx = pcl::getFieldIndex (*cloud, "rgb", fields);
//    if (rgb_idx == -1)
//      rgb_idx = pcl::getFieldIndex (*cloud, "rgba", fields);

    vtkSmartPointer<vtkUnsignedCharArray> colors_array;
#if 1
    if (!colors.empty())
    {
      colors_array = vtkSmartPointer<vtkUnsignedCharArray>::New ();
      colors_array->SetNumberOfComponents (3);
      colors_array->SetName ("Colors");

      const unsigned char* data = colors.ptr<unsigned char>();

      //TODO check mask
      CV_Assert(mask.empty()); //because not implemented;

      for(int i = 0; i < colors.cols; ++i)
          colors_array->InsertNextTupleValue(&data[i*3]);

//      pcl::RGB rgb_data;
//      for (size_t i = 0; i < cloud->size (); ++i)
//      {
//        if (!isFinite (cloud->points[i]))
//          continue;
//        memcpy (&rgb_data, reinterpret_cast<const char*> (&cloud->points[i]) + fields[rgb_idx].offset, sizeof (pcl::RGB));
//        unsigned char color[3];
//        color[0] = rgb_data.r;
//        color[1] = rgb_data.g;
//        color[2] = rgb_data.b;
//        colors->InsertNextTupleValue (color);
//      }
    }
#endif

    // Create points from polyMesh.cloud
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkIdType nr_points = cloud.size().area();

    points->SetNumberOfPoints (nr_points);


    // Get a pointer to the beginning of the data array
    float *data = static_cast<vtkFloatArray*> (points->GetData ())->GetPointer (0);


    std::vector<int> lookup;
    // If the dataset is dense (no NaNs)
    if (mask.empty())
    {
      cv::Mat hdr(cloud.size(), CV_32FC3, (void*)data);
      cloud.copyTo(hdr);
    }
    else
    {
        lookup.resize (nr_points);

        const unsigned char *mdata = mask.ptr<unsigned char>();
        const cv::Point3f *cdata = cloud.ptr<cv::Point3f>();
        cv::Point3f* out = reinterpret_cast<cv::Point3f*>(data);

        int j = 0;    // true point index
        for (int i = 0; i < nr_points; ++i)
            if(mdata[i])
            {
                lookup[i] = j;
                out[j++] = cdata[i];
            }
        nr_points = j;
        points->SetNumberOfPoints (nr_points);
    }

    // Get the maximum size of a polygon
    int max_size_of_polygon = -1;
    for (size_t i = 0; i < vertices.size (); ++i)
      if (max_size_of_polygon < static_cast<int> (vertices[i].vertices.size ()))
        max_size_of_polygon = static_cast<int> (vertices[i].vertices.size ());

    vtkSmartPointer<vtkLODActor> actor;

    if (vertices.size () > 1)
    {
      // Create polys from polyMesh.polygons
      vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();
      vtkIdType *cell = cell_array->WritePointer (vertices.size (), vertices.size () * (max_size_of_polygon + 1));
      int idx = 0;
      if (lookup.size () > 0)
      {
        for (size_t i = 0; i < vertices.size (); ++i, ++idx)
        {
          size_t n_points = vertices[i].vertices.size ();
          *cell++ = n_points;
          //cell_array->InsertNextCell (n_points);
          for (size_t j = 0; j < n_points; j++, ++idx)
            *cell++ = lookup[vertices[i].vertices[j]];
            //cell_array->InsertCellPoint (lookup[vertices[i].vertices[j]]);
        }
      }
      else
      {
        for (size_t i = 0; i < vertices.size (); ++i, ++idx)
        {
          size_t n_points = vertices[i].vertices.size ();
          *cell++ = n_points;
          //cell_array->InsertNextCell (n_points);
          for (size_t j = 0; j < n_points; j++, ++idx)
            *cell++ = vertices[i].vertices[j];
            //cell_array->InsertCellPoint (vertices[i].vertices[j]);
        }
      }
      vtkSmartPointer<vtkPolyData> polydata;
      allocVtkPolyData (polydata);
      cell_array->GetData ()->SetNumberOfValues (idx);
      cell_array->Squeeze ();
      polydata->SetStrips (cell_array);
      polydata->SetPoints (points);

      if (colors_array)
        polydata->GetPointData ()->SetScalars (colors_array);

      createActorFromVTKDataSet (polydata, actor, false);
    }
    else
    {
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
      size_t n_points = vertices[0].vertices.size ();
      polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

      if (lookup.size () > 0)
      {
        for (size_t j = 0; j < (n_points - 1); ++j)
          polygon->GetPointIds ()->SetId (j, lookup[vertices[0].vertices[j]]);
      }
      else
      {
        for (size_t j = 0; j < (n_points - 1); ++j)
          polygon->GetPointIds ()->SetId (j, vertices[0].vertices[j]);
      }
      vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
      allocVtkUnstructuredGrid (poly_grid);
      poly_grid->Allocate (1, 1);
      poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
      poly_grid->SetPoints (points);
      poly_grid->Update ();
      if (colors_array)
        poly_grid->GetPointData ()->SetScalars (colors_array);

      createActorFromVTKDataSet (poly_grid, actor, false);
    }
    addActorToRenderer (actor, viewport);
    actor->GetProperty ()->SetRepresentationToSurface ();
    // Backface culling renders the visualization slower, but guarantees that we see all triangles
    actor->GetProperty ()->BackfaceCullingOff ();
    actor->GetProperty ()->SetInterpolationToFlat ();
    actor->GetProperty ()->EdgeVisibilityOff ();
    actor->GetProperty ()->ShadingOff ();

    // Save the pointer/ID pair to the global actor map
    (*cloud_actor_map_)[id].actor = actor;
    //if (vertices.size () > 1)
    //  (*cloud_actor_map_)[id].cells = static_cast<vtkPolyDataMapper*>(actor->GetMapper ())->GetInput ()->GetVerts ()->GetData ();

    const Eigen::Vector4f& sensor_origin = Eigen::Vector4f::Zero ();
    const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternionf::Identity ();

    // Save the viewpoint transformation matrix to the global actor map
    vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
    convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
    (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

    return (true);


}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<pcl::Vertices> &vertices, const std::string &id, int viewport)
{
  if (vertices.empty () || cloud->points.empty ())
    return (false);

  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
    return std::cout << "[addPolygonMesh] A shape with id <" << id << "> already exists! Please choose a different id and retry." << std::endl, false;

  int rgb_idx = -1;
  std::vector<sensor_msgs::PointField> fields;
  vtkSmartPointer<vtkUnsignedCharArray> colors;
  rgb_idx = pcl::getFieldIndex (*cloud, "rgb", fields);
  if (rgb_idx == -1)
    rgb_idx = pcl::getFieldIndex (*cloud, "rgba", fields);

  if (rgb_idx != -1)
  {
    colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");

    pcl::RGB rgb_data;
    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (!isFinite (cloud->points[i]))
        continue;
      memcpy (&rgb_data, reinterpret_cast<const char*> (&cloud->points[i]) + fields[rgb_idx].offset, sizeof (pcl::RGB));
      unsigned char color[3];
      color[0] = rgb_data.r;
      color[1] = rgb_data.g;
      color[2] = rgb_data.b;
      colors->InsertNextTupleValue (color);
    }
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);
  vtkSmartPointer<vtkLODActor> actor;

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

      lookup[i] = static_cast<int> (j);
      memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
      j++;
      ptr += 3;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  // Get the maximum size of a polygon
  int max_size_of_polygon = -1;
  for (size_t i = 0; i < vertices.size (); ++i)
    if (max_size_of_polygon < static_cast<int> (vertices[i].vertices.size ()))
      max_size_of_polygon = static_cast<int> (vertices[i].vertices.size ());

  if (vertices.size () > 1)
  {
    // Create polys from polyMesh.polygons
    vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();
    vtkIdType *cell = cell_array->WritePointer (vertices.size (), vertices.size () * (max_size_of_polygon + 1));
    int idx = 0;
    if (lookup.size () > 0)
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        //cell_array->InsertNextCell (n_points);
        for (size_t j = 0; j < n_points; j++, ++idx)
          *cell++ = lookup[vertices[i].vertices[j]];
          //cell_array->InsertCellPoint (lookup[vertices[i].vertices[j]]);
      }
    }
    else
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        //cell_array->InsertNextCell (n_points);
        for (size_t j = 0; j < n_points; j++, ++idx)
          *cell++ = vertices[i].vertices[j];
          //cell_array->InsertCellPoint (vertices[i].vertices[j]);
      }
    }
    vtkSmartPointer<vtkPolyData> polydata;
    allocVtkPolyData (polydata);
    cell_array->GetData ()->SetNumberOfValues (idx);
    cell_array->Squeeze ();
    polydata->SetStrips (cell_array);
    polydata->SetPoints (points);

    if (colors)
      polydata->GetPointData ()->SetScalars (colors);

    createActorFromVTKDataSet (polydata, actor, false);
  }
  else
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    size_t n_points = vertices[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    if (lookup.size () > 0)
    {
      for (size_t j = 0; j < (n_points - 1); ++j)
        polygon->GetPointIds ()->SetId (j, lookup[vertices[0].vertices[j]]);
    }
    else
    {
      for (size_t j = 0; j < (n_points - 1); ++j)
        polygon->GetPointIds ()->SetId (j, vertices[0].vertices[j]);
    }
    vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
    allocVtkUnstructuredGrid (poly_grid);
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (points);
    poly_grid->Update ();
    if (colors)
      poly_grid->GetPointData ()->SetScalars (colors);

    createActorFromVTKDataSet (poly_grid, actor, false);
  }
  addActorToRenderer (actor, viewport);
  actor->GetProperty ()->SetRepresentationToSurface ();
  // Backface culling renders the visualization slower, but guarantees that we see all triangles
  actor->GetProperty ()->BackfaceCullingOff ();
  actor->GetProperty ()->SetInterpolationToFlat ();
  actor->GetProperty ()->EdgeVisibilityOff ();
  actor->GetProperty ()->ShadingOff ();

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  //if (vertices.size () > 1)
  //  (*cloud_actor_map_)[id].cells = static_cast<vtkPolyDataMapper*>(actor->GetMapper ())->GetInput ()->GetVerts ()->GetData ();

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
  convertToVtkMatrix (cloud->sensor_origin_, cloud->sensor_orientation_, transformation);
  (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

  return (true);
}


inline bool pcl::visualization::PCLVisualizer::updatePolygonMesh (const cv::Mat& cloud, const cv::Mat& colors, const cv::Mat& mask, const std::vector<pcl::Vertices> &vertices, const std::string &id)
{
    CV_Assert(cloud.type() == CV_32FC3 && cloud.rows == 1 && !vertices.empty ());
    CV_Assert(colors.empty() || (!colors.empty() && colors.size() == cloud.size() && colors.type() == CV_8UC3));
    CV_Assert(mask.empty() || (!mask.empty() && mask.size() == cloud.size() && mask.type() == CV_8U));

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
      return (false);

    // Get the current poly data
    vtkSmartPointer<vtkPolyData> polydata = static_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
    if (!polydata)
      return (false);
    vtkSmartPointer<vtkCellArray> cells = polydata->GetStrips ();
    if (!cells)
      return (false);
    vtkSmartPointer<vtkPoints> points   = polydata->GetPoints ();
    // Copy the new point array in
    vtkIdType nr_points = cloud.size().area();
    points->SetNumberOfPoints (nr_points);

    // Get a pointer to the beginning of the data array
    float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

    int ptr = 0;
    std::vector<int> lookup;
    // If the dataset is dense (no NaNs)
    if (mask.empty())
    {
        cv::Mat hdr(cloud.size(), CV_32FC3, (void*)data);
        cloud.copyTo(hdr);

    }
    else
    {
        lookup.resize (nr_points);

        const unsigned char *mdata = mask.ptr<unsigned char>();
        const cv::Point3f *cdata = cloud.ptr<cv::Point3f>();
        cv::Point3f* out = reinterpret_cast<cv::Point3f*>(data);

        int j = 0;    // true point index
        for (int i = 0; i < nr_points; ++i)
            if(mdata[i])
            {
                lookup[i] = j;
                out[j++] = cdata[i];
            }
        nr_points = j;
        points->SetNumberOfPoints (nr_points);;
    }

    // Update colors
    vtkUnsignedCharArray* colors_array = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());

    if (!colors.empty() && colors_array)
    {
        if (mask.empty())
        {
            const unsigned char* data = colors.ptr<unsigned char>();
            for(int i = 0; i < colors.cols; ++i)
                colors_array->InsertNextTupleValue(&data[i*3]);
        }
        else
        {
            const unsigned char* color = colors.ptr<unsigned char>();
            const unsigned char* mdata = mask.ptr<unsigned char>();

            int j = 0;
            for(int i = 0; i < colors.cols; ++i)
                if (mdata[i])
                    colors_array->SetTupleValue (j++, &color[i*3]);

        }
    }

    // Get the maximum size of a polygon
    int max_size_of_polygon = -1;
    for (size_t i = 0; i < vertices.size (); ++i)
      if (max_size_of_polygon < static_cast<int> (vertices[i].vertices.size ()))
        max_size_of_polygon = static_cast<int> (vertices[i].vertices.size ());

    // Update the cells
    cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkIdType *cell = cells->WritePointer (vertices.size (), vertices.size () * (max_size_of_polygon + 1));
    int idx = 0;
    if (lookup.size () > 0)
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        for (size_t j = 0; j < n_points; j++, cell++, ++idx)
          *cell = lookup[vertices[i].vertices[j]];
      }
    }
    else
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        for (size_t j = 0; j < n_points; j++, cell++, ++idx)
          *cell = vertices[i].vertices[j];
      }
    }
    cells->GetData ()->SetNumberOfValues (idx);
    cells->Squeeze ();
    // Set the the vertices
    polydata->SetStrips (cells);
    polydata->Update ();
    return (true);
}



/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool pcl::visualization::PCLVisualizer::updatePolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const std::vector<pcl::Vertices> &verts, const std::string &id)
{
  if (verts.empty ())
  {
     pcl::console::print_error ("[addPolygonMesh] No vertices given!\n");
     return (false);
  }

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it == cloud_actor_map_->end ())
    return (false);

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
  float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

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

  // Update colors
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
  int rgb_idx = -1;
  std::vector<sensor_msgs::PointField> fields;
  rgb_idx = pcl::getFieldIndex (*cloud, "rgb", fields);
  if (rgb_idx == -1)
    rgb_idx = pcl::getFieldIndex (*cloud, "rgba", fields);
  if (rgb_idx != -1 && colors)
  {
    pcl::RGB rgb_data;
    int j = 0;
    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (!isFinite (cloud->points[i]))
        continue;
      memcpy (&rgb_data,
              reinterpret_cast<const char*> (&cloud->points[i]) + fields[rgb_idx].offset,
              sizeof (pcl::RGB));
      unsigned char color[3];
      color[0] = rgb_data.r;
      color[1] = rgb_data.g;
      color[2] = rgb_data.b;
      colors->SetTupleValue (j++, color);
    }
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


/*
  vtkSmartPointer<vtkLODActor> actor;
  if (vertices.size () > 1)
  {
  }
  else
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    size_t n_points = vertices[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    for (size_t j = 0; j < (n_points - 1); ++j)
      polygon->GetPointIds ()->SetId (j, vertices[0].vertices[j]);

    vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
    allocVtkUnstructuredGrid (poly_grid);
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (points);
    poly_grid->Update ();

    createActorFromVTKDataSet (poly_grid, actor);
  }
*/

  // Get the colors from the handler
  //vtkSmartPointer<vtkDataArray> scalars;
  //color_handler.getColor (scalars);
  //polydata->GetPointData ()->SetScalars (scalars);
  //polydata->Update ();

  //am_it->second.actor->GetProperty ()->BackfaceCullingOn ();
  //am_it->second.actor->Modified ();

  return (true);
}
