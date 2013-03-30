#include <opencv2/viz/viz3d.hpp>
#include <opencv2/core.hpp>
#include <q/visualization/common/shapes.h>

void pcl::visualization::PCLVisualizer::setFullScreen (bool mode)
{
  if (win_)
    win_->SetFullScreen (mode);
}

void pcl::visualization::PCLVisualizer::setWindowName (const std::string &name)
{
  if (win_)
    win_->SetWindowName (name.c_str ());
}

void pcl::visualization::PCLVisualizer::addPointCloud(const cv::Mat& cloud, const cv::Mat& colors, const std::string& id, const cv::Mat& mask, int viewport)
{
    CV_Assert(cloud.type() == CV_32FC3 && colors.type() == CV_8UC3 && colors.size() == cloud.size());
    CV_Assert(mask.empty() || (mask.type() == CV_8U && mask.size() == cloud.size()));

    vtkSmartPointer<vtkPolyData> polydata;

    allocVtkPolyData(polydata);
    //polydata = vtkSmartPointer<vtkPolyData>::New ();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
    polydata->SetVerts (vertices);

    vtkSmartPointer<vtkIdTypeArray> initcells;
    vtkIdType nr_points = cloud.size().area();
    vtkSmartPointer<vtkPoints> points = polydata->GetPoints ();

    if (!points)
    {
        points = vtkSmartPointer<vtkPoints>::New ();
        points->SetDataTypeToFloat ();
        polydata->SetPoints (points);
    }
    points->SetNumberOfPoints (nr_points);

    // Get a pointer to the beginning of the data array
    float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

    if (mask.empty())
    {
        int j = 0;
        for(int y = 0; y < cloud.rows; ++y)
        {
            const cv::Point3f* crow = cloud.ptr<cv::Point3f>(y);
            for(int x = 0; x < cloud.cols; ++x)
                memcpy (&data[j++ * 3], &crow[x], sizeof(cv::Point3f));
        }
    }
    else
    {
        int j = 0;
        for(int y = 0; y < cloud.rows; ++y)
        {
            const cv::Point3f* crow = cloud.ptr<cv::Point3f>(y);
            const unsigned char* mrow = mask.ptr<unsigned char>(y);
            for(int x = 0; x < cloud.cols; ++x)
                if (mrow[x])
                    memcpy (&data[j++ * 3], &crow[x], sizeof(cv::Point3f));
        }
        nr_points = j;
        points->SetNumberOfPoints (nr_points);
    }

    vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
    updateCells (cells, initcells, nr_points);

    // Set the cells and the vertices
    vertices->SetCells (nr_points, cells);

    /////////////////////////////////////////////////////////////////////////////////

    // use the given geometry handler
    polydata->Update ();

    // Get the colors from the handler
    bool has_colors = false;
    double minmax[2];
    vtkSmartPointer<vtkDataArray> scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    scalars->SetNumberOfComponents (3);
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

    // Get a random color
    unsigned char* colors_data = new unsigned char[nr_points * 3];

    if (mask.empty())
    {
        int j = 0;
        for(int y = 0; y < colors.rows; ++y)
        {
            const cv::Vec3b* crow = colors.ptr<cv::Vec3b>(y);
            for(int x = 0; x < colors.cols; ++x)
                memcpy (&colors_data[j++ * 3], &crow[x], sizeof(cv::Vec3b));
        }
    }
    else
    {
        int j = 0;
        for(int y = 0; y < colors.rows; ++y)
        {
            const cv::Vec3b* crow = colors.ptr<cv::Vec3b>(y);
            const unsigned char* mrow = mask.ptr<unsigned char>(y);
            for(int x = 0; x < colors.cols; ++x)
                if (mrow[x])
                    memcpy (&colors_data[j++ * 3], &crow[x], sizeof(cv::Vec3b));
        }
    }

    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors_data, 3 * nr_points, 0);

    /////////////////////////////////////////
    has_colors = true;

    if (has_colors)
    {
        polydata->GetPointData ()->SetScalars (scalars);
        scalars->GetRange (minmax);
    }

    // Create an Actor
    vtkSmartPointer<vtkLODActor> actor;
    createActorFromVTKDataSet (polydata, actor);
    if (has_colors)
        actor->GetMapper ()->SetScalarRange (minmax);

    // Add it to all renderers
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*cloud_actor_map_)[id].actor = actor;
    (*cloud_actor_map_)[id].cells = initcells;

    const Eigen::Vector4f& sensor_origin = Eigen::Vector4f::Zero ();
    const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternionf::Identity ();

    // Save the viewpoint transformation matrix to the global actor map
    vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
    convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
    (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;
}


bool pcl::visualization::PCLVisualizer::updatePointCloud (const cv::Mat& cloud, const cv::Mat& colors, const std::string& id, const cv::Mat& mask)
{
    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
    if (am_it == cloud_actor_map_->end ())
        return (false);

    // Get the current poly data
    vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
    if (!polydata)
        return (false);
    vtkSmartPointer<vtkCellArray> vertices = polydata->GetVerts ();
    vtkSmartPointer<vtkPoints> points      = polydata->GetPoints ();
    // Copy the new point array in
    vtkIdType nr_points = cloud.size().area();
    points->SetNumberOfPoints (nr_points);

    // Get a pointer to the beginning of the data array
    float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

    if (mask.empty())
    {
        int j = 0;
        for(int y = 0; y < cloud.rows; ++y)
        {
            const cv::Point3f* crow = cloud.ptr<cv::Point3f>(y);
            for(int x = 0; x < cloud.cols; ++x)
                memcpy (&data[j++ * 3], &crow[x], sizeof(cv::Point3f));
        }
    }
    else
    {
        int j = 0;
        for(int y = 0; y < cloud.rows; ++y)
        {
            const cv::Point3f* crow = cloud.ptr<cv::Point3f>(y);
            const unsigned char* mrow = mask.ptr<unsigned char>(y);
            for(int x = 0; x < cloud.cols; ++x)
                if (mrow[x])
                    memcpy (&data[j++ * 3], &crow[x], sizeof(cv::Point3f));
        }
        nr_points = j;
        points->SetNumberOfPoints (nr_points);
    }

    vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
    updateCells (cells, am_it->second.cells, nr_points);


    // Set the cells and the vertices
    vertices->SetCells (nr_points, cells);

#if 1
    // Get the colors from the handler
    //  vtkSmartPointer<vtkDataArray> scalars;
    //  color_handler.getColor (scalars);
    //  double minmax[2];
    //  scalars->GetRange (minmax);

    //  // Update the data
    //  polydata->GetPointData ()->SetScalars (scalars);
    //  polydata->Update ();

    //  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();
    //  am_it->second.actor->GetMapper ()->SetScalarRange (minmax);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Get the colors from the handler
    bool has_colors = false;
    double minmax[2];
    vtkSmartPointer<vtkDataArray> scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    scalars->SetNumberOfComponents (3);
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

    // Get a random color
    unsigned char* colors_data = new unsigned char[nr_points * 3];

    if (mask.empty())
    {
        int j = 0;
        for(int y = 0; y < colors.rows; ++y)
        {
            const cv::Vec3b* crow = colors.ptr<cv::Vec3b>(y);
            for(int x = 0; x < colors.cols; ++x)
                memcpy (&colors_data[j++ * 3], &crow[x], sizeof(cv::Vec3b));
        }
    }
    else
    {
        int j = 0;
        for(int y = 0; y < colors.rows; ++y)
        {
            const cv::Vec3b* crow = colors.ptr<cv::Vec3b>(y);
            const unsigned char* mrow = mask.ptr<unsigned char>(y);
            for(int x = 0; x < colors.cols; ++x)
                if (mrow[x])
                    memcpy (&colors_data[j++ * 3], &crow[x], sizeof(cv::Vec3b));
        }
    }

    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors_data, 3 * nr_points, 0);

    /////////////////////////////////////////
    has_colors = true;

    if (has_colors)
    {
        polydata->GetPointData ()->SetScalars (scalars);
        scalars->GetRange (minmax);
    }

#else
    vtkSmartPointer<vtkDataArray> scalars;
    polydata->GetPointData ()->SetScalars (scalars);
    polydata->Update ();
    double minmax[2];
    minmax[0] = std::numeric_limits<double>::min ();
    minmax[1] = std::numeric_limits<double>::max ();
    am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();
    am_it->second.actor->GetMapper ()->SetScalarRange (minmax);
#endif


    // Update the mapper
    reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
    return (true);
}



bool pcl::visualization::PCLVisualizer::addPointCloudNormals (const cv::Mat &cloud, const cv::Mat& normals, int level, float scale, const std::string &id, int viewport)
{
    CV_Assert(cloud.size() == normals.size() && cloud.type() == CV_32FC3 && normals.type() == CV_32FC3);

  if (cloud_actor_map_->find (id) != cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  points->SetDataTypeToFloat ();
  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);

  vtkIdType nr_normals = 0;
  float* pts = 0;

  // If the cloud is organized, then distribute the normal step in both directions
  if (cloud.cols > 1 && cloud.rows > 1)
  {
    vtkIdType point_step = static_cast<vtkIdType> (sqrt (double (level)));
    nr_normals = (static_cast<vtkIdType> ((cloud.cols - 1)/ point_step) + 1) *
                 (static_cast<vtkIdType> ((cloud.rows - 1) / point_step) + 1);
    pts = new float[2 * nr_normals * 3];

    vtkIdType cell_count = 0;
    for (vtkIdType y = 0; y < cloud.rows; y += point_step)
      for (vtkIdType x = 0; x < cloud.cols; x += point_step)
      {
        cv::Point3f p = cloud.at<cv::Point3f>(y, x);
        cv::Point3f n = normals.at<cv::Point3f>(y, x) * scale;

        pts[2 * cell_count * 3 + 0] = p.x;
        pts[2 * cell_count * 3 + 1] = p.y;
        pts[2 * cell_count * 3 + 2] = p.z;
        pts[2 * cell_count * 3 + 3] = p.x + n.x;
        pts[2 * cell_count * 3 + 4] = p.y + n.y;
        pts[2 * cell_count * 3 + 5] = p.z + n.z;

        lines->InsertNextCell (2);
        lines->InsertCellPoint (2 * cell_count);
        lines->InsertCellPoint (2 * cell_count + 1);
        cell_count++;
    }
  }
  else
  {
    nr_normals = (cloud.size().area() - 1) / level + 1 ;
    pts = new float[2 * nr_normals * 3];

    for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * level)
    {
      cv::Point3f p = cloud.ptr<cv::Point3f>()[i];
      cv::Point3f n = normals.ptr<cv::Point3f>()[i] * scale;

      pts[2 * j * 3 + 0] = p.x;
      pts[2 * j * 3 + 1] = p.y;
      pts[2 * j * 3 + 2] = p.z;
      pts[2 * j * 3 + 3] = p.x + n.x;
      pts[2 * j * 3 + 4] = p.y + n.y;
      pts[2 * j * 3 + 5] = p.z + n.z;

      lines->InsertNextCell (2);
      lines->InsertCellPoint (2 * j);
      lines->InsertCellPoint (2 * j + 1);
    }
  }

  data->SetArray (&pts[0], 2 * nr_normals * 3, 0);
  points->SetData (data);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints (points);
  polyData->SetLines (lines);

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (polyData);
  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUsePointData();

  // create actor
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  actor->SetMapper (mapper);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  return (true);
}


////////////////////////////////////////////////////////////////////////////////////////////
bool pcl::visualization::PCLVisualizer::addLine (const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createLine (pt1.getVector4fMap (), pt2.getVector4fMap ());

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->GetMapper ()->ScalarVisibilityOff ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}
