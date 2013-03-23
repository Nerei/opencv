#pragma once

#include <pcl/pcl_macros.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PointCloudColorHandlerRandom<PointT>::getColor (vtkSmartPointer<vtkDataArray> &scalars) const
{
  if (!capable_ || !cloud_)
    return (false);

  if (!scalars)
    scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  scalars->SetNumberOfComponents (3);
  
  vtkIdType nr_points = cloud_->points.size ();
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);

  // Get a random color
  unsigned char* colors = new unsigned char[nr_points * 3];
  double r, g, b;
  pcl::visualization::getRandomColors (r, g, b);

  int r_ = static_cast<int> (pcl_lrint (r * 255.0)), 
      g_ = static_cast<int> (pcl_lrint (g * 255.0)), 
      b_ = static_cast<int> (pcl_lrint (b * 255.0));

  // Color every point
  for (vtkIdType cp = 0; cp < nr_points; ++cp)
  {
    colors[cp * 3 + 0] = static_cast<unsigned char> (r_);
    colors[cp * 3 + 1] = static_cast<unsigned char> (g_);
    colors[cp * 3 + 2] = static_cast<unsigned char> (b_);
  }
  reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3 * nr_points, 0);
  return (true);
}
