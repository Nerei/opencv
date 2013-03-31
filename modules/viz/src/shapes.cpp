#include <q/visualization/common/shapes.h>

inline float rad2deg (float alpha)
{ return (alpha * 57.29578f); }

inline double rad2deg (double alpha){return (alpha * 57.29578);}


////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createCylinder (const pcl::ModelCoefficients &coefficients, int numsides)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
  line->SetPoint1 (coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  line->SetPoint2 (coefficients.values[3]+coefficients.values[0], coefficients.values[4]+coefficients.values[1], coefficients.values[5]+coefficients.values[2]);

  vtkSmartPointer<vtkTubeFilter> tuber = vtkSmartPointer<vtkTubeFilter>::New ();
  tuber->SetInputConnection (line->GetOutputPort ());
  tuber->SetRadius (coefficients.values[6]);
  tuber->SetNumberOfSides (numsides);

  return (tuber->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createSphere (const pcl::ModelCoefficients &coefficients, int res)
{
  // Set the sphere origin
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity (); t->Translate (coefficients.values[0], coefficients.values[1], coefficients.values[2]);

  vtkSmartPointer<vtkSphereSource> s_sphere = vtkSmartPointer<vtkSphereSource>::New ();
  s_sphere->SetRadius (coefficients.values[3]);
  s_sphere->SetPhiResolution (res);
  s_sphere->SetThetaResolution (res);
  s_sphere->LatLongTessellationOff ();

  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (s_sphere->GetOutputPort ());

  return (tf->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createCube (const pcl::ModelCoefficients &coefficients)
{
  // coefficients = [Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth]
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity ();
  t->Translate (coefficients.values[0], coefficients.values[1], coefficients.values[2]);

  Eigen::AngleAxisf a (Eigen::Quaternionf (coefficients.values[6], coefficients.values[3],
                                           coefficients.values[4], coefficients.values[5]));
  t->RotateWXYZ (rad2deg (a.angle ()), a.axis ()[0], a.axis ()[1], a.axis ()[2]);

  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetXLength (coefficients.values[7]);
  cube->SetYLength (coefficients.values[8]);
  cube->SetZLength (coefficients.values[9]);

  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (cube->GetOutputPort ());

  return (tf->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
                                double width, double height, double depth)
{
  // coefficients = [Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth]
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity ();
  t->Translate (translation.x (), translation.y (), translation.z ());

  Eigen::AngleAxisf a (rotation);
  t->RotateWXYZ (rad2deg (a.angle ()), a.axis ()[0], a.axis ()[1], a.axis ()[2]);

  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetXLength (width);
  cube->SetYLength (height);
  cube->SetZLength (depth);

  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (cube->GetOutputPort ());

  return (tf->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createCube (double x_min, double x_max,
                                double y_min, double y_max,
                                double z_min, double z_max)
{
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();

  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (x_min, x_max, y_min, y_max, z_min, z_max);

  return (cube->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet> pcl::visualization::createLine (const pcl::ModelCoefficients &coefficients)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
  line->SetPoint1 (coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  line->SetPoint2 (coefficients.values[3] + coefficients.values[0],
                   coefficients.values[4] + coefficients.values[1],
                   coefficients.values[5] + coefficients.values[2]);
  line->Update ();

  return (line->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet> pcl::visualization::createPlane (const pcl::ModelCoefficients &coefficients)
{
  vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New ();
  plane->SetNormal (coefficients.values[0], coefficients.values[1], coefficients.values[2]);

  double norm_sqr = coefficients.values[0] * coefficients.values[0]
                  + coefficients.values[1] * coefficients.values[1]
                  + coefficients.values[2] * coefficients.values[2];

  plane->Push (-coefficients.values[3] / sqrt(norm_sqr));
  return (plane->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet> pcl::visualization::createPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z)
{
  vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New ();


  double norm_sqr = 1.0 / (coefficients.values[0] * coefficients.values[0] +
                           coefficients.values[1] * coefficients.values[1] +
                           coefficients.values[2] * coefficients.values[2] );

//  double nx = coefficients.values [0] * norm;
//  double ny = coefficients.values [1] * norm;
//  double nz = coefficients.values [2] * norm;
//  double d  = coefficients.values [3] * norm;

//  plane->SetNormal (nx, ny, nz);
  plane->SetNormal (coefficients.values[0], coefficients.values[1], coefficients.values[2]);

  double t = x * coefficients.values[0] + y * coefficients.values[1] + z * coefficients.values[2] + coefficients.values[3];
  x -= coefficients.values[0] * t * norm_sqr;
  y -= coefficients.values[1] * t * norm_sqr;
  z -= coefficients.values[2] * t * norm_sqr;
  plane->SetCenter (x, y, z);

  return (plane->GetOutput ());
}


////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet> pcl::visualization::create2DCircle (const pcl::ModelCoefficients &coefficients, double z)
{
  vtkSmartPointer<vtkDiskSource> disk = vtkSmartPointer<vtkDiskSource>::New ();
  // Maybe the resolution should be lower e.g. 50 or 25
  disk->SetCircumferentialResolution (100);
  disk->SetInnerRadius (coefficients.values[2] - 0.001);
  disk->SetOuterRadius (coefficients.values[2] + 0.001);
  disk->SetCircumferentialResolution (20);

  // An alternative to <vtkDiskSource> could be <vtkRegularPolygonSource> with <vtkTubeFilter>
  /*
  vtkSmartPointer<vtkRegularPolygonSource> circle = vtkSmartPointer<vtkRegularPolygonSource>::New();
  circle->SetRadius (coefficients.values[2]);
  circle->SetNumberOfSides (100);

  vtkSmartPointer<vtkTubeFilter> tube = vtkSmartPointer<vtkTubeFilter>::New();
  tube->SetInput (circle->GetOutput());
  tube->SetNumberOfSides (25);
  tube->SetRadius (0.001);
  */

  // Set the circle origin
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity ();
  t->Translate (coefficients.values[0], coefficients.values[1], z);

  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (disk->GetOutputPort ());
  /*
  tf->SetInputConnection (tube->GetOutputPort ());
  */

  return (tf->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createCone (const pcl::ModelCoefficients &coefficients)
{
  vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New ();
  cone->SetHeight (1.0);
  cone->SetCenter (coefficients.values[0] + coefficients.values[3] * 0.5,
                   coefficients.values[1] + coefficients.values[1] * 0.5,
                   coefficients.values[2] + coefficients.values[2] * 0.5);
  cone->SetDirection (-coefficients.values[3], -coefficients.values[4], -coefficients.values[5]);
  cone->SetResolution (100);
  cone->SetAngle (coefficients.values[6]);

  return (cone->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createSphere (const Eigen::Vector4f &center, double radius, int res)
{
  // Set the sphere origin
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity ();
  t->Translate (center[0], center[1], center[2]);

  vtkSmartPointer<vtkSphereSource> s_sphere = vtkSmartPointer<vtkSphereSource>::New ();
  s_sphere->SetRadius (radius);
  s_sphere->SetPhiResolution (res);
  s_sphere->SetThetaResolution (res);
  s_sphere->LatLongTessellationOff ();

  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (s_sphere->GetOutputPort ());
  tf->Update ();

  return (tf->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::createLine (const Eigen::Vector4f &pt1, const Eigen::Vector4f &pt2)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
  line->SetPoint1 (pt1.x (), pt1.y (), pt1.z ());
  line->SetPoint2 (pt2.x (), pt2.y (), pt2.z ());
  line->Update ();

  return (line->GetOutput ());
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata)
{
  polydata = vtkSmartPointer<vtkUnstructuredGrid>::New ();
}

