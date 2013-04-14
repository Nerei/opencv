#pragma once

#include <opencv2/core.hpp>
// PCL includes
#include <opencv2/viz/events.hpp>
#include <q/visualization/interactor_style.h>
#include <q/visualization/point_picking_event.h>
#include <q/visualization/viz_types.h>
#include <q/visualization/common.h>
#include <opencv2/viz/types.hpp>
#include <opencv2/core/affine.hpp>

// VTK includes
#include <q/visualization/3rdparty.h>


#include <opencv2/viz/viz3d.hpp>

//#include <Eigen/Geometry>

namespace temp_viz
{
    inline cv::Scalar VIZ_RGB(unsigned char r, unsigned char g, unsigned char b) { return cv::Scalar(b, g, r); }



    class CV_EXPORTS Viz3d::VizImpl
    {
      public:
        typedef cv::Ptr<VizImpl> Ptr;

        VizImpl (const std::string &name = "Viz");

        virtual ~VizImpl ();
        void setFullScreen (bool mode);
        void setWindowName (const std::string &name);

        /** \brief Register a callback boost::function for keyboard events
          * \param[in] cb a boost function that will be registered as a callback for a keyboard event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> cb);
        inline boost::signals2::connection registerKeyboardCallback (void (*callback) (const cv::KeyboardEvent&, void*), void* cookie = NULL)
        { return (registerKeyboardCallback (boost::bind (callback, _1, cookie))); }

        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the member function that will be registered as a callback for a keyboard event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection registerKeyboardCallback (void (T::*callback) (const cv::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        { return (registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie))); }

        /** \brief Register a callback function for mouse events
          * \param[in] cb a boost function that will be registered as a callback for a mouse event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection registerMouseCallback (boost::function<void (const cv::MouseEvent&)> cb);
        inline boost::signals2::connection registerMouseCallback (void (*callback) (const cv::MouseEvent&, void*), void* cookie = NULL)
        { return (registerMouseCallback (boost::bind (callback, _1, cookie))); }

        /** \brief Register a callback function for mouse events
          * \param[in] callback  the member function that will be registered as a callback for a mouse event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection registerMouseCallback (void (T::*callback) (const cv::MouseEvent&, void*), T& instance, void* cookie = NULL)
        { return (registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie))); }

        /** \brief Register a callback function for point picking events
          * \param[in] cb a boost function that will be registered as a callback for a point picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection registerPointPickingCallback (boost::function<void (const cv::PointPickingEvent&)> cb);
        inline boost::signals2::connection registerPointPickingCallback (void (*callback) (const cv::PointPickingEvent&, void*), void* cookie = NULL)
        { return (registerPointPickingCallback (boost::bind (callback, _1, cookie))); }

        /** \brief Register a callback function for point picking events
          * \param[in] callback  the member function that will be registered as a callback for a point picking event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerPointPickingCallback (void (T::*callback) (const cv::PointPickingEvent&, void*), T& instance, void* cookie = NULL)
        { return (registerPointPickingCallback (boost::bind (callback, boost::ref (instance), _1, cookie))); }

        void spin ();
        void spinOnce (int time = 1, bool force_redraw = false);

         /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z, Roll,Pitch,Yaw
           *
           * \param[in] scale the scale of the axes (default: 1)
           * \param[in] t transformation matrix
           *
           * RPY Angles
           * Rotate the reference frame by the angle roll about axis x
           * Rotate the reference frame by the angle pitch about axis y
           * Rotate the reference frame by the angle yaw about axis z
           *
           * Description:
           * Sets the orientation of the Prop3D.  Orientation is specified as
           * X,Y and Z rotations in that order, but they are performed as
           * RotateZ, RotateX, and finally RotateY.
           *
           * All axies use right hand rule. x=red axis, y=green axis, z=blue axis
           * z direction is point into the screen.
           *     z
           *      \
           *       \
           *        \
           *         -----------> x
           *         |
           *         |
           *         |
           *         |
           *         |
           *         |
           *         y
           */
        void addCoordinateSystem (double scale, const cv::Affine3f& t, const std::string &id = "coordinate");

        /** \brief Removes a previously added 3D axes (coordinate system)
          */
        bool removeCoordinateSystem (const std::string &id = "coordinate");
        bool removePointCloud (const std::string &id = "cloud");
        inline bool removePolygonMesh (const std::string &id = "polygon")
        {
          // Polygon Meshes are represented internally as point clouds with special cell array structures since 1.4
          return removePointCloud (id);
        }
        bool removeShape (const std::string &id = "cloud");

        bool removeText3D (const std::string &id = "cloud");
        bool removeAllPointClouds ();
        bool removeAllShapes ();

        void setBackgroundColor (const cv::Scalar& color = cv::Scalar::all(0));

        bool addText (const std::string &text, int xpos, int ypos, const cv::Scalar& color = cv::Scalar(255, 255, 255), int fontsize = 10, const std::string &id = "");
        bool updateText (const std::string &text, int xpos, int ypos, const cv::Scalar& color  = cv::Scalar(255, 255, 255), int fontsize = 10, const std::string &id = "");

        /** \brief Set the pose of an existing shape. Returns false if the shape doesn't exist, true if the pose was succesfully updated. */
        bool updateShapePose (const std::string &id, const cv::Affine3f& pose);

        bool addText3D (const std::string &text, const cv::Point3f &position, double textScale = 1.0, double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "");

        bool addPointCloudNormals (const cv::Mat &cloud, const cv::Mat& normals, int level = 100, float scale = 0.02f, const std::string &id = "cloud");
        void addPointCloud(const cv::Mat& cloud, const cv::Mat& colors, const std::string& id = "cloud", const cv::Mat& mask = cv::Mat());
        bool updatePointCloud (const cv::Mat& cloud, const cv::Mat& colors, const std::string& id = "cloud", const cv::Mat& mask = cv::Mat());

        bool addPolygonMesh (const cv::Mat& cloud, const cv::Mat& colors, const cv::Mat& mask, const std::vector<temp_viz::Vertices> &vertices, const std::string &id = "polygon");
        bool updatePolygonMesh (const cv::Mat& cloud, const cv::Mat& colors, const cv::Mat& mask, const std::vector<temp_viz::Vertices> &vertices, const std::string &id = "polygon");

        bool addPolylineFromPolygonMesh (const cv::Mat& cloud, const std::vector<temp_viz::Vertices> &vertices, const std::string &id = "polyline");

        bool setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const std::string &id = "cloud");
        bool setPointCloudRenderingProperties (int property, double value, const std::string &id = "cloud");
        bool getPointCloudRenderingProperties (int property, double &value, const std::string &id = "cloud");

        bool setShapeRenderingProperties (int property, double value, const std::string &id);
        bool setShapeRenderingProperties (int property, double val1, double val2, double val3, const std::string &id);

        /** \brief Set whether the point cloud is selected or not
         *  \param[in] selected whether the cloud is selected or not (true = selected)
         *  \param[in] id the point cloud object id (default: cloud)
         */
        bool setPointCloudSelected (const bool selected, const std::string &id = "cloud" );

        /** \brief Returns true when the user tried to close the window */
        bool wasStopped () const { if (interactor_ != NULL) return (stopped_); else return true; }

        /** \brief Set the stopped flag back to false */
        void resetStoppedFlag () { if (interactor_ != NULL) stopped_ = false; }

        /** \brief Stop the interaction and close the visualizaton window. */
        void close ()
        {
          stopped_ = true;
          // This tends to close the window...
          interactor_->TerminateApp ();
        }

        //void createViewPortCamera ();

        bool addPolygon(const cv::Mat& cloud, const cv::Scalar& color = cv::Scalar(255, 255, 255), const std::string &id = "polygon");

        /** \brief Add a line segment from two points
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the line id/name (default: "line")
          */
        bool addLine (const cv::Point3f &pt1, const cv::Point3f &pt2, double r, double g, double b, const std::string &id = "line");

        /** \brief Add a line arrow segment between two points, and display the distance between them
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the arrow id/name (default: "arrow")
          */
        bool addArrow (const cv::Point3f &pt1, const cv::Point3f &pt2, double r, double g, double b, const std::string &id = "arrow");

        /** \brief Add a line arrow segment between two points, and display the distance between them
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] display_length true if the length should be displayed on the arrow as text
          * \param[in] id the line id/name (default: "arrow")
          */
        bool addArrow (const cv::Point3f &pt1, const cv::Point3f &pt2, double r, double g, double b, bool display_length, const std::string &id = "arrow");

        /** \brief Add a line arrow segment between two points, and display the distance between them in a given color
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r_line the red channel of the color that the line should be rendered with
          * \param[in] g_line the green channel of the color that the line should be rendered with
          * \param[in] b_line the blue channel of the color that the line should be rendered with
          * \param[in] r_text the red channel of the color that the text should be rendered with
          * \param[in] g_text the green channel of the color that the text should be rendered with
          * \param[in] b_text the blue channel of the color that the text should be rendered with
          * \param[in] id the line id/name (default: "arrow")
          */
        bool addArrow (const cv::Point3f &pt1, const cv::Point3f &pt2, double r_line, double g_line, double b_line,
                      double r_text, double g_text, double b_text, const std::string &id = "arrow");


        bool addSphere (const cv::Point3f &center, double radius, double r, double g, double b, const std::string &id = "sphere");
        bool updateSphere (const cv::Point3f &center, double radius, double r, double g, double b, const std::string &id = "sphere");

         /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] id the model id/name (default: "PolyData")
          */
        bool addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, const std::string & id = "PolyData");

        /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PolyData")
          */
        bool addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PolyData");

        /** \brief Add a PLYmodel as a mesh
          * \param[in] filename of the ply file
          * \param[in] id the model id/name (default: "PLYModel")
          */
        bool addModelFromPLYFile (const std::string &filename, const std::string &id = "PLYModel");

        /** \brief Add a PLYmodel as a mesh and applies given transformation
          * \param[in] filename of the ply file
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PLYModel")
          */
        bool addModelFromPLYFile (const std::string &filename, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PLYModel");

        /** \brief Add a cylinder from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radius)
          * \param[in] id the cylinder id/name (default: "cylinder")
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCylinder for more information.
          * // float radius;
          *
          * temp_viz::ModelCoefficients cylinder_coeff;
          * cylinder_coeff.values.resize (7);    // We need 7 values
          * cylinder_coeff.values[0] = pt_on_axis.x ();
          * cylinder_coeff.values[1] = pt_on_axis.y ();
          * cylinder_coeff.values[2] = pt_on_axis.z ();
          *
          * cylinder_coeff.values[3] = axis_direction.x ();
          * cylinder_coeff.values[4] = axis_direction.y ();
          * cylinder_coeff.values[5] = axis_direction.z ();
          *
          * cylinder_coeff.values[6] = radius;
          *
          * addCylinder (cylinder_coeff);
          * \endcode
          */
        bool addCylinder (const temp_viz::ModelCoefficients &coefficients, const std::string &id = "cylinder");

        /** \brief Add a plane from a set of given model coefficients
          * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
          * \param[in] id the plane id/name (default: "plane")
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelPlane for more information

          *
          * temp_viz::ModelCoefficients plane_coeff;
          * plane_coeff.values.resize (4);    // We need 4 values
          * plane_coeff.values[0] = plane_parameters.x ();
          * plane_coeff.values[1] = plane_parameters.y ();
          * plane_coeff.values[2] = plane_parameters.z ();
          * plane_coeff.values[3] = plane_parameters.w ();
          *
          * addPlane (plane_coeff);
          * \endcode
          */
        bool addPlane (const temp_viz::ModelCoefficients &coefficients, const std::string &id = "plane");
        bool addPlane (const temp_viz::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id = "plane");

        /** \brief Add a circle from a set of given model coefficients
          * \param[in] coefficients the model coefficients (x, y, radius)
          * \param[in] id the circle id/name (default: "circle")
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCircle2D for more information
          * // float x, y, radius;
          *
          * temp_viz::ModelCoefficients circle_coeff;
          * circle_coeff.values.resize (3);    // We need 3 values
          * circle_coeff.values[0] = x;
          * circle_coeff.values[1] = y;
          * circle_coeff.values[2] = radius;
          *
          * vtkSmartPointer<vtkDataSet> data = temp_viz::create2DCircle (circle_coeff, z);
          * \endcode
           */
        bool addCircle (const temp_viz::ModelCoefficients &coefficients, const std::string &id = "circle");

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] coefficients the model coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
          * \param[in] id the cube id/name (default: "cube")
          */
        bool addCube (const temp_viz::ModelCoefficients &coefficients, const std::string &id = "cube");

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] translation a translation to apply to the cube from 0,0,0
          * \param[in] rotation a quaternion-based rotation to apply to the cube
          * \param[in] width the cube's width
          * \param[in] height the cube's height
          * \param[in] depth the cube's depth
          * \param[in] id the cube id/name (default: "cube")
          */
        bool addCube (const cv::Vec3f& translation, const cv::Vec3f quaternion, double width, double height, double depth, const std::string &id = "cube");

        /** \brief Add a cube
          * \param[in] x_min the min X coordinate
          * \param[in] x_max the max X coordinate
          * \param[in] y_min the min Y coordinate
          * \param[in] y_max the max Y coordinate
          * \param[in] z_min the min Z coordinate
          * \param[in] z_max the max Z coordinate
          * \param[in] r how much red (0.0 -> 1.0)
          * \param[in] g how much green (0.0 -> 1.0)
          * \param[in] b how much blue (0.0 -> 1.0)
          * \param[in] id the cube id/name (default: "cube")
          */
        bool addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                 double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube");

        /** \brief Changes the visual representation for all actors to surface representation. */
        void setRepresentationToSurfaceForAllActors ();

        /** \brief Changes the visual representation for all actors to points representation. */
        void setRepresentationToPointsForAllActors ();

        /** \brief Changes the visual representation for all actors to wireframe representation. */
        void setRepresentationToWireframeForAllActors ();

        /** \brief Initialize camera parameters with some default values. */
        void initCameraParameters ();

        /** \brief Search for camera parameters at the command line and set them internally.
        bool getCameraParameters (int argc, char **argv);

        /** \brief Checks whether the camera parameters were manually loaded from file.*/
        bool cameraParamsSet () const;

        /** \brief Update camera parameters and render. */
        void updateCamera ();

        /** \brief Reset camera parameters and render. */
        void resetCamera ();

        /** \brief Reset the camera direction from {0, 0, 0} to the center_{x, y, z} of a given dataset.
          * \param[in] id the point cloud object id (default: cloud)
          */
        void resetCameraViewpoint (const std::string &id = "cloud");

        /** \brief Set the camera pose given by position, viewpoint and up vector
          * \param[in] pos_x the x coordinate of the camera location
          * \param[in] pos_y the y coordinate of the camera location
          * \param[in] pos_z the z coordinate of the camera location
          * \param[in] view_x the x component of the view point of the camera
          * \param[in] view_y the y component of the view point of the camera
          * \param[in] view_z the z component of the view point of the camera
          * \param[in] up_x the x component of the view up direction of the camera
          * \param[in] up_y the y component of the view up direction of the camera
          * \param[in] up_z the y component of the view up direction of the camera
          */
        void setCameraPosition (const cv::Vec3d& pos, const cv::Vec3d& view, const cv::Vec3d& up);

        /** \brief Set the camera location and viewup according to the given arguments
          * \param[in] pos_x the x coordinate of the camera location
          * \param[in] pos_y the y coordinate of the camera location
          * \param[in] pos_z the z coordinate of the camera location
          * \param[in] up_x the x component of the view up direction of the camera
          * \param[in] up_y the y component of the view up direction of the camera
          * \param[in] up_z the z component of the view up direction of the camera
          */
        void setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z);

        /** \brief Set the camera parameters via an intrinsics and and extrinsics matrix
          * \note This assumes that the pixels are square and that the center of the image is at the center of the sensor.
          * \param[in] intrinsics the intrinsics that will be used to compute the VTK camera parameters
          * \param[in] extrinsics the extrinsics that will be used to compute the VTK camera parameters
          */
        void setCameraParameters (const cv::Matx33f& intrinsics, const cv::Affine3f& extrinsics);

        /** \brief Set the camera parameters by given a full camera data structure.
          * \param[in] camera camera structure containing all the camera parameters.
          */
        void setCameraParameters (const Camera &camera);

        /** \brief Set the camera clipping distances.
          * \param[in] near the near clipping distance (no objects closer than this to the camera will be drawn)
          * \param[in] far the far clipping distance (no objects further away than this to the camera will be drawn)
          */
        void setCameraClipDistances (double near, double far);

        /** \brief Set the camera vertical field of view.
          * \param[in] fovy vertical field of view in radians
          */
        void setCameraFieldOfView (double fovy);

        /** \brief Get the current camera parameters. */
        void getCameras (std::vector<Camera>& cameras);

        /** \brief Get the current viewing pose. */
        cv::Affine3f getViewerPose ();
        void saveScreenshot (const std::string &file);

        /** \brief Return a pointer to the underlying VTK Render Window used. */
        vtkSmartPointer<vtkRenderWindow> getRenderWindow () { return (win_); }

        void setPosition (int x, int y);
        void setSize (int xw, int yw);

      protected:
        /** \brief The render window interactor. */

        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
      private:
        struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New()
          {
            return new ExitMainLoopTimerCallback;
          }
          virtual void Execute(vtkObject* vtkNotUsed(caller), unsigned long event_id, void* call_data)
          {
            if (event_id != vtkCommand::TimerEvent)
              return;
            int timer_id = * static_cast<int*> (call_data);
            //PCL_WARN ("[temp_viz::PCLVisualizer::ExitMainLoopTimerCallback] Timer %d called.\n", timer_id);
            if (timer_id != right_timer_id)
              return;
            // Stop vtk loop and send notification to app to wake it up
            pcl_visualizer->interactor_->TerminateApp ();
          }
          int right_timer_id;
          VizImpl* pcl_visualizer;
        };
        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return new ExitCallback;
          }
          virtual void Execute (vtkObject*, unsigned long event_id, void*)
          {
            if (event_id != vtkCommand::ExitEvent)
              return;
            pcl_visualizer->stopped_ = true;
            // This tends to close the window...
            pcl_visualizer->interactor_->TerminateApp ();
          }
          VizImpl* pcl_visualizer;
        };

        /** \brief Set to false if the interaction loop is running. */
        bool stopped_;

        double s_lastDone_;

        /** \brief Global timer ID. Used in destructor only. */
        int timer_id_;

        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;

        /** \brief The collection of renderers used. */
        vtkSmartPointer<vtkRenderer> ren_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The render window interactor style. */
        vtkSmartPointer<PCLVisualizerInteractorStyle> style_;

        /** \brief Internal list with actor pointers and name IDs for point clouds. */
        CloudActorMapPtr cloud_actor_map_;

        /** \brief Internal list with actor pointers and name IDs for shapes. */
        ShapeActorMapPtr shape_actor_map_;

        /** \brief Internal pointer to widget which contains a set of axes */
        vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;

        /** \brief Boolean that holds whether or not the camera parameters were manually initialized*/
        bool camera_set_;

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          */
        bool removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor);

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          */
        bool removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object */
        void addActorToRenderer (const vtkSmartPointer<vtkProp> &actor);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object
          */
        bool removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkActor> &actor, bool use_scalars = true);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkLODActor> &actor, bool use_scalars = true);

        /** \brief Updates a set of cells (vtkIdTypeArray) if the number of points in a cloud changes
          * \param[out] cells the vtkIdTypeArray object (set of cells) to update
          * \param[out] initcells a previously saved set of cells. If the number of points in the current cloud is
          * higher than the number of cells in \a cells, and initcells contains enough data, then a copy from it
          * will be made instead of regenerating the entire array.
          * \param[in] nr_points the number of points in the new cloud. This dictates how many cells we need to
          * generate
          */
        void updateCells (vtkSmartPointer<vtkIdTypeArray> &cells, vtkSmartPointer<vtkIdTypeArray> &initcells, vtkIdType nr_points);

        void allocVtkPolyData (vtkSmartPointer<vtkAppendPolyData> &polydata);
        void allocVtkPolyData (vtkSmartPointer<vtkPolyData> &polydata);

        /** \brief Allocate a new unstructured grid smartpointer. Internal
          * \param[out] polydata the resultant poly data
          */
        void allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata);

    };

    //void getTransformationMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternionf& orientation, Eigen::Matrix4f &transformation);

    //void convertToVtkMatrix (const Eigen::Matrix4f &m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

    void convertToVtkMatrix (const cv::Matx44f& m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

    /** \brief Convert origin and orientation to vtkMatrix4x4
      * \param[in] origin the point cloud origin
      * \param[in] orientation the point cloud orientation
      * \param[out] vtk_matrix the resultant VTK 4x4 matrix
      */
    void convertToVtkMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternion<float> &orientation, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);
    void convertToEigenMatrix (const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix, Eigen::Matrix4f &m);

}



