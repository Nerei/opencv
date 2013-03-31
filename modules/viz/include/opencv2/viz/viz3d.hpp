#pragma once

#include <opencv2/core.hpp>
// PCL includes
//#include <pcl/point_types.h>
//#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <q/PolygonMesh.h>
#include <q/planar_polygon.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>

//#include <q/visualization/common/common.h>
//#include <q/visualization/common/shapes.h>
#include <q/visualization/interactor_style.h>
//#include <q/visualization/window.h>

#include <q/visualization/keyboard_event.h>
#include <q/visualization/mouse_event.h>
#include <q/visualization/point_picking_event.h>
#include <q/visualization/common/actor_map.h>

// VTK includes
#include <q/visualization/vtk.h>
#include <q/visualization/boost.h>

#include <Eigen/Geometry>

namespace pcl
{
  namespace visualization
  {
    class CV_EXPORTS PCLVisualizer
    {
      public:
        typedef boost::shared_ptr<PCLVisualizer> Ptr;

        PCLVisualizer (const std::string &name = "Viz", const bool create_interactor = true);

        virtual ~PCLVisualizer ();
        void setFullScreen (bool mode);
        void setWindowName (const std::string &name);

        /** \brief Register a callback boost::function for keyboard events
          * \param[in] cb a boost function that will be registered as a callback for a keyboard event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> cb);
        inline boost::signals2::connection
        registerKeyboardCallback (void (*callback) (const cv::KeyboardEvent&, void*), void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback, _1, cookie)));
        }

        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the member function that will be registered as a callback for a keyboard event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerKeyboardCallback (void (T::*callback) (const cv::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie)));
        }

        /** \brief Register a callback function for mouse events
          * \param[in] cb a boost function that will be registered as a callback for a mouse event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerMouseCallback (boost::function<void (const cv::MouseEvent&)> cb);
        inline boost::signals2::connection
        registerMouseCallback (void (*callback) (const cv::MouseEvent&, void*), void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, _1, cookie)));
        }

        /** \brief Register a callback function for mouse events
          * \param[in] callback  the member function that will be registered as a callback for a mouse event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerMouseCallback (void (T::*callback) (const cv::MouseEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        /** \brief Register a callback function for point picking events
          * \param[in] cb a boost function that will be registered as a callback for a point picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)> cb);
        inline boost::signals2::connection registerPointPickingCallback (void (*callback) (const pcl::visualization::PointPickingEvent&, void*), void* cookie = NULL)
        {
          return (registerPointPickingCallback (boost::bind (callback, _1, cookie)));
        }

        /** \brief Register a callback function for point picking events
          * \param[in] callback  the member function that will be registered as a callback for a point picking event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerPointPickingCallback (void (T::*callback) (const pcl::visualization::PointPickingEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerPointPickingCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        void spin ();
        void spinOnce (int time = 1, bool force_redraw = false);

         /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z, Roll,Pitch,Yaw
           *
           * \param[in] scale the scale of the axes (default: 1)
           * \param[in] t transformation matrix
           * \param[in] viewport the view port where the 3D axes should be added (default: all)
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
        void addCoordinateSystem (double scale, const Eigen::Affine3f& t, int viewport = 0);

        /** \brief Removes a previously added 3D axes (coordinate system)
          * \param[in] viewport view port where the 3D axes should be removed from (default: all)
          */
        bool removeCoordinateSystem (int viewport = 0);

        /** \brief Removes a Point Cloud from screen, based on a given ID.
          * \param[in] id the point cloud object id (i.e., given on \a addPointCloud)
          * \param[in] viewport view port from where the Point Cloud should be removed (default: all)
          * \return true if the point cloud is successfully removed and false if the point cloud is
          * not actually displayed
          */
        bool removePointCloud (const std::string &id = "cloud", int viewport = 0);

        /** \brief Removes a PolygonMesh from screen, based on a given ID.
          * \param[in] id the polygon object id (i.e., given on \a addPolygonMesh)
          * \param[in] viewport view port from where the PolygonMesh should be removed (default: all)
          */
        inline bool
        removePolygonMesh (const std::string &id = "polygon", int viewport = 0)
        {
          // Polygon Meshes are represented internally as point clouds with special cell array structures since 1.4
          return (removePointCloud (id, viewport));
        }

        /** \brief Removes an added shape from screen (line, polygon, etc.), based on a given ID
          * \note This methods also removes PolygonMesh objects and PointClouds, if they match the ID
          * \param[in] id the shape object id (i.e., given on \a addLine etc.)
          * \param[in] viewport view port from where the Point Cloud should be removed (default: all)
          */
        bool removeShape (const std::string &id = "cloud", int viewport = 0);

        /** \brief Removes an added 3D text from the scene, based on a given ID
          * \param[in] id the 3D text id (i.e., given on \a addText3D etc.)
          * \param[in] viewport view port from where the 3D text should be removed (default: all)
          */
        bool removeText3D (const std::string &id = "cloud", int viewport = 0);

        /** \brief Remove all point cloud data on screen from the given viewport.
          * \param[in] viewport view port from where the clouds should be removed (default: all)
          */
        bool removeAllPointClouds (int viewport = 0);

        /** \brief Remove all 3D shape data on screen from the given viewport.
          * \param[in] viewport view port from where the shapes should be removed (default: all)
          */
        bool removeAllShapes (int viewport = 0);

        /** \brief Set the viewport's background color.
          * \param[in] r the red component of the RGB color
          * \param[in] g the green component of the RGB color
          * \param[in] b the blue component of the RGB color
          * \param[in] viewport the view port (default: all)
          */
        void setBackgroundColor (const double &r, const double &g, const double &b, int viewport = 0);

        bool addText (const std::string &text, int xpos, int ypos, const cv::Scalar& color = cv::Scalar(255, 255, 255), int fontsize = 10, const std::string &id = "", int viewport = 0);
        bool updateText (const std::string &text, int xpos, int ypos, const cv::Scalar& color  = cv::Scalar(255, 255, 255), int fontsize = 10, const std::string &id = "");

        /** \brief Set the pose of an existing shape.
          *
          * Returns false if the shape doesn't exist, true if the pose was succesfully
          * updated.
          *
          * \param[in] id the shape or cloud object id (i.e., given on \a addLine etc.)
          * \param[in] pose the new pose
          * \return false if no shape or cloud with the specified ID was found
          */
        bool updateShapePose (const std::string &id, const Eigen::Affine3f& pose);

        bool addText3D (const std::string &text, const PointXYZ &position, double textScale = 1.0,
                   double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "", int viewport = 0);

        bool addPointCloudNormals (const cv::Mat &cloud, const cv::Mat& normals, int level = 100, float scale = 0.02f, const std::string &id = "cloud", int viewport = 0);
        void addPointCloud(const cv::Mat& cloud, const cv::Mat& colors, const std::string& id = "cloud", const cv::Mat& mask = cv::Mat(), int viewport = 0);
        bool updatePointCloud (const cv::Mat& cloud, const cv::Mat& colors, const std::string& id = "cloud", const cv::Mat& mask = cv::Mat());


        /** \brief Add a PolygonMesh object to screen
          * \param[in] polymesh the polygonal mesh
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        bool addPolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polygon", int viewport = 0);

        /** \brief Add a PolygonMesh object to screen
          * \param[in] cloud the polygonal mesh point cloud
          * \param[in] vertices the polygonal mesh vertices
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        template <typename PointT> bool
        addPolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<pcl::Vertices> &vertices,
                        const std::string &id = "polygon", int viewport = 0);

        /** \brief Update a PolygonMesh object on screen
          * \param[in] cloud the polygonal mesh point cloud
          * \param[in] vertices the polygonal mesh vertices
          * \param[in] id the polygon object id (default: "polygon")
          * \return false if no polygonmesh with the specified ID was found
          */
        template <typename PointT> bool
        updatePolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<pcl::Vertices> &vertices, const std::string &id = "polygon");

        /** \brief Update a PolygonMesh object on screen
          * \param[in] polymesh the polygonal mesh
          * \param[in] id the polygon object id (default: "polygon")
          * \return false if no polygonmesh with the specified ID was found
          */
        bool updatePolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polygon");

        /** \brief Add a Polygonline from a polygonMesh object to screen
          * \param[in] polymesh the polygonal mesh from where the polylines will be extracted
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        bool addPolylineFromPolygonMesh (const pcl::PolygonMesh &polymesh, const std::string &id = "polyline", int viewport = 0);


        /** \brief Set the rendering properties of a PointCloud (3x values - e.g., RGB)
          * \param[in] property the property type
          * \param[in] val1 the first value to be set
          * \param[in] val2 the second value to be set
          * \param[in] val3 the third value to be set
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
          */
        bool setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const std::string &id = "cloud", int viewport = 0);

       /** \brief Set the rendering properties of a PointCloud
         * \param[in] property the property type
         * \param[in] value the value to be set
         * \param[in] id the point cloud object id (default: cloud)
         * \param[in] viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
         */
        bool setPointCloudRenderingProperties (int property, double value, const std::string &id = "cloud", int viewport = 0);

       /** \brief Get the rendering properties of a PointCloud
         * \param[in] property the property type
         * \param[in] value the resultant property value
         * \param[in] id the point cloud object id (default: cloud)
         */
        bool getPointCloudRenderingProperties (int property, double &value, const std::string &id = "cloud");

        /** \brief Set whether the point cloud is selected or not
         *  \param[in] selected whether the cloud is selected or not (true = selected)
         *  \param[in] id the point cloud object id (default: cloud)
         */
        bool setPointCloudSelected (const bool selected, const std::string &id = "cloud" );

       /** \brief Set the rendering properties of a shape
         * \param[in] property the property type
         * \param[in] value the value to be set
         * \param[in] id the shape object id
         * \param[in] viewport the view port where the shape's properties should be modified (default: all)
         */
        bool
        setShapeRenderingProperties (int property, double value,
                                     const std::string &id, int viewport = 0);

        /** \brief Set the rendering properties of a shape (3x values - e.g., RGB)
          * \param[in] property the property type
          * \param[in] val1 the first value to be set
          * \param[in] val2 the second value to be set
          * \param[in] val3 the third value to be set
          * \param[in] id the shape object id
          * \param[in] viewport the view port where the shape's properties should be modified (default: all)
          */
         bool
         setShapeRenderingProperties (int property, double val1, double val2, double val3,
                                      const std::string &id, int viewport = 0);


        /** \brief Returns true when the user tried to close the window */
        bool
        wasStopped () const { if (interactor_ != NULL) return (stopped_); else return (true); }

        /** \brief Set the stopped flag back to false */
        void
        resetStoppedFlag () { if (interactor_ != NULL) stopped_ = false; }

        /** \brief Stop the interaction and close the visualizaton window. */
        void
        close ()
        {
          stopped_ = true;
          // This tends to close the window...
          interactor_->TerminateApp ();
        }

        /** \brief Create a new viewport from [xmin,ymin] -> [xmax,ymax].
          * \param[in] xmin the minimum X coordinate for the viewport (0.0 <= 1.0)
          * \param[in] ymin the minimum Y coordinate for the viewport (0.0 <= 1.0)
          * \param[in] xmax the maximum X coordinate for the viewport (0.0 <= 1.0)
          * \param[in] ymax the maximum Y coordinate for the viewport (0.0 <= 1.0)
          * \param[in] viewport the id of the new viewport
          *
          * \note If no renderer for the current window exists, one will be created, and
          * the viewport will be set to 0 ('all'). In case one or multiple renderers do
          * exist, the viewport ID will be set to the total number of renderers - 1.
          */
        void
        createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport);

        /** \brief Create a new separate camera for the given viewport.
          * \param[in] viewport the viewport to create a new camera for.
          */
        void
        createViewPortCamera (const int viewport);

        /** \brief Add a polygon (polyline) that represents the input point cloud (connects all
          * points in order)
          * \param[in] cloud the point cloud dataset representing the polygon
          * \param[in] r the red channel of the color that the polygon should be rendered with
          * \param[in] g the green channel of the color that the polygon should be rendered with
          * \param[in] b the blue channel of the color that the polygon should be rendered with
          * \param[in] id (optional) the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    double r, double g, double b,
                    const std::string &id = "polygon", int viewport = 0);

        /** \brief Add a polygon (polyline) that represents the input point cloud (connects all
          * points in order)
          * \param[in] cloud the point cloud dataset representing the polygon
          * \param[in] id the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &id = "polygon", int viewport = 0);

        /** \brief Add a planar polygon that represents the input point cloud (connects all points in order)
          * \param[in] polygon the polygon to draw
          * \param[in] r the red channel of the color that the polygon should be rendered with
          * \param[in] g the green channel of the color that the polygon should be rendered with
          * \param[in] b the blue channel of the color that the polygon should be rendered with
          * \param[in] id the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const pcl::PlanarPolygon<PointT> &polygon, double r, double g, double b, const std::string &id = "polygon", int viewport = 0);

        /** \brief Add a line segment from two points
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the line id/name (default: "line")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addLine (const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, double r, double g, double b, const std::string &id = "line", int viewport = 0);

        /** \brief Add a line arrow segment between two points, and display the distance between them
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the arrow id/name (default: "arrow")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id = "arrow", int viewport = 0);

        /** \brief Add a line arrow segment between two points, and display the distance between them
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] display_length true if the length should be displayed on the arrow as text
          * \param[in] id the line id/name (default: "arrow")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length, const std::string &id = "arrow", int viewport = 0);

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
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
          template <typename P1, typename P2> bool
          addArrow (const P1 &pt1, const P2 &pt2, double r_line, double g_line, double b_line,
                      double r_text, double g_text, double b_text, const std::string &id = "arrow", int viewport = 0);


        bool addSphere (const PointXYZ &center, double radius, double r, double g, double b, const std::string &id = "sphere", int viewport = 0);
        bool updateSphere (const PointXYZ &center, double radius, double r, double g, double b, const std::string &id = "sphere");

         /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] id the model id/name (default: "PolyData")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, const std::string & id = "PolyData", int viewport = 0);

        /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PolyData")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PolyData", int viewport = 0);

        /** \brief Add a PLYmodel as a mesh
          * \param[in] filename of the ply file
          * \param[in] id the model id/name (default: "PLYModel")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addModelFromPLYFile (const std::string &filename, const std::string &id = "PLYModel", int viewport = 0);

        /** \brief Add a PLYmodel as a mesh and applies given transformation
          * \param[in] filename of the ply file
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PLYModel")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addModelFromPLYFile (const std::string &filename, vtkSmartPointer<vtkTransform> transform, const std::string &id = "PLYModel", int viewport = 0);

        /** \brief Add a cylinder from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radius)
          * \param[in] id the cylinder id/name (default: "cylinder")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCylinder for more information.
          * // Eigen::Vector3f pt_on_axis, axis_direction;
          * // float radius;
          *
          * pcl::ModelCoefficients cylinder_coeff;
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
        bool addCylinder (const pcl::ModelCoefficients &coefficients, const std::string &id = "cylinder", int viewport = 0);

        /** \brief Add a sphere from a set of given model coefficients
          * \param[in] coefficients the model coefficients (sphere center, radius)
          * \param[in] id the sphere id/name (default: "sphere")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelSphere for more information
          * // Eigen::Vector3f sphere_center;
          * // float radius;
          *
          * pcl::ModelCoefficients sphere_coeff;
          * sphere_coeff.values.resize (4);    // We need 4 values
          * sphere_coeff.values[0] = sphere_center.x ();
          * sphere_coeff.values[1] = sphere_center.y ();
          * sphere_coeff.values[2] = sphere_center.z ();
          *
          * sphere_coeff.values[3] = radius;
          *
          * addSphere (sphere_coeff);
          * \endcode
          */
        bool addSphere (const pcl::ModelCoefficients &coefficients, const std::string &id = "sphere", int viewport = 0);

        /** \brief Add a line from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_line, direction)
          * \param[in] id the line id/name (default: "line")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelLine for more information
          * // Eigen::Vector3f point_on_line, line_direction;
          *
          * pcl::ModelCoefficients line_coeff;
          * line_coeff.values.resize (6);    // We need 6 values
          * line_coeff.values[0] = point_on_line.x ();
          * line_coeff.values[1] = point_on_line.y ();
          * line_coeff.values[2] = point_on_line.z ();
          *
          * line_coeff.values[3] = line_direction.x ();
          * line_coeff.values[4] = line_direction.y ();
          * line_coeff.values[5] = line_direction.z ();
          *
          * addLine (line_coeff);
          * \endcode
          */
        bool addLine (const pcl::ModelCoefficients &coefficients, const std::string &id = "line", int viewport = 0);

        /** \brief Add a plane from a set of given model coefficients
          * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
          * \param[in] id the plane id/name (default: "plane")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelPlane for more information
          * // Eigen::Vector4f plane_parameters;
          *
          * pcl::ModelCoefficients plane_coeff;
          * plane_coeff.values.resize (4);    // We need 4 values
          * plane_coeff.values[0] = plane_parameters.x ();
          * plane_coeff.values[1] = plane_parameters.y ();
          * plane_coeff.values[2] = plane_parameters.z ();
          * plane_coeff.values[3] = plane_parameters.w ();
          *
          * addPlane (plane_coeff);
          * \endcode
          */
        bool addPlane (const pcl::ModelCoefficients &coefficients, const std::string &id = "plane", int viewport = 0);
        bool addPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id = "plane", int viewport = 0);

        /** \brief Add a circle from a set of given model coefficients
          * \param[in] coefficients the model coefficients (x, y, radius)
          * \param[in] id the circle id/name (default: "circle")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCircle2D for more information
          * // float x, y, radius;
          *
          * pcl::ModelCoefficients circle_coeff;
          * circle_coeff.values.resize (3);    // We need 3 values
          * circle_coeff.values[0] = x;
          * circle_coeff.values[1] = y;
          * circle_coeff.values[2] = radius;
          *
          * vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff, z);
          * \endcode
           */
        bool addCircle (const pcl::ModelCoefficients &coefficients, const std::string &id = "circle", int viewport = 0);

        /** \brief Add a cone from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radiu)
          * \param[in] id the cone id/name (default: "cone")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addCone (const pcl::ModelCoefficients &coefficients, const std::string &id = "cone", int viewport = 0);

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] coefficients the model coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
          * \param[in] id the cube id/name (default: "cube")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool addCube (const pcl::ModelCoefficients &coefficients, const std::string &id = "cube", int viewport = 0);

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] translation a translation to apply to the cube from 0,0,0
          * \param[in] rotation a quaternion-based rotation to apply to the cube
          * \param[in] width the cube's width
          * \param[in] height the cube's height
          * \param[in] depth the cube's depth
          * \param[in] id the cube id/name (default: "cube")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
                 double width, double height, double depth,
                 const std::string &id = "cube",
                 int viewport = 0);

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
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                 double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);

        /** \brief Changes the visual representation for all actors to surface representation. */
        void setRepresentationToSurfaceForAllActors ();

        /** \brief Changes the visual representation for all actors to points representation. */
        void setRepresentationToPointsForAllActors ();

        /** \brief Changes the visual representation for all actors to wireframe representation. */
        void setRepresentationToWireframeForAllActors ();

        /** \brief Sets whether the 2D overlay text showing the framerate of the window is displayed or not.
          * \param[in] show_fps determines whether the fps text will be shown or not.
          */
        void setShowFPS (bool show_fps);

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
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void setCameraPosition (double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z, double up_x, double up_y, double up_z, int viewport = 0);

        /** \brief Set the camera location and viewup according to the given arguments
          * \param[in] pos_x the x coordinate of the camera location
          * \param[in] pos_y the y coordinate of the camera location
          * \param[in] pos_z the z coordinate of the camera location
          * \param[in] up_x the x component of the view up direction of the camera
          * \param[in] up_y the y component of the view up direction of the camera
          * \param[in] up_z the z component of the view up direction of the camera
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport = 0);

        /** \brief Set the camera parameters via an intrinsics and and extrinsics matrix
          * \note This assumes that the pixels are square and that the center of the image is at the center of the sensor.
          * \param[in] intrinsics the intrinsics that will be used to compute the VTK camera parameters
          * \param[in] extrinsics the extrinsics that will be used to compute the VTK camera parameters
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void setCameraParameters (const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics, int viewport = 0);

        /** \brief Set the camera parameters by given a full camera data structure.
          * \param[in] camera camera structure containing all the camera parameters.
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void setCameraParameters (const Camera &camera, int viewport = 0);

        /** \brief Set the camera clipping distances.
          * \param[in] near the near clipping distance (no objects closer than this to the camera will be drawn)
          * \param[in] far the far clipping distance (no objects further away than this to the camera will be drawn)
          */
        void setCameraClipDistances (double near, double far, int viewport = 0);

        /** \brief Set the camera vertical field of view.
          * \param[in] fovy vertical field of view in radians
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void setCameraFieldOfView (double fovy, int viewport = 0);

        /** \brief Get the current camera parameters. */
        void getCameras (std::vector<Camera>& cameras);

        /** \brief Get the current viewing pose. */
        Eigen::Affine3f getViewerPose (int viewport = 0);        
        void saveScreenshot (const std::string &file);

        /** \brief Return a pointer to the underlying VTK Render Window used. */
        vtkSmartPointer<vtkRenderWindow> getRenderWindow () { return (win_); }

        /** \brief Return a pointer to the underlying VTK Renderer Collection. */
        vtkSmartPointer<vtkRendererCollection> getRendererCollection () { return (rens_); }

        /** \brief Return a pointer to the CloudActorMap this visualizer uses. */
        CloudActorMapPtr getCloudActorMap () { return (cloud_actor_map_); }

        void setPosition (int x, int y);
        void setSize (int xw, int yw);

        /** \brief Use Vertex Buffer Objects renderers.
          * \param[in] use_vbos set to true to use VBOs
          */
        void setUseVbos (bool use_vbos)
        {
          use_vbos_ = use_vbos;
          style_->setUseVbos (use_vbos_);
        }

        /** \brief Create the internal Interactor object. */
        void createInteractor ();

        /** \brief Get a pointer to the current interactor style used. */
        inline vtkSmartPointer<PCLVisualizerInteractorStyle> getInteractorStyle (){ return (style_); }
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
            //PCL_WARN ("[pcl::visualization::PCLVisualizer::ExitMainLoopTimerCallback] Timer %d called.\n", timer_id);
            if (timer_id != right_timer_id)
              return;
            // Stop vtk loop and send notification to app to wake it up
            pcl_visualizer->interactor_->TerminateApp ();
          }
          int right_timer_id;
          PCLVisualizer* pcl_visualizer;
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
          PCLVisualizer* pcl_visualizer;
        };

        //////////////////////////////////////////////////////////////////////////////////////////////
        struct FPSCallback : public vtkCommand
        {
          static FPSCallback *New () { return (new FPSCallback); }

          FPSCallback () : actor (), pcl_visualizer (), decimated () {}
          FPSCallback (const FPSCallback& src) : vtkCommand (), actor (src.actor), pcl_visualizer (src.pcl_visualizer), decimated (src.decimated) {}
          FPSCallback& operator = (const FPSCallback& src) { actor = src.actor; pcl_visualizer = src.pcl_visualizer; decimated = src.decimated; return (*this); }

          virtual void
          Execute (vtkObject* caller, unsigned long event_id, void* call_data);

          vtkTextActor *actor;
          PCLVisualizer* pcl_visualizer;
          bool decimated;
        };

        /** \brief The FPSCallback object for the current visualizer. */
        vtkSmartPointer<FPSCallback> update_fps_;

        /** \brief Set to false if the interaction loop is running. */
        bool stopped_;

        /** \brief Global timer ID. Used in destructor only. */
        int timer_id_;

        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;

        /** \brief The collection of renderers used. */
        vtkSmartPointer<vtkRendererCollection> rens_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The render window interactor style. */
        vtkSmartPointer<PCLVisualizerInteractorStyle> style_;

        /** \brief Internal list with actor pointers and name IDs for point clouds. */
        CloudActorMapPtr cloud_actor_map_;

        /** \brief Internal list with actor pointers and name IDs for shapes. */
        ShapeActorMapPtr shape_actor_map_;

        /** \brief Internal list with actor pointers and viewpoint for coordinates. */
        CoordinateActorMap coordinate_actor_map_;

        /** \brief Internal pointer to widget which contains a set of axes */
        vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;

        /** \brief Boolean that holds whether or not the camera parameters were manually initialized*/
        bool camera_set_;

        /** \brief Boolean that holds whether or not to use the vtkVertexBufferObjectMapper*/
        bool use_vbos_;

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be removed from (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be removed from (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport port where the actor should be added to (default: 0/all)
          *
          * \note If viewport is set to 0, the actor will be added to all existing
          * renders. To select a specific viewport use an integer between 1 and N.
          */
        void
        addActorToRenderer (const vtkSmartPointer<vtkProp> &actor,
                            int viewport = 0);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be added to (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void
        createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                   vtkSmartPointer<vtkActor> &actor,
                                   bool use_scalars = true);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void
        createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                   vtkSmartPointer<vtkLODActor> &actor,
                                   bool use_scalars = true);


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

        /** \brief Transform the point cloud viewpoint to a transformation matrix
          * \param[in] origin the camera origin
          * \param[in] orientation the camera orientation
          * \param[out] transformation the camera transformation matrix
          */
        void getTransformationMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternion<float> &orientation, Eigen::Matrix4f &transformation);

        //There's no reason these conversion functions shouldn't be public and static so others can use them.
      public:
        /** \brief Convert Eigen::Matrix4f to vtkMatrix4x4
          * \param[in] m the input Eigen matrix
          * \param[out] vtk_matrix the resultant VTK matrix
          */
        static void convertToVtkMatrix (const Eigen::Matrix4f &m, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

        /** \brief Convert origin and orientation to vtkMatrix4x4
          * \param[in] origin the point cloud origin
          * \param[in] orientation the point cloud orientation
          * \param[out] vtk_matrix the resultant VTK 4x4 matrix
          */
        static void convertToVtkMatrix (const Eigen::Vector4f &origin, const Eigen::Quaternion<float> &orientation, vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

        /** \brief Convert vtkMatrix4x4 to an Eigen4f
          * \param[in] vtk_matrix the original VTK 4x4 matrix
          * \param[out] m the resultant Eigen 4x4 matrix
          */
        static void convertToEigenMatrix (const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix, Eigen::Matrix4f &m);

    };
  }
}

#include <opencv2/viz/viz3d_impl.hpp>


