#pragma once

#include <q/visualization/3rdparty.h>
#include <q/visualization/viz_types.h>
#include <opencv2/viz/events.hpp>
#include <q/visualization/point_picking_event.h>

namespace temp_viz
{
        /** \brief PCLVisualizerInteractorStyle defines an unique, custom VTK
          * based interactory style for PCL Visualizer applications. Besides
          * defining the rendering style, we also create a list of custom actions
          * that are triggered on different keys being pressed:
          *
          * -        p, P   : switch to a point-based representation
          * -        w, W   : switch to a wireframe-based representation (where available)
          * -        s, S   : switch to a surface-based representation (where available)
          * -        j, J   : take a .PNG snapshot of the current window view
          * -        c, C   : display current camera/window parameters
          * -        f, F   : fly to point mode
          * -        e, E   : exit the interactor\
          * -        q, Q   : stop and call VTK's TerminateApp
          * -       + / -   : increment/decrement overall point size
          * -  r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]
          * -  ALT + s, S   : turn stereo mode on/off
          * -  ALT + f, F   : switch between maximized window mode and original size
          * -
          * -  SHIFT + left click   : select a point
          *
          * \author Radu B. Rusu
          * \ingroup visualization
          */
        class CV_EXPORTS PCLVisualizerInteractorStyle : public vtkInteractorStyleTrackballCamera
        {
            typedef cv::Ptr<CloudActorMap> CloudActorMapPtr;

        public:

            enum InteractorKeyboardModifier
            {
                INTERACTOR_KB_MOD_ALT,
                INTERACTOR_KB_MOD_CTRL,
                INTERACTOR_KB_MOD_SHIFT
            };

            static PCLVisualizerInteractorStyle *New ();

            /** \brief Empty constructor. */
            PCLVisualizerInteractorStyle () {}

            /** \brief Empty destructor */
            virtual ~PCLVisualizerInteractorStyle () {}

            // this macro defines Superclass, the isA functionality and the safe downcast method
            vtkTypeMacro (PCLVisualizerInteractorStyle, vtkInteractorStyleTrackballCamera);

            /** \brief Initialization routine. Must be called before anything else. */
            virtual void Initialize ();

            /** \brief Pass a pointer to the actor map
                  * \param[in] actors the actor map that will be used with this style
                  */
            inline void setCloudActorMap (const CloudActorMapPtr &actors) { actors_ = actors; }

            /** \brief Get the cloud actor map pointer. */
            inline CloudActorMapPtr getCloudActorMap () { return (actors_); }

            /** \brief Pass a set of renderers to the interactor style.
                  * \param[in] rens the vtkRendererCollection to use
                  */
            void setRendererCollection (vtkSmartPointer<vtkRendererCollection> &rens) { rens_ = rens; }

            /** \brief Register a callback function for mouse events
                  * \param[in] cb a boost function that will be registered as a callback for a mouse event
                  * \return a connection object that allows to disconnect the callback function.
                  */
            boost::signals2::connection registerMouseCallback (boost::function<void (const cv::MouseEvent&)> cb);

            /** \brief Register a callback boost::function for keyboard events
                  * \param[in] cb a boost function that will be registered as a callback for a keyboard event
                  * \return a connection object that allows to disconnect the callback function.
                  */
            boost::signals2::connection registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> cb);

            /** \brief Register a callback function for point picking events
                  * \param[in] cb a boost function that will be registered as a callback for a point picking event
                  * \return a connection object that allows to disconnect the callback function.
                  */
            boost::signals2::connection registerPointPickingCallback (boost::function<void (const cv::PointPickingEvent&)> cb);

            /** \brief Save the current rendered image to disk, as a PNG screenshot.
                  * \param[in] file the name of the PNG file
                  */
            void saveScreenshot (const std::string &file);

            /** \brief Change the default keyboard modified from ALT to a different special key.
                  * Allowed values are:
                  * - INTERACTOR_KB_MOD_ALT
                  * - INTERACTOR_KB_MOD_CTRL
                  * - INTERACTOR_KB_MOD_SHIFT
                  * \param[in] modifier the new keyboard modifier
                  */
            inline void setKeyboardModifier (const InteractorKeyboardModifier &modifier) { modifier_ = modifier; }
        protected:
            /** \brief Set to true after initialization is complete. */
            bool init_;

            /** \brief Collection of vtkRenderers stored internally. */
            vtkSmartPointer<vtkRendererCollection> rens_;

            /** \brief Actor map stored internally. */
            CloudActorMapPtr actors_;

            /** \brief The current window width/height. */
            int win_height_, win_width_;

            /** \brief The current window position x/y. */
            int win_pos_x_, win_pos_y_;

            /** \brief The maximum resizeable window width/height. */
            int max_win_height_, max_win_width_;

            /** \brief A PNG writer for screenshot captures. */
            vtkSmartPointer<vtkPNGWriter> snapshot_writer_;
            /** \brief Internal window to image filter. Needed by \a snapshot_writer_. */
            vtkSmartPointer<vtkWindowToImageFilter> wif_;

            boost::signals2::signal<void (const cv::MouseEvent&)> mouse_signal_;
            boost::signals2::signal<void (const cv::KeyboardEvent&)> keyboard_signal_;
            boost::signals2::signal<void (const cv::PointPickingEvent&)> point_picking_signal_;

            /** \brief Interactor style internal method. Gets called whenever a key is pressed. */
            virtual void OnChar ();

            // Keyboard events
            virtual void OnKeyDown ();
            virtual void OnKeyUp ();

            // mouse button events
            virtual void OnMouseMove ();
            virtual void OnLeftButtonDown ();
            virtual void OnLeftButtonUp ();
            virtual void OnMiddleButtonDown ();
            virtual void OnMiddleButtonUp ();
            virtual void OnRightButtonDown ();
            virtual void OnRightButtonUp ();
            virtual void OnMouseWheelForward ();
            virtual void OnMouseWheelBackward ();

            /** \brief Interactor style internal method. Gets called periodically if a timer is set. */
            virtual void OnTimer ();


            void zoomIn ();
            void zoomOut ();

            /** \brief True if we're using red-blue colors for anaglyphic stereo, false if magenta-green. */
            bool stereo_anaglyph_mask_default_;

            /** \brief A VTK Mouse Callback object, used for point picking. */
            vtkSmartPointer<PointPickingCallback> mouse_callback_;

            /** \brief The keyboard modifier to use. Default: Alt. */
            InteractorKeyboardModifier modifier_;

            friend class PointPickingCallback;
        };
}
