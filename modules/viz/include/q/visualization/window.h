#pragma once

#include <opencv2/core/cvdef.h>
#include <q/visualization/vtk.h>
#include <q/visualization/interactor_style.h>


namespace cv
{
class MouseEvent;
class KeyboardEvent;

class CV_EXPORTS Window
{
public:
    Window (const std::string& window_name = "Viz");
    Window (const Window &src);
    Window& operator = (const Window &src);

    virtual ~Window ();

    void spin ();
    void spinOnce (int time = 1, bool force_redraw = false);
    bool wasStopped () const { return (stopped_); }

    /**
          * @brief registering a callback function for keyboard events
          * @param callback  the function that will be registered as a callback for a keyboard event
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
    boost::signals2::connection registerKeyboardCallback (void (*callback) (const cv::KeyboardEvent&, void*), void* cookie = NULL)
    {
        return registerKeyboardCallback (boost::bind (callback, _1, cookie));
    }

    /**
          * @brief registering a callback function for keyboard events
          * @param callback  the member function that will be registered as a callback for a keyboard event
          * @param instance  instance to the class that implements the callback function
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
    template<typename T> boost::signals2::connection registerKeyboardCallback (void (T::*callback) (const cv::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
    {
        return registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie));
    }

    /**
          * @brief
          * @param callback  the function that will be registered as a callback for a mouse event
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
    boost::signals2::connection registerMouseCallback (void (*callback) (const cv::MouseEvent&, void*), void* cookie = NULL)
    {
        return registerMouseCallback (boost::bind (callback, _1, cookie));
    }

    /**
          * @brief registering a callback function for mouse events
          * @param callback  the member function that will be registered as a callback for a mouse event
          * @param instance  instance to the class that implements the callback function
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
    template<typename T> boost::signals2::connection
    registerMouseCallback (void (T::*callback) (const cv::MouseEvent&, void*), T& instance, void* cookie = NULL)
    {
        return registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie));
    }

protected: // methods

    void resetStoppedFlag () { stopped_ = false; }

    /**
          * @brief   registering a callback function for mouse events
          * @param   the boost function that will be registered as a callback for a mouse event
          * @return  connection object that allows to disconnect the callback function.
          */
    boost::signals2::connection registerMouseCallback (boost::function<void (const cv::MouseEvent&)> );

    /**
         * @brief   registering a callback boost::function for keyboard events
         * @param   the boost function that will be registered as a callback for a keyboard event
         * @return  connection object that allows to disconnect the callback function.
         */
    boost::signals2::connection registerKeyboardCallback (boost::function<void (const cv::KeyboardEvent&)> );

    void emitMouseEvent (unsigned long event_id);

    void emitKeyboardEvent (unsigned long event_id);

    // Callbacks used to register for vtk command
    static void MouseCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);
    static void KeyboardCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);

protected: // types
    struct ExitMainLoopTimerCallback : public vtkCommand
    {
        static ExitMainLoopTimerCallback* New()
        {
            return (new ExitMainLoopTimerCallback);
        }

        ExitMainLoopTimerCallback () : right_timer_id (), window () {}
        ExitMainLoopTimerCallback (const ExitMainLoopTimerCallback& src) : vtkCommand (), right_timer_id (src.right_timer_id), window (src.window) {}
        ExitMainLoopTimerCallback& operator = (const ExitMainLoopTimerCallback& src) { right_timer_id = src.right_timer_id; window = src.window; return (*this); }

        virtual void
        Execute (vtkObject*, unsigned long event_id, void* call_data)
        {
            if (event_id != vtkCommand::TimerEvent)
                return;
            int timer_id = *static_cast<int*> (call_data);
            //PCL_WARN ("[cv::Window::ExitMainLoopTimerCallback] Timer %d called.\n", timer_id);
            if (timer_id != right_timer_id)
                return;
            window->interactor_->TerminateApp ();
            //            window->interactor_->stopLoop ();
        }
        int right_timer_id;
        Window* window;
    };

    struct ExitCallback : public vtkCommand
    {
        static ExitCallback* New ()
        {
            return (new ExitCallback);
        }

        ExitCallback () : window () {}
        ExitCallback (const ExitCallback &src) : vtkCommand (), window (src.window) {}
        ExitCallback& operator = (const ExitCallback &src) { window = src.window; return (*this); }

        virtual void
        Execute (vtkObject*, unsigned long event_id, void*)
        {
            if (event_id != vtkCommand::ExitEvent)
                return;
            window->interactor_->TerminateApp ();
            window->stopped_ = true;
        }
        Window* window;
    };

    bool stopped_;
    int timer_id_;

protected: // member fields
    boost::signals2::signal<void (const cv::MouseEvent&)> mouse_signal_;
    boost::signals2::signal<void (const cv::KeyboardEvent&)> keyboard_signal_;

    vtkSmartPointer<vtkRenderWindow> win_;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
    vtkCallbackCommand* mouse_command_;
    vtkCallbackCommand* keyboard_command_;
    /** \brief The render window interactor style. */
    vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle> style_;
    /** \brief The collection of renderers used. */
    vtkSmartPointer<vtkRendererCollection> rens_;
    vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
    vtkSmartPointer<ExitCallback> exit_callback_;
};

}
