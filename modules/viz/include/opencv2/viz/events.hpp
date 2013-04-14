#pragma once

#include <string>
#include <opencv2/viz/types.hpp>

namespace cv
{
    /** /brief Class representing key hit/release events */
    class KeyboardEvent
    {
    public:
        /** \brief bit patter for the ALT key*/
        static const unsigned int Alt   = 1;
        /** \brief bit patter for the Control key*/
        static const unsigned int Ctrl  = 2;
        /** \brief bit patter for the Shift key*/
        static const unsigned int Shift = 4;

        /** \brief Constructor
              * \param[in] action    true for key was pressed, false for released
              * \param[in] key_sym   the key-name that caused the action
              * \param[in] key       the key code that caused the action
              * \param[in] alt       whether the alt key was pressed at the time where this event was triggered
              * \param[in] ctrl      whether the ctrl was pressed at the time where this event was triggered
              * \param[in] shift     whether the shift was pressed at the time where this event was triggered
              */
        inline KeyboardEvent (bool action, const std::string& key_sym, unsigned char key,
                              bool alt, bool ctrl, bool shift);

        inline bool isAltPressed () const;
        inline bool isCtrlPressed () const;
        inline bool isShiftPressed () const;

        inline unsigned char getKeyCode () const;

        inline const std::string& getKeySym () const;
        inline bool keyDown () const;
        inline bool keyUp () const;

    protected:

        bool action_;
        unsigned int modifiers_;
        unsigned char key_code_;
        std::string key_sym_;
    };

    class MouseEvent
    {
    public:
        enum Type
        {
            MouseMove = 1,
            MouseButtonPress,
            MouseButtonRelease,
            MouseScrollDown,
            MouseScrollUp,
            MouseDblClick
        } ;

        enum MouseButton
        {
            NoButton      = 0,
            LeftButton,
            MiddleButton,
            RightButton,
            VScroll /*other buttons, scroll wheels etc. may follow*/
        } ;

        /** Constructor.
                  * \param[in] type   event type
                  * \param[in] button The Button that causes the event
                  * \param[in] x      x position of mouse pointer at that time where event got fired
                  * \param[in] y      y position of mouse pointer at that time where event got fired
                  * \param[in] alt    whether the ALT key was pressed at that time where event got fired
                  * \param[in] ctrl   whether the CTRL key was pressed at that time where event got fired
                  * \param[in] shift  whether the Shift key was pressed at that time where event got fired
                  */
        inline MouseEvent (const Type& type, const MouseButton& button, unsigned int x, unsigned int y, bool alt, bool ctrl, bool shift);


        inline const Type& getType () const;
        inline void  setType (const Type& type);
        inline const MouseButton& getButton () const;

        inline void setButton (const MouseButton& button);

        //the x position of the mouse pointer at that time where the event got fired
        inline unsigned int getX () const;
        inline unsigned int  getY () const;

        inline unsigned int getKeyboardModifiers () const;

    protected:
        Type type_;
        MouseButton button_;
        unsigned int pointer_x_;
        unsigned int pointer_y_;
        unsigned int key_state_;
    };


    /** /brief Class representing 3D point picking events. */
    class CV_EXPORTS PointPickingEvent
    {
      public:
        PointPickingEvent (int idx);
        PointPickingEvent (int idx, const Vec3d& p);
        PointPickingEvent (int idx1, int idx2, const Vec3d& p1, const Vec3d& p2);

        /** \brief Obtain the ID of a point that the user just clicked on. */
        int getPointIndex () const;

        /** \brief Obtain the XYZ point coordinates of a point that the user just clicked on. */
        Vec3d getPoint () const;

        /** \brief For situations when multiple points are selected in a sequence, return the point coordinates.
          * \return true, if two points are available and have been clicked by the user, false otherwise
          */
        bool getPoints (Vec3d& p1, Vec3d& p2) const;

        /** \brief For situations where multiple points are selected in a sequence, return the points indices.
          * \return true, if two points are available and have been clicked by the user, false otherwise
          */
        bool getPointIndices (int &index_1, int &index_2) const;
      private:
        int idx_, idx2_;
        Vec3d p1, p2;
    };

}


cv::KeyboardEvent::KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, bool alt, bool ctrl, bool shift)
  : action_ (action), modifiers_ (0), key_code_(key), key_sym_ (key_sym)
{
  if (alt)
    modifiers_ = Alt;

  if (ctrl)
    modifiers_ |= Ctrl;

  if (shift)
    modifiers_ |= Shift;
}

bool cv::KeyboardEvent::isAltPressed () const { return (modifiers_ & Alt) != 0; }
bool cv::KeyboardEvent::isCtrlPressed () const { return (modifiers_ & Ctrl) != 0; }
bool cv::KeyboardEvent::isShiftPressed () const { return (modifiers_ & Shift) != 0; }
unsigned char cv::KeyboardEvent::getKeyCode () const { return key_code_; }
const std::string& cv::KeyboardEvent::getKeySym () const { return (key_sym_); }
bool cv::KeyboardEvent::keyDown () const { return action_; }
bool cv::KeyboardEvent::keyUp () const { return !action_; }

cv::MouseEvent::MouseEvent (const Type& type, const MouseButton& button, unsigned x, unsigned y,  bool alt, bool ctrl, bool shift)
    : type_ (type), button_ (button), pointer_x_ (x), pointer_y_ (y), key_state_ (0)
{
    if (alt)
        key_state_ = KeyboardEvent::Alt;

    if (ctrl)
        key_state_ |= KeyboardEvent::Ctrl;

    if (shift)
        key_state_ |= KeyboardEvent::Shift;
}

const cv::MouseEvent::Type& cv::MouseEvent::getType () const { return type_; }
void cv::MouseEvent::setType (const Type& type) { type_ = type; }
const cv::MouseEvent::MouseButton& cv::MouseEvent::getButton () const { return (button_); }
void cv::MouseEvent::setButton (const MouseButton& button) { button_ = button;}
unsigned int cv::MouseEvent::getX () const { return pointer_x_; }
unsigned int cv::MouseEvent::getY () const { return (pointer_y_); }
unsigned int cv::MouseEvent::getKeyboardModifiers () const { return key_state_; }


inline cv::PointPickingEvent::PointPickingEvent(int idx) : idx_(idx), idx2_(-1) {}
inline cv::PointPickingEvent::PointPickingEvent(int idx, const Vec3d& p) : idx_(idx), idx2_(-1), p1(p) {}
inline cv::PointPickingEvent::PointPickingEvent(int idx1, int idx2, const Vec3d& _p1, const Vec3d& _p2) :  idx_(idx1), idx2_(idx2), p1(_p1), p2(_p2) {}
inline int cv::PointPickingEvent::getPointIndex() const { return (idx_); }
inline cv::Vec3d cv::PointPickingEvent::getPoint() const { return p1; }

inline bool cv::PointPickingEvent::getPoints(Vec3d& _p1, Vec3d& _p2) const
{
    if (idx2_ == -1)
        return false;
    _p1 = p1;
    _p2 = p2;
    return true;
}

inline bool cv::PointPickingEvent::getPointIndices (int &index_1, int &index_2) const
{
  if (idx2_ == -1)
    return false;
  index_1 = idx_;
  index_2 = idx2_;
  return true;
}


