#pragma once

#include "keyboard_event.h"

namespace cv
{
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

}

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
