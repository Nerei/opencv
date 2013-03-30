#pragma once

#include <string>

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

    KeyboardEvent::KeyboardEvent (bool action, const std::string& key_sym, unsigned char key, bool alt, bool ctrl, bool shift)
      : action_ (action), modifiers_ (0), key_code_(key), key_sym_ (key_sym)
    {
      if (alt)
        modifiers_ = Alt;

      if (ctrl)
        modifiers_ |= Ctrl;

      if (shift)
        modifiers_ |= Shift;
    }
}

bool cv::KeyboardEvent::isAltPressed () const { return (modifiers_ & Alt) != 0; }
bool cv::KeyboardEvent::isCtrlPressed () const { return (modifiers_ & Ctrl) != 0; }
bool cv::KeyboardEvent::isShiftPressed () const { return (modifiers_ & Shift) != 0; }
unsigned char cv::KeyboardEvent::getKeyCode () const { return key_code_; }
const std::string& cv::KeyboardEvent::getKeySym () const { return (key_sym_); }
bool cv::KeyboardEvent::keyDown () const { return action_; }
bool cv::KeyboardEvent::keyUp () const { return !action_; }

