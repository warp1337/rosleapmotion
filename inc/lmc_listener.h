//!  LeapListener 
/*!
The LeapListener overrides the Listener class that defines a set of callback functions that 
one can override in a subclass to respond to events dispatched by the Controller object. 
*/

#ifndef LMC_LISTENER_H
#define LMC_LISTENER_H

#include "ros/ros.h"
#include "Leap.h"

using namespace Leap;

class LeapListener : public Listener
{
    public:
        LeapListener();
        LeapListener(bool args[7]);
        virtual void onConnect(const Controller&);
        virtual void onInit(const Controller&);
        virtual void onDisconnect(const Controller&);
        virtual void onExit(const Controller&);
        virtual void onFrame(const Controller&);

        ros::Publisher ros_publisher;
        std::string header_frame_id_;

        bool enable_controller_info_;
        bool enable_frame_info_;
        bool enable_hand_info_;
        
        bool enable_gesture_circle_;
        bool enable_gesture_swipe_;
        bool enable_gesture_screen_tap_;
        bool enable_gesture_key_tap_;
};

#endif /* LMC_LISTENER_H */