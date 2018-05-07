#include <iostream>

#include "ros/ros.h"
#include "leap_motion/Human.h"

#include "../inc/lmc_listener.h"
#include "Leap.h"

using namespace Leap;

int main(int argc, char** argv) {

    ros::init(argc, argv, "leap_motion");
    ros::NodeHandle nh("leap_motion");

    bool setup_params[7];

    // Read parameters from the defined in listener_params.yaml
    nh.getParam("/enable_controller_info", setup_params[0] );
    nh.getParam("/enable_frame_info", setup_params[1] );
    nh.getParam("/enable_hand_info", setup_params[2] );

    nh.getParam("/enable_gesture_circle", setup_params[3] );
    nh.getParam("/enable_gesture_swipe", setup_params[4] );
    nh.getParam("/enable_gesture_screen_tap", setup_params[5] );
    nh.getParam("/enable_gesture_key_tap", setup_params[6] );

    LeapListener listener(setup_params);
    // Add a publisher to the leapListener object
    listener.ros_publisher = nh.advertise<leap_motion::Human>("leap_device", 1);

    Controller controller;
    controller.addListener(listener);
    // Keep doing ROS spin until shutdown() or Ctrl+C
    ros::spin();
    controller.removeListener(listener);

    return 0;
}