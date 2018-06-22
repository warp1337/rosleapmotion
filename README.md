# ROS LEAP MOTION

ROS driver for the Leap Motion Controller

[![Build Status](https://travis-ci.org/nowittyusername/leap_motion.svg?branch=hydro)](https://travis-ci.org/nowittyusername/leap_motion)

## REQUIREMENTS

You should have [ROS Kinetic](http://wiki.ros.org/kinetic) or a [newer version](http://wiki.ros.org/Distributions) installed on your device and the [Leap Motion SDK](https://developer.leapmotion.com/sdk/v2) for Linux.

## FEATURES

Currently, this ROS package supports one person (left and right arm), publishing raw camera images from the controller, basic visualization using RViz and a pointcloud2 generated from [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) node.

There is also a filter node implementing a 2nd-order Butterworth lowpass filter that is used to filter the hand x, y, z coordinates coming from the Leap Controller via Human.msg. For more information refer to Julius O. Smith III, Intro to Digital Filters with Audio Applications.

## INSTALLATION

**1.** You need to append the location of your LeapSDK to your environment variables. This step differs depending on where you saved the SDK. The LeapSDK folder should contain the following files: include/Leap.h, include/LeapMath.h, lib/x64/libLeap.so and lib/x86/libLeap.so.

Example:

```bash
export LEAP_SDK=~/lib/LeapSDK
```

**2.** (OPTIONAL) You can edit your ~/.bashrc file to remove the need to export the location of your LeapSDK every time you open a new shell. Just append the LeapSDK location to the end of the file.

**3.** Go to the src folder of your catkin workspace.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/ros-drivers/leap_motion.git
    cd ~/catkin_ws
    catkin_make
```

**4.** Start the Leap control panel in another terminal.

```bash
LeapControlPanel
```

**5.** (OPTIONAL) If it gives you an error about the leap daemon not running, stop the LeapControlPanel have a look [here](https://forums.leapmotion.com/t/error-in-leapd-malloc/4271/13) and use the following command:

```bash
sudo service leapd restart
```

**6.** Source your current catkin workspace.

```bash
source ~/catkin_ws/devel/setup.bash
```

**7.** Launch the demo.launch file to see if you have set everything up correctly. If you wish to enable a lowpass filter change "enable_filter" to true in filter_params.yaml file.

```bash
roslaunch leap_motion everything.launch
```

**8.** You are done! You should see an RViz window opening up displaying the detected hands from the controller.