# ROS LEAP MOTION

ROS driver for the Leap Motion Controller

[![Build Status](https://travis-ci.org/ros-drivers/leap_motion.svg?branch=hydro)](https://travis-ci.org/ros-drivers/leap_motion)

## REQUIREMENTS

You should have [ROS Kinetic](http://wiki.ros.org/kinetic) or a [newer version](http://wiki.ros.org/Distributions) installed on your device and the [Leap Motion SDK](https://developer.leapmotion.com/sdk/v2) for Linux.

## FEATURES

Currently, this ROS package supports one person (left and right arm), publishing raw camera images from the controller, basic visualization using RViz and a pointcloud2 generated from [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) node.

There is also a filter node implementing a 2nd-order Butterworth lowpass filter that is used to filter the hand x, y, z coordinates coming from the Leap Controller via Human.msg. For more information refer to Julius O. Smith III, [Intro to Digital Filters with Audio Applications](https://ccrma.stanford.edu/~jos/filters/).

## INSTALLATION

### Python API installation

**1.** If you wish to use the old deprecated Python API you need to append the location of your LeapSDK to your environment variables. This step differs depending on where you saved the SDK. The LeapSDK folder should contain the following [files](https://developer-archive.leapmotion.com/documentation/v2/python/devguide/Project_Setup.html): lib/Leap.py, lib/x86/LeapPython.so, lib/x86/libLeap.so, lib/x64/LeapPython.so, lib/x64/libLeap.so lib/LeapPython.so, lib/libLeap.dylib.

Example:

```bash
# 64-bit operating system
export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64

# 32-bit operating system
export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x86
```

**2.** (OPTIONAL) You can edit your ~/.bashrc file to remove the need to export the location of your LeapSDK every time you open a new shell. Just append the LeapSDK location to the end of the PYTHONPATH.

```bash
# 64-bit operating system
echo "export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64" >> ~/.bashrc
source ~/.bashrc

# 32-bit operating system
echo "export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x86" >> ~/.bashrc
source ~/.bashrc
```

### Usage

**1.** Just go to the src folder of your catkin workspace.

```bash
    cd ~/catkin_ws/src
    git clone https://github.com/ros-drivers/leap_motion.git
    cd ~/catkin_ws
    catkin_make
```

**2.** Start the Leap control panel in another terminal.

```bash
LeapControlPanel
```

**3.** (OPTIONAL) If it gives you an error about the leap daemon not running, stop the LeapControlPanel have a look [here](https://forums.leapmotion.com/t/error-in-leapd-malloc/4271/13) and use the following command:

```bash
sudo service leapd restart
```

**4.** Source your current catkin workspace.

```bash
source ~/catkin_ws/devel/setup.bash
```

**5.** Launch the demo.launch file to see if you have set everything up correctly. If you wish to enable a lowpass filter change "enable_filter" to true in filter_params.yaml file.

```bash
roslaunch leap_motion demo.launch
```

**6.** You are done! You should see an RViz window opening up displaying the detected hands from the controller.