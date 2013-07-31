ROS LEAP MOTION
=============

Leap Motion ROS integration


REQUIREMENTS
============

ROS Groovy installed including rospy and geometry_msg and the LEAP MOTION SDK for Linux


FEATURES
========

Release 0.0.1 Includes:

Currently, this ros package features the extraction of one hand, the first to be recognized by the LEAP DEVICE.

(Vector3)
direction.x
direction.y
direction.z

(Vector3)
normal.x
normal.y
normal.z

(Point)
pos.x
pos.y
pos.z

(Vector3)
hand.pitch
hand.yaw
hand.roll


INSTALLATION
=======

1. If you don't already have a catkin workspace, please follow these instructions before starting with 2.: http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

2. cd ~/catkin_ws/src

3. git clone https://github.com/warp1337/rosleapmotion.git

4. cd ~/catkin_ws && catkin_make

5. Start a roscore (another shell) and leapd (another shell)

6. You need to append the location of your LeapSDK (especially /lib and /lib/x64 or x86) to your PYTHONPATH,e.g., export PYTHONPATH=$PYTHONPATH:/path/to/SDK/lib:/path/to/SDK/lib/x64
Remember that you will need to have your path set at least in the "sender" shell. If you don't want to set it every time, you can also alter the leapinterface.py file (have a look at it).

6. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion sender.py (another shell)

7. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion subscriber.py (another shell) 

8. You are done, you should see the LEAP MOTION coordinates in your shell prompt
