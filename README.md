rosleapmotion
=============

Leap Motion ROS integration


FEATURES
========

Release 0.0.1 Includes:

Currently this ros package features the extraction of one hand, the first to be recognized by the LEAP DEVICE.

Hand-...

Vector3
-direction.x
-direction.y
-direction.z

Vector3
-normal.x
-normal.y
-normal.z

Point
-pos.x
-pos.y
-pos.z

Vector3
-hand.pitch
-hand.yaw
-hand.roll


INSTALLATION
=======

1. If you don't already have a catkin workspace please follow these instructions: http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

2. cd ~/catkin_ws/src

3. git clone https://github.com/warp1337/rosleapmotion.git

4. cd ~/catkin_ws && catkin_make

5. Start a roscore (another shell) and leapd

6. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion sender.py (another shell)

7. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion subscriber.py (another shell) 

8. You are done, you should see the LEAP MOTION coordinates in your shell prompt.

