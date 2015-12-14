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

For each finger in [‘thumb’, ‘index’, ‘middle’, ‘ring’, ‘pinky’], the coordinates of each bone in [‘metacarpal’, ‘proximal’, ‘intermediate’, ‘distal’] are available as finger_bone (e.g., thumb_metacarpal).

(Point)
finger_bone.x
finger_bone.y
finger_bone.z

These coordinates are taken at the base of each bone (closest to the wrist). To access the end of the distal bone, use finger_tip (e.g., thumb_tip).

(Point)
finger_tip.x
finger_tip.y
finger_tip.z



INSTALLATION
==============

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


USE LEAP AS STEREO CAMERA
============================
if you want to use leap_motion as stereo_camera, You can use by compiling it.

1. Make sure that You are using LeapSDK ver 2.*

2. You need to append the location of your LeapSDK (especially /lib and /lib/x64 or x86) to your LEAP_SDK_PATH,e.g., add "export LEAP_SDK=/home/Your_name/LeapDebeloperKit_2.*/LeapSDK " in ~/.bashrc

3. command LeapControlPanel in your shell and enable the feature in the Leap Motion control panel for any application to get the raw camera images.

4. in your catkin_ws, catkin_make --only-pkg-with-depth leap_motion

5. roslaunch leap_motion leap_camera.launch

6. roslaunch leap_motion leap_stereo.launch

