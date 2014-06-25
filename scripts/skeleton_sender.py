#!/usr/bin/python

__author__ = 'Igor Zubrycki'

import rospy
import leap_interface
import tf
import sys
import Leap
import math
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import std_msgs.msg
import sensor_msgs.msg
import actionlib_msgs.msg
import trajectory_msgs.msg
import collections
import scipy.signal as signal
import geometry_msgs.msg
import PyKDL

finger_names=["thumb","index","middle","ring","pinky"]
bones_names=["metacarpal","proximal","intermediate","distal"]

def make_kdl_frame(leap_basis_matrix,leap_position_vector,is_left=False):
    # Makes kdl frame from Leap Motion matrix and vector formats
    
    if is_left:
       basis=([-el for el in leap_basis_matrix.x_basis.to_float_array()]+
       leap_basis_matrix.y_basis.to_float_array()+
       leap_basis_matrix.z_basis.to_float_array())
       
    else:
       basis=leap_basis_matrix.to_array_3x3() 
    rotation_mat=PyKDL.Rotation(*basis).Inverse()
    # rotation_mat=PyKDL.Rotation(*[1,0,0,0,1,0,0,0,1])#.Inverse()
    placement=PyKDL.Vector(*leap_position_vector.to_float_array())/1000.0 #to m
    return PyKDL.Frame(rotation_mat,placement)
    
def relative_frame(frame_A,frame_B):

    return (frame_A.Inverse())*frame_B


hand_ground_tf=PyKDL.Frame(PyKDL.Rotation.EulerZYX(0, 0, math.pi/2.0),PyKDL.Vector(0,0,0))

def make_tf_dict(hand,hand_name):
    
    hand_dict={}
    # hand_ground_tf=
    hand_dict["hand_ground"]=["ground",hand_ground_tf]
    
            
    hand_tf=make_kdl_frame(hand.basis,hand.palm_position,hand.is_left)
    # hand_tf=make_kdl_frame(hand.basis,hand.palm_position)
    hand_dict[hand_name]=["hand_ground",hand_tf]
    for finger in hand.fingers:
        finger_name=hand_name+"_"+finger_names[finger.type()]
        
        prev_bone_name=hand_name
        for num in range(0,4):
            
            bone=finger.bone(num)

            bone_absolute=make_kdl_frame(bone.basis,bone.prev_joint,hand.is_left)
            if num==0:
                bone_tf=relative_frame(hand_tf,bone_absolute)
            else:
                bone_tf=relative_frame(prev_bone_absolute,bone_absolute)

            bone_name=finger_name+"_"+bones_names[num]
            hand_dict[bone_name]=[prev_bone_name,bone_tf]
            
            prev_bone_name=bone_name
            prev_bone_absolute=bone_absolute
            
        tip=PyKDL.Frame(PyKDL.Rotation(1,0,0, 0,1,0, 0,0,1),PyKDL.Vector(0,0,-bone.length/1000.0))
        hand_dict[finger_name+"_tip"]=[prev_bone_name,tip]    
    return hand_dict    
     # now sending to ROS       
        

def broadcast_hand(hand,hand_name,timenow):
    hand_dict=make_tf_dict(hand,hand_name)
    br = tf.TransformBroadcaster()


    for tf_name,tf_array in hand_dict.items():
        tf_matrix=tf_array[1]
        tf_prev_name=tf_array[0]
        br.sendTransform(tf_matrix.p,tf_matrix.M.GetQuaternion(),timenow,tf_name,tf_prev_name)

def sender():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    # pub_ros   = rospy.Publisher('leapmotion/data',leapros)
    rospy.init_node('leap_skeleton_pub')

    
    while not rospy.is_shutdown():
        timenow=rospy.Time.now()
        if li.listener.left_hand:
            broadcast_hand(li.listener.left_hand,"left_hand",timenow)
        if li.listener.right_hand:
            broadcast_hand(li.listener.right_hand,"right_hand",timenow)

        # save some CPU time, circa 100Hz publishing.
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
