#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "leap_motion/Finger.h"
#include "leap_motion/Bone.h"

#include "Leap.h"
#include <iostream>

static ros::Publisher* marker_pub;
const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

/*!
 * \brief Adds circles at the intersection of bones
 * 
 * Takes a pointer to human.msg, and a start of a bone. Using that information creates a
 * sphere visualization marker that will later be appended to a marker array.
 * 
 * \param human     Pointer to the received Human.msg
 * \param hand_ns   Used to create a unique namespace
 * \param finger_ns Used to create a unique namespace
 * \param location  Tip position of a bone object 
 */
visualization_msgs::Marker createJointMarker(const leap_motion::Human::ConstPtr& human, 
    std::string hand_ns, std::string finger_ns, unsigned int id, geometry_msgs::Pose location ){

    visualization_msgs::Marker joint_marker;

    joint_marker.header.frame_id = human->header.frame_id;
    joint_marker.header.stamp = human->header.stamp;
    joint_marker.ns = hand_ns+"_"+finger_ns;
    joint_marker.id = id;
    joint_marker.type =  visualization_msgs::Marker::SPHERE;
    joint_marker.action = visualization_msgs::Marker::ADD;

    // Location data in meters from origin
    joint_marker.pose.position.x = location.position.x;
    joint_marker.pose.position.y = location.position.y; 
    joint_marker.pose.position.z = location.position.z;

    joint_marker.scale.x = 0.015;
    joint_marker.scale.y = 0.015;
    joint_marker.scale.z = 0.015;
    
    joint_marker.color.r = 1.0f;
    joint_marker.color.g = 0.0f;
    joint_marker.color.b = 0.0f;
    joint_marker.color.a = 1.0f;
    joint_marker.lifetime = ros::Duration(0.1);

    return joint_marker;
}

/*!
 * \brief Adds lines between the tips of the bones.
 * 
 * Takes a pointer to human.msg, and a bone.msg. Using that information
 * adds lines between the tips of the bones.
 * 
 * \param human         Pointer to the received Human.msg
 * \param hand_ns       Used to create a unique namespace
 * \param finger_ns     Used to create a unique namespace
 * \param current_bone  The bone that will be represented as a line.
 */
visualization_msgs::Marker createFingerLines(const leap_motion::Human::ConstPtr &human, 
    std::string hand_ns, std::string finger_ns, unsigned int id, leap_motion::Bone current_bone){
    
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = human -> header.frame_id;
    line_list.header.stamp = human -> header.stamp;
    line_list.ns = hand_ns + "_" + finger_ns;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.id = id;

    line_list.scale.x = 0.0075;

    // Line list is white
    line_list.color.r = 1.0f;
    line_list.color.g = 1.0f;
    line_list.color.b = 1.0f;
    line_list.color.a = 1.0f;
    line_list.lifetime = ros::Duration(0.1);

    geometry_msgs::Point p;
    p.x = current_bone.bone_start.position.x;
    p.y = current_bone.bone_start.position.y;
    p.z = current_bone.bone_start.position.z;
    line_list.points.push_back(p);

    p.x = current_bone.bone_end.position.x;
    p.y = current_bone.bone_end.position.y;
    p.z = current_bone.bone_end.position.z;
    line_list.points.push_back(p);

    return line_list;
}

/*!
 * \brief Creates a representation of palm
 * 
 * Takes a pointer to human.msg, and a hand.msg. Using that information
 * adds draw lines between fingers to create a palm.
 * 
 * \param human         Pointer to the received Human.msg
 * \param hand_ns       Used to create a unique namespace
 * \param finger_ns     Used to create a unique namespace
 * \param current_hand  The hand from which the palm will be created
 */
visualization_msgs::Marker createHandOutline(const leap_motion::Human::ConstPtr &human, 
    std::string hand_ns, unsigned int id, leap_motion::Hand current_hand){

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = human -> header.frame_id;
    line_list.header.stamp = human -> header.stamp;
    line_list.ns = hand_ns;
    line_list.id = id;
    
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;

    line_list.scale.x = 0.0075;

    // Line list is white
    line_list.color.r = 1.0f;
    line_list.color.g = 1.0f;
    line_list.color.b = 1.0f;
    line_list.color.a = 1.0f;
    line_list.lifetime = ros::Duration(0.1);

    for(unsigned int j = 0; j < current_hand.finger_list.size(); j++){
        
        geometry_msgs::Point p;

        leap_motion::Finger finger = current_hand.finger_list[j];
        leap_motion::Bone bone = finger.bone_list[0];

        // Connects thumb start and index finger
        if(finger.type == Leap::Finger::Type::TYPE_THUMB)
        {
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);

            // Assumption that the hand has all 5 fingers and 
            // that the fingers are given in order from thumb[idx 0] to pinky[idx 4].
            finger = current_hand.finger_list[1];
            bone = finger.bone_list[1];
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);
        }
        // Connect wrist and pinky
        else if(finger.type == Leap::Finger::Type::TYPE_PINKY)
        {
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);

            p.x = bone.bone_end.position.x;
            p.y = bone.bone_end.position.y;
            p.z = bone.bone_end.position.z;
            line_list.points.push_back(p);

            // Creates the line from index to pinky at the wrist
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);

            finger = current_hand.finger_list[0];
            bone = finger.bone_list[0];
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);
        }
        else
        {   
            // Connects current finger and the follwing finger at the start of proximal bone (id 1)
            leap_motion::Bone bone = finger.bone_list[1];
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);

            leap_motion::Finger temp_finger = current_hand.finger_list[j+1];
            bone = temp_finger.bone_list[1];
            p.x = bone.bone_start.position.x;
            p.y = bone.bone_start.position.y;
            p.z = bone.bone_start.position.z;
            line_list.points.push_back(p);
        }
    }

    return line_list;
}

/*!
 * \brief Represents the midpoint of the palm
 * 
 * Takes a pointer to human.msg, and pose the the centre point of the palm. 
 * Using that information creates a sphere at that location.
 * 
 * \param human         Pointer to a received Human.msg
 * \param hand_ns       Used to create a unique namespace
 * \param id            A unique id to the created marker
 * \param centre_point  The palm centre of a hand
 */
visualization_msgs::Marker createPalmPosition(const leap_motion::Human::ConstPtr &human, 
    std::string hand_ns, unsigned int id, geometry_msgs::Point centre_point){
    
    visualization_msgs::Marker palm_centre;
    palm_centre.header.frame_id = human -> header.frame_id;
    palm_centre.header.stamp = human -> header.stamp;

    palm_centre.ns = hand_ns;
    palm_centre.id = id;
    palm_centre.type = visualization_msgs::Marker::SPHERE;
    palm_centre.action = visualization_msgs::Marker::ADD;

    // Location data in meters from origin
    palm_centre.pose.position.x = centre_point.x;
    palm_centre.pose.position.y = centre_point.y; 
    palm_centre.pose.position.z = centre_point.z;

    palm_centre.scale.x = 0.025;
    palm_centre.scale.y = 0.025;
    palm_centre.scale.z = 0.025;
   
    palm_centre.color.r = 0.0f;
    palm_centre.color.g = 0.0f;
    palm_centre.color.b = 1.0f;
    palm_centre.color.a = 1.0f;
    palm_centre.lifetime = ros::Duration(0.1);

    return palm_centre;
}

/*!
 * \brief Creates a visualization of an entire hand from he wrist up.
 * 
 * Using the given information calls out different functions to create a visualization of
 * a detected hand for displaying in Rviz.
 * 
 * \param marker_array  An array into which all created markers will be added
 * \param human         Pointer to a received Human.msg
 * \param hand          A hand.msg from which the hand visualization will be created.
 * \param hand_ns       Used to create a unique namespace
 * \param hand_id       A unique id to the create markers
 */

void createVisualization(visualization_msgs::MarkerArray* marker_array, const leap_motion::Human::ConstPtr& human,
    leap_motion::Hand hand, std::string hand_ns, uint8_t hand_id){

    marker_array->markers.push_back(createPalmPosition(human, hand_ns, 55, hand.palm_center));
    marker_array->markers.push_back(createHandOutline(human, hand_ns, hand_id, hand) );

    leap_motion::Finger finger;
    for(unsigned int j = 0; j < hand.finger_list.size(); j++)
    {
        finger = hand.finger_list[j];
        std::string ns_finger = fingerNames[finger.type];
        unsigned int id_offset = finger.bone_list.size();
        
        leap_motion::Bone bone;
        for(unsigned int k = 1; k < finger.bone_list.size(); k++)
        {
            bone = finger.bone_list[k];
            marker_array->markers.push_back(createFingerLines(human, hand_ns, ns_finger, k, bone));
            marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger, k + id_offset, bone.bone_start));
            
            // The circle at the very bottom of the pinky
            if(finger.type == Leap::Finger::Type::TYPE_PINKY)
            {
                leap_motion::Bone temp_bone = finger.bone_list[0];
                marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger,
                    k + id_offset+1, temp_bone.bone_start));    
            }
            // Fingertip circles
            if(k == Leap::Bone::Type::TYPE_DISTAL)
            {
                marker_array->markers.push_back(createJointMarker(human, hand_ns, ns_finger,
                    k + id_offset + 2, bone.bone_end));
            }
        }
    }
}

/*!
 * \brief Called when a new Human.msg is received.
 * 
 * \param human    Pointer to thr received Human.msg
 */
void frameCallback(const leap_motion::Human::ConstPtr& human){

    visualization_msgs::MarkerArray marker_array;
    leap_motion::Hand hand;

    if( human -> right_hand.is_present )
    {
        hand = human -> right_hand;
        createVisualization(&marker_array, human, hand, "Right", 0);
    }
    
    if( human -> left_hand.is_present )
    {
        hand = human -> left_hand;
        createVisualization(&marker_array, human, hand, "Left", 1);
    }
    
    marker_pub->publish(marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leap_motion");
    ros::NodeHandle nh("leap_motion");
    bool enable_filter;

    nh.getParam("/enable_filter", enable_filter);
    ros::Subscriber human_sub;

    human_sub = nh.subscribe<leap_motion::Human>("leap_device", 1, frameCallback);
    if(enable_filter)
    {
        human_sub = nh.subscribe<leap_motion::Human>("leap_filtered", 1, frameCallback);
    }

    ROS_INFO("enable_filter: %s", enable_filter ? "true" : "false");
    
    ros::Publisher m_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    marker_pub = &m_pub;

    ros::spin();
    return 0;
}

