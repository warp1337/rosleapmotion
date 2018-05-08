#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "leap_motion/Finger.h"
#include "leap_motion/Bone.h"

#include <iostream>
#include "Leap.h"
#include "../inc/lmc_filter_node.h"

static ros::Publisher* filt_pub;

std::vector< std::vector<lpf> > hands;
bool set_up_right = true;
bool set_up_left = true;

/*!
 * \brief Uses default values for initial setup.
 *
 * Called when a Lpf object is created with this constructor.
 * 
 */
lpf::lpf()
{
    c_ = 4.0;
}

/*!
 * \brief Uses parameter server values for initial setup.
 *
 * Called when a Lpf object is created with this constructor.
 * 
 * \param cutoff    Related to the cutoff frequency of the filter.
 */
lpf::lpf(double cutoff)
{
    c_ = cutoff;
}

/*!
 * \brief Use this to change the cutoff of a lpf object.
 *
 * Use this to change the cutoff of a lpf object.
 * 
 * \param cutoff    Related to the cutoff frequency of the filter.
 */
void lpf::setCutoff(double cutoff)
{
    c_ = cutoff;
}

/*!
 * \brief This is used to set the inital position values of a filter.
 * 
 * \param initial_msrmt    Measurement will be used as the base for further filtering.
 * \param b_idx            Used to access the values of the bone using that index.
 * \param dim              Each bone has a value of x, y, z (0, 1, 2).
 */
void lpf::setInitialPos(double initial_msrmt, int b_idx, int dim)
{
    prev_msrmts_[b_idx][dim][0] = initial_msrmt;
    prev_msrmts_[b_idx][dim][1] = initial_msrmt;
    prev_msrmts_[b_idx][dim][2] = initial_msrmt;

    prev_filtered_msrmts_[b_idx][dim][0] = initial_msrmt;
    prev_filtered_msrmts_[b_idx][dim][1] = initial_msrmt;
}

/*!
 * \brief This is used to filter the position values of a bone.
 * 
 * See bitbucket.org/AndyZe/pid if you want to get more sophisticated.
 * Larger c --> trust the filtered data more, trust the measurements less.
 * 
 * \param new_msrmt    New measurement that will be used to calculate the new filtered value.
 * \param b_idx        Used to access the values of the bone using that index.
 * \param dim          Each bone has a coordinate of x, y, z (0, 1, 2).
 */
double lpf::filter(const double& new_msrmt, int b_idx, int dim)
{
    // Push in the new measurement
    prev_msrmts_[b_idx][dim][2] = prev_msrmts_[b_idx][dim][1];
    prev_msrmts_[b_idx][dim][1] = prev_msrmts_[b_idx][dim][0];
    prev_msrmts_[b_idx][dim][0] = new_msrmt;

    double new_filtered_msrmt = (1 / (1 + c_*c_ + 1.414*c_) ) * (prev_msrmts_[b_idx][dim][2] + 2*prev_msrmts_[b_idx][dim][1] 
        + prev_msrmts_[b_idx][dim][0] - (c_*c_ - 1.414*c_ + 1) * prev_filtered_msrmts_[b_idx][dim][1] 
        - (-2 * c_*c_ + 2) * prev_filtered_msrmts_[b_idx][dim][0]);

    // Store the new filtered measurement
    prev_filtered_msrmts_[b_idx][dim][1] = prev_filtered_msrmts_[b_idx][dim][0];
    prev_filtered_msrmts_[b_idx][dim][0] = new_filtered_msrmt;

    return new_filtered_msrmt;
}

/*!
 * \brief This is used to get a filtered value of a bone.
 * 
 * \param b_idx    Used to access the values of the bone using that index.
 * \param dim      Each bone has a coordinate of x, y, z (0, 1, 2).
 */
double lpf::getFilteredMsrmt(int b_idx, int dim){
    return prev_filtered_msrmts_[b_idx][dim][0];
}

/*!
 * \brief This is used to filter the unfiltered human.msg.
 * 
 * It receives the unfiltered Human.msg, filters every coordinate value and forwards it under
 * the topic "filtered_values".
 * 
 * \param hand      An instance of leap_motion::Hand to be filtered 
 * \param hand_idx  Used to access the values of the bone using that index.
 */
void filterMessage(leap_motion::Hand hand, uint8_t hand_idx){
    if(set_up_right || set_up_left ){
        hands[hand_idx][5].setInitialPos( hand.palm_center.x, 0 , 0);
        hands[hand_idx][5].setInitialPos( hand.palm_center.y, 0 , 1);
        hands[hand_idx][5].setInitialPos( hand.palm_center.z, 0 , 2);
    }

    hand.palm_center.x = hands[hand_idx][5].filter( hand.palm_center.x, 0 , 0);
    hand.palm_center.y = hands[hand_idx][5].filter( hand.palm_center.y, 0 , 1);
    hand.palm_center.z = hands[hand_idx][5].filter( hand.palm_center.z, 0 , 2);

    leap_motion::Finger finger;
    for(unsigned int j = 0; j < hand.finger_list.size(); j++)
    {
        finger = hand.finger_list[j];
        leap_motion::Bone bone;
        for(unsigned int k = 0; k < finger.bone_list.size(); k++)
        {
            bone = finger.bone_list[k];
            if(set_up_right || set_up_left)
            {
                hands[hand_idx][finger.type].setInitialPos(bone.bone_end.position.x, bone.type, 0);
                hands[hand_idx][finger.type].setInitialPos(bone.bone_end.position.y, bone.type, 1);
                hands[hand_idx][finger.type].setInitialPos(bone.bone_end.position.z, bone.type, 2);
            }

            bone.bone_end.position.x = hands[hand_idx][finger.type].filter(bone.bone_end.position.x, bone.type, 0);
            bone.bone_end.position.y = hands[hand_idx][finger.type].filter(bone.bone_end.position.y, bone.type, 1);
            bone.bone_end.position.z = hands[hand_idx][finger.type].filter(bone.bone_end.position.z, bone.type, 2);
                   
            if(bone.type != 0)
            {
                bone.bone_start.position.x = hands[hand_idx][finger.type].getFilteredMsrmt( bone.type - 1, 0);
                bone.bone_start.position.y = hands[hand_idx][finger.type].getFilteredMsrmt( bone.type - 1, 1);
                bone.bone_start.position.z = hands[hand_idx][finger.type].getFilteredMsrmt( bone.type - 1, 2);
            }
            else
            {
                if( set_up_right || set_up_left )
                {
                    hands[hand_idx][finger.type].setInitialPos(bone.bone_start.position.x, 5, 0);
                    hands[hand_idx][finger.type].setInitialPos(bone.bone_start.position.y, 5, 1);
                    hands[hand_idx][finger.type].setInitialPos(bone.bone_start.position.z, 5, 2);
                }
                bone.bone_start.position.x = hands[hand_idx][finger.type].filter(bone.bone_start.position.x, 5, 0);
                bone.bone_start.position.y = hands[hand_idx][finger.type].filter(bone.bone_start.position.y, 5, 1);
                bone.bone_start.position.z = hands[hand_idx][finger.type].filter(bone.bone_start.position.z, 5, 2);
            }
        }
    }
    if(set_up_right)
    {
        set_up_right = false;
    }
    if(set_up_left)
    {
        set_up_left = false;
    }
}

/*!
 * \brief A callback function that is run everytime a new Human.msg is received.
 * 
 * It receives the unfiltered Human.msg, filters every coordinate value and forwards it under
 * the topic "filtered_values".
 * 
 * \param human    A pointer to the received human.msg 
 */

void messageCallback(const leap_motion::Human::ConstPtr& human){ 
    leap_motion::Hand hand;
    
    if( human -> right_hand.is_present )
    {
        hand = human -> right_hand;
        filterMessage(hand, 0);
    }
    
    if( human -> left_hand.is_present )
    {
        hand = human -> left_hand;
        filterMessage(hand, 1);
    }

    // Forward the now filtered human message
    filt_pub->publish(human);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leap_motion");
    ros::NodeHandle nh("leap_motion");
    
    double cutoff;
    bool is_enabled;
    nh.getParam("/filter_cutoff", cutoff);
    nh.getParam("/enable_filter", is_enabled);
    
    if(is_enabled)
    {
        // Left and right hand setup
        hands.push_back( std::vector<lpf> ( 6, lpf( cutoff ) ) );
        hands.push_back( std::vector<lpf> ( 6, lpf( cutoff ) ) );

        ros::Subscriber human_sub = nh.subscribe<leap_motion::Human>("leap_device", 1, messageCallback);
        ros::Publisher f_pub = nh.advertise<leap_motion::Human>("leap_filtered", 1);
        filt_pub = &f_pub;

        ros::spin();
    }

    return 0;
}