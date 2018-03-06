std_msgs/Header header

# A unique ID for this Frame.
int32 lmc_frame_id

# How many hands were detected in the frame
int32 nr_of_hands

# How many fingers were detected in the frame
int32 nr_of_fingers

# How many gestures were detected in the frame
int32 nr_of_gestures

# The rate at which the Leap Motion software is providing frames of data (in frames per second). 
float32 current_frames_per_second

# A string containing a brief, human readable description of the Frame object. 
string to_string

# If there were any hands detected in the frame then the 
# hand.msg will be added here
Hand right_hand
Hand left_hand

