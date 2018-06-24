std_msgs/Header header

# A unique id given to this finger
int32 lmc_finger_id

# The name of this finger. 
uint8 type

# The estimated length of the finger in meters. 
float32 length

# The estimated width of the finger in meters.
float32 width

# A string containing a brief, human readable description of the Finger object. 
string to_string

# List of bones associated with this finger
Bone[] bone_list