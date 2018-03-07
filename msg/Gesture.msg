# A unique id given to this gesture
int32 lmc_gesture_id

# Reports whether this Gesture instance represents a valid Gesture. 
bool is_valid

# The elapsed duration of the recognized movement up to the frame containing this 
# Gesture object, in microseconds. 
int64 duration_us

# The elapsed duration in seconds.
float32 duration_s

# Recognized movements occur over time and have a beginning, a middle, and an end. 
# The 'state' attribute reports where in that sequence this Gesture object falls.
int32 gesture_state
### STATE_INVALID == -1
### STATE_START == 1
### STATE_UPDATE == 2
### STATE_STOP == 3

# The supported types of gestures. 
int32 gesture_type
### TYPE_INVALID == -1
### TYPE_SWIPE == 1
### TYPE_CIRCLE == 4
### TYPE_SCREEN_TAP == 5
### TYPE_KEY_TAP == 6

# A string containing a brief, human-readable description of this Gesture. 
string to_string

# The list of fingers, tools ids associated with this Gesture, if any. 
int32[] pointable_ids

