std_msgs/Header header

int8 CREATE_EVENT = 0 # only fill in sdf_string        
int8 DELETE_EVENT = 1 # only fill in entity or vessel_name
int8 OTHER_EVENT = 1 # only fill in entity or vessel_name
uint8 event_type

# Name of vessel in gazebo
string vessel_name

geometry_msgs/Pose vessel_position

# Description of event.
string event 