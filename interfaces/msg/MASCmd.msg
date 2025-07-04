# Built in commands
int8 CREATE_CMD = 0 # only fill in sdf_string        
int8 DELETE_CMD = 1 # only fill in entity or vessel_name
int8 MOVE_CMD   = 2 # only fill in (entity or vessel_name) and vessel_position
uint8 cmd_type

# sdf definition in string if creation type
string sdf_string

# If model has been created with folder in `assets/models` directory, sdf will be retrieved. 
# Lotusim param will be retrieved from sdf_string
string model_name

# Name of vessel in gazebo
string vessel_name

# ID of vessel in gazebo. 
# During creation, the entity entered does not matter and will be overwritten
uint16 entity

# vessel position if create or modify cmd
geometry_msgs/Pose vessel_position
geographic_msgs/GeoPoint geo_point
float32 heading
