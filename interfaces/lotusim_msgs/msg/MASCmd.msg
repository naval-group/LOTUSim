# Built in commands
int8 CREATE_CMD = 0 # only fill in sdf_string        
int8 DELETE_CMD = 1 # only fill in entity or vessel_name
int8 MOVE_CMD   = 2 # only fill in (entity or vessel_name), geo_point and heading
uint8 cmd_type

# sdf definition in string if creation type
string sdf_string

# If model has been created with folder in `assets/models` directory, sdf will be retrieved. 
# Lotusim param will be retrieved from sdf_string
string model_name

# Optional SDF file name inside the model folder.
# If not specified, the default file `model.sdf` will be used.
# Only applicable when the model exists in `assets/models`.
string sdf_file

# Name of vessel in gazebo. Names must not start with number of special character
string vessel_name

# ID of vessel in gazebo. 
# During creation, the entity entered does not matter and will be overwritten
uint16 entity

# Where the vessel is placed, for a create or a move command, always given
# (0 is a value: the equator, the Greenwich meridian, the sea surface):
# WGS84 latitude and longitude in degrees, altitude in metres from sea level,
# positive up, negative when submerged (a submarine at 30 m is at -30).
geographic_msgs/GeoPoint geo_point

# Heading, where the bow points: degrees from true North, clockwise
# (0 = North, 90 = East), for a create or a move command, always given.
float32 heading
