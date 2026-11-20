# One-off renderer events. 
# For the set of live vessels, use renderer_state (RendererState).

std_msgs/Header header

int8 CREATE_CMD = 0 # only fill in sdf_string
int8 DELETE_CMD = 1 # only fill in vessel_name
int8 EXPLODE_CMD = 2 # only fill in vessel_name
uint8 cmd_type

# sdf definition in string if creation type
string renderer_obj_name

string vessel_name

# vessel pose if create or modify cmd: WGS84 latitude and longitude in
# degrees, altitude in metres from sea level, attitude in the ENU world frame.
geographic_msgs/GeoPose vessel_pose