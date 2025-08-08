std_msgs/Header header

int8 CREATE_CMD = 0 # only fill in sdf_string        
int8 DELETE_CMD = 1 # only fill in vessel_name
int8 EXPLODE_CMD = 2 # only fill in vessel_name
uint8 cmd_type

# sdf definition in string if creation type
string renderer_obj_name

string vessel_name

# vessel position if create or modify cmd
geometry_msgs/Pose vessel_position