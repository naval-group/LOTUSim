# A vessel's pose in the simulation's local frame, for LOTUSim's own
# components (renderer, physics interfaces): x East, y North, z Up in metres,
# the sea surface at z = 0; orientation of the body frame front-left-up.
# Users get positions in geographic conventions from VesselPosition.
string vessel_name
geometry_msgs/Pose pose
