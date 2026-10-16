# One tapered frustum segment of a turbine's wake, chained with other
# segments (this one's r_end == the next one's r_start) to approximate a
# continuous cone. World ENU. origin carries the rotor hub altitude in z;
# axis is horizontal (wind direction, z=0). The segment is a tube centred on
# the hub: r_start/r_end bound the distance from that axis in every direction
# perpendicular to it, vertical offset included, not lateral offset alone.
# WindRegionBox remains altitude-independent; only cones have vertical extent.
geometry_msgs/Point origin
geometry_msgs/Vector3 axis
float64 length
float64 r_start
float64 r_end
