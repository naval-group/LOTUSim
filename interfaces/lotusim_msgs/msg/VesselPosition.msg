string vessel_name
string model_name

# WGS84 latitude and longitude in degrees, altitude in metres from sea level,
# positive up, negative when submerged.
geographic_msgs/GeoPoint geo_point

# Heading, where the bow points: degrees from true North, clockwise, in
# [0, 360).
float32 heading

# Course over ground, the direction of motion: degrees from true North,
# clockwise, in [0, 360); 360 when the vessel is not moving (below 0.05 m/s),
# as AIS does.
float32 course

# Horizontal speed over ground, m/s.
float32 speed_over_ground
