std_msgs/Header header
geometry_msgs/Vector3 velocity        # body-frame velocity (m/s)
float64[4] beam_range                 # range per beam (m) - if simulated
float64[4] beam_velocity              # vel along each beam (m/s)
bool bottom_lock                      # check if the DVL has a valid bottom lock