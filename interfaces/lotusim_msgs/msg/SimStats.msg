# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html
builtin_interfaces/Time sim_time
builtin_interfaces/Time real_time

uint64 iterations
float64 real_time_factor
builtin_interfaces/Duration step_size