#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/lotusim_ws/install/setup.bash
export PATH="/root/.nvm/versions/node/v18.20.8/bin:$PATH"

# Execute the command passed to the container
exec "$@"