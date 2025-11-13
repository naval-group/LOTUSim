#!/bin/bash
##
## Copyright (c) 2025 Naval Group
##
## This program and the accompanying materials are made available under the
## terms of the Eclipse Public License 2.0 which is available at
## https://www.eclipse.org/legal/epl-2.0.
##
## SPDX-License-Identifier: EPL-2.0
##
set -e

# Source ROS environment
source /opt/ros/humble/setup.bash
source /home/lotusim_ws/install/setup.bash
export PATH="/root/.nvm/versions/node/v18.20.8/bin:$PATH"

# Execute the command passed to the container
exec "$@"