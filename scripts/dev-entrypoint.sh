#!/bin/bash
# Dockerfile.dev's entrypoint: `lotusim ui`, which runs the backend and the
# frontend as separate processes, in the background, then the container's command
# (bash by default) in the foreground. The UI stops with the container, when that
# command exits.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${LOTUSIM_WS}/install/setup.bash"

# Its own session and log file, so the UI neither prints over the shell nor
# depends on it.
ui_log=/var/log/lotusim-ui.log
setsid lotusim ui </dev/null >"$ui_log" 2>&1 &

echo "LOTUSim UI"
echo "  open http://localhost:${LOTUSIM_UI_PORT}   (the backend serves the API on :5000)"
echo "  log: $ui_log"

exec "$@"
