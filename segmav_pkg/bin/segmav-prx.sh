#!/usr/bin/env bash
set -euo pipefail
# Source ROS and your workspace
source /opt/ros/noetic/setup.bash
if [ -f "${HOME}/catkin_ws/devel/setup.bash" ]; then
  source "${HOME}/catkin_ws/devel/setup.bash"
fi

# Make sure mavlink-router is up before we start (optional grace wait)
sleep 2

# Launch PRX publisher
exec roslaunch segmav prx.launch
