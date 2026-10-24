#!/usr/bin/env bash
# Start the robot-hosted web API after boot for Wi-Fi browser access.
# Keep Nav2 goal requests disabled at startup. Planner testing requires a
# separate, deliberate opt-in after the live system has been reviewed.
set -eo pipefail

workspace="$HOME/lite3_autonomy_ws"
if [[ ! -r /opt/ros/foxy/setup.bash || ! -r "$workspace/install/setup.bash" ]]; then
  echo "ROS Foxy or the project workspace is not installed." >&2
  exit 1
fi

mkdir -p "$workspace/logs" "$HOME/.cache"
exec 9>"$HOME/.cache/lite3_robot_api.lock"
if ! flock -n 9; then
  echo "Lite3 robot API is already running."
  exit 0
fi

source /opt/ros/foxy/setup.bash
source "$workspace/install/setup.bash"
# Keep the API, its odometry-only bridge, and mapping launch on the same DDS
# implementation.  This does not enable any motion output.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd "$workspace"

exec ros2 run lite3_odom_tf_adapter robot_api
