#!/usr/bin/env bash
# Start the project-owned vendor command relay in its disabled state.
#
# This is deliberately separate from the vendor services.  It may be started
# at boot so the ROS graph is ready for a later, explicitly commissioned test,
# but it never enables motion by itself.
set -euo pipefail

workspace="${HOME}/lite3_autonomy_ws"
install="${workspace}/install/setup.bash"
executable="${workspace}/install/lite3_odom_tf_adapter/lib/lite3_odom_tf_adapter/vendor_cmd_vel_bridge"
log_dir="${workspace}/logs"
pid_file="${log_dir}/vendor_cmd_vel_bridge.pid"
log_file="${log_dir}/vendor_cmd_vel_bridge_autostart.log"

if [[ ! -r /opt/ros/foxy/setup.bash || ! -r "$install" || ! -x "$executable" ]]; then
  echo "ROS Foxy, the project install, or the guarded relay is not installed." >&2
  exit 1
fi

mkdir -p "$log_dir" "$HOME/.cache"
exec 9>"${HOME}/.cache/lite3_vendor_cmd_vel_bridge.lock"
if ! flock -n 9; then
  exit 0
fi

if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file" 2>/dev/null)" 2>/dev/null; then
  exit 0
fi

env -i HOME="$HOME" PATH=/opt/ros/foxy/bin:/usr/bin:/bin \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  bash --noprofile --norc -lc \
  "source /opt/ros/foxy/setup.bash; source '$install'; exec '$executable' --ros-args -p physical_stop_verified:=false" \
  >>"$log_file" 2>&1 &
echo $! >"$pid_file"
echo "Started the disabled project vendor command relay (PID $!)."
