#!/usr/bin/env bash
# Read-only ROS/network snapshot for Lite3 mapping and planner preparation.
# Does not start/stop nodes, change parameters, or publish ROS messages.
set -o pipefail

section() { printf '\n\n=== %s ===\n' "$1"; }

# The web API gives this complete snapshot 45 seconds.  ROS graph CLIs can
# otherwise wait indefinitely when discovery or the ROS 1 master is degraded,
# turning a read-only diagnostic into a permanently occupied API action.  Keep
# every remote graph query bounded and report the timeout in the snapshot.
run_bounded() {
  local seconds="$1"
  local label="$2"
  shift 2
  timeout -k 1s "${seconds}s" "$@"
  local rc=$?
  if [[ $rc -eq 124 || $rc -eq 137 ]]; then
    printf 'WARN: %s timed out after %s seconds.\n' "$label" "$seconds"
  fi
  return $rc
}

section "Host and time"
hostname
date -Is 2>/dev/null || date
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-<unset>}"
printf 'ROS_MASTER_URI=%s\n' "${ROS_MASTER_URI:-<default localhost>}"

section "Project environment"
if [[ -r /opt/ros/foxy/setup.bash ]]; then
  # Isolate ROS setup to this process; do not mutate the caller's shell.
  source /opt/ros/foxy/setup.bash
else
  echo "ERROR: /opt/ros/foxy/setup.bash not found"
fi
if [[ -r "$HOME/lite3_autonomy_ws/install/setup.bash" ]]; then
  source "$HOME/lite3_autonomy_ws/install/setup.bash"
else
  echo "ERROR: project Foxy install/setup.bash not found"
fi
printf 'Project package prefix: '
timeout -k 1s 5s python3 - <<'PY' 2>&1 || true
from ament_index_python.packages import get_package_prefix
print(get_package_prefix('lite3_odom_tf_adapter'))
PY
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-<default>}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY:-<unset>}"

section "ROS 2 Python type support (read-only)"
# Keep this check in the Foxy-only project shell.  Sourcing Noetic in the
# ROS 1 graph subshell below is intentional and must not affect the API or
# project Python nodes.
timeout -k 1s 5s python3 - <<'PY' 2>&1 || true
import geometry_msgs
import nav2_msgs
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.type_support import check_for_type_support
check_for_type_support(Twist)
check_for_type_support(NavigateToPose)
print('geometry_msgs:', geometry_msgs.__file__)
print('nav2_msgs:', nav2_msgs.__file__)
print('Twist type support: valid')
print('NavigateToPose type support: valid')
PY

section "Project and vendor processes"
ps -eo pid,args | grep -E '[r]obot_api|[p]arameter_bridge|[b]ringup_launch.py|[m]apping.launch.py|[n]avigation.launch.py|[j]etson2motion|[r]os2qnx' || true

section "ROS 2 topics and endpoints"
# One short-lived rclpy node avoids starting a separate ros2 CLI process for
# every topic.  That is both faster on the Jetson and independent of a stale
# ros2cli daemon, while remaining a passive graph query.
timeout -k 1s 15s python3 - <<'PY' 2>&1 || true
import time
import rclpy
from rclpy.node import Node

topics_to_check = (
    '/camera/depth/image_rect_raw', '/camera/depth/color/points',
    '/leg_odom2', '/odom', '/scan', '/map', '/amcl_pose',
    '/cmd_vel', '/nav2/cmd_vel', '/autonomy/cmd_vel',
    '/autonomy/cmd_vel_arbited', '/cmd_vel_corrected', '/commands/joy_raw',
    '/autonomy/manual_override', '/autonomy/ready', '/autonomy/context',
    '/autonomy/gate/status', '/autonomy/arbiter/status',
    '/autonomy/vendor_bridge/status', '/autonomy/supervisor/status',
    '/perception/depth_obstacles',
)
rclpy.init()
node = Node('lite3_readonly_preflight_graph')
try:
    # Fresh DDS participants on this Jetson can need several seconds to
    # discover all of the Nav2 processes.  Three seconds consistently found
    # the long-lived vendor participants but omitted newer Nav2 endpoints.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    graph = dict(node.get_topic_names_and_types())
    for topic in sorted(graph):
        print(topic)
    for topic in topics_to_check:
        if topic not in graph:
            print(f'--- {topic}: absent ---')
            continue
        print(f'--- {topic} ---')
        print('Type: ' + ', '.join(graph[topic]))
        print(f'Publisher count: {node.count_publishers(topic)}')
        print(f'Subscription count: {node.count_subscribers(topic)}')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

section "Project web API status (read-only GET)"
if command -v curl >/dev/null 2>&1; then
  curl --connect-timeout 1 --max-time 3 -fsS http://127.0.0.1:8080/api/v1/status \
    || echo "API is not responding on localhost:8080 (it may be intentionally stopped)."
else
  echo "curl is unavailable; skipped API status request."
fi

section "ROS 1 command route and bridge configuration (read-only)"
if [[ -r /opt/ros/noetic/setup.bash ]]; then
  (
    source /opt/ros/noetic/setup.bash
    if command -v python3 >/dev/null 2>&1; then
      # Query the ROS master once.  `rostopic info` also contacts every node's
      # XML-RPC server and can hang on an otherwise healthy vendor publisher.
      # The master system state is sufficient for this read-only route check.
      run_bounded 8 'ROS 1 master graph query' python3 - <<'PY' 2>&1 || true
import socket
import rosgraph

socket.setdefaulttimeout(3.0)
master = rosgraph.Master('/lite3_readonly_preflight_ros1')
publishers, subscribers, _services = master.getSystemState()
topic_types = dict(master.getTopicTypes())
publisher_map = dict(publishers)
subscriber_map = dict(subscribers)

checks = (
    ('/leg_odom2', 'ROS 1 /leg_odom2 telemetry source graph:'),
    ('/cmd_vel', 'ROS 1 /cmd_vel graph:'),
    ('/commands/joy_raw', 'ROS 1 /commands/joy_raw graph:'),
)
for topic, heading in checks:
    print(heading)
    print('Type: ' + topic_types.get(topic, '<unknown>'))
    print('Publishers:')
    nodes = publisher_map.get(topic, [])
    print('\n'.join(' * ' + name for name in nodes) if nodes else ' None')
    print('Subscribers:')
    nodes = subscriber_map.get(topic, [])
    print('\n'.join(' * ' + name for name in nodes) if nodes else ' None')
PY
      echo "ROS 1 bridge /topics parameter:"
      run_bounded 5 'ROS 1 bridge parameter query' \
        rosparam get /topics 2>&1 || true
    else
      echo "Noetic setup loaded but Python 3 is unavailable for the ROS 1 master query."
    fi
  )
else
  echo "Noetic CLI is unavailable in this shell; skipped ROS 1 graph queries."
fi

section "UDP 43897 ownership (read-only)"
if command -v ss >/dev/null 2>&1; then
  ss -lunp 2>&1 | grep -E '(^|:)43897([[:space:]]|$)' || echo "No UDP 43897 listener reported."
else
  echo "ss is unavailable; skipped UDP listener query."
fi

section "Saved robot maps"
if [[ -d "$HOME/lite3_autonomy_ws/webapp/maps" ]]; then
  find "$HOME/lite3_autonomy_ws/webapp/maps" -maxdepth 1 -type f \
    \( -name '*.yaml' -o -name '*.pgm' \) -printf '%f\t%s bytes\n' | sort
else
  echo "Robot project map folder is missing."
fi

section "Safety note"
echo "Read-only snapshot complete. No node/service was started or stopped; no parameter or ROS message was changed/published."
echo "A listed /cmd_vel subscriber does not prove safe RC/autonomy arbitration. Keep motor output isolated until that path is commissioned."
