# Lite3 odometry and TF adapter

This small ROS 2 Foxy node adapts the vendor `/leg_odom2` topic for ROS navigation:

- Copies each `nav_msgs/Odometry` message to `/odom`.
- Sets `header.frame_id` to `odom` and `child_frame_id` to `base_link`.
- Replaces the vendor steady-clock timestamp with the adapter's ROS/system clock time so it can be compared with RealSense timestamps.
- Publishes the matching `odom -> base_link` transform.

It does not publish velocity commands or control the robot.

## Build on the perception computer

The project package is staged separately from vendor source in `~/lite3_autonomy_ws/src/lite3_odom_tf_adapter` on the perception computer. Build it with:

```bash
cd ~/lite3_autonomy_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select lite3_odom_tf_adapter
source install/setup.bash
```

After rebuilding this package, save a retained map with the included QoS-aware one-shot saver:

```bash
ros2 run lite3_odom_tf_adapter save_live_map /home/ysc/maps/lite3_room_scan
```

It waits up to 15 seconds for `/map` using reliable, transient-local QoS and writes a matching PGM/YAML pair. Use a new prefix for every scan so earlier maps remain available. Override the wait with `--timeout 30` if needed. The saver does not validate map quality or control the robot.

The package includes `mapping.launch.py`, which starts the odometry adapter, camera transforms, depth-to-scan conversion, and `slam_toolbox`. Foxy's installed `depthimage_to_laserscan` is version 2.3.1 and has no `scan_offset` parameter. `scan_info_adjuster` therefore shifts a private CameraInfo principal row to compensate the vendor's 0.34907 rad camera pitch, and publishes the resulting scan in a level `camera_scan_frame`. This geometry has not yet been validated with a new scan; verify the mounting transform and resulting RViz scan plane before trusting a map.

## Robot-hosted mapping UI/API

The package also installs a foreground `robot_api` command. On the perception computer:

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/lite3_autonomy_ws/install/setup.bash
ros2 run lite3_odom_tf_adapter robot_api
```

The browser app supports first-browser pairing: from a trusted robot-Wi-Fi browser, the first client can create a random API key and save it in browser storage and the robot token file. This source was deployed and rebuilt on 2026-10-04, but the API and pairing flow have not yet been started or verified. A user-level `@reboot` cron entry is installed; its launcher keeps Nav2 goal requests disabled. Direct Chrome reachability is unverified. Do not start the API until the owner is present for a supervised stationary API-only check. See [the API notes](../../../docs/WEB_APP_API.md).

Mapping is sensor-only and remains separate from the vendor RC manual-control path. The web API can launch the shared odometry/depth-scan pipeline with Nav2, set an AMCL initial pose, and request/cancel planner goals when explicitly enabled. Nav2 velocity output remains isolated at `/autonomy/cmd_vel`; there is no relay to the vendor controller. Physical click-and-go is therefore not commissioned.

The package also contains an uncommissioned Nav2 overlay and velocity gate:

- `config/nav2_camera_overrides.yaml` replaces the LiDAR costmap input with the RealSense point cloud, disallows planning through unknown map cells, and applies low initial DWB limits (0.10 m/s forward, 0.20 rad/s yaw, no lateral speed). It merges over the vendor Foxy Nav2 file; it is not a standalone config.
- `launch/perception.launch.py` starts the odometry adapter and RealSense depth-to-scan pipeline. Both `mapping.launch.py` and `navigation.launch.py` include it.
- `launch/navigation.launch.py` merges that overlay at launch and remaps Nav2 `cmd_vel` producers to `/nav2/cmd_vel`.
- `cmd_vel_gate` defaults disabled, requires a fresh `/autonomy/ready` true signal before its enable service accepts autonomy, clamps velocity, and stops output on stale commands/readiness loss. Its only output is `/autonomy/cmd_vel`.
- `motion_arbiter` is a separate fail-closed commissioning component. It consumes the gated topic, requires an explicit service enable plus fresh readiness and a launch-time physical-stop verification flag, applies the same conservative limits, and sends zero on manual override, stale readiness, stale commands, or process exit. The provided launch publishes only to isolated `/autonomy/cmd_vel_arbited` and never opens the vendor `/cmd_vel` or UDP path.
- The current `lite3_room` map is packaged as a launch default with corrected unknown-space thresholds. It remains a rough proof-of-concept map and should be replaced after the scan-plane correction is validated.

No bridge from `/autonomy/cmd_vel` to the vendor ROS 1 `/cmd_vel` subscriber is included. This package therefore cannot move the robot by itself. The RC command route and its interaction with ROS velocity commands still need verification before adding that bridge. The camera-frame correction, merged Nav2 launch, and gate are installed in the project workspace but have not been launched or commissioned on the robot.

## ROS 1 odometry relay diagnostic

On this robot, Noetic `qnx2ros` receives telemetry on UDP `43897`, while Foxy `jetson2motion` also tries to bind that port and exits. Noetic-dependent vendor services require the ROS 1 transfer launch, so do not stop that service or change either port as an ad hoc fix. The perception computer now has the prebuilt `ros-foxy-ros1-bridge` package installed.

The project config `config/ros1_bridge.yaml` selects only `/leg_odom2`; it deliberately excludes TF and all command topics. A temporary `parameter_bridge` test forwarded this odometry into Foxy, and the odometry adapter published `/odom`. Scan-time checks found intermittent extrapolation because the camera scans arrived before the corresponding bridged odometry TF. The launch now routes generated scans through `scan_delay`, which waits 0.5 seconds using a monotonic timer and preserves their source timestamps. Stationary and manually guided trials produced OccupancyGrid maps, but the latest full-room map is sparse/distorted. Tomorrow's larger-room scan is the next mapping acceptance run; the stack remains a project workflow and is not a boot service.

Do not run the broad `dynamic_bridge` for routine mapping: it bridged additional status/TF topics and services. Do not enable a persistent bridge service or bridge `/cmd_vel` until the command path and arbitration have been reviewed.

For diagnosis without the web API, after reviewing the frames and confirming the camera mount is unchanged, the stationary stack can be launched manually with:

```bash
source /home/ysc/lite3_autonomy_ws/install/setup.bash
ros2 launch lite3_odom_tf_adapter mapping.launch.py
```

Do not launch the mapping stack until the odometry adapter and frame connections have been reviewed. A room scan requires the user to physically guide the robot; confirm the user's explicit direction before that activity.

The vendor VOA launch source includes a camera mount transform for this Lite3 setup:

```bash
ros2 run tf2_ros static_transform_publisher 0.25489 0 0.07249 0 0.34907 0 base_link camera_link
```

That source defines translation `(0.25489, 0, 0.07249)` m and RPY `(0, 0.34907, 0)` rad. It is a vendor-configured starting point; confirm the camera has not been remounted before using it for mapping.
