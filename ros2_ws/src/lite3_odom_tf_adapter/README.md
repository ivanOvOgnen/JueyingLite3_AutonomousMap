# Lite3 odometry and TF adapter

## Current Nav2 trial configuration (2026-10-24)

The current supervised trial uses a rolling odom-frame local costmap with a depth-derived obstacle cloud, and a saved-map global costmap that blocks unknown cells. The local observation update requirement is 0.5 seconds. Live probes measure roughly 0.8-1.13 seconds of depth transport latency, so the producer and API accept a bounded 1.5-second acquisition envelope while rejecting older/future data. The producer validates acquisition-time base TF; the API separately requires fresh current TF, localization, and both Nav2 costmaps instead of duplicating asynchronous historical TF lookups. The attended `scripts/test_auto_mode_move.py` vendor check remains separate from map-click Nav2 navigation.

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

Mapping is sensor-only and remains separate from the vendor RC manual-control path. The web API can launch the shared odometry/depth-scan pipeline with Nav2, set an AMCL initial pose, and request/cancel planner goals when explicitly enabled. Nav2 outputs are remapped to `/autonomy/cmd_vel` through a disabled-by-default gate. A separate guarded vendor relay exists, but navigation does not launch it. Verify the actual command graph before claiming motor isolation. Physical click-and-go is not commissioned.

The package also contains an uncommissioned Nav2 overlay and velocity gate:

- `config/nav2_camera_overrides.yaml` replaces the LiDAR costmap input with the RealSense point cloud, disallows planning through unknown map cells, and applies low initial DWB limits (0.10 m/s forward, 0.20 rad/s yaw, no lateral speed). It merges over the vendor Foxy Nav2 file; it is not a standalone config.
- `launch/perception.launch.py` starts the odometry adapter and RealSense depth-to-scan pipeline. Both `mapping.launch.py` and `navigation.launch.py` include it.
- `launch/navigation.launch.py` merges that overlay at launch and remaps Nav2 `cmd_vel` producers to `/nav2/cmd_vel`.
- `cmd_vel_gate` defaults disabled, requires a fresh `/autonomy/ready` true signal before its enable service accepts autonomy, and clamps velocity. A stale command/readiness or non-finite command disables it and sends zero; returning traffic cannot re-arm it. Its only output is `/autonomy/cmd_vel`.
- `motion_arbiter` is a separate commissioning component. It consumes the gated topic, requires explicit service enable, fresh readiness/manual state, a clear fresh safety record, and a launch-time physical-stop verification flag. It applies conservative limits and latches disabled on faults. The provided launch publishes to `/autonomy/cmd_vel_arbited` and never opens a UDP socket.
- `vendor_cmd_vel_bridge` separately forwards arbiter output to the vendor's documented navigation input `/cmd_vel` only when explicitly enabled under its commissioning checks. The live vendor `/safety_node` already publishes `/cmd_vel_corrected`, so the project must not publish there. Arbiter and relay start with manual state unavailable and require fresh `/autonomy/manual_override` observations; `manual_override_timeout` defaults to 1.0 s. No authoritative manual-state publisher has been commissioned in this repo, so they must refuse arming until that integration exists. Both recheck safety/manual/readiness/deadlines before forwarding, as well as on timer ticks.
- The current `lite3_room` map is packaged as a launch default with corrected unknown-space thresholds. It remains a rough proof-of-concept map and should be replaced after the scan-plane correction is validated.

Mapping uses the telemetry-only profile below. The separate combined bridge profiles include `/cmd_vel`, which can reach the vendor ROS 1 subscriber when deliberately connected. Preserve vendor manual control and keep command-capable components disabled until exclusive ownership, stop preemption, and downstream timeout behavior are verified. The current 2026-10-23 fixes are source-only and pass local inert tests; target Foxy/runtime acceptance remains pending. See [the execution log](../../../docs/EXECUTION_LOG.md).

## ROS 1 odometry relay diagnostic

On this robot, Noetic `qnx2ros` receives telemetry on UDP `43897`, while Foxy `jetson2motion` also tries to bind that port and exits. Noetic-dependent vendor services require the ROS 1 transfer launch, so do not stop that service or change either port as an ad hoc fix. The perception computer now has the prebuilt `ros-foxy-ros1-bridge` package installed.

The project config `config/mapping_ros1_bridge.yaml` selects only `/leg_odom2`; it excludes TF and all command topics. The differently named `ros1_bridge.yaml` is a combined commissioning profile and must not be substituted for mapping. Historical diagnostics forwarded odometry into Foxy and found scan-time extrapolation; `scan_delay` now delays delivery by 0.5 seconds while preserving source timestamps. New full-pipeline and moving-map acceptance must be performed from the accepted passive-recorder checkpoint; historical maps remain review-only.

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
