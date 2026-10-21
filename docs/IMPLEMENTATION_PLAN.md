# Simple Implementation Plan

## 2026-10-19 current readiness

The saved-map navigation software path is ready through the planner boundary. With `v3_soba_ogi_test2` active, automatic AMCL localization converged, required transforms and both costmaps are fresh, the Nav2 action server is ready, and the API reports `planner_ready=true`. The final read-only preflight also confirmed the combined telemetry/command bridge configuration and vendor subscriber. Planner requests remain opted out, no goal is active, and no velocity or motion was commanded.

Do not begin physical travel from this checkpoint. The displayed pose still needs physical alignment confirmation, `physical_estop_verified` is false, exclusive manual/autonomy arbitration has not been independently commissioned, and the guarded relay remains disabled with motor output isolated. The next milestone is a supervised commissioning sequence that proves command ownership and immediate stop preemption before a bounded, very short, known-free walking goal.

## 2026-10-18 deployment result

Version 15 API/UI is deployed, clean-built in Foxy, and served through the laptop tunnel. The software STOP control is visible in the deployed UI; the API starts with the stop latch engaged and goal requests disabled. Live GET verification passed. The robot is not ready for a walk test: `/leg_odom2` and downstream map/localization topics are absent, and motor output remains disconnected. Next work is to restore and verify odometry/mapping inputs, then commission sensor costmaps, manual/autonomy arbitration, and an independent physical stop. Keep physical motor output disabled until those checks are complete.

## 2026-10-18 local update

Added a local source-only software STOP path: goal latch, project gate disable request, Nav2 goal cancel, and project Nav2 shutdown, plus a paired-browser control. This is a software stop, not a physical E-stop; motor output stays isolated. Version 15 has not been built, deployed, or exercised. The UI startup script refuses to install version 15 unless the backend advertises its global-localization and safety-stop status fields.

The latest available robot preflight reported `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose` without publishers. These are sequential dependencies: vendor odometry must return first, then the project odometry/TF adapter and depth-to-scan+SLAM pipeline can supply `/scan` and `/map`, and localization/Nav2 can later supply `/amcl_pose`. Version 15 is deployed and the current tunnel works; next, diagnose the upstream odometry publisher without changing vendor ownership and verify each dependency before starting mapping. Keep physical movement blocked until an independent hardware stop and manual/autonomy arbitration are verified.

> **Prior API snapshot (2026-10-18):** The laptop tunnel returned HTTP 200 for status, reporting live camera/cloud only; odometry, scan, map, localization and stitched cloud were unavailable; mapping/Nav2 were stopped; goals were disabled and motor output isolated. The UI supported map selection/click preview and gated Nav2 requests. In the latest session, the tunnel was not listening, so do not treat that snapshot as current. Preserve manual RC and keep motion gated. See [current status](../STATUS.md) and [API details](WEB_APP_API.md).

> **Session update:** A new laptop-local status request could not connect to `127.0.0.1:18080`; do not assume the tunnel is currently up. Local UI/API version 14 implements AMCL global localization for no-start-pose operation but is not built or deployed. Physical walking also remains blocked by isolated motor output and unresolved command arbitration/stop path, despite the user's stated presence and authorization.
>
> The UI also has a typed-confirmation shutdown action for battery changes. It stops only project-owned mapping/navigation/bridge work and closes the project API; the owner must close the laptop tunnel and power off the complete robot through its normal procedure.

## Intended result

The user sees a live robot map on the laptop, clicks a goal, and the robot walks there while using the depth camera for obstacle awareness.

## Keep the architecture small

1. **Robot perception computer:** use the existing ROS 2 Foxy and RealSense installation. Keep camera and robot sensor drivers close to their hardware.
2. **Robot motion interface:** reuse the vendor ROS-to-motion bridge and its `/cmd_vel` interface. Do not send direct joint commands from the mapping/navigation app.
   - Preserve the vendor manual-controller app and motion SDK. Keep autonomy manually launched and inactive by default. Never let manual control and Nav2 publish to `/cmd_vel` at the same time; add an explicit mode switch or command mux before autonomous driving.
3. **Laptop app:** the intended user interface is a Windows app (or local web app) opened on the laptop. It should connect over robot Wi-Fi to a small robot-side service that starts/stops ROS functions, reports status, saves/loads maps, and submits/cancels goals. SSH is for development and maintenance, not the normal user workflow. RViz is useful during development, but the final app can show a map and accept clicked goals without exposing RViz to the user.
4. **Mapping and navigation:** use a depth-camera-compatible SLAM/localization and costmap pipeline. The mapping method must work without LiDAR and provide a map plus robot pose. The goal planner must output velocity commands compatible with the dog.

## Intended product workflow

1. Turn on the robot, connect the laptop to its Wi-Fi, and open the app. The app checks connectivity and reports camera, odometry, mapping, and motion readiness.
2. Choose **Map room**. The app starts mapping but does not drive the robot. The user walks the dog with the existing RC/manual controller; SLAM builds the map from the depth camera and odometry.
3. Stop and save the raw 2D map. Review occupancy coverage, wall continuity, start pose, and the registered point cloud. Provide reversible range/outlier filtering for the 3D review/export, with raw data retained for comparison. Filtering must not alter the occupancy map used by localization/Nav2.
4. Choose **Autonomous** explicitly only after the selected map passes review and the operator confirms clear space. Load the saved map and start localization/Nav2 with live depth-based obstacle data. Either set a known AMCL initial pose manually or request global relocalization, then verify the pose and costmap health before proceeding.
5. Click a known-free point with clearance on the 2D map. Preview the map-frame coordinates and robot-to-goal route, then request a goal. Show planner/robot progress and provide cancel plus a stop action. Physical travel is unavailable until exclusive RC/autonomy arbitration, velocity limits, watchdog/stop behavior, obstacle response, and vendor command routing are commissioned and separately supervised.
6. Return to **Manual** to hand control back to the vendor controller. Only one command source may control the robot at a time. Autonomy stays inactive by default after boot or a connection failure.

### Map and point-cloud cleanup policy

- Keep the original scan, saved occupancy map, and raw stitched cloud unchanged. Apply point-cloud filters as a separate review/export layer with recorded settings (range, voxel size, neighborhood/outlier threshold) and a way to compare against raw data.
- Remove isolated depth returns and truncate points beyond a configurable sensor range only after validating the RealSense range/noise behavior. Current stitching samples and transforms data for display; it is not a durable calibrated point-cloud map.
- Do not force room walls into a rectangle or smooth the navigation occupancy grid to improve appearance. A room may not be square; openings, furniture, and real wall irregularities are navigation-relevant. Any geometric correction needs independent measurements and explicit review.
- Local implementation now provides automatic display-only range/voxel filtering with a raw-view toggle, plus `scripts/filter_map_candidate.py` for a separate conservative PGM/YAML candidate. The map filter changes isolated occupied cells to unknown and never overwrites source files. Neither output is navigation-approved.

Laptop-to-API access is currently provided through an SSH tunnel via YSC, and a read-only API request has succeeded in this session. Retain this as a maintenance/demo path while the startup script rediscovers the perception address at each location. The normal workflow should eventually avoid manual SSH interaction, but that is packaging work; do not expose the ROS 2 DDS graph across subnets as a shortcut.

## Stages

### Current motion-path inventory - 2026-10-19

Read-only inspection while Nav2 was running found the following command route:

- The project gate publishes `/autonomy/cmd_vel` but has zero subscribers.
- ROS 1 `/cmd_vel` is `geometry_msgs/Twist` with one subscriber, vendor node `/ros2qnx`.
- `/ros2qnx` is configured with `local_port=43894`, `remote_ip=192.168.1.120`, and `remote_port=43893`; it is the existing vendor bridge toward the motion computer. No UDP packet or ROS command was sent during this inventory.
- The vendor joystick topic `/commands/joy_raw` is published by `/nx2app` and has no subscriber. It is not evidence of the manual-control arbitration path.
- Vendor `/leg_odom2` is published by `/qnx2ros`; the project ROS bridge also subscribes to it for telemetry.
- The project `cmd_vel_gate` is intentionally isolated at `/autonomy/cmd_vel`. No bridge from that topic to ROS 1 `/cmd_vel` exists.

Before connecting these topics, commission a single command owner that preserves the vendor manual controller, has an explicit manual/autonomy mode, rejects stale or conflicting commands, sends zero on timeout/readiness loss/process exit, and has a separately verified physical stop. Do not change UDP `43897`, `43893`, or vendor services as an exploratory fix. The next software deliverable is a dry-run arbiter/bridge with graph and watchdog tests; deployment must keep its output disconnected until the owner and stop behavior are verified.

The first dry-run deliverable is now present locally and deployed/built on the robot as `motion_arbiter` with `launch/motion_arbiter.launch.py`. It defaults disabled, requires `physical_stop_verified=true` at launch before accepting enable, monitors project readiness/manual override/command heartbeat, clamps to 0.10 m/s and 0.20 rad/s, and sends zero on every fault or exit. Its launch output is `/autonomy/cmd_vel_arbited`, which has no vendor subscriber. The executable and launch file were verified present; the node remains unlaunched and the vendor command path remains isolated. Rollback: `/home/ysc/lite3_autonomy_ws/backups/20261019/motion_arbiter_predeploy_123000/`.

The vendor MotionSDK README was also read-only checked on the motion computer. It says SDK joint commands take priority and the underlying controller enters damping protection after more than one second without SDK commands. That behavior is useful as a secondary timeout observation, but it is not accepted as the project's independent emergency stop or manual/autonomy arbiter. The project must still use the vendor-supported mode/stop path and verify it physically before adding any `/cmd_vel` bridge.

The deployed vendor safety package was inspected without changing its service configuration. The motion computer runs a passive Retroid STOP observer; the perception computer receives authenticated relay frames on `192.168.1.103:43910`. Starting the existing ROS 1 safety supervisor with defaults produced `DISARMED`, `dynamic_armed=false`, `transmit_enabled=false`, fresh zeroed Retroid input, and healthy telemetry. This monitor is useful for the next supervised commissioning step, but the project must not set transmission or connect `/autonomy/cmd_vel` until a physical stop test and manual/autonomy handoff are verified.

The arbiter now also reads the relay receiver's seqlock record at `/dev/shm/emotion_bot_lite3_safety_state` and fails closed on a missing, stale, torn, malformed, STOP-active, or non-zero-axis record. The robot package rebuild succeeded; the node remains unlaunched and the output topic remains disconnected.

### 1. Inventory the perception computer (complete)

- Perception host is `PERCEPTION_HOST_IP`, reachable from the motion computer. The laptop Wi-Fi network cannot currently reach this address directly.
- Both Noetic and Foxy are installed. The vendor switch script selected Foxy, and Foxy transfer/RealSense services are active and enabled. After reboot, however, Noetic `transfer.service` also became active through required dependencies. It starts both ROS1 and ROS2 transfer launches; their motion receiver nodes collide on UDP port `43897`, preventing Foxy's `jetson2motion` from starting. Keep both service states untouched until the vendor RC/manual path and a safe service arrangement are understood.
- Verified sensor topics include `/camera/depth/color/points` (~30 Hz), `/leg_odom2`, `/imu/data`, and `/joint_states`. Earlier inspection saw `/cmd_vel`; latest inspection shows only the ROS 1 `/cmd_vel` subscriber, while ROS 2 has no `/cmd_vel` due the UDP receiver conflict described below.
- Ground-truth details are in [GROUND_TRUTH.md](../GROUND_TRUTH.md).

### 2. View data without moving the robot

- Connect laptop tools to the robot ROS graph.
- Verify camera images/depth, IMU, odometry, and transforms in RViz.
- Save a short bag/log if useful. Do not publish motion commands during this stage.
- Decide the laptop display path. Because it cannot directly reach `PERCEPTION_HOST_IP` from its current Wi-Fi subnet, check a vendor-supported remote desktop or tunneled ROS connection before installing ROS on Windows.
- NoMachine server is active on the perception host at TCP port `4000`. The local SSH tunnel reaches it, but the client treats `127.0.0.1:14000` as its own physical desktop; the server log confirms that the session fails. Continue using SSH for robot-side inspection while leaving remote desktop as a separate networking issue.
- The camera cloud frame is `camera_depth_optical_frame`. The robot's `/leg_odom2` frame IDs are empty and the vendor bridge does not currently broadcast `odom -> base_link`. Add a small ROS2 TF/odometry adapter before mapping.
- Foxy has `depthimage_to_laserscan` and `slam_toolbox`; `rtabmap_ros` is missing. Prefer a 2D mapping proof of concept using the depth image converted to `LaserScan`, once TF is correct.
- Vendor odometry stamps use `RCL_STEADY_TIME` uptime (about 5,200 s), while camera messages use host time (about 1.7 billion s). Restamp odometry on ROS receipt before using it with camera scans.
- The [lite3_odom_tf_adapter package](../ros2_ws/src/lite3_odom_tf_adapter/README.md) republishes `/leg_odom2` as `/odom`, fills `odom`/`base_link` frame IDs, stamps with ROS time, and publishes `odom -> base_link`. It built successfully in `/home/ysc/lite3_adapter_ws`.
- Its `mapping.launch.py` has been run while stationary. `/scan` published at about 30 Hz; Slam Toolbox published an 81 by 114 cell OccupancyGrid at 5 cm resolution. This is only the current stationary view, not a room map.
- The latest reboot still has Noetic `qnx2ros` and Foxy `jetson2motion` competing for UDP `43897`. The standard `ros-foxy-ros1-bridge` package is installed, and a temporary bridge limited to `/leg_odom2` forwards the stream into Foxy. Scan-time checks showed odometry TF lag/extrapolation. A sensor-only node buffers scans for 0.5 seconds while preserving source timestamps; four of five sampled transforms then resolved, and a stationary OccupancyGrid was received. A full manual room scan exists but is sparse/distorted; a larger-room rescan is pending. No bridge service is enabled.
- The launch is stopped. `async_slam_toolbox_node` exited with code `-11` on Ctrl+C; investigate before relying on repeated runs. The next complete room scan requires physically moving the robot, and the user has asked us to stop and get explicit permission before that.
- Camera node `/camera/camera` uses `camera_link` and `camera_depth_optical_frame`. Vendor VOA source defines `base_link -> camera_link` translation `(0.25489, 0, 0.07249)` m and RPY `(0, 0.34907, 0)` rad. It is not active now; confirm the camera has not been remounted, then launch this static transform.

### 3. Make a map while manually controlling the robot

- The project uses Foxy `depthimage_to_laserscan` and `slam_toolbox`, a local `/leg_odom2` bridge/odometry adapter, a scan buffer, and a QoS-matched map saver. The perception ROS package and the robot-hosted mapping API are built in the separate `/home/ysc/lite3_autonomy_ws` workspace.
- A user-guided scan produced a 131 x 181 PGM/YAML, but approximately 74.8% of cells are unknown and visible structure is sparse/distorted. Keep it for comparison; it is not a localization/navigation map.
- Tomorrow's larger-room scan is the remaining mapping acceptance run. Follow [the checklist](TOMORROW_SCAN_CHECKLIST.md); use the robot API if Wi-Fi can reach it, otherwise record the route failure and use the documented maintenance workflow. Review the map before treating the mapping demo as packageable.

### 4. Add clicked-goal navigation

- Configure localization against the saved map and obstacle data from the depth camera.
- Adapt the existing Nav2 configuration: it currently expects `/odom` and `/rslidar_points`, while this robot publishes `/leg_odom2` and `/camera/depth/color/points`.
- The latest post-reboot inspection found ROS1 `/ros2qnx` subscribing to ROS1 `/cmd_vel` with no publisher, while the ROS2 `jetson2motion` intended to accept ROS2 `/cmd_vel` cannot start because it tries to bind UDP port `43897`, already occupied by ROS1 `/qnx2ros`. ROS2 currently has no `/cmd_vel` topic. Autonomy must not be enabled until the bridge conflict is resolved without disrupting manual control.
- Choose one motion-command route only after preserving the manual RC path: ROS 1 `/ros2qnx` currently subscribes to ROS 1 `/cmd_vel`, while the ROS 2 `jetson2motion` process fails on the UDP port conflict. Although `ros1_bridge` is installed for odometry relay diagnosis, no command bridge or motion route is configured. Add explicit command arbitration, conservative speed limits, timeout, and stop behavior before connecting planner output.
- Test first with the robot safely supported or with motion disabled, then on the floor in a clear area with an operator at the emergency stop.
- Confirm stop, obstacle, lost-localization, and Wi-Fi-loss behavior before attempting a room-scale goal.

### 5. Package it as a usable app/launcher

- Added API and UI controls to start/stop localization/Nav2 using a saved robot map, set the AMCL initial pose from a map click, and submit/cancel an explicitly gated Nav2 planner request. Added a shared RealSense-to-scan and odometry launch included by mapping and navigation. The revision still needs deployment/build/runtime verification; planner goals do not reach the dog because the gate output remains isolated. See [webapp instructions](../webapp/README.md) and [API implementation/status](WEB_APP_API.md).
- Remaining acceptance work: verify laptop/Retroid reachability, mapping start/stop/save/reload and map quality; verify AMCL initialization plus Nav2 goal acceptance/cancel with the gate isolated; resolve UDP 43897 ownership and establish a tested RC/autonomy interlock before any physical motion test. A no-SSH service startup is later packaging work.
- Start safe sensor services at boot if verified; keep mapping and especially motion navigation explicitly started by the user. Do not require SSH for routine use.
- Preserve the vendor RC/manual app and MotionSDK. Identify how its commands enter the motion stack and add a mutually exclusive mode gate/mux before Nav2 can publish to `/cmd_vel`; verify the controller still works in Manual mode.
- Keep configuration in this project and document startup, recovery, stop, and Wi-Fi-loss behavior.

## Current blocker / next action

The latest read-only laptop API check confirms connection but reports only live camera/cloud readiness. Odometry, scan, map, localization, and map-frame cloud are unavailable, and mapping/Nav2 remain stopped. The browser already has the click-to-coordinate preview; it cannot send an allowed planner request in this state. First re-establish a supervised manual mapping session, stop/save a new map, and inspect map/TF/cloud registration. Add reversible cloud review filters without changing the saved occupancy map. Only then progress through localization, live obstacle-layer validation, and command arbitration. Keep actual goals and motor output disabled until those checks and a separate supervised motion test are authorized.

Historical saved maps include an 89 x 99 map with 63.3% unknown cells and a later 131 x 181 scan with about 74.8% unknown cells; neither is approved for navigation. Earlier work showed `/scan` and `/map` can be produced using the odometry adapter, delayed scan transform, and QoS-matched map saver, but these are not live in the current API snapshot.

Current hard blockers for click-to-go are absent live odometry/scan/map/localization, no current registered cloud, no validated depth obstacle layer, unresolved UDP `43897`/vendor command ownership, and no exclusive Manual/Autonomous arbitration. The API tunnel is reachable now, but this alone does not commission navigation. First perform a supervised manual map session when explicitly requested, then save and review a new map and cloud. Keep Nav2 goals and all motor output disabled until each navigation dependency is independently validated.

## Assumptions to verify

- The user's Lite3 Pro has the vendor ROS2 perception stack enabled.
- The installed RealSense camera and its ROS wrapper publish usable depth data.
- The motion/perception bridge publishes sufficiently stable pose/odometry for camera-only mapping and navigation.
- The robot's gait controller accepts the velocity commands needed by the planned navigator.
- Laptop and onboard ROS versions can communicate over the robot Wi-Fi, or a simple supported remote-viewing arrangement is available.
