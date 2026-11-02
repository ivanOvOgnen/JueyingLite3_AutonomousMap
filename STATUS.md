# Project Status

## Current status — 2026-10-24: map-click movement not viable

The user ran the backed-up one-shot install at 18:38 local time. `log/nav2_one_shot_20261024_183816.json` records `deployed=true`, a robot backup at `/home/ysc/lite3_autonomy_ws/backups/20240912/supervised_nav_20240912_072542`, a laptop copy under `log/nav2_robot_backup_20261024_183816/`, and `manual_verified=true`. The separate Manual + Move restore report records no active project navigation process or ROS 1 `/cmd_vel` publisher before sending the vendor mode packets; those packets have no acknowledgement.

After that install, the user's Room Mapper screenshot still reports **both local and global Nav2 costmaps unavailable** because their source timestamps are stale, future, or out of order. It also reports **depth obstacle observations unavailable** because the acquisition timestamp or map/odom transform is not current, plus no current operator lease. A lease alone cannot cure either sensor/costmap failure. The screenshot does not include numeric source ages or a TF breakdown, so the exact remaining cause is unverified. The 1.0-second depth limit and 0.5-second cloud interval added earlier did **not** establish robot-side readiness. Do not weaken these checks or claim that a selected/accepted goal can move the robot.

**Physical map-click movement is not viable as of 2026-10-24.** Target selection and saved-map inspection are UI functions; they do not prove a usable Nav2 route or motor command. No successful post-update Nav2 target walk has been observed or recorded. The separate short vendor Auto Mode command test is not a Nav2 target-navigation result. Before another movement attempt, collect one live, read-only report while localization/Nav2 is running that includes each costmap's receipt/source age, depth observation source age and rejection reason, and acquisition-time map/odom TF. Then repair the demonstrated clock, stamp, or TF fault and recheck manual joystick operation. The [current test guide](docs/STEP_BY_STEP_ROBOT_TEST_20261024.md) gives the report command and recovery path. Older dated entries below are historical and do not supersede this status.

## Current verified checkpoint - 2026-10-19 (latest)

The saved map `v3_soba_ogi_test2` is loaded and Localization/Nav2 is running. Automatic global localization recovered the stale-localization condition without motion. The final read-only API check reported fresh camera, odometry, scan, point cloud, AMCL, required TF, and both costmaps; the Nav2 action server is ready and `planner_ready=true`. AMCL had 58 stable samples at preflight completion with low covariance; the later no-write confirmation reached 86 stable samples at `(x=1.124 m, y=0.083 m, yaw=-1.829 rad / -104.8 deg)`. The active map is 92 x 91 cells at 0.05 m/cell.

Planner goal requests remain deliberately disabled (`goal_requests_enabled=false`), no goal is active, the software stop is not latched, `physical_estop_verified=false`, and motor output is still isolated from the vendor command transport. No goal, velocity, gate enable, relay enable, or physical movement was issued in this checkpoint.

The deployed read-only preflight/UI revision is v23. It distinguishes incomplete discovery by its short-lived ROS 2 graph probe from the fresh data already observed by the long-running API. Deployment backup: `/home/ysc/lite3_autonomy_ws/backups/20261019/preflight23_predeploy_20261019_031912/`. The completed preflight confirmed ROS 1 `/leg_odom2` publishers `/qnx2ros` and `/ros_bridge`, vendor `/cmd_vel` subscriber `/ros2qnx`, the combined `/leg_odom2` plus `/cmd_vel_corrected` bridge configuration, and UDP 43897 listening. It changed no service, parameter, route, or ROS message.

Software localization/planner readiness is complete. Before the first walking test, the operator must independently confirm that the blue map marker matches the robot's physical pose, commission exclusive manual/autonomy command ownership, and verify that a physical/vendor stop immediately preempts the bounded autonomous command path. Only then, with the operator beside the robot and a known-free very short target selected, may planner requests and the guarded motor path be enabled for the supervised test.

## ROS compatibility and reboot readiness — 2026-10-19

Live Foxy checks pass for the API and guarded relay: `geometry_msgs` and `nav2_msgs` resolve under `/opt/ros/foxy`, and `Twist`/`NavigateToPose` have valid `_TYPE_SUPPORT`. The C++ `ros1_bridge` is the sole intentional Noetic-plus-Foxy process because it translates the vendor ROS 1 graph to ROS 2. The project boot helper now starts the guarded vendor relay with a clean Foxy environment and `physical_stop_verified=false`; no motion route is enabled.

The current live graph has one `/leg_odom2` publisher, one guarded `/autonomy/cmd_vel_arbited` subscriber, and the corrected command topic reaches the vendor ROS 1 subscriber. The API currently reports localization refresh stopped after stale input and therefore `planner_ready=false`; no goal or velocity was issued. Re-run the UI's explicit localization/global-localization action and verify fresh AMCL before planner opt-in. The independent stop and arbitration gate remain required for physical walking.

## Vendor STOP log adaptation - 2026-10-19

The project-owned motion observer now supplements the Retroid packet detector with the confirmed vendor protection log event. It tails only new vendor log data and relays a time-limited authenticated STOP when it sees `Soft Emergency Pressed!!!` or `is_robot_lose_control = 1`. Deployment backup: `/home/ysc/lite3_autonomy_ws/backups/20261019_074913/vendor_stop_observer_predeploy/`. The observer compiled and restarted successfully; the perception receiver reports fresh relay traffic with STOP clear. No vendor process, UDP port, ROS velocity topic, arbiter output, or networking configuration was changed.

This is a secondary stop signal, not independent physical-stop verification. A fresh owner-supervised stationary vendor STOP check is still required to confirm that the project safety state latches as intended. Do not enable the arbiter or connect `/autonomy/cmd_vel_arbited` to the vendor route based on this adaptation alone.

The observer was redeployed after a log-rotation race fix. The current deployed SHA-256 is `6b358a7bd33a7025bdd92c09a751132ddff90ea58d7c9a20e4e5a76b2000497b`; rollback backup: `/home/ysc/lite3_autonomy_ws/backups/20261019_075055/vendor_stop_observer_predeploy/`. The service is active and no vendor event was generated during the restart.

The service sandbox initially hid `/home/ysc`, which would have prevented log observation. Only the observer unit was changed from `ProtectHome=true` to `ProtectHome=read-only`, with rollback backup `/home/ysc/lite3_autonomy_ws/backups/20261019_075209/vendor_stop_observer_sandbox_predeploy/`. Namespace verification confirms the active observer can read the current vendor log; the service remains active and relay traffic remains fresh/clear.

## Supervised vendor STOP relay check - 2026-10-19

The owner supervised a stationary Stand then vendor STOP test. The robot stood, the vendor log emitted `Soft Emergency Pressed!!!` at `07:55:25.933`, and the robot returned to belly/resting state. The project relay observed `stop=true`; the safety supervisor entered `FAULT` with `fault_reason="Retroid STOP observed"`, `robot_basic_state=1`, fresh Retroid/telemetry, and zero axes. The current API still reports `physical_estop_verified=false` and `motion_output=not_ready`; no goal, velocity, bridge, or arbiter enable was issued. This validates event propagation and project fault latching, not autonomous-motion preemption.

## Command-owner dry run - 2026-10-19

The deployed ROS 2 arbiter owns `/autonomy/cmd_vel` as input and publishes only `/autonomy/cmd_vel_arbited`; that output has zero subscribers and remains disconnected from vendor `/cmd_vel`. A clean Foxy-only enable request was rejected with `Refusing autonomy: independent physical stop is not verified.` No command or velocity reached the vendor path and no physical motion occurred.

## Post-battery reconnect - 2026-10-19

The tunnel and API are back after the battery change. Current read-only status: sensors, map, AMCL, Nav2, transforms, and costmaps are ready; planner requests are enabled; no active goal exists. The STOP relay and session safety supervisor are restored. The isolated `motion_arbiter` is running disabled; `/autonomy/cmd_vel_arbited` has no subscribers, and its enable service correctly refuses because independent physical stop verification is still false. No motion command was sent.

The follow-up read-only supervisor check reports `DISARMED`, `stop=false`, fresh zeroed Retroid axes and telemetry, but `link_fresh=false`, `stable_stand=false`, and `sdk_feedback_healthy=false`. These are safety observations, not reasons to lower thresholds. The next step is a stationary, operator-supervised STOP signal check; the vendor motor path remains disconnected.

The direct safety topic reports vendor state `98`, `contacts_healthy=false`, `operator_planted_confirmed=false`, and reason `four-contact sensor gate unavailable or operator planted confirmation absent; EmotionBot link stale`. Battery is 93%, attitude is level, and vendor joint/IMU feedback is healthy. The project will not synthesize the planted confirmation or bypass this gate.

During a supervised stationary check, vendor Stand reached state `6` with `stable_stand=true` and healthy SDK feedback, but the Retroid STOP relay stayed `False`. Vendor Sit/Lay down returned the robot to resting state `1`. Motion remains blocked because STOP preemption and the four-contact/direct-feedback gate are not verified.

After the vendor STOP action, the supervisor entered `FAULT` with `fault_reason="robot protection state"` while the robot reported resting state `1`. The project `stop` flag remained false; this confirms vendor protection occurred but does not certify the independent project interlock.

A read-only `tcpdump` on motion `p2p0` showed the Retroid link active and, during another vendor STOP press, only normal `0x21040001` reply packets. The expected project STOP code `0x21020C0E` was absent. The STOP relay detector must not be treated as verified.

## Isolated motion-arbiter deployment - 2026-10-19

`lite3_odom_tf_adapter` was built on the robot with the new fail-closed `motion_arbiter` executable and launch description. Rollback backup: `/home/ysc/lite3_autonomy_ws/backups/20261019/motion_arbiter_predeploy_123000/`. The node was not launched; output is still disconnected from the vendor command path, the physical-stop flag remains false, and motor output is isolated. This is a software readiness step only and does not make the robot walk-ready.

The vendor safety chain is present: the motion computer's Retroid STOP observer is active, the perception STOP relay receiver is listening on `192.168.1.103:43910`, and a session-only ROS 1 safety supervisor reports `DISARMED`, `dynamic_armed=false`, `transmit_enabled=false`, fresh zeroed Retroid input, and healthy telemetry. No command transmitter was enabled. Physical stop preemption still needs a supervised verification before any bridge can be connected.

The arbiter has been rebuilt with a shared-memory safety gate reading `/dev/shm/emotion_bot_lite3_safety_state`; the current record is valid and fresh with STOP clear and zero axes. Backup for this revision: `/home/ysc/lite3_autonomy_ws/backups/20261019/motion_arbiter_predeploy_124000/`. The arbiter remains unlaunched and vendor output remains isolated.

## Live session - 2026-10-19

The browser is paired and the saved map `v3_soba_ogi_test2` is loaded. Camera, odometry, scan, map, required transforms, Nav2, and both costmaps are fresh/ready. Automatic AMCL localization did not converge (`stable_samples=0`, localization `not_ready`), so navigation reports `planner_ready=false`, planner requests remain disabled, and there is no active goal (`active_goal_state=idle`). The physical stop is unverified and motor output remains isolated from the vendor command transport. No goal, velocity, or movement command was sent. Set a known initial pose on the 2D map and let AMCL stabilize before considering planner opt-in; do not lower readiness thresholds.

## Updated live session - 2026-10-19

AMCL now reports ready with 12 stable samples and bounded covariance. Camera, odometry, scan, map, transforms, Nav2, both costmaps, and the action server are ready; `planner_ready=true`. There is still no active goal, and planner requests are disabled until the explicit UI opt-in is pressed. The physical stop is unverified and motor output remains isolated from the vendor command transport. No goal or velocity command was sent.

## Goal submission diagnostic - 2026-10-19

The live API now reports planner requests enabled, `planner_ready=true`, and `active_goal_state=idle`; no goal has been accepted. The UI was masking rejected goal errors by refreshing the idle status immediately after the request. UI v21 was deployed without restarting the API or Nav2 on a fresh tunnel at `http://192.168.2.39:18084/`; its send-goal handler preserves the HTTP rejection message. Robot UI backup: `/home/ysc/lite3_autonomy_ws/backups/20261019/startup_ui_predeploy_20261019_002239/`. Use port 18084 for the diagnostic retry. No goal, velocity, or movement command was sent.

## Nav2 action result - 2026-10-19

The submitted planner goal completed with ROS action status `4`, which is `SUCCEEDED` (not failure). The API now reports `active_goal_state=finished_4`; the UI v22 deployment at `http://192.168.2.39:18086/` renders this as “Nav2 reports SUCCEEDED (status 4).” This confirms planner/action completion only. The vendor motor path remains isolated, so no physical walking occurred. No new goal or velocity command was sent.

## Live session - 2026-10-18, v20 deployed

API v20 was deployed and restarted. The robot API is listening on port 8080 (PID 15939); the laptop page at `http://192.168.2.39:18082/` serves UI v20 (`app.js?v=20`). A fresh status GET after restart reports camera and point cloud ready; odometry, scan, map, localization, transforms, costmaps and Nav2 not ready. The map is not loaded in API memory, no goal is active, planner requests are disabled, software STOP is latched, physical E-stop is unverified, and motion output is isolated. The API restart stopped its owned Nav2 stack and cleared the in-memory map; saved map files were not deleted.

Robot-side pre-deployment backup: `/home/ysc/lite3_autonomy_ws/backups/20261018/costmap_goaloptin_predeploy_01/`. The v20 source fixes costmap message type and AMCL stable-sample bookkeeping, and adds guarded planner-only opt-in requiring full readiness and the active map. Nav2 is intentionally left stopped. No goal, velocity, or motion command was issued; vendor services, networking and UDP settings were not changed. Next: reload/reconnect the browser, load the saved robot map, start Localization/Nav2 via UI, then verify fresh sensors, transforms, stable AMCL, both costmaps, and map-pose alignment before planner opt-in. Motor output remains isolated; this does not enable a walking test.

## Previous deployed checkpoint - API/UI v19 (superseded)

The laptop tunnel at `http://192.168.2.39:18082/` serves API v19 and UI v19. The project API source and installed module match SHA-256 `c0b6268a8e5dac7a6826fc5340004ec43cdb23f2e5ac3d6675d571895d59e2f4`; the Nav2 overlay requests periodic full local/global costmaps. The Foxy build succeeded; the installed Python module was synchronized and compiled after the build left the prior installed copy unchanged. A read-only laptop GET confirms API/UI availability and the new `transforms` and `costmaps` status fields. Robot rollback copy: `/home/ysc/lite3_autonomy_ws/backups/20261018/nav_readiness_predeploy_01/`.

Live API status after restarting only the project API: camera and point cloud ready; odometry, scan, map, all required TF checks, costmaps, localization and navigation not ready. The restart cleared the API's in-memory map; saved files remain in the robot webapp maps directory, but none is currently loaded. Mapping and Nav2 are stopped. API is paired; software STOP is latched; goal requests are disabled; `physical_estop_verified=false`; velocity output remains isolated. No bridge, mapping, Nav2, goal, velocity, or vendor process was started or changed. The last preflight identified vendor ROS 1 `/leg_odom2` but no project bridge; ROS 2 data remains absent. This is not ready for a goal or a walk. Next: while the robot is stationary and operator-supervised, restore the narrow odometry bridge and launch the saved-map localization/Nav2 stack, then verify live TF, scan/map, costmaps, AMCL convergence, and pose alignment. Physical motion additionally requires a verified independent stop and exclusive RC/autonomy arbitration.


## 2026-10-18 — reconnected after battery change

The laptop tunnel is live at `http://192.168.2.39:18082/`; the robot API is v16 and now serves UI v18. UI files were backed up at `/home/ysc/lite3_autonomy_ws/backups/20261018/ui_v18_predeploy_01/` before copying. Current authenticated API status: camera and point cloud are fresh; odometry, scan, and map data are stale; localization, mapping, and Nav2 are not ready/running. The 92x91 cached map metadata is not proof of a fresh live `/map`. Goal requests are disabled, `physical_estop_verified=false`, and velocity output remains isolated. The API's software stop latch currently reports not latched after reconnect, so do not treat it as an active stop.

The read-only preflight confirms zero ROS 2 publishers on `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose`; `/odom` is absent. A follow-up ROS 1 check found the vendor `/leg_odom2` source publisher `/qnx2ros`, while the project bridge's `/topics` parameter is unset. The immediate blocker is the stopped project odometry bridge; mapping/Nav2 startup normally creates that narrow bridge. The scan/map publishers are also absent while the stack is stopped. Preflight backup: `/home/ysc/lite3_autonomy_ws/backups/20261018/preflight_ros1_odom_predeploy_01/`. No process, service, parameter, route, goal, or velocity was changed. This is not ready for a walk.

## Historical diagnostic run before battery reconnect — 2026-10-18

API/UI version 16 is live through the laptop tunnel at `http://192.168.2.39:18082/`. Robot-side API source and installed-module hashes match (`b14719e6…4724e5`); the UI serves `UI_DEPLOY_VERSION = 16`. Backups: API `/home/ysc/lite3_autonomy_ws/backups/20261018/bridge_reuse_predeploy_01/`, UI `/home/ysc/lite3_autonomy_ws/backups/20261018/amcl_ui_predeploy_01/`, and perception launch `/home/ysc/lite3_autonomy_ws/backups/20261018/camera_tf_predeploy_01/`.

The API adds transient-local AMCL pose subscription, repeated `/request_nomotion_update` requests during global localization, and a three-sample stable/low-covariance gate. It adopts an already-running ROS 1 bridge only if it is the sole `parameter_bridge` and `/topics` exactly matches the project’s `/leg_odom2` telemetry config. The live bridge is present and publishes `/leg_odom2`; no command topic is bridged. API/UI deployment and Python syntax checks passed; Node.js is not installed locally, so no Node syntax check was available.

The API's action lock became stuck after the prior diagnostics request. I restarted only the project API; it came back with goal requests disabled, software STOP latched and output isolated. Starting Localization/Nav2 then produced a fresh `/scan` and active saved map. A best-effort `/scan` echo confirmed `frame_id=camera_scan_frame`; standalone `tf2_echo` resolved `odom → base_link`, `base_link → camera_scan_frame`, and `base_link → camera_depth_optical_frame`. However, Nav2 continued to log `odom` and `camera_scan_frame` as unknown, and ROS lifecycle queries returned “Node not found”; the Nav2 transform/costmap path is therefore **not verified working**. A second 90-second global localization attempt produced candidate AMCL poses, but covariance remained high and no stable samples accumulated. Latest observed estimate was `(x=0.688 m, y=-0.067 m, yaw=-1.768 rad)`, with x/y variances about `0.198/0.090 m²` and yaw variance `0.428 rad²`. The Nav2 stack was stopped and software STOP latched again.

Current API: camera/point cloud/odometry ready; scan/map/localization/Nav2 not ready because the project stack is stopped; goals disabled; software STOP latched; `physical_estop_verified=false`; `/autonomy/cmd_vel` remains isolated. The map server and scan were live while the stack ran. Standalone TF checks succeed, but Nav2 cannot yet use those transforms and its lifecycle nodes are absent from the node list. The final software STOP stopped Nav2 and latched planner requests, but could not confirm the cmd_vel gate-disable service response; output remains isolated from robot transport. **Do not attempt a walk.** Next diagnose the Nav2 ROS graph/remappings and lifecycle failure, then obtain a physically confirmed initial pose or improve map/scan alignment until AMCL converges; verify the map marker and obstacle costmap before planner use. Physical motion additionally requires independent motor routing, arbitration, watchdog, obstacle response, and stop commissioning.

## 2026-10-18 — saved-map navigation blocked by live ROS inputs

The user reports saving a new room map and selecting a point in the browser. The current authenticated, read-only API status and ROS preflight show why the workflow stops: the browser click is a map-coordinate preview, while navigation is gated until localization and planner readiness.

The live preflight completed successfully (`exit_code=0`). `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose` each have zero publishers; `/odom` is absent. Camera and point-cloud publishers are present. Mapping and Nav2 are stopped, the cached 92x91 grid at 0.05 m/cell is stale, global localization is not ready, the planner/action server is unavailable, goal requests are disabled, and the software stop latch is engaged. The map catalog lists `v3_soba_Ogi_2026` and `v3_soba_ogi_test2` among six saved maps; the API snapshot does not establish which is the user's newest map or that it is loaded for navigation.

No localization/Nav2 start, stop action, goal, velocity command, or process/service change was made. Do not try a walking test from this state. First restore and verify the vendor `/leg_odom2` stream; then validate project `/odom` and TF, live `/scan` and `/map`, selected-map load, localization pose, Nav2 action server and obstacle costmap. Separately commission command arbitration, motor routing, watchdog behavior, and an independent physical stop before any supervised motion test.

## 2026-10-18 — version 15 deployment complete

Deployed local API/UI version 15 to the perception computer after creating `/home/ysc/lite3_autonomy_ws/backups/20261018/v15_predeploy_20261018_01/`. A clean Foxy package build succeeded; source and installed API hashes match, Foxy `NavigateToPose` type support passes, and generated package setup has no Noetic prefix. Restarted only the project API with its existing launcher. Startup cron, vendor services, network routes, UDP ports, and ROS parameters were not changed.

The laptop tunnel at `http://192.168.2.39:18082/` serves the page, JS (marker `UI_DEPLOY_VERSION = 15`), and CSS with HTTP 200. Read-only status confirms camera/point cloud ready; odometry, scan, map, localization, and stitched cloud not ready; mapping/Nav2 stopped; goals disabled; STOP latch engaged; motion output isolated. `safety_stop.physical_estop_verified` is false. The stop control is deployed but was not invoked as a live test.

This is not ready for a walk test. The latest available preflight found no `/leg_odom2`, `/scan`, `/map`, or `/amcl_pose` publishers; current API status continues to report odometry/scan/map/localization unavailable. Restore the upstream odometry stream and verify downstream mapping/localization, costmap health, exclusive RC/autonomy arbitration, and a separate physical stop before any movement test.

## 2026-10-18 — predeployment reconnect snapshot (superseded)

The user ran `Start-Lite3Session.ps1 -LocalPort 18082`. YSC discovery succeeded and the launcher reached the robot API, but correctly stopped before installing UI assets because the API lacks the v15 `global_localization` and `safety_stop` status fields. The laptop's read-only `GET /api/v1/status` through port 18082 returned HTTP 200: camera/point cloud are fresh; odometry, scan, map, localization and stitched cloud are not ready; mapping/Nav2 are stopped; goals are disabled; motor output is isolated. No control action was called. SSH shell access timed out during banner exchange, so API deployment is not yet available.

This is not a walk-test-ready state. Restore vendor `/leg_odom2` first, then verify project `/odom`/TF, `/scan`, `/map`, AMCL pose, obstacle layer and the independent stop/command arbitration before any supervised motion test.

## 2026-10-18 — version 15 source before deployment

Local source version 15 adds **STOP AUTONOMY**. It software-latches planner goals, requests the project gate’s SetBool disable service, cancels an accepted Nav2 goal where possible, and stops the project Nav2 process. The latch clears only when Localization/Nav2 is explicitly started again. The UI warns that this is not a physical emergency stop. This source has not been built or deployed; no stop action was triggered. The robot motor route remains isolated and an independent emergency stop/interlock is unverified.

The last robot-side read-only preflight found camera data but no publishers for `/leg_odom2`, `/scan`, `/map`, or `/amcl_pose`. `/leg_odom2` must be restored from the vendor transfer path before the project’s bridge/adapter can provide `/odom` and TF; the mapping launch can then produce `/scan` and `/map`, while AMCL publishes `/amcl_pose` after localization starts. The current tunnel probe on port 18081 failed, so current runtime state is unverified. No robot launch, vendor-service operation, or motion command occurred.

Last updated: 2026-10-18 (Europe/Skopje)

## Earlier 2026-10-18 API and preflight findings (before version 15 deployment)

**Latest reconnect (2026-10-18):** The visible startup launcher established a second SSH tunnel on laptop Wi-Fi port 18081 after finding port 18080 occupied by an older SSH process. `GET /api/v1/status` on 18081 returned HTTP 200. It reports camera/point cloud ready; odometry, scan, and map stale by about 14 minutes; localization unavailable; mapping and Nav2 stopped; goal requests disabled; action server unavailable; motor output isolated. Five saved-map names are in the catalog, but the active map data is stale. This is a live API status check, not a new robot-side process/service preflight. The API does not advertise `components.global_localization`, so local UI v14 was not copied. No control endpoint or motion command was used. The 18081 tunnel is left open for this session.

**Fresh read-only preflight (2026-10-18):** The authenticated API diagnostics route ran `robot_preflight.sh` successfully (`exit_code=0`). Camera image and point cloud each have a publisher. `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose` have zero publishers; `/odom`, `/cmd_vel`, `/nav2/cmd_vel`, and `/autonomy/cmd_vel` are absent. ROS 1 `/cmd_vel` has no publisher and `/ros2qnx` subscriber; ROS 1 `/commands/joy_raw` has `/nx2app` publisher but no subscriber. Bridge `/topics` is unset. UDP 43897 is bound with process owner unidentified. The API reports odometry/scan/map stale by about 18 minutes; mapping/Nav2 stopped, goals disabled, motion output isolated. Five saved maps exist; the cached 86x91 map is about 49% known but stale. Robot system clock reports 2024-09-12. The preflight started/stopped no process/service and changed/published no ROS parameter/message. This verifies the current blocker: mapping/navigation inputs and the manual/autonomy command path are not active or commissioned.

Timeline clarification: an earlier noninteractive SSH attempt on 18081 was canceled at authentication; a later visible launcher run succeeded in establishing the tunnel. A separate batch-mode SSH attempt for the read-only preflight timed out during SSH banner exchange, so no shell preflight was obtained.

**Session correction (2026-10-18):** The user reports mapping works, but the latest successful API snapshot was stale: odometry, scan and map were marked not ready (ages about 258–263 seconds); it showed mapping/Nav2 stopped, goal requests disabled and motor output isolated. In the current session, `http://127.0.0.1:18080/api/v1/status` did not connect, so neither the tunnel nor current robot state could be refreshed. UI version 14 and the global-localization API are local only and not deployed. No control endpoint or motion command was used. User confirms they are beside the robot and authorizes a goal test; physical motion must still wait for verified deployment/readiness, motor ownership/arbitration and an independent stop path. Browser cancel/shutdown is not an emergency stop.

The tunnel diagnosis was refined: the laptop is on `YSC-JYML-he2asx-5G` at `192.168.2.39`, gateway `192.168.2.1`; port 18080 is bound to the laptop's Wi-Fi address by an SSH process with no established remote connection, targeting the old perception address `192.168.1.103`. The startup helper refused that port. A read-only retry on port 18081 stopped at SSH authentication before current YSC configuration was read. Current perception/API state remains unknown.

The user reports the Windows startup terminal/tunnel is running. A current read-only request from the laptop to the local tunnel at port 18080 returned HTTP 200 from `/api/v1/status`; this verifies that the tunnel reaches the project API on the perception computer. It does not verify a fresh SSH shell or a robot-side service inventory.

API status: browser session paired; control mode `manual`; camera and live point cloud ready; odometry, scan, map, localization, and stitched map-frame cloud not ready; mapping stopped; localization/Nav2 stopped; navigation goal requests disabled; planner/action server not ready; velocity output isolated. The saved-map catalog responds with four entries. The live camera cloud responds, but `/api/v1/map/grid` and `/api/v1/pointcloud/map` return HTTP 503 because there is no live map/TF registration. No control endpoint was called and no motion was commanded.

The laptop is on `YSC-JYML-he2asx-5G`; the existing port-18080 tunnel continues to serve the API and older UI. A fresh SSH attempt through the gateway stopped at interactive authentication; the provided password input closed the connection before the YSC configuration could be read. UI deployment version 14 and the backend global-localization endpoint therefore remain local and undeployed. No robot process or control endpoint was touched.

The workflow therefore cannot yet do what the user wants: there is no active or selected navigation map/localization state and the motor path is intentionally disconnected. Existing UI map-click/goal-preview behavior is implemented; actual planner requests remain gated. The current browser API status does not freshly verify vendor service health, UDP ownership, RC/autonomy arbitration, or Retroid access.


The user reports mapping now works. The latest API GET contains a cached 86x91 map at 0.05 m resolution (~49% observed), but odometry, scan, and map data are 258–263 seconds old, so the API correctly marks them stale. Camera and live cloud are fresh; mapping and Nav2 are stopped. The motor output remains isolated and Nav2 goal requests are disabled.

Local source adds a Foxy AMCL global-localization request for the no-start-pose workflow, fresh-pose display on the 2D map, and map-name matching before a clicked goal is submitted. These changes have not been syntax-checked, built, or deployed. Global localization is not guaranteed to converge on an incomplete or symmetric map. It does not move the robot; planner goals still cannot produce walking while the motor gate is disconnected.
## UI and cleanup implementation

- The 3D viewer now filters far camera points and sparse isolated voxels by default. It offers a raw-view toggle and live-camera range setting. Filtering is display-only; raw API buffers, stitched accumulation, saved occupancy maps, and navigation inputs are unchanged.
- The UI explains the intended sequence: build the map under manual RC control, save/review it, start localization/Nav2 using a robot-saved map, set the initial pose, click a known-free destination, and submit it to the planner. It explains that depth obstacle avoidance requires a ready sensor costmap and that a map click is a coordinate preview.
- `scripts/filter_map_candidate.py` creates a separate map pair and only turns isolated occupied cells into unknown. It preserves free and unknown cells and map geometry. It refuses to overwrite input or existing output files; the generated candidate is not navigation-approved.
- `Start-Lite3Session.ps1` detects the UI deployment marker and checks API capabilities before copying `index.html`, `app.js`, and `styles.css` together without restarting robot services. Version 15 was deployed manually in this session; the launcher's asset-copy branch has not been verified end to end.
- The deployed assets were HTTP-verified. Browser visual rendering remains unverified. Current API has no live odometry/scan/map/localization or stitched cloud. Nav2 goals remain disabled and motor output isolated.

## Repeatable startup implementation (local source)

Added `scripts/Start-Lite3Session.ps1` for Windows laptop startup. It identifies the active Wi-Fi IPv4/default gateway dynamically, discovers the perception IP through YSC, and opens an SSH ProxyJump tunnel. A visible SSH window receives host-key/password interaction; credentials are not saved. The script polls `GET /api/v1/status` for up to 100 seconds. If the served UI lacks deployment version 12, it backs up and copies `index.html`, `app.js`, and `styles.css` from this repo to the robot's project web directory; it does not restart the API or touch ROS/vendor processes. It then prints the HTTP URL for the laptop and Retroid and opens the page in the laptop's default browser. The UI's **Connect & check status** creates the current browser session and refreshes all component states via GET only.

The first 2026-10-18 launcher attempt authenticated to gateway `192.168.2.1` but could not resolve `lite`. After the launcher was changed to query YSC's `~/jy_exe/conf/network.toml`, the user reports a successful startup connection and the laptop's read-only status GET now succeeds through the local tunnel. Automatic UI deployment, new-location startup, and Retroid access through the laptop-bound tunnel remain unverified. The dashboard does not detect the handheld/vendor-controller link; check it separately in the Lite3 app.

## Live robot state

The following robot-side service and ROS graph details are historical (2026-10-06 and earlier), not reverified by the latest laptop API GET. The current API component state is recorded above.

- Vendor `transfer.service`, `transfer_ros2.service`, and `realsense_ros2.service`: active.
- RealSense depth image and `/camera/depth/color/points`: publishing; camera status fresh.
- `/leg_odom2` currently has no publisher; `/scan` and `/map` have no publishers. No localization or Nav2 stack is active.
- Robot API: active on port 8080, reachable by laptop through the SSH tunnel. It reports camera ready, mapping stopped, navigation stopped, Nav2 goal requests disabled, and the motor output isolated.
- Saved robot maps are present, including `room_scan_review_20261004_01`; they are review artifacts and are not approved for navigation.
- ROS 1 `/cmd_vel` has subscriber `/ros2qnx` but no publisher. ROS 1 `/commands/joy_raw` has publisher `/nx2app` but no subscriber. UDP 43897 is bound by an unidentified process. These do not establish a safe manual/autonomy route.
- No robot movement or velocity command was sent. The read-only preflight started/stopped no service and changed/published no parameter or ROS message.

## Current browser path

The robot API/UI responds through the current laptop tunnel. Status, saved-map list, and live camera cloud were confirmed with read-only requests; map grid and stitched cloud return HTTP 503 until a live map/TF is available. Page asset status and Retroid browser access are not freshly reverified. Open using `http://`, not `https://`.

Local source adds:

- an HttpOnly, same-origin browser session, so no API key needs to be pasted into each browser;
- mapping pause/resume controls, with a warning that pausing SLAM does not stop robot movement;
- live 2D OccupancyGrid rendering;
- live orbitable RealSense point-cloud rendering;
- a bounded map-frame point cloud accumulated from camera points only when TF can register them during mapping.

The files were copied only into the project workspace. Dated backups of the old deployment and the pre-shutdown-button deployment are under `/home/ysc/lite3_autonomy_ws/backups/20261006/`. Clean Foxy package build, installed-source hash comparison, Foxy/Nav2 imports, JavaScript parsing, and DOM ID checks succeeded. Browser visual verification remains pending because no Chrome surface is exposed in this session.

## Navigation and 3D limitations

The browser can expose existing localization/Nav2 controls, but physical navigation is not commissioned. Goal requests stay disabled, motor output remains isolated, and no navigation start/goal was issued. The existing maps are sparse and require review.

The live cloud is in the camera frame. The accumulated cloud is a sparse review aid in the SLAM `map` frame; it depends on fresh map-to-camera TF during mapping and does not replace a calibrated 3D pose graph. A successful endpoint/build alone does not prove the cloud is correctly registered or useful for navigation.

## Controlled power-off preparation

The UI includes an explicit **Stop project and disconnect UI** action. It requires typing `STOP PROJECT`, serializes against other control actions, cancels any accepted planner goal, stops project-owned mapping/localization/Nav2/odometry-bridge processes, and then closes the project API. Its invalid-confirmation path was checked and returned HTTP 400 without shutting down the API. Vendor transfer/RealSense services remain active. It does not stop the Linux computer or terminate the laptop-side SSH tunnel. After success, close that tunnel and power off the robot normally before changing its battery. Do not pull the battery while the computer is running.

## Next work

1. Confirm the current status in Chrome/Retroid. Do not start controls just to test the visual page.
2. For a scan, explicitly confirm a supervised manual-mapping session, then verify readiness, manually guide, stop/save, and review the 2D map and map-frame cloud.
3. Add reversible point-cloud outlier/range filtering for visualization/export while preserving the raw capture. Do not rectangularize or otherwise smooth the navigation occupancy grid.
4. Navigation remains blocked pending a suitable saved map, validated localization and live depth obstacle layer, camera/scan geometry checks, vendor command path ownership and Manual/Autonomous arbitration. The map-click preview already exists; a physical click-to-move demonstration is not safe or available yet.

See [HANDOVER.md](HANDOVER.md) for the full dated record and [GROUND_TRUTH.md](GROUND_TRUTH.md) for observations.

## User-supplied follow-up preflight (2026-10-18)

The later read-only snapshot confirms the same bridge blocker: ROS 1 `/leg_odom2` is published by vendor node `/qnx2ros`, but ROS 2 `/leg_odom2` still has zero publishers and the bridge `/topics` parameter remains unset. ROS 2 `/scan`, `/map`, and `/amcl_pose` also have zero publishers; `/odom` and motion-output topics are absent. Camera image and point cloud are publishing. API status says manual mode and paired browser; camera/cloud ready, odometry not ready (last data age 31.16 s), scan/map not ready (about 19 minutes old), localization not ready, mapping/Nav2 stopped, goal requests disabled, software STOP not latched, physical E-stop unverified, and velocity output isolated. Cached map dimensions do not mean the map is live.

The robot clock in this snapshot is `2024-09-12T15:07:49+08:00`, so its wall-clock timestamp is stale; use the API monotonic age values for freshness. This is user-provided output, not a new live check performed from this workstation. No process, service, ROS parameter/message, route, goal, or velocity was changed. The bridge and downstream mapping/localization stack remain to be brought up and verified before navigation.

## Local navigation-readiness hardening (2026-10-18; not deployed)

The local API now requires fresh, valid `/local_costmap/costmap` and `/global_costmap/costmap` messages, plus `odom -> base_link`, `base_link -> camera_scan_frame`, `base_link -> camera_depth_optical_frame`, and `map -> base_link` transforms before reporting planner readiness or accepting a goal. The Nav2 overlay requests periodic full costmap publication so the API can check that data remains live. UI v19 displays transform and costmap readiness; the Windows launcher refuses to install it unless the robot API reports both new components. Goal requests still default off, and `/autonomy/cmd_vel` remains disconnected from motor transport.

Current-session access to the local tunnel at `127.0.0.1:18082` failed, so the API change cannot be built/deployed or checked against robot Nav2 topics now. No robot process, service, route, gate, goal, or velocity was changed. After the bridge is restored, first check whether the installed Foxy Nav2 publishes the expected Costmap message topics and whether all TF edges resolve; the added gates are intentionally expected to remain not-ready otherwise.
# Current live check - 2026-10-18

- Laptop tunnel/API responds at `http://192.168.2.39:18082/`; browser is paired.
- Latest GET: camera, odometry, scan, point cloud, active map server, Nav2 process/action server, and required TF checks report ready.
- AMCL global search is active; stable sample count was 0 at this check. Local/global Nav2 costmaps are not fresh. `planner_ready=false`; goal requests are disabled.
- `/local_costmap/costmap` and `/global_costmap/costmap` are published as `nav_msgs/msg/OccupancyGrid`; the deployed API subscribes expecting `nav2_msgs/msg/Costmap`, so it cannot receive those messages.
- Local API/UI v20 source fixes the subscription type and stale AMCL convergence flag; it adds a guarded planner-only opt-in requiring full readiness and the active map. It revokes opt-in on readiness loss. It has not been built/deployed. The API shutdown handler stops its owned Nav2 stack, so schedule deployment when the operator is ready to restart Localization/Nav2.
- Motor output remains isolated; physical E-stop verification is false. No goal, velocity, threshold relaxation, process stop, or motion command was issued.


## Current live readiness check (2026-10-19)

A read-only inspection through the authenticated `lite` shell found the project bridge and Nav2 stack active. ROS 2 `/ros_bridge` publishes `/leg_odom2`; `/odom` consumers, `/scan`, the active map, TF, and both Nav2 costmaps are present. The API status is fresh for camera, odometry, scan, map, transforms, costmaps, and the Nav2 action server. AMCL has a pose estimate but is not converged (`stable_samples=1`), so `planner_ready=false` and goal requests remain disabled.

The vendor ROS 1 graph shows `/ros2qnx` subscribed to both `/cmd_vel` and `/cmd_vel_corrected`; its launch configuration sends commands to the motion computer over UDP. The project still has no ROS 2-to-ROS 1 command bridge: `/autonomy/cmd_vel` is consumed only by the isolated project arbiter, whose output has zero subscribers. The vendor safety supervisor is in `FAULT` with `fault_reason="Retroid STOP observed"`, `dynamic_armed=false`, and `posture_armed=false`. This confirms the stop/fault state is visible to the vendor safety layer, but it does not by itself certify autonomous command ownership or independent preemption during motion.

No gate, goal, velocity, vendor service, network route, or motion process was changed by this inspection. The fastest safe completion path is: establish stable AMCL convergence; demonstrate a planner-only goal while output remains isolated; then separately commission a reviewed ROS1 command bridge, exclusive RC/autonomy ownership, watchdog, and independent stop/preemption before any walking test.

## AMCL convergence and planner gate (2026-10-19)

A live API check after the operator set the starting pose reports AMCL ready and converged: pose age 1.41 seconds, `stable_samples=65`, bounded covariance, and `refresh_active=true`. TF and both costmaps are fresh. Navigation reports `planner_ready=true`, `goal_requests_enabled=true`, and the previous action state is `finished_6`; status 6 is the Nav2 canceled result. No motion command was sent.

The motion boundary is unchanged. `/autonomy/cmd_vel_arbited` has one publisher and zero subscribers; `/autonomy/cmd_vel` feeds only the isolated arbiter. The vendor ROS1 `ros2qnx` subscribers and UDP route remain downstream but disconnected from project output. The vendor safety supervisor remains the required stop/fault authority; this session did not enable the gate, add a command bridge, or send velocity.

## Guarded vendor command bridge (source-only, 2026-10-19)

Added `vendor_cmd_vel_bridge.py` and `vendor_cmd_vel_bridge.launch.py` as a separate commissioning component. It is disabled by default and is not included in the navigation launch. When explicitly started, it requires the physical-stop verification parameter, a fresh authenticated stop-relay record, fresh project readiness, no manual override, and a command heartbeat; stale or invalid conditions publish zero and disable the relay. A separate `ros1_bridge` configuration maps `/cmd_vel_corrected` to the vendor ROS1 `ros2qnx` subscriber.

This component has not been built, deployed, enabled, or connected to the robot in this session. The existing isolated arbiter and vendor services are unchanged. Before enabling it, verify the bridge's ROS1/ROS2 topic mapping on the robot and complete the supervised stop-preemption test.

## Guarded bridge staging (2026-10-19)

The new fail-closed vendor bridge source and launch files were staged on `lite` under the project workspace and built successfully in an isolated temporary Foxy install (`/tmp/lite3_bridge_install`). A robot-side backup is under `/home/ysc/lite3_autonomy_ws/backups/20240912_160111/vendor_cmd_bridge_predeploy/`. The component was not installed into the active runtime and was not launched.

The existing odometry parameter bridge owns the global ROS 1 `/topics` configuration. Starting a second parameter bridge without a reviewed combined configuration could replace the live odometry mapping, so the command bridge remains staged only. No gate, bridge, goal, velocity, vendor service, or network setting changed; the active API still reports AMCL converged and motor output isolated.

## Combined bridge deployment (2026-10-19)

Deployed one combined project ROS 1 bridge configuration on `lite` for `/leg_odom2` and `/cmd_vel_corrected`, replacing only the project `parameter_bridge` process. The guarded relay is installed in the active project workspace with `physical_stop_verified=false` and is disabled by default. ROS 1 inspection confirms `/ros_bridge` publishes `/cmd_vel_corrected` to the vendor `/ros2qnx` subscriber, while ROS 2 `/autonomy/cmd_vel_arbited` has one guarded-relay subscriber. `/leg_odom2` still publishes and has five subscribers. Backup: `/home/ysc/lite3_autonomy_ws/backups/20240912_160556/combined_bridge_predeploy/`.

No vendor service, networking, UDP route, gate, goal, velocity, or motion process was changed. The API remains reachable; after the bridge restart its localization refresh is stopped/stale and planner readiness is false, so the UI must restart Localization/Nav2 and confirm fresh AMCL/costmaps before a planner-only goal. Physical output remains disabled.
