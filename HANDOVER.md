# Lite3 Project Handover

## Latest live checkpoint - 2026-10-19

Reconnect with `scripts/Start-Lite3Session.ps1`; do not reuse the address or tunnel port without rediscovery. In the current session the page is `http://192.168.2.39:18080/`. The saved map `v3_soba_ogi_test2` is loaded, Localization/Nav2 is running, and automatic global localization has converged. The final status showed fresh sensors, all required TF, both costmaps, a ready action server, and `planner_ready=true`; the latest no-write readback had 86 stable AMCL samples near `(1.124, 0.083, -104.8 deg)`.

Leave `goal_requests_enabled=false` until the operator physically confirms the blue marker is aligned with the robot. Do not connect or enable the motor route yet: `physical_estop_verified=false`, the guarded relay is disabled, and `motion_output=not_ready` because output remains isolated. There is no active goal. No goal or velocity was sent during this checkpoint.

UI/preflight v23 is deployed with rollback backup `/home/ysc/lite3_autonomy_ws/backups/20261019/preflight23_predeploy_20261019_031912/`. The final read-only preflight completed and confirmed the combined bridge and vendor subscriber. Its short-lived ROS 2 graph probe may report incomplete DDS discovery even when the long-running API has fresh messages; use the API freshness, TF, costmap, and action checks as the live readiness evidence without lowering any threshold.

Remaining before the first supervised walk: (1) compare the displayed pose and heading with the robot in the room; (2) independently commission exclusive manual/autonomy ownership; (3) demonstrate immediate vendor/physical stop preemption through the actual bounded autonomy command path; (4) select a known-free very short goal and explicitly reconfirm the operator is beside the robot with the controller/stop ready. Only after all four are true should planner requests be opted in and the guarded relay be enabled for the bounded test.

## Latest compatibility checkpoint — 2026-10-19

The live API and guarded relay were verified in ROS 2 Foxy-only Python environments with valid `Twist` and `NavigateToPose` type support. The only process using both Noetic and Foxy is the C++ `ros1_bridge`, intentionally. `scripts/start_vendor_cmd_vel_bridge.sh` is deployed and called by the API boot helper; it starts disabled and cannot forward motion by itself.

Current navigation state is not walk-ready: sensors, transforms, costmaps, and the action server are present, but the API's localization refresh is inactive after stale input, so planner readiness is false. Use the explicit localization action in the UI, then re-check AMCL freshness and planner opt-in. Do not remove the physical-stop/arbitration interlocks.

## Vendor STOP log adaptation - 2026-10-19

The known vendor STOP behavior is now watched by the existing motion-host observer. New `Soft Emergency Pressed!!!` or `is_robot_lose_control = 1` lines in the vendor `deeprcs` log generate the existing authenticated STOP relay to perception. The observer still owns the existing `192.168.1.120:43911` source and the receiver remains at `192.168.1.103:43910`; no new port or vendor service was introduced. The deployed file is backed up under `/home/ysc/lite3_autonomy_ws/backups/20261019_074913/vendor_stop_observer_predeploy/` and the systemd observer is active.

The relay is currently fresh and clear. This improves project STOP detection but does not establish an independent physical e-stop or safe motor arbitration. Keep the motion arbiter disabled and output isolated until a supervised stationary event check confirms the safety latch and the separate command-owner/stop-preemption commissioning is complete.

The observer was redeployed once more with a log-rotation race fix (SHA-256 `6b358a7bd33a7025bdd92c09a751132ddff90ea58d7c9a20e4e5a76b2000497b`). The latest rollback backup is `/home/ysc/lite3_autonomy_ws/backups/20261019_075055/vendor_stop_observer_predeploy/`; systemd reports the observer active.

The observer's systemd sandbox was then adjusted from `ProtectHome=true` to `ProtectHome=read-only` so the watcher can read the vendor log. The unit backup is `/home/ysc/lite3_autonomy_ws/backups/20261019_075209/vendor_stop_observer_sandbox_predeploy/`; namespace inspection confirms the current `deeprcs` file is visible to the observer. This change is limited to the project observer and is reversible.

The owner then performed the supervised stationary verification: vendor Stand, followed by vendor STOP. The log event reached the relay, which reported `stop=true`; the perception supervisor latched `FAULT` with `fault_reason="Retroid STOP observed"` and vendor state `1` (belly/resting). This is the first verified end-to-end project STOP event. It still does not certify an independent stop during autonomous motion, so keep the arbiter disabled and vendor `/cmd_vel` disconnected.

A clean ROS 2 dry run confirmed the arbiter topology and fail-closed gate. `/autonomy/cmd_vel_arbited` has no subscriber, and `/autonomy/arbiter/set_enabled` rejected the request because independent physical stop verification is false. No command or motion path was enabled.

## Post-battery reconnect - 2026-10-19

The current tunnel/API session is healthy after the battery change. The saved map, AMCL, Nav2, transforms, and both costmaps are ready, with no active goal. The Retroid STOP receiver and session-only fail-closed safety supervisor are running. The isolated `motion_arbiter` is launched for monitoring only; its output has no subscriber and an enable request was rejected because physical stop verification is false. The vendor `/cmd_vel` route remains disconnected.

The current read-only supervisor check reports `DISARMED`, `stop=false`, fresh zeroed Retroid input and telemetry, but `link_fresh=false`, `stable_stand=false`, and `sdk_feedback_healthy=false`. The API still reports `physical_estop_verified=false` and `motion_output=not_ready`. First verify the Retroid STOP signal while the robot is stationary. Do not enable the arbiter or connect vendor `/cmd_vel` from this state.

The direct safety topic now identifies the exact blockers: vendor basic state `98` (belly/resting), no four-contact feedback or planted confirmation, and a stale EmotionBot link. Battery and attitude are healthy, but these gates must become true through the vendor-supported stand/control path; do not set the parameters from the project or lower thresholds.

The owner-supervised Stand/Sit check reached standing state `6` and then returned to resting state `1`. The Retroid STOP observer never saw `STOP=true`; therefore physical stop preemption is still unverified. Keep the arbiter disabled and vendor `/cmd_vel` disconnected.

The supervisor now remains fail-closed in `FAULT` with reason `robot protection state`, caused by the vendor app STOP action. Do not clear or override this fault from the project. The vendor action is useful evidence, but the missing authenticated Retroid STOP event still blocks autonomy commissioning.

Read-only motion-side packet capture confirms the Retroid exchange is alive, but the vendor STOP press does not emit the project detector's expected `0x21020C0E`; only regular `0x21040001` reply packets were seen. The autonomy interlock therefore needs a vendor-supported stop source or a reviewed protocol update before any velocity bridge is connected.

## Latest motion-path checkpoint - 2026-10-19

The isolated fail-closed `motion_arbiter` is now built and installed in `/home/ysc/lite3_autonomy_ws` after backup `/home/ysc/lite3_autonomy_ws/backups/20261019/motion_arbiter_predeploy_123000/`. It has not been launched or connected to the vendor `/cmd_vel` route. The node defaults disabled, refuses enablement without an explicit physical-stop verification parameter, watches readiness/manual override/heartbeats, and emits zero on faults. Independent vendor mode arbitration and physical stop behavior remain unverified; do not enable the arbiter or test walking from this checkpoint.

The vendor STOP observer/relay chain is active and a session-only ROS 1 safety supervisor is running with `transmit_enabled=false` and `dynamic_actions_enabled=false`. It reports `DISARMED` with fresh zeroed Retroid input and healthy telemetry. This confirms the safety telemetry path is live, not that physical stop preemption has been demonstrated under motion.

The arbiter was rebuilt with an additional `/dev/shm/emotion_bot_lite3_safety_state` gate. It requires a fresh valid authenticated relay record (`stop=false`, `observed_fresh=true`, `axes_zero=true`) before enablement, while still requiring explicit physical-stop verification and project readiness. Revision backup: `/home/ysc/lite3_autonomy_ws/backups/20261019/motion_arbiter_predeploy_124000/`. It remains unlaunched.

## Current session - 2026-10-18, v20 deployed; Nav2 stopped

API v20 has been deployed and the robot API restarted. The laptop URL `http://192.168.2.39:18082/` serves the v20 page (confirmed `app.js?v=20` and `UI_DEPLOY_VERSION = 20`), while the robot API is listening on port 8080 as PID 15939. Deployment used a dated robot-side backup at `/home/ysc/lite3_autonomy_ws/backups/20261018/costmap_goaloptin_predeploy_01/`. Only the project-owned mapping/Nav2/API path was stopped/restarted; vendor services, networking and UDP settings were not changed.

The fresh post-restart API status is fail-closed: camera and point cloud ready; odometry, scan, map, AMCL, transforms, costmaps and Nav2 not ready; map is not loaded; no goal active; planner requests disabled; software STOP latched; physical E-stop unverified; motor output isolated. The API restart cleared its in-memory map and stopped the owned Nav2 stack. Saved map files are expected to remain on the robot. No goal, velocity or movement command was sent.

The v20 fix subscribes to public costmaps as `nav_msgs/OccupancyGrid`, clears AMCL convergence when stable samples are lost, and adds a guarded planner-only opt-in that requires full readiness and active-map match. Reload the page, reconnect, load the saved robot map and start Localization/Nav2 from the UI. Do not lower thresholds. Wait for fresh odometry, scan/map, transforms, stable AMCL, and both costmaps; visually check pose alignment before enabling planner requests. This only enables Nav2 planner requests; motion output remains isolated, so the robot cannot walk through this path. Independent stop and RC/autonomy arbitration still require commissioning.

## Previous session snapshot - before v20 deployment

The laptop tunnel at `http://192.168.2.39:18082/` responds to GETs; the browser is paired. At the latest live status check, camera, odometry, scan, point cloud, active static map, Nav2 process/action server, and required TFs reported ready. AMCL global search was active; its stable sample count was zero. Both costmaps were `not_ready` with no recent messages, so `planner_ready=false`; planner requests were disabled and the motion output remained isolated. The screenshot’s map pose marker is not enough to override those live checks.

ROS graph inspection found `/local_costmap/costmap` and `/global_costmap/costmap` publishing `nav_msgs/msg/OccupancyGrid`, while the project API subscribed to `nav2_msgs/msg/Costmap`. That type mismatch explains the missing API costmap freshness. Local source now subscribes using `OccupancyGrid`, validates grid dimensions/resolution, and adds a planner-request opt-in route/UI control that requires all existing checks and the same active map to pass. It revokes opt-in when readiness drops and on Nav2 stop. AMCL convergence now follows the latest three consecutive stable samples, rather than retaining a previous `converged=true` through a later uncertain search. The physical velocity output remains isolated; no navigation goal or velocity was sent.

API/UI v20 source is local only: it has not been built or deployed. Do not deploy by restarting the API while the current Nav2 stack is active without planning the interruption: the API shutdown path stops its owned Nav2/mapping processes. After a deployment window, restart the saved-map Localization/Nav2 from the UI and recheck AMCL convergence and both costmaps. Physical walking still requires separate command arbitration and verified physical stop commissioning.

## Latest deployed checkpoint - 2026-10-18

The laptop tunnel at `http://192.168.2.39:18082/` serves API v19 and UI v19. The project API source and installed module match SHA-256 `c0b6268a8e5dac7a6826fc5340004ec43cdb23f2e5ac3d6675d571895d59e2f4`; the Nav2 overlay requests periodic full local/global costmaps. The Foxy build succeeded; the installed Python module was synchronized and compiled after the build left the prior installed copy unchanged. A read-only laptop GET confirms API/UI availability and the new `transforms` and `costmaps` status fields. Robot rollback copy: `/home/ysc/lite3_autonomy_ws/backups/20261018/nav_readiness_predeploy_01/`.

Live API status after restarting only the project API: camera and point cloud ready; odometry, scan, map, all required TF checks, costmaps, localization and navigation not ready. The restart cleared the API's in-memory map; saved files remain in the robot webapp maps directory, but none is currently loaded. Mapping and Nav2 are stopped. API is paired; software STOP is latched; goal requests are disabled; `physical_estop_verified=false`; velocity output remains isolated. No bridge, mapping, Nav2, goal, velocity, or vendor process was started or changed. The last preflight identified vendor ROS 1 `/leg_odom2` but no project bridge; ROS 2 data remains absent. This is not ready for a goal or a walk. Next: while the robot is stationary and operator-supervised, restore the narrow odometry bridge and launch the saved-map localization/Nav2 stack, then verify live TF, scan/map, costmaps, AMCL convergence, and pose alignment. Physical motion additionally requires a verified independent stop and exclusive RC/autonomy arbitration.


## Latest after battery reconnect — 2026-10-18

The laptop tunnel is available at `http://192.168.2.39:18082/`; the robot serves API v16 and UI v18. UI files were backed up at `/home/ysc/lite3_autonomy_ws/backups/20261018/ui_v18_predeploy_01/`. The current authenticated API status is: camera/point cloud fresh; odometry/scan/map stale; localization and Nav2 not ready; mapping/Nav2 stopped; goal requests disabled; motion output isolated. The software STOP latch reports not latched after reconnect; the physical E-stop is unverified. Do not attempt a walk.

Read-only preflight found zero ROS 2 publishers on `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose`, with `/odom` absent. A ROS 1 graph check found vendor `/leg_odom2` publisher `/qnx2ros`, but the project bridge `/topics` parameter is unset. The immediate blocker is the stopped narrow project bridge; mapping/Nav2 startup creates it. Preflight backup: `/home/ysc/lite3_autonomy_ws/backups/20261018/preflight_ros1_odom_predeploy_01/`. UI v18 summarizes the difference. Next, start localization/Nav2 only with the robot stationary and supervised, then confirm fresh TF/scan/map, AMCL convergence and costmap health. Commission motor arbitration and a physical stop separately before any walk.

## Historical navigation diagnostics before battery reconnect — 2026-10-18, API/UI v16

The laptop tunnel at `http://192.168.2.39:18082/` serves API/UI version 16. A final read-only status check confirms camera, point cloud and odometry ready; scan/map/localization/Nav2 not ready because the project stack is stopped; planner goals disabled; software STOP latched; and velocity output isolated. The selected saved map `v3_soba_ogi_test2` is 92x91 at 0.05 m/cell.

The API was restarted after a stale HTTP action lock blocked diagnostics; only the project API was restarted. Starting Localization/Nav2 then produced a fresh scan and active map. Standalone TF checks resolved `odom → base_link`, `base_link → camera_scan_frame`, and `base_link → camera_depth_optical_frame`; scan messages use `camera_scan_frame`. Nav2 nevertheless continued logging `odom` and `camera_scan_frame` as unknown; its lifecycle nodes were absent from the ROS node list and lifecycle queries returned “Node not found.” A 90-second no-motion AMCL search produced a candidate pose but failed the covariance/stability gate (latest estimate x=0.688 m, y=-0.067 m, yaw=-1.768 rad; zero stable samples). Nav2 was stopped and software STOP relatched. The last stop response could not confirm the gate-disable service, but the velocity output remains isolated. No goal, velocity, or physical movement was commanded. `physical_estop_verified=false`; project software STOP is not a physical emergency stop. **Do not attempt a walk.**

Robot-side backups: API `/home/ysc/lite3_autonomy_ws/backups/20261018/bridge_reuse_predeploy_01/`, UI `/home/ysc/lite3_autonomy_ws/backups/20261018/amcl_ui_predeploy_01/`, and perception launch `/home/ysc/lite3_autonomy_ws/backups/20261018/camera_tf_predeploy_01/`. Next diagnose why Nav2 cannot see TF/lifecycle nodes despite a separate listener resolving the same frames, then obtain a physically confirmed initial pose or improve map/scan alignment so AMCL converges. Verify the obstacle costmap and compare the pose marker to the real robot. Motor routing, command arbitration, watchdog, obstacle-stop behavior, and an independent physical stop also require commissioning before any walk test.

## Historical snapshot — 2026-10-18 (superseded by the live state above)

The laptop tunnel/API is live at `http://192.168.2.39:18082/` (API/UI v15). Project Nav2 was started against the saved robot map `v3_soba_ogi_test2`. The status reports fresh camera, odometry and scan, active static map server, ready Nav2 action server, and ready AMCL global-localization service. Map size is 92x91 at 5 cm/cell, about 52.9% known.

Two launch defects were fixed and deployed: the perception computer's vendor Foxy `nav2_bringup/localization_launch.py` has its AMCL node commented out, and Foxy's local costmap expects integer `width`/`height`. The project now owns `localization.launch.py`, launching map_server and AMCL under one lifecycle manager. AMCL is configured for `map`, `odom`, `base_link`; the scan frame is project-created as `camera_scan_frame` with a static transform to `base_link`.

The global-localization API request returned successfully and explicitly reported `motion_command_sent=false`, but no fresh AMCL pose arrived. A ROS CLI TF check did not confirm `odom → base_link`; at the endpoint check `/odom` had a publisher but zero subscribers. No goal or velocity command was submitted. Goal requests remain disabled (`goal_requests_enabled=false`), motion output is still isolated, and `physical_estop_verified=false`. Do not try a walk yet.

Backups: launch files at `/home/ysc/lite3_autonomy_ws/backups/20261018/amcl_launch_predeploy_01/`; AMCL frame overlay at `/home/ysc/lite3_autonomy_ws/backups/20261018/amcl_frames_predeploy_01/`; earlier corrected costmap overlay at `/home/ysc/lite3_autonomy_ws/backups/20261018/nav2_readiness_predeploy_01/`. Source/install launch and config hashes were verified where applicable. The active Nav2 stack is left running with output isolated. Next, resolve and verify the odometry TF and fresh AMCL pose, check that the map marker matches the robot physically, then commission the physical E-stop/manual-autonomy arbitration/motor path separately before considering motion. The software STOP button is not an independent hardware stop.

## 2026-10-18 — current navigation blocker

The user reports a new room map was saved and a point can be selected in the UI. The current read-only preflight says this is only a coordinate preview: `/leg_odom2`, `/scan`, `/map`, `/amcl_pose` have zero ROS publishers and `/odom` is absent. Camera/point cloud are present, but the API's cached 92x91 map is stale. Mapping/Nav2 are stopped, global localization and planner/action server are unavailable, goals remain disabled, software STOP is latched, and motion output is isolated. The catalog now contains six entries, including `v3_soba_Ogi_2026` and `v3_soba_ogi_test2`; the API snapshot cannot identify which is the map just saved or show that it is active.

Next engineering step is diagnosis/restoration of the vendor odometry publisher. After that, verify the project odom/TF adapter, depth scan and map production, load the chosen robot-saved map, then verify AMCL pose, Nav2 action server and obstacle costmap. Physical motion remains blocked until command routing/arbitration and an independent physical stop are commissioned. No motion-capable control was called in the latest check.

## Latest deployment checkpoint (2026-10-18)

API/UI version 15 is deployed on the perception computer. The dated predeployment backup is `/home/ysc/lite3_autonomy_ws/backups/20261018/v15_predeploy_20261018_01/`. A clean Foxy package build completed, installed/source API hashes match, Foxy Nav2 action type support passed, and the existing launcher restarted only the project API. The page, version-15 JavaScript, and CSS return HTTP 200 through the live laptop tunnel at `http://192.168.2.39:18082/`.

Current API state: camera and point cloud ready; odometry, scan, map, localization, and stitched map cloud unavailable; mapping and Nav2 stopped; goals disabled; software stop latched; velocity output isolated. STOP is software-only (`physical_estop_verified=false`), not a hardware E-stop. The latest available ROS preflight found no `/leg_odom2`, `/scan`, `/map`, or `/amcl_pose` publishers. Do not attempt autonomous walking. Restore odometry, verify the perception/map/localization pipeline and obstacle costmap, and commission manual/autonomy arbitration plus an independent physical stop first.

Only the project API process was restarted. Vendor services, startup cron, routes, UDP ports, and ROS parameters were not modified. No control endpoint, mapping/Nav2 launch, goal, velocity, or STOP action was invoked. Browser GET checks confirmed the deployed assets and status; visual Chrome/Retroid rendering remains for the user to confirm.

Last updated: 2026-10-18 (Europe/Skopje)

This handoff was first prepared after the robot was passed to another operator on 2026-10-01. The current live state and recent deployment are recorded in the final dated subsections below; older dated sections are historical and superseded where they conflict. Always recheck live network/software state before acting.

## Goal

Build a browser workflow for manual RealSense room mapping and review with a Jueying Lite3 Pro, then evaluate navigation only after safe localization, obstacle, and command arbitration are commissioned. Physical autonomous walking is not commissioned; do not describe the project as a finished autonomous walking product.

### Previous predeployment session snapshot (2026-10-18; superseded below)

**After battery replacement:** the user ran the startup helper on port 18082. It discovered the perception host and reached the API, then correctly refused UI version 15 because the deployed backend lacks the global-localization and software-stop fields. A current read-only API GET now succeeds: camera/cloud fresh; odometry, scan, map, localization and stitched cloud unavailable; mapping/Nav2 stopped; goals disabled; output isolated. SSH shell access timed out during banner exchange. Nothing was deployed or started. Do not attempt the walk test from this state.

**Local follow-up:** source version 15 adds a software STOP latch, `/api/v1/safety/stop`, and an always-available UI control after browser pairing. The action requests gate disable, cancels any accepted goal, stops project Nav2, and blocks later goals until localization/Nav2 is explicitly restarted. This is not a physical E-stop: `/autonomy/cmd_vel` remains isolated from the vendor motor path, and no independent stop/interlock has been verified. Local source is not built/deployed. The laptop tunnel probe to `127.0.0.1:18081` now fails, so there is no current live robot verification.

The latest robot preflight available from the preceding session found camera publishers but no `/leg_odom2`, `/scan`, `/map`, or `/amcl_pose` publishers. `/leg_odom2` is the upstream blocker; the project launch can create the odometry adapter, scan and map only once a fresh vendor odometry stream arrives. `/amcl_pose` requires the later localization/Nav2 launch and convergence. Do not fake data or alter vendor transfer services/UDP ownership to make the graph look ready.

Latest connection follow-up: the laptop is on the robot SSID at `192.168.2.39` with gateway `192.168.2.1`. An older SSH process still occupies port 18080 but does not have a working final connection. The visible launcher established a separate port-18081 tunnel by discovering the perception address through YSC. The API returned HTTP 200 status: camera/cloud fresh; odometry, scan, and map about 14 minutes stale; mapping/Nav2 stopped; goal requests disabled; motion output isolated. Five saved maps exist. The API lacks the new `global_localization` status capability, so the launcher correctly refused to copy UI v14. The active tunnel is left open. This was not a robot-side process/service inventory.

The authenticated read-only API preflight then completed successfully. Camera image and cloud publish; `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose` have no publishers. `/odom` and all ROS 2 velocity topics are absent. The vendor ROS 1 `/cmd_vel` subscriber exists without a publisher; `/commands/joy_raw` has a publisher but no subscriber; the ROS 1 bridge `/topics` parameter is unset; UDP 43897 is bound by an unidentified process. Mapping/Nav2 are stopped and goal requests are disabled. This confirms that AMCL cannot localize and Nav2 cannot plan from the current graph. The API reported a cached 86x91 map at ~49% known, but it is stale. No process/service or ROS state was changed by preflight.

The user's no-start-pose workflow is implemented in local source as a Foxy AMCL global relocalization request, pose marker, and exact saved-map name match for goal submission. AMCL global localization searches map free space; it can be ambiguous on incomplete/repetitive maps, so the marker must be checked against the physical room before selecting a goal. Local UI deployment version 14 and the backend route are **not deployed or built**. The last successful API snapshot had stale odometry/scan/map data and showed mapping/Nav2 stopped; a new GET through `127.0.0.1:18080` failed because no tunnel listener was available in this session.

The user confirms they are beside the robot and authorizes a physical goal test. That authorization does not establish readiness or a safe motor path: Nav2 goal requests remain off by default, `/autonomy/cmd_vel` is disconnected, vendor command ownership/Manual-Autonomous arbitration is unresolved, and there is no independently verified emergency stop path. **Do not connect or enable motor output or submit a walking goal until those items are resolved.** Browser cancel and shutdown buttons do not constitute a physical emergency stop. Do not use the old preflight snapshot as evidence of current sensor or service health.

## Previous API status before version 15 deployment (2026-10-18)

The user reports the laptop startup terminal/tunnel is running. Read-only laptop requests through its local tunnel confirm the project API is reachable: status, saved-map catalog, and live camera-cloud endpoints return HTTP 200. The status reports browser paired and manual mode; camera/live point cloud ready; odometry, scan, map, localization and stitched map-frame cloud not ready; mapping/Nav2 stopped; goal requests disabled; motion output isolated. The grid and stitched-cloud endpoints return HTTP 503 because live map/TF data are absent. No control endpoint was called and no motion was commanded. This API check does not freshly verify vendor service state, UDP ownership, command arbitration, or Retroid access.

The current UI provides saved-map selection, map click-to-coordinate preview, initial-pose selection, and gated Nav2 goal/cancel actions. Local UI version 14 adds a no-start-pose AMCL global-localization request, displays its fresh pose estimate on the map, and requires the clicked image name to match the selected robot map. After relocalization, verify the marker before choosing a known-free destination. Depth obstacle avoidance requires a ready sensor costmap. The existing port-18080 tunnel still serves the older UI. The fresh SSH attempt stopped at interactive authentication; the provided password input closed the connection before deployment, so no robot control or service was changed. Physical navigation remains uncommissioned. The appropriate sequence is manual scan → stop/save → review raw 2D map and registered 3D cloud → localize on saved map → verify depth obstacle layer and command arbitration → only then perform a separately authorized supervised motion test. Point-cloud filtering should be reversible and review-only; do not force the occupancy map into rectangular geometry.

## UI and cleanup implementation

The 3D browser viewer now has default display-only filtering for far live-camera points and isolated voxels, plus a raw-view toggle. `scripts/filter_map_candidate.py` writes a separate conservative map candidate by changing isolated occupied cells to unknown; it preserves free cells, unknown regions, and map geometry and refuses source/output overwrites. The UI text now describes map building, saving, localization/Nav2, map-click destination selection, and planner submission. The Windows startup launcher detects UI version 14, backs up, and deploys the HTML, JS, and CSS together only if the API status advertises its matching global-localization capability. The launcher never deploys/builds the Python API. These local changes have not been deployed or visually checked. Current API state is not freshly available; use only the last-read snapshot as historical evidence.

### Historical robot-side snapshot (2026-10-06)

The laptop authenticated to YSC, then to lite. The current local SSH tunnel reaches the robot-hosted API. On lite, vendor transfer/RealSense services and the project API are active. Camera image and point cloud publish; `/leg_odom2`, `/scan`, and `/map` currently have no publishers. Mapping and Nav2 are stopped; Nav2 goal requests are disabled; motor output is isolated. ROS 1 command routing remains unresolved and UDP 43897 has an unidentified listener. The read-only preflight changed no service, process, parameter, ROS message, or motion state.

The updated API/UI is deployed in the project workspace. Through the laptop tunnel, session, status, map catalog, live camera cloud, page, JavaScript, and CSS return HTTP 200. Live grid and stitched map-frame cloud return HTTP 503 until live map/TF data exists. The UI/API deployment backup is `/home/ysc/lite3_autonomy_ws/backups/20261006/live_ui_predeploy_01/`; the shutdown-button deployment backup is `/home/ysc/lite3_autonomy_ws/backups/20261006/poweroff_button_predeploy_01/`. The package was clean-built under Foxy and only the project API was restarted. Browser rendering and Retroid tunnel access remain unverified because no Chrome surface is exposed to this agent. Use only trusted devices on robot Wi-Fi: automatic browser sessions grant 12-hour control to each browser that can reach the robot-hosted UI. Never put the robot token or password in repo/chat. See [STATUS.md](STATUS.md), [GROUND_TRUTH.md](GROUND_TRUTH.md), and [docs/WEB_APP_API.md](docs/WEB_APP_API.md).

The current UI also has a deliberate battery-change shutdown action. It requires typing `STOP PROJECT`, serializes with other control requests, cancels any active accepted planner goal, stops only project-owned mapping/Nav2/odometry bridge work, then closes the project API. An invalid confirmation was rejected with HTTP 400 and did not stop the API. Vendor services stayed active. It does not power off the Linux computer or close the laptop-side SSH tunnel; those are separate power-off steps.

## First startup attempt of this session (2026-10-18)

The user reports OpenSSH authenticated successfully to the robot Wi-Fi gateway `192.168.2.1`, but the gateway could not resolve the final SSH destination `lite` (`Temporary failure in name resolution`). The perception computer/API was not reached by that attempt. The launcher was changed to read the current target from YSC's `~/jy_exe/conf/network.toml` rather than assume `lite`; the later laptop API GET confirms the current tunnel now reaches the perception API. No service or robot configuration was changed in response to the failed tunnel.

## Repeatable startup work (2026-10-18; local source only)

Added a Windows laptop launcher at `scripts/Start-Lite3Session.ps1` and a **Connect & check status** button in the web UI. The launcher derives the current Wi-Fi address and default gateway at runtime, queries YSC's `~/jy_exe/conf/network.toml` for the current `ip` value, starts a visible SSH ProxyJump tunnel to that address, and polls the robot API's read-only status endpoint. It no longer assumes the perception host resolves as `lite` or asks the operator to select an IP. If the robot serves an older UI (deployment version below 12), it backs up and copies the local `index.html`, `app.js`, and `styles.css`; it does not restart the API. `-PerceptionHost` remains an explicit fallback override. Passwords and any host-key prompt remain interactive; credentials are not saved. The UI button obtains a browser session and refreshes all component status over GET only; it starts/stops no robot process.

The latest static UI changes remain local unless separately deployed; the successful read-only API GET does not prove auto-deployment or UI asset freshness. New-location boot, browser visuals, and Retroid access through the tunnel remain unverified. The launcher queries YSC configuration for the target address instead of relying on `lite`; keep it interactive and do not store credentials. The UI cannot verify the Lite3 vendor controller link; check that in the vendor app.

## Current state at handoff

### Verified during earlier sessions

- The robot has separate motion and perception computers. SSH observations are recorded in [GROUND_TRUTH.md](GROUND_TRUTH.md); its network details are private project information.
- The perception computer ran ROS 2 Foxy, the RealSense camera, and robot vendor services. ROS 1 Noetic services are also part of the vendor system.
- A selected-topic ROS 1 bridge for `/leg_odom2` and the local odometry/TF adapter provided odometry for sensor mapping. `/scan` was observed around 28 Hz and a transient-local `/map` was retrieved.
- The user manually guided the dog during previous room scans. No movement command was sent by the assistant.
- The Retroid successfully opened the laptop-hosted web app in Chrome and loaded a map preview.
- A project workspace is built on the robot at `/home/ysc/lite3_autonomy_ws`; the 2026-10-06 clean Foxy build includes the current browser session and 2D/3D UI work.
- A user `@reboot` cron entry is installed on the perception computer. It waits 45 seconds and runs `/home/ysc/lite3_autonomy_ws/scripts/start_robot_api.sh`. The current API is active after a project-only restart. Reboot-to-browser behavior is still not freshly verified.

### Historical pairing status (superseded)

Earlier dated handoff notes below record the pairing implementation as local-only. That status was superseded by the 2026-10-06 deployment: the robot now issues a same-origin, HttpOnly browser session on page load, and the refreshed UI does not require the user to paste an API key. See the current verified state above and the latest ground-truth/debug-log entries.

At the initial 2026-10-01 handoff, the laptop could not verify TCP/22 reachability to the perception host. The latest laptop-side check is recorded below; it still has not reached either robot computer.

### New-location check (2026-10-03)

- User reports the robot/perception computer is powered on. Windows Settings shows the laptop on the robot Wi-Fi SSID; the user reports the Retroid Pocket 3 Plus YZX is on that same SSID. The SSID and IP addresses are omitted from this repository. Both client devices have addresses on the same private `/24` network and report the same gateway. The controller app and Chrome have not been opened during this session.
- From the laptop, one ICMP probe succeeded to the Wi-Fi gateway and Retroid. A one-probe discovery across the connected `/24` found no other responding host. Limited TCP checks of port 22 and the documented API port 8080 across that subnet, including the gateway, found no listener. This does not prove the robot is offline; its computers may use different addresses, disable these services, or be isolated from Wi-Fi clients.
- Local name resolution returned `ysc.local` on the internal private subnet. Laptop SSH to `ysc` did not connect, but the user's verbose SSH command from `lite` to `ysc.local` succeeded and returned `hostname: ysc`, interface addresses, and routes. This proves the internal `lite` â†’ `ysc` SSH path works, not laptop reachability.
- A non-interactive `ssh ysc 'hostname'` attempt using the laptop's default OpenSSH configuration also failed while opening the connection, before authentication. No password was entered or requested; the laptop has no configured SSH alias for `ysc`.
- User accessed `lite` by HDMI/local terminal, then SSHed from `lite` to `ysc`. The reported `ysc` output shows `p2p0` on the robot Wi-Fi subnet (the gateway seen by the laptop) and `eth1` on the internal subnet shared with `lite`; `wlan0` is down. The expected `~/jy_exe/conf/network.toml` path on `lite` was absent. See the clearly labeled source sections in [GROUND_TRUTH.md](GROUND_TRUTH.md).
- Laptop-to-perception TCP/22 checks to the reported address failed on both laptop interfaces. The current evidence points to a routing/topology gap: the laptop is on robot Wi-Fi, while perception is on the internal Ethernet subnet and has no default route. This remains an inference until the motion computer's interfaces and routes are read.
- The laptop's static preview server is not running. The Windows Python launcher reports no installed Python interpreter, so the guide's Python-based local preview fallback was not started. This does not establish anything about the robot-hosted API.
- Next safe steps from the current `lite` terminal: run `bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh` (read-only), then query the motion host's SSH listeners with `ssh ysc@ysc.local 'ss -ltn'`. Share those outputs. This will establish the perception software state and whether SSH listens on `ysc`'s robot-Wi-Fi interface. Do not alter routes, services, or robot networking during discovery.

### Mapping quality

The saved map pairs are useful for demonstrating the preview but are not cleared for navigation:

- The larger saved scan is `131 x 181` cells at `0.05 m/cell` (`6.55 x 9.05 m`), with `74.8%` unknown cells.
- The older scan is `89 x 99` cells at `0.05 m/cell`, with `63.3%` unknown cells and fragmented returns.
- Offline reports are under `docs/map_reports/` if present; `scripts/map_quality_report.py` can analyze a new PGM/YAML pair. Its percentages are not a navigation-safety verdict.

## Critical robot and navigation blockers

1. **Do not disturb vendor services or ports casually.** The recorded boot state had both Noetic and Foxy transfer stacks active. Their motion receivers contend for UDP `43897`; the ROS 2 receiver exits with `Address already in use`. Noetic services depend on their transfer path. Stopping services or changing ports could break the vendor controller.
2. **Manual command routing is unknown.** Recorded ROS graphs showed joystick data on `/commands/joy_raw`, no confirmed subscriber consuming it, a Noetic `/ros2qnx` subscriber on `/cmd_vel` with no publisher, and no verified route from this project to the motors. Preserve the original Lite3 controller app.
3. **No safe autonomy arbitration exists yet.** `/autonomy/cmd_vel` is an isolated, closed-by-default output with conservative limits. It is not connected to the vendor motor path. There is no validated exclusive Manual/Autonomous switch, readiness publisher, or stop-on-loss behavior.
4. **Nav2 has not been commissioned.** The vendor Nav2 configuration expects LiDAR (`/rslidar_points`), while this robot setup is being developed around a RealSense camera. A project depth-camera overlay and launch path exist, but obstacle behavior/localization have not been validated live.
5. **The maps need a better room scan.** Existing maps are sparse/distorted and mostly unknown. Do not test navigation on them.
6. **A no-shell browser workflow is partly verified.** The API responds through the active laptop SSH tunnel and automatically creates a browser session. Retroid access through that tunnel and reboot-to-browser behavior remain unverified. The API is plain HTTP and should be used only by trusted clients on robot Wi-Fi.

## Safe first session for the next operator

1. Read this file, [Lite3_New_Location_Setup_Guide.md](Lite3_New_Location_Setup_Guide.md), [STATUS.md](STATUS.md), the latest sections of [GROUND_TRUTH.md](GROUND_TRUTH.md), and [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md).
2. Establish with the robot owner whether the dog is stationary and whether they authorize any physical movement. Until explicit authorization, treat all work as read-only. Do not publish velocity, click a navigation goal, enable the gate, start Nav2 in a mode that can reach motors, or alter vendor services/ports.
3. Confirm laptop/controller Wi-Fi and current robot IPs from the device/network itself. Do not assume recorded addresses are current. Check the perception host/API and cron logs without changing services if reachable.
4. For any future deployment, compare local and robot source first. The current UI/API is deployed in the project workspace; preserve owner changes and do not overwrite robot/vendor workspaces.
5. Before changing robot runtime, use the read-only checklist in `scripts/robot_preflight.sh` (robot copy: `~/lite3_autonomy_ws/scripts/robot_preflight.sh`) and record output without secrets. The script starts/stops no ROS processes and publishes no messages.
6. Continue useful software/documentation work while the robot state or authorization is unavailable. Keep README, STATUS, GROUND_TRUTH, and DEBUG_LOG aligned with what was actually checked.

## Useful paths and commands

- Local ROS package: `ros2_ws/src/lite3_odom_tf_adapter/`
- Local browser application: `webapp/`
- Robot project workspace observed earlier: `/home/ysc/lite3_autonomy_ws`
- Read-only robot snapshot: `bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh`
- Foxy project build (on the perception computer, after confirming source/workspace):

  ```bash
  source /opt/ros/foxy/setup.bash
  cd ~/lite3_autonomy_ws
  colcon build --packages-select lite3_odom_tf_adapter
  ```

Do not treat a successful build as evidence that the API is reachable, the map is suitable, Nav2 works, or the robot can safely walk.

## Handoff docs

- [Copy-ready Codex prompt](docs/CODEX_HANDOVER_PROMPT.md)
- [Project status](STATUS.md)
- [Verified observations](GROUND_TRUTH.md)
- [Debug history](docs/DEBUG_LOG.md)
- [Web/API contract and caveats](docs/WEB_APP_API.md)
- [Larger-room mapping checklist](docs/TOMORROW_SCAN_CHECKLIST.md)

## GitHub note

This handoff is prepared for the provided public GitHub repository. Historical private LAN addresses in Markdown have been replaced by host labels; ask the robot owner for current addresses. Never commit passwords, API tokens, pairing keys, private keys, or unreviewed logs.

### Latest robot snapshot (2026-10-03)

The user's read-only `lite` preflight and YSC terminal screenshots supersede earlier “preflight not yet run” and “YSC listener unknown” notes. RealSense publishes image/point-cloud topics, but odometry, scan, map, localization, Nav2, and autonomy velocity topics are absent. The project API on localhost:8080 is stopped/refusing connections. UDP 43897 is bound with unknown owner. YSC SSH listens on wildcard IPv4/IPv6, forwarding is enabled, and its return route to the laptop uses the Wi-Fi interface. A TCP probe from this session was blocked by the local sandbox before connection; laptop SSH/authentication remains unresolved. Read-only checks changed no runtime or motion state. The first-browser pairing code remains undeployed.

### Laptop reachability follow-up (2026-10-03)

A single laptop TCP connection to YSC's confirmed Wi-Fi SSH port 22 succeeded after the execution was allowed network access. Laptop-to-YSC TCP reachability is verified; SSH host-key verification/authentication still needs a safe check. The project API remains stopped on `lite`, and mapping/navigation inputs are absent. No robot software or motion state changed.

### Laptop SSH login confirmed (2026-10-03)

The user's screenshot confirms PowerShell SSH login to YSC succeeded after the user accepted its ED25519 host key and entered credentials. YSC showed an Ubuntu 20.04.5 LTS login banner and roughly four minutes of uptime at capture time. This supersedes earlier SSH-authentication-unverified statements. The project execution environment has no configured robot credentials, so its separate non-interactive read-only SSH attempt failed authentication; no password was handled by the assistant. Continue with read-only checks in the user's authenticated terminal.

### Post-battery-replacement check (2026-10-03)

The user ran the read-only preflight through the laptop → YSC → `lite` SSH path after the battery replacement. It matches the earlier sensor-only snapshot: RealSense publishes, while odometry/map/scan/Nav2/autonomy and the API remain absent/stopped; UDP 43897 has unknown owner. No runtime or motion state changed. Continue read-only diagnosis; the browser API workflow is still unavailable.

### API autostart diagnostic (2026-10-03)

The user confirmed the `@reboot` crontab entry is installed and directs output to `~/lite3_autonomy_ws/logs/robot_api_autostart.log`. The API was not listening in the post-reboot preflight, so startup success is still unverified. A broad `/var/log/syslog` search was interrupted by the user with Ctrl+C; this only stopped the diagnostic command. No robot service or runtime state was changed. Check the specific API autostart log with a bounded tail next.

### API startup traceback (2026-10-03)

The user supplied the targeted autostart log output. The cron launcher runs, but `robot_api` crashes while creating the `NavigateToPose` ActionClient: Foxy type-support lookup raises `AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'`, followed by a partial-client destructor warning. Port 8080 has no listener. Likely a ROS Python/generated action type mismatch; exact cause remains unverified. A local fail-closed mitigation was made in `robot_api.py` so invalid Nav2 type support is logged, Nav2 reports unavailable, and the API can continue non-navigation work. It is unbuilt and undeployed. No robot runtime or source was changed.

### New session (2026-10-04)

The user reports all devices are reconnected. Current Windows state shows laptop Wi-Fi is disconnected; Ethernet is up on a separate LAN. No robot network endpoint was probed because the prior gateway may no longer be current and is not routed through this Ethernet connection. Yesterday's confirmed YSC login and API startup failure remain the latest robot-side observations, not a fresh check for today. Reconnect the laptop to robot Wi-Fi, then verify its active SSID/gateway and repeat checks. No settings or robot state changed.

### Reconnection recheck (2026-10-04)

The laptop Wi-Fi adapter is now up and has an active gateway. Windows could not independently return the current SSID. The gateway accepts TCP/22; authentication was not tested. Gateway port 8080 refuses connections, which does not verify the API because the API runs on `lite`, not YSC. Current `lite` IP/status, preflight, and API state remain unchecked in this session because no robot SSH credentials are configured in the assistant environment. Next run the read-only preflight and inspect the targeted API log through the authenticated laptop → YSC → `lite` SSH path. No robot or network settings changed.

### Assistant-session SSH attempt (2026-10-04)

A batch-mode read-only SSH query to the then-current gateway failed authentication (`Permission denied (publickey,password)`) despite TCP/22 being reachable. No password was tried; the password value supplied in chat was incomplete. This does not indicate an SSH/network service failure. The user may run `ssh ysc@ROBOT_WIFI_GATEWAY "hostname; ip -br addr"` locally and enter the password at the terminal prompt. Robot-side preflight/API remained unverified at that time.

### Current YSC interfaces (2026-10-04)

The user reports `hostname` and `ip -br addr` from a successful laptop SSH session. YSC is up; internal `eth1` and Wi-Fi-facing `p2p0` are both up, `wlan0`/`eth0` down, and most CAN interfaces up. This verifies the motion host and SSH login path, not current perception/API health. Assistant-side SSH still lacks authentication. Next resolve the current `lite` address from YSC's internal link, then run preflight and inspect the API log. No configuration changed.

### Live check and root cause (2026-10-04)

The assistant authenticated through YSC to lite and reran the read-only preflight. Vendor `transfer`, `transfer_ros2`, and `realsense_ros2` are active; camera image/point cloud publish. Odometry, scan, map, localization, Nav2, and autonomy velocity topics remain absent. API process/listener are absent; the startup log has the action type-support traceback. UDP 43897 owner remains unknown and lite's clock is still 2024-09-12. No services or processes were started/stopped and no ROS messages/configuration changed.

Read-only testing in a fresh environment isolated the crash: Foxy alone imports Foxy `geometry_msgs`; sourcing the current project install then adds Noetic `geometry_msgs` to `PYTHONPATH` while `nav2_msgs` remains Foxy, reproducing the missing `_TYPE_SUPPORT`. The current install setup is mixed-distribution. Build a clean Foxy project workspace separately and compare before replacing the current install. Do not launch API/Nav2 during verification.

## Clean Foxy rebuild and launcher state (2026-10-04)

This section supersedes the older new-location/connectivity steps above. Laptop SSH through YSC to `lite` was authenticated and verified. A fresh read-only preflight found the three vendor transfer/RealSense services active and RealSense image/point-cloud topics publishing; `/leg_odom2`, `/odom`, `/scan`, `/map`, localization, Nav2, and autonomy velocity topics are absent. UDP 43897 is bound with owner unknown. The API is stopped and port 8080 is not listening.

The API startup traceback was traced to the active project install mixing Noetic `geometry_msgs` with Foxy `nav2_msgs`. The active robot package was rebuilt cleanly under Foxy; its generated setup contains no Noetic path, and the `NavigateToPose` type-support check passes. The previous install and launcher were saved at `~/lite3_autonomy_ws/backups/20261004/`. The robot launcher was updated to keep Nav2 goal requests disabled. API startup was not attempted after the rebuild. The later “Pairing source deployed” subsection records the subsequent deployment.

Next: perform an owner-supervised stationary API-only check with goal requests disabled, then confirm browser reachability and pairing from laptop and Retroid. Restore and verify the odometry-only `/leg_odom2` bridge before mapping. Keep navigation blocked pending map quality, localization/obstacle checks, UDP ownership, and manual/autonomous arbitration.


### Pairing source deployed (2026-10-04)

Compared local and robot package/web manifests. Only `robot_api.py`, `webapp/app.js`, and `webapp/index.html` differed in the pairing implementation; other package source files and web assets matched. Backed up those three robot files under `~/lite3_autonomy_ws/backups/20261004/pairing_predeploy/`, copied the local versions into the project workspace, and rebuilt `lite3_odom_tf_adapter`. The first incremental build left the future-dated installed Python module stale, so the package-local build/install outputs were preserved separately and rebuilt cleanly. The installed module hash now matches source; Foxy imports and `NavigateToPose` type support pass, and setup files contain no Noetic paths. The API process/port were subsequently started for the supervised UI check; see the following section.

### Supervised API/UI check (2026-10-04)

The owner confirmed the robot is stationary in its belly-ready position, is present beside it, and has the DEEP Robotics controller running. Fresh preflight: vendor transfer/RealSense services active; camera topics publishing; `/leg_odom2` exists in the graph with zero publishers; `/odom`, `/scan`, `/map`, localization, and velocity topics are absent. UDP 43897 owner remains unknown. Started only `scripts/start_robot_api.sh`; it logged `Nav2 goal opt-in=False` and the motion output remains isolated. No mapping/Nav2/bridge/motion process was started and no mapping or navigation control was pressed.

Verified through a temporary SSH tunnel bound to the laptop's robot-Wi-Fi interface: `/api/v1/status`, `/`, `/app.js`, `/styles.css`, and `/api/v1/maps` return successfully. Status reports camera ready, odometry/scan/map not ready, mapping/navigation not running, goal requests disabled, and `api_paired=true`. The robot already has a token, so the first-claim Pair button is expected to stay hidden. Chrome rendering, whether the browser already has the saved token, and Retroid reachability remain unverified. Do not share the token; if needed, retrieve it locally and enter it only in the trusted browser. Keep this supervised session stationary and avoid mapping/Nav2 controls.

### Browser check and battery swap (2026-10-04)

The user confirmed the UI and per-topic status load on the Retroid over robot Wi-Fi. Laptop Chrome reported `ERR_SSL_PROTOCOL_ERROR` for the laptop address; the API is plain HTTP, so retry with the explicit `http://` scheme after reconnecting the tunnel. The API was stopped and both SSH sessions closed in preparation for an urgent battery replacement. No vendor services or network settings were changed. After the swap and boot, reconnect and verify API autostart (cron normally waits about 45 seconds) before opening a fresh tunnel; continue to keep mapping/navigation controls untouched.

### Post-swap reconnect (2026-10-04)

Laptop Wi-Fi is again on the robot network; YSC SSH is reachable at the current Wi-Fi gateway, with the perception host reachable through its internal address. The vendor transfer/RealSense services remain active. The project API autostarted and listens on port 8080. Read-only status reports camera ready, odometry/scan/map/localization not ready, mapping and navigation stopped, and goal requests disabled. Through a fresh SSH tunnel bound to the laptop Wi-Fi interface, `/api/v1/status`, `/`, `/app.js`, `/styles.css`, and `/api/v1/maps` all return HTTP 200. Use the laptop's current Wi-Fi address at local port 18080 in Chrome. No mapping, bridge, Nav2, or motion process was started.

### Supervised mapping start (2026-10-04)

At the user's explicit request to start mapping, invoked the API's authenticated guarded mapping-start route. It started the project's odometry-only `parameter_bridge`, received fresh `/leg_odom2`, and launched `mapping.launch.py` (depth-to-scan plus SLAM). Immediate API status showed camera, odometry, scan, map, and mapping ready; localization and Nav2 stopped; navigation goals disabled; motion output isolated. An initial partial map was 23 x 38 cells at 0.05 m/cell (about 47.9% known) shortly after launch. This is a live scan, not a saved/quality-reviewed map. No movement or velocity command was sent; the room must be scanned by the user using the vendor RC in manual mode. Keep supervising, watch the live map for distortion, stop mapping if it breaks, and stop/save under a new unique name when finished. Do not enable Nav2 or autonomous motion.

The user then reported moving the robot through the room but could not see the map in the browser. A fresh read-only status still showed mapping active, camera/odometry/scan/map fresh, and the current map at 94 x 104 cells (0.05 m/cell). The UI requires the robot's existing API key for control POSTs, including map save; `api_paired=true` means do not claim a new pairing. The current UI only previews saved maps: `/api/v1/map` returns metadata, not live grid cells. Saving a snapshot from the UI auto-loads that saved map for review. Do not rebuild/restart the API while the current scan is active; implement live map display after preserving/stopping the current scan.

To give the user a view without waiting for browser automation, saved a review snapshot through the authenticated robot API as `room_scan_review_20261004_01` (94 x 104 cells). `/api/v1/maps` confirms it is available; mapping remains active and scan/map/odometry are still fresh. Laptop Chrome is not exposed through the available computer-use browser surface, so the existing API key could not be typed into its field from this session. The snapshot itself is readable from the browser without the key: reload the page, choose the review snapshot, and load the hosted map. For buttons, the key still must be entered in the browser locally; do not share it in chat. Later save the final scan under a different unique name after stopping mapping.

### Mapping stopped at user request (2026-10-04)

Called the authenticated `/api/v1/mapping/stop` API. It returned success; follow-up status shows mapping stopped, with camera still ready and the last 94 x 104 map metadata retained. The odometry bridge and mapping/scan processes exited; `transfer.service`, `transfer_ros2.service`, and `realsense_ros2.service` remain active. The saved `room_scan_review_20261004_01` snapshot remains in the map catalog. Browser pairing/controls and live 3D cloud display are drawing-board items. The current UI previews saved 2D maps; it does not provide live map-cell rendering, 3D accumulation, or browser teleoperation. Preserve the vendor manual controller and keep Nav2/motor routing disabled.

## Current live check and UI work (2026-10-06)

The user reports the robot was powered on and Wi-Fi configured. From this laptop, the existing local SSH tunnel on port 18080 answers the robot-hosted UI/API. Password-authenticated SSH to YSC and onward to lite succeeded. Fresh `robot_preflight.sh` on lite confirms the vendor transfer/RealSense services are active, the RealSense image and point cloud publish, and the project API process is listening on port 8080. API status reports camera ready, `/leg_odom2`/scan/map not fresh, mapping and Nav2 stopped, navigation goal requests disabled, and motor output isolated. The preflight shows no `/leg_odom2`, `/scan`, or `/map` publishers, no ROS 2 velocity topics, an unidentified UDP 43897 owner, ROS 1 `/cmd_vel` subscribed by `/ros2qnx` with no publisher, and ROS 1 `/commands/joy_raw` published by `/nx2app` with no subscriber. No ROS process/service was started or stopped by the preflight; no motion was commanded.

The updated UI/API is now deployed and running. The old project files and package build/install output were preserved under `/home/ysc/lite3_autonomy_ws/backups/20261006/live_ui_predeploy_01/`. The package built under a clean Foxy environment; the installed Python module matches source, `tf2_ros` and Nav2 action type support import, and generated setup files contain no Noetic path. Only the project API process group was stopped/restarted. Vendor services remain active.

Through the laptop tunnel, session, status, map catalog, live camera cloud, page, JavaScript, and CSS return HTTP 200. Live 2D grid and stitched map-frame cloud return HTTP 503 because odometry, scan, map, and map-to-camera TF are not currently fresh. Fresh preflight confirms mapping and Nav2 remain stopped, Nav2 goal requests disabled, and motor output isolated. Browser rendering and Retroid tunnel access remain unverified because the browser surface is unavailable. Keep Nav2 goals disabled. Pausing the mapping process does not stop manual robot movement; hold the robot stationary while paused. Use auto-session controls only from trusted devices on robot Wi-Fi because each browser that can reach the service receives a 12-hour control cookie.
# Current handoff — 2026-10-18

API/UI version 16 is deployed through the active laptop tunnel `http://192.168.2.39:18082/`. The latest source/install API hash is `b14719e6f7a5de5378e7e633f7e24032d5efa28005dd50e63da25ba8434724e5`, and the page serves UI marker 16. The API includes bounded AMCL no-motion refresh and conservative stable-pose readiness; it also reuses the sole preexisting ROS 1 bridge only when `/topics` exactly matches the narrow `/leg_odom2` config.

The saved map is `v3_soba_ogi_test2` (92x91, 0.05 m/cell, about 52.9% known). Global localization produced poses but failed the three-consistent-sample/covariance gate after 90 seconds. During the next Nav2 restart, the map server did not provide a fresh map; launch logs reported `map` and `camera_depth_optical_frame` unknown. A standard RealSense optical-frame transform is in the project launch and its process started, but the costmap continued to drop those point clouds. The unusable project Nav2 stack was stopped; the project software STOP is now latched.

Current API status: camera, point cloud and odometry ready; scan/map/localization/Nav2 not ready; goal requests disabled; output `/autonomy/cmd_vel` isolated; physical E-stop unverified. No navigation goal or velocity was submitted. **Do not attempt autonomous walking.** The essential remaining technical work is to make the active map server and TF graph visible to Nav2 and verify both `odom → base_link` and `base_link → camera_depth_optical_frame`; then rerun AMCL and confirm the estimated marker against the real robot. Before any physical goal test, separately commission motor routing, RC/autonomy arbitration, watchdog, obstacle-stop behavior and an independent physical stop.

Robot-side backups:

- API: `/home/ysc/lite3_autonomy_ws/backups/20261018/bridge_reuse_predeploy_01/`
- UI: `/home/ysc/lite3_autonomy_ws/backups/20261018/amcl_ui_predeploy_01/`
- Perception launch: `/home/ysc/lite3_autonomy_ws/backups/20261018/camera_tf_predeploy_01/`

See [STATUS.md](STATUS.md), [GROUND_TRUTH.md](GROUND_TRUTH.md), and the last session in [docs/DEBUG_LOG.md](docs/DEBUG_LOG.md). Older handoff snapshots below are historical and superseded where they conflict.


## Follow-up preflight supplied by user (2026-10-18)

The later read-only output confirms the vendor ROS 1 `/leg_odom2` publisher `/qnx2ros` is alive, while ROS 2 `/leg_odom2` has no publisher because the project ROS 1 bridge's `/topics` parameter is unset. ROS 2 `/scan`, `/map`, and `/amcl_pose` also have no publishers; mapping and Nav2 are stopped. Camera image and point cloud are live. API reports manual mode, goal requests disabled, motion output isolated, software STOP not latched, and physical E-stop unverified. Odometry's last message is 31.16 seconds old; scan/map are about 19 minutes old. Robot wall clock says 2024-09-12 and should not be trusted for current time.

This was user-provided read-only output, not a new live check by the assistant. No state-changing request was made in response. Do not send a goal based on cached map metadata or this preflight; no walking readiness is established. Next engineering work is to safely bring up and verify the project bridge and downstream TF/scan/map/localization chain.

## Local navigation-readiness hardening (2026-10-18; not deployed)

The local API/UI source has been hardened to keep Nav2 from appearing planner-ready on sensor freshness alone. It now requires fresh, structurally valid local and global costmap messages and the required odom/base, scan, depth-camera, and map/base transforms. The Nav2 profile requests periodic full costmap messages; UI v19 exposes the two readiness groups. The startup script checks for the new API fields before copying UI v19. Goal requests remain disabled by default and velocity remains isolated.

This workstation cannot currently reach `127.0.0.1:18082`, so these source changes have not been built, deployed, or checked against Foxy topic availability on the robot. The next session needs a live startup tunnel, API deployment/build, and read-only confirmation of those topic/TF paths before any goal test.

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

The single project ROS 1 `/topics` parameter now contains `/leg_odom2` and `/cmd_vel_corrected`; the project `parameter_bridge` was restarted with this combined configuration. The guarded `vendor_cmd_vel_bridge` is installed in the active project workspace but remains disabled (`physical_stop_verified=false`). ROS 1 `/cmd_vel_corrected` is wired from `/ros_bridge` to vendor `/ros2qnx`; ROS 2 `/autonomy/cmd_vel_arbited` has one guarded-relay subscriber. `/leg_odom2` remains bridged. No vendor process/service, route, UDP port, gate, goal, velocity, or motion command was changed. Backup: `/home/ysc/lite3_autonomy_ws/backups/20240912_160556/combined_bridge_predeploy/`.

Current follow-up: API status is reachable, but localization refresh stopped and planner readiness is false after the bridge restart. Restart Localization/Nav2 from the UI and wait for fresh AMCL/costmaps. Do not enable the relay or submit a physical goal until the independent stop/preemption and command-owner commissioning is separately verified.
