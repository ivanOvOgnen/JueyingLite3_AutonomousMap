# Lite3 Pro Room Mapping and Navigation Prototype

## Current test path — 2026-10-24

Use the [step-by-step robot test guide](docs/STEP_BY_STEP_ROBOT_TEST_20261024.md) for the current Linux-laptop install, map/localization button order, bounded Auto Mode motion test, and recovery. The backed-up install command is `python3 scripts/nav2_one_shot.py` after manually joining the robot Wi-Fi. The separate attended vendor test is `python3 scripts/test_auto_mode_move.py --apply`; it does not navigate to a map target. The sensor timing revision accepts up to 1.0 second of depth acquisition delay and 0.5 seconds between valid cloud arrivals while retaining timestamp, TF, STOP, and route checks. It passed 226 local Python tests and the UI wiring check. **The 18:38 robot deployment did not make map-click movement viable:** a later screenshot still reports both costmaps and depth-obstacle timing/TF unavailable. See [current status](STATUS.md). Earlier dated checkpoints below are historical observations.

## Current live compatibility checkpoint — 2026-10-19

The active robot session was checked after the combined bridge deployment. The project API and guarded vendor relay run in a clean ROS 2 Foxy Python environment; `geometry_msgs/Twist` and `nav2_msgs/NavigateToPose` both report valid type support from `/opt/ros/foxy`. The only intentional mixed environment is the C++ `ros1_bridge`, which must load ROS 1 Noetic and ROS 2 Foxy to translate `/leg_odom2` and `/cmd_vel_corrected`.

The project-owned relay is now included in the boot helper in its fail-closed disabled state. It does not enable the vendor command path or publish motion on startup. The read-only preflight also reports Python type support so a future reboot catches an accidental mixed ROS shell early.

## Vendor STOP signal integration - 2026-10-19

The project safety observer keeps the Retroid packet detector and now also
watches new vendor `deeprcs` log lines for the confirmed STOP event
(`Soft Emergency Pressed!!!` or `is_robot_lose_control = 1`). It sends that
event through the existing authenticated STOP relay to the perception
computer. This is a fail-closed project signal: it does not write to vendor
motion services, publish velocity, or prove an independent physical emergency
stop. Keep the motion output isolated until the supervised event check and
command arbitration commissioning are complete.

## Current supervised motion checkpoint - 2026-10-19

The saved-map perception and Nav2 planner path is live through the current tunnel: camera, odometry, scan, map, transforms, AMCL, Nav2, and both costmaps are ready, with no active goal. A supervised vendor Stand/Sit check reached standing state `6` and returned to resting state `1`. The project safety supervisor never observed the authenticated Retroid STOP signal (`STOP=false` throughout); direct safety still reports no four-contact gate and a stale EmotionBot link. The project arbiter remains fail-closed and disconnected from vendor `/cmd_vel`. Do not attempt autonomous walking or lower safety gates until the vendor stop/interlock path is verified.

The vendor STOP action subsequently put the project supervisor into fail-closed `FAULT` (`robot protection state`) while the robot returned to state `1`. This demonstrates vendor protection behavior, but the independent authenticated STOP relay still did not assert; autonomy remains disabled.

The observer was subsequently adapted to the vendor log event and verified under supervision: Stand reached the standing state, vendor STOP returned the robot to belly/resting state, and the project supervisor reported `FAULT` with `fault_reason="Retroid STOP observed"`. This confirms STOP event propagation. It does not certify autonomous-motion preemption, so the arbiter output remains isolated.

Motion-side packet capture confirms the Retroid exchange is alive, but the vendor STOP press emits the normal `0x21040001` reply rather than the project detector's expected `0x21020C0E`. A vendor-supported stop/interlock source or reviewed protocol change is required before connecting planner velocity to the robot.

This repository contains a research prototype for manually guided room mapping with a Jueying Lite3 Pro and RealSense depth camera. The browser UI shows live 2D SLAM and live/registered 3D depth data when the corresponding robot topics are available.

## Read this first

- [Handover](HANDOVER.md) — current robot state, safety limits, and deployment notes.
- [New-location guide](Lite3_New_Location_Setup_Guide.md) — network and browser workflow.
- [Status](STATUS.md) and [ground truth](GROUND_TRUTH.md) — dated observations separated from assumptions.
- [Web/API notes](docs/WEB_APP_API.md), [implementation plan](docs/IMPLEMENTATION_PLAN.md), and [debug log](docs/DEBUG_LOG.md).

## Latest live check and v20 deployment - 2026-10-18

API v20 was deployed to the robot and restarted. The local tunnel at `http://192.168.2.39:18082/` serves the v20 UI; a read-only GET confirms the updated planner opt-in UI and API status endpoint. Robot API port 8080 is listening as PID 15939. The API restart intentionally left project mapping and Nav2 stopped. Current fresh status: camera and point cloud ready; odometry, scan, map, localization, transforms, costmaps, and navigation not ready; no active goal; goal requests disabled; software STOP latched; physical E-stop unverified; motor output isolated. No goal, velocity, or movement command was sent.

The deployment fixes the public costmap subscription type (`OccupancyGrid`) and stale AMCL convergence bookkeeping. A new explicit planner-request opt-in is available only after full readiness and matching active-map checks; readiness loss disables it. **Do not lower thresholds.** Reload the page, reconnect, select the saved robot map, and start Localization/Nav2 from the UI. Only consider planner requests after fresh odometry/scan/map, TF, stable AMCL pose, and both costmaps are ready and pose alignment is checked. Physical walking remains unavailable because motor output is isolated and independent stop/RC arbitration are uncommissioned.

## Previous deployed checkpoint - API/UI v19 (superseded)

The laptop tunnel at `http://192.168.2.39:18082/` serves API v19 and UI v19. The project API source and installed module match SHA-256 `c0b6268a8e5dac7a6826fc5340004ec43cdb23f2e5ac3d6675d571895d59e2f4`; the Nav2 overlay requests periodic full local/global costmaps. The Foxy build succeeded; the installed Python module was synchronized and compiled after the build left the prior installed copy unchanged. A read-only laptop GET confirms API/UI availability and the new `transforms` and `costmaps` status fields. Robot rollback copy: `/home/ysc/lite3_autonomy_ws/backups/20261018/nav_readiness_predeploy_01/`.

Live API status after restarting only the project API: camera and point cloud ready; odometry, scan, map, all required TF checks, costmaps, localization and navigation not ready. The restart cleared the API's in-memory map; saved files remain in the robot webapp maps directory, but none is currently loaded. Mapping and Nav2 are stopped. API is paired; software STOP is latched; goal requests are disabled; `physical_estop_verified=false`; velocity output remains isolated. No bridge, mapping, Nav2, goal, velocity, or vendor process was started or changed. The last preflight identified vendor ROS 1 `/leg_odom2` but no project bridge; ROS 2 data remains absent. This is not ready for a goal or a walk. Next: while the robot is stationary and operator-supervised, restore the narrow odometry bridge and launch the saved-map localization/Nav2 stack, then verify live TF, scan/map, costmaps, AMCL convergence, and pose alignment. Physical motion additionally requires a verified independent stop and exclusive RC/autonomy arbitration.
### Previous navigation diagnostic (before the battery reconnect)

API/UI version 16 is deployed and served at `http://192.168.2.39:18082/`. The project API now refreshes AMCL with bounded no-motion scan updates and reports localization ready only after stable low-covariance estimates. It can reuse the existing ROS 1 bridge only when that bridge is the sole `parameter_bridge` and its `/topics` list exactly matches the project’s `/leg_odom2` telemetry config. A standard RealSense depth-optical transform was added to the project perception launch; its runtime effect is **not verified**.

The selected saved map is `v3_soba_ogi_test2` (92x91, 0.05 m/cell, about 52.9% known). A best-effort scan sample and the saved map became available while Nav2 ran; a standalone TF listener resolved the odom and scan transforms, but Nav2 itself continued to report `odom` and `camera_scan_frame` unknown. The ROS 2 node list did not show Nav2 lifecycle nodes, and lifecycle queries returned “Node not found.” AMCL produced a candidate pose but did not converge during its 90-second search; the latest estimate had high covariance and zero stable samples. Nav2 was stopped again and software STOP latched; the last stop response could not confirm the gate-disable service, though motion output remains isolated. Current API: camera, point cloud and odometry ready; scan/map/localization/Nav2 not ready; goal requests disabled. No goal or velocity was sent. This is not ready for walking. See [HANDOVER.md](HANDOVER.md), [STATUS.md](STATUS.md), [GROUND_TRUTH.md](GROUND_TRUTH.md), and [DEBUG_LOG.md](docs/DEBUG_LOG.md).

## Earlier version-15 deployment snapshot (runtime state superseded by live checkpoint above)

API/UI version 15 is deployed on the perception computer and served through the existing laptop tunnel at `http://192.168.2.39:18082/`. Robot-side API and UI hashes match this workspace. A clean Foxy build succeeded, the installed API matches its source, and Foxy `NavigateToPose` type support passes. The page, JavaScript, and CSS return HTTP 200; served JavaScript reports UI version 15.

Live status after restart: camera and point cloud ready; odometry, scan, map, localization, and stitched map cloud not ready; mapping/Nav2 stopped; goals disabled; software stop latched; motion output isolated. The STOP AUTONOMY control is deployed, but it is software-only (`physical_estop_verified=false`). The backup is `/home/ysc/lite3_autonomy_ws/backups/20261018/v15_predeploy_20261018_01/`. Do not attempt autonomous walking until `/leg_odom2` and downstream map/localization inputs return and command arbitration, obstacle response, and an independent physical stop are commissioned.

## Version 15 safety implementation

The authenticated API action latches future planner requests off, asks `/autonomy/set_enabled` to disable the project velocity gate, cancels an accepted Nav2 action when present, and stops the project Nav2 launch. Starting localization/Nav2 explicitly clears the software latch. The UI states that this is not a physical emergency stop. Motor output remains isolated, and an independent vendor/hardware stop and command arbitration have not been verified. Version 15 is deployed; the STOP action itself has not been invoked as a live test.

The requested ROS pipeline is represented in project launches: vendor `/leg_odom2` → project bridge/adapter `/odom` and `odom → base_link` TF → depth image `/scan` → SLAM `/map`; localization/Nav2 later supplies `/amcl_pose`. The latest available preflight reported **no `/leg_odom2` publisher**, and the current post-deployment API GET still reports odometry/scan/map/localization unavailable. No mapping/Nav2 launch, stop endpoint, or motion command was issued during deployment.

## Earlier 2026-10-18 predeployment findings

**Latest connection:** the startup launcher established a read-only tunnel on port 18081 (port 18080 was held by an older SSH process). Current API status: camera/cloud fresh, odometry/scan/map stale (~14 minutes), mapping/Nav2 stopped, goal requests disabled, action server unavailable, motor output isolated. The API does not advertise the v14 global-localization capability, so the launcher did not deploy the new UI. No control endpoint was called. The new tunnel remains open.

The authenticated read-only preflight now confirms `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose` have no publishers; `/odom` and every velocity topic are absent. The manual ROS 1 joystick topic has no subscriber, the bridge `/topics` parameter is unset, and UDP 43897 has an unidentified owner. The cached map is 86x91 at 5 cm/cell and about 49% known, but stale. Mapping/Nav2 are stopped. These inputs must be restored and verified before localization or goals can work.

**Predeployment check:** the user reported that mapping works, while an earlier API response showed stale odometry/scan/map and stopped mapping/Nav2. At that point the port-18080 tunnel was unavailable and version 14 was not built or deployed. The user authorized a supervised goal test, but motion was blocked by uncommissioned motor routing, arbitration, and independent stop. These findings are superseded for API/UI version by the deployment checkpoint above and remain relevant to physical safety.

The local laptop-to-robot SSH tunnel is now returning read-only API responses. The current status response reports camera and live point cloud ready; odometry, scan, map, localization, and stitched cloud not ready; mapping and Nav2 stopped; navigation goal requests disabled; and motor output isolated. The maps catalog and current camera cloud respond, while map grid and map-frame cloud return HTTP 503 until mapping/TF data are available. This is a live API check, not a new robot-side SSH preflight; vendor service state and command routing are not freshly reverified. No movement command was sent.

The deployed UI supports map clicks, goal previews, initial-pose selection, gated Nav2 requests, and no-start-pose AMCL global relocalization with a pose marker. It checks that the clicked map matches the selected robot map. Version 15 is now served over tunnel port 18082; the API is not currently navigation-ready because odometry, scan, map, localization, and Nav2 are unavailable. Chrome visual rendering and Retroid access through this tunnel still need confirmation. The shutdown confirmation endpoint rejected an invalid confirmation with HTTP 400 in an earlier session; the API remained active.

The UI also has a confirmation-gated battery-change action that stops project-owned mapping/navigation/bridge work and closes the project API. It does not power off the computer or stop vendor services; close the laptop tunnel and use the normal robot power-off procedure before changing the battery.

The intended map-to-goal sequence is: manually guide the robot while mapping, stop and save the 2D map, inspect map quality and the registered cloud, then localize against that saved map and start obstacle-aware navigation only after its data and command path are validated. Cloud cleanup should be reversible and review-only. Do not force a rectangular shape or alter the navigation occupancy grid to make the room look smoother; that can erase openings and obstacles.

## UI and map cleanup tools

The deployed 3D viewer applies automatic far-range and isolated-point filtering for display by default, with a raw-view toggle and adjustable live-camera range. This changes only browser rendering. `scripts/filter_map_candidate.py` creates a separate PGM/YAML candidate that converts isolated occupied cells to unknown; it never marks space free, changes geometry, or overwrites the source. The API/UI deployment was HTTP-verified, but visual rendering in Chrome has not been inspected in this session.

For the current session, open **HTTP** (not HTTPS) at `http://192.168.2.39:18082/` on the laptop. The laptop tunnel has been confirmed by read-only API requests; Retroid access through this port still needs confirmation. On page load, the UI creates a browser session without asking for an API key. Any trusted device that can reach this URL receives control access. Physical walking is not operational and planner goal requests remain disabled by default.

## Repeatable startup — 2026-10-18

After powering on, connect the laptop and Retroid to the robot Wi-Fi and keep the vendor app in manual mode. From the repository root on Windows, run `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Lite3Session.ps1 -LocalPort 18082`. The script detects the current laptop Wi-Fi address and gateway, reads the perception IP from YSC's current network configuration, opens a visible SSH tunnel, and waits for a read-only API status response. It can back up/copy HTML, JavaScript, and CSS when the API advertises matching capabilities. No SSH password is stored. It prints an HTTP URL for laptop and Retroid and opens it in the laptop's default browser; keep the SSH window open while using the UI. If automatic discovery fails at a new site, verify the current internal address before using `-PerceptionHost <address>`. In this deployment the API/UI files were copied through YSC because laptop ProxyJump SCP stalled; the current startup run was not repeated after deployment.

In the page, **Connect & check status** creates the browser's session and refreshes all reported component states using GET requests only. It does not start mapping, localization, navigation, or other processes. The dashboard cannot verify the vendor controller link; check that separately in the Lite3 app. The API/UI deployment succeeded manually; the launcher's future asset-copy branch has not been verified. Retroid reachability through the laptop tunnel remains subject to Windows Firewall and must be checked on the device.

## Project layout

- `ros2_ws/src/lite3_odom_tf_adapter/` — ROS 2 Foxy mapping, perception, API, and gated navigation source.
- `webapp/` — browser UI and saved-map files.
- `scripts/` — preflight, API startup, map review, and packaging helpers.
- `docs/` — runbooks, API contract, plans, and dated engineering history.

## Safety boundary

Preserve the DEEP Robotics manual controller path. Do not publish velocity, send a navigation goal, enable the autonomy gate, launch a motion-capable path, or modify/stop vendor services or networking/UDP ports without explicit authorization for that supervised test. Mapping must be started only when the owner is ready and supervising. A good build or a visual map does not certify safe walking.

## Repository hygiene

Never commit passwords, API tokens, pairing keys, private keys, or unreviewed logs. Historical private LAN details belong in the user's local environment, not public project docs.


## Readiness diagnostics update (local source only, 2026-10-18)

The project now requires the API to observe fresh valid local and global Nav2 costmaps and the required map/odom/base/scan/depth transforms before it reports planner readiness. Deployed UI v19 displays these states. The costmap profile requests periodic full costmap publication. The launcher will only install v19 when the robot API exposes the matching status fields. The robot output remains isolated and goal requests remain disabled by default.

The last user-supplied snapshot shows the ROS1 `/leg_odom2` source but no project bridge, no ROS2 `/leg_odom2`, `/scan`, `/map`, or `/amcl_pose`, and stopped mapping/Nav2. The current local tunnel probe at `127.0.0.1:18082` fails, so robot runtime state is not refreshed. Local source changes have not been built, deployed, or verified on Foxy. Do not submit a goal from cached map data.

## Fastest route to the finish line

The software navigation demo is now close: the live bridge, odometry, scan, saved map, TF, Nav2 action server, and both costmaps are present. The current blocker is AMCL stability (`stable_samples=1`, not converged). Once the pose is stable, the browser can enable planner requests and submit a goal; this exercises the planner while the project output remains isolated.

A physical walk still requires one separate commissioning step. The vendor ROS1 `ros2qnx` process accepts `/cmd_vel` and `/cmd_vel_corrected`, but the project arbiter output has no subscribers by design. Do not connect that path or lower the gate. The vendor safety supervisor is currently faulted from the observed STOP event and reports dynamic motion disarmed. A reviewed bridge, exclusive manual/autonomy ownership, watchdog, and an independently verified stop/preemption test are required before connecting planner velocity to the dog.

## Guarded vendor command bridge (source-only, 2026-10-19)

Added `vendor_cmd_vel_bridge.py` and `vendor_cmd_vel_bridge.launch.py` as a separate commissioning component. It is disabled by default and is not included in the navigation launch. When explicitly started, it requires the physical-stop verification parameter, a fresh authenticated stop-relay record, fresh project readiness, no manual override, and a command heartbeat; stale or invalid conditions publish zero and disable the relay. A separate `ros1_bridge` configuration maps `/cmd_vel_corrected` to the vendor ROS1 `ros2qnx` subscriber.

This component has not been built, deployed, enabled, or connected to the robot in this session. The existing isolated arbiter and vendor services are unchanged. Before enabling it, verify the bridge's ROS1/ROS2 topic mapping on the robot and complete the supervised stop-preemption test.

## Guarded bridge staging (2026-10-19)

The new fail-closed vendor bridge source and launch files were staged on `lite` under the project workspace and built successfully in an isolated temporary Foxy install (`/tmp/lite3_bridge_install`). A robot-side backup is under `/home/ysc/lite3_autonomy_ws/backups/20240912_160111/vendor_cmd_bridge_predeploy/`. The component was not installed into the active runtime and was not launched.

The existing odometry parameter bridge owns the global ROS 1 `/topics` configuration. Starting a second parameter bridge without a reviewed combined configuration could replace the live odometry mapping, so the command bridge remains staged only. No gate, bridge, goal, velocity, vendor service, or network setting changed; the active API still reports AMCL converged and motor output isolated.

## Combined bridge deployment (2026-10-19)

The project bridge was deployed on `lite` with one combined ROS 1 `/topics` parameter containing both `/leg_odom2` and `/cmd_vel_corrected`. The old project `parameter_bridge` was replaced by the same project bridge process; vendor transfer, safety, and networking services were not changed. The guarded `vendor_cmd_vel_bridge` is installed in the active project workspace with `physical_stop_verified=false`, so it remains disabled and publishes no motion command.

Live checks: ROS 1 `/cmd_vel_corrected` has `/ros_bridge` as publisher and vendor `/ros2qnx` as subscriber; ROS 2 `/autonomy/cmd_vel_arbited` has exactly one subscriber (the guarded relay); `/leg_odom2` still has one publisher and five subscribers; API status responds. The relay executable is now in the active project install. Backup: `/home/ysc/lite3_autonomy_ws/backups/20240912_160556/combined_bridge_predeploy/`.

This wires the path for later commissioning but does not make walking ready. The relay, arbiter, physical-stop verification, planner opt-in, and goal submission remain disabled; no velocity, goal, gate enable, or physical movement occurred. Current API status after the bridge restart reports localization refresh/planner not ready because the prior refresh stopped; restart Localization/Nav2 from the UI and recheck convergence before any planner test.
