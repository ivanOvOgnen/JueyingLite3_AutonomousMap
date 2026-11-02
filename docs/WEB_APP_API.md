# Robot-hosted web app and mapping API

> **Current supervised-trial note (2026-10-24):** The version-15 and planner-only sections below are retained as historical API checkpoints. The current local bundle adds a separately armed, time-limited supervised Nav2 relay. Target selection and a saved-map inspection can work before motion readiness; sending and arming a real goal still require current localization, both Nav2 costmaps, validated depth observations, a matching map, and an operator lease. The 18:38 depth/costmap timing repair was deployed, but a subsequent screenshot still showed those sensor checks failing. **Physical map-click movement is not viable.** See [the current step-by-step guide](STEP_BY_STEP_ROBOT_TEST_20261024.md) for the evidence needed before another attempt and the separate bounded Auto Mode movement check.

## Deployed version 15 — software STOP

`POST /api/v1/safety/stop` is authenticated and serialized with other control requests. It latches goal submission off, requests `SetBool(false)` from `/autonomy/set_enabled` when available, requests cancellation for an accepted Nav2 goal, and stops the project Nav2 launch. Its response reports whether gate disable was confirmed and states this is not a physical E-stop. The latch is reset only by explicitly starting localization/Nav2 again. Version 15 is deployed, but this route has not been invoked as a live test. It cannot stop motion through vendor paths and is not a substitute for the handheld controller/hardware stop. `components.safety_stop.physical_estop_verified` is false.

The Windows launcher now recognizes UI asset marker 15 and requires the backend to expose both `components.global_localization` and `components.safety_stop` before copying the static UI. It still does not deploy or build the Python API, so backend deployment must precede UI deployment.

Last reviewed: 2026-10-18 (Europe/Skopje)

## Safe reconnect flow

**Earlier predeployment check (superseded):** `127.0.0.1:18080` did not accept a connection and version 14 had not been deployed. Version 15 is now deployed; the current API state is recorded in [STATUS.md](../STATUS.md). No control endpoint was called during deployment.

**Later reconnect in the same session:** the visible launcher established the tunnel on Wi-Fi port 18081 and the API returned HTTP 200. Status reported stale odometry/scan/map (~14 minutes), mapping/Nav2 stopped, goals disabled, and motor output isolated. The backend did not advertise `components.global_localization`; the helper therefore refused UI v14 deployment. The tunnel remains open.

The Windows laptop helper `scripts/Start-Lite3Session.ps1` discovers the active Wi-Fi IPv4/default gateway, reads the perception IP from YSC's `network.toml`, then opens an SSH ProxyJump and waits for `GET /api/v1/status`. It supports `-PerceptionHost` as a verified-address override. The SSH window remains visible for host-key and password prompts; the helper stores no credentials. If the served UI lacks deployment version 14, it backs up and copies local `index.html`, `app.js`, and `styles.css` into the project's configured web directory; it does not launch/restart the API or send a control POST. The UI's **Connect & check status** button requests `GET /api/v1/session` and then `GET /api/v1/status`; it is a status/session check only.

Before copying version 15 UI assets, the launcher requires `/api/v1/status` to advertise both `components.global_localization` and `components.safety_stop`. This prevents deploying the new UI against an older API that lacks corresponding capabilities. It does not deploy or build the Python API.

The UI/API additions described here are deployed as version 15. This does not mean the mapping/localization inputs or motor path are ready. Retroid can use the printed laptop Wi-Fi URL only if the Windows tunnel listener is reachable from that device; do not change firewall policy automatically.

For localization without an initial pose, start Localization/Nav2 with a saved robot map, wait for fresh map/scan/odometry, then use **Locate robot automatically (no start pose)**. AMCL globally initializes particles over map free space; wait for the fresh estimate and compare its blue marker with the real room. Repetitive or incomplete maps can produce an ambiguous/wrong estimate. The UI's **Cancel Nav2 goal** cancels a planner action; it is not a physical emergency stop.

## 2026-10-06 deployment record (superseded by version 15 above)

The updated UI/API has been copied into the robot project workspace, clean-built under Foxy, and restarted as the only project process changed. Deployment backups exist at `/home/ysc/lite3_autonomy_ws/backups/20261006/live_ui_predeploy_01/` and `/home/ysc/lite3_autonomy_ws/backups/20261006/poweroff_button_predeploy_01/`. Through the laptop tunnel, session, status, map catalog, live camera cloud, page, JS, and CSS return HTTP 200. Live grid and stitched cloud return HTTP 503 while there is no fresh map or map-to-camera TF. Current robot status: RealSense camera/point cloud fresh; odometry/scan/map not fresh; mapping and Nav2 stopped; goal requests disabled; motor output isolated. The new shutdown action is deployed; a bad confirmation returned HTTP 400 and did not shut down the API. Retroid visual access is not freshly verified.

## Browser session and network boundary

On page load the updated UI fetches `GET /api/v1/session`. The robot issues an opaque 12-hour, host-only `HttpOnly; SameSite=Strict` cookie. It does not return the robot API token to the browser or store a pairing key in local storage. Control `POST` requests may authenticate with that cookie plus a same-origin HTTP `Origin` check; the existing token header remains accepted for maintenance clients. Reloading in a browser creates a new session. This makes setup shell-free, but **any browser/device that can reach the robot-hosted service can obtain control access**. Use only on the password-protected robot Wi-Fi with trusted devices. The service uses plain HTTP; do not expose it to public, guest, or untrusted networks. Do not use HTTPS against the plain-HTTP port.

The one-time `/api/v1/pair` route remains for a robot without a token. Automatic browser sessions do not reveal an existing token. The API token file remains owner-only on the robot.

## Battery-change shutdown

The authenticated `POST /api/v1/shutdown/prepare` route requires `{"confirm":"STOP PROJECT"}`. It serializes against other control actions, cancels an accepted planner goal if one exists, stops only API-owned localization/Nav2, mapping, and odometry-bridge process groups, and closes the project HTTP API after returning its result. If a project process does not stop cleanly, it returns an error and leaves the API available for diagnosis/retry. The API never sends a velocity or gait command.

This does not power off the Linux computer, stop vendor services, turn off robot Wi-Fi, or close a laptop-side SSH tunnel. After the UI confirms success, close the tunnel separately and use the robot's normal power-off procedure before changing the battery. The existing user autostart entry starts the API again after reboot.

## Endpoints

| Method | Endpoint | Behavior |
| --- | --- | --- |
| GET | `/api/v1/session` | Issues the short-lived same-origin browser session cookie. |
| GET | `/api/v1/status` | Component freshness, mapping pause/running state, cloud status, navigation gates, and map metadata. |
| GET | `/api/v1/map` | Current map metadata. |
| GET | `/api/v1/map/grid` | Fresh `OccupancyGrid` cells as JSON for the live 2D view; 503 until a fresh map exists. |
| GET | `/api/v1/maps` | Saved PGM/YAML pairs. New-format entries also expose an optional manifest URL and whether independently registered 3D evidence exists. |
| GET | `/api/v1/pointcloud` | Latest sparse camera-frame XYZ cloud as little-endian float32 triplets, capped at about 5,000 points. Headers identify frame, count, and age. |
| GET | `/api/v1/pointcloud/map` | Accumulated sparse XYZ cloud registered in the SLAM `map` frame as little-endian float32 triplets. 503 until TF registration succeeds. |
| POST | `/api/v1/mapping/start` | Starts the guarded odometry-only bridge and mapping launch if sensor checks pass. Clears prior in-memory stitched points. It does not move the robot. |
| POST | `/api/v1/mapping/pause` | Sends SIGSTOP to the mapping launch process group. Mapping pauses; the robot itself does not stop. Hold it stationary using the vendor controller. |
| POST | `/api/v1/mapping/resume` | Sends SIGCONT to the mapping launch process group. |
| POST | `/api/v1/mapping/stop` | Resumes a paused group if needed, stops only API-owned mapping/bridge processes, and retains latest map/cloud review data in memory. |
| POST | `/api/v1/maps/save` | Saves the latest map under a unique name; existing map files are not overwritten. It also writes `<name>.lite3-map.json` recording the 2D coordinate convention and explicitly marks 3D as absent unless separately registered. JSON: `{"name":"room_scan_01"}`. |
| POST | `/api/v1/diagnostics/preflight` | Runs the bundled read-only robot snapshot. |
| POST | `/api/v1/shutdown/prepare` | After explicit confirmation, stops project-owned mapping/navigation/bridge work, then closes the project API. Vendor/system services remain until normal computer power-off. |
| POST | `/api/v1/navigation/start` | Starts project localization/Nav2 for a saved robot map. It does not connect velocity to motors. Keep stopped until separately authorized. |
| POST | `/api/v1/navigation/stop` | Stops the project localization/Nav2 launch and bridge. |
| POST | `/api/v1/localization/initial_pose` | Publishes an AMCL start pose after known-free clearance checks. |
| POST | `/api/v1/localization/global` | Requests AMCL's global relocalization across saved-map free space after fresh map, scan, odometry, and Nav2 checks. Wait for a fresh estimate and verify its map marker before choosing a goal; this request does not move the robot. |
| POST | `/api/v1/navigation/goal` | Gated Nav2 planner action. Requires explicit API opt-in and readiness checks; disabled by default. The request must name the same saved robot map selected for Nav2. It is not physical robot control. |
| POST | `/api/v1/navigation/cancel` | Cancels the active planner goal. |
| POST | `/api/v1/pair` | One-time first claim for a robot without an existing token. |

## 2D and 3D views

The live 2D panel renders fresh ROS `OccupancyGrid` cells (row 0 is flipped for canvas display). The live 3D mode samples the latest RealSense `PointCloud2` and displays the cloud's camera frame in an orbitable canvas.

The stitched mode uses ROS TF to transform sampled camera points into `map` coordinates while mapping is active. It samples at most roughly 1,000 points per half-second and retains the newest 200,000 points in memory. Starting another mapping session clears the prior cloud; stopping mapping retains it until the next start or API restart. If map-to-camera TF is unavailable for a cloud timestamp, that sample is dropped. The current raw RealSense cloud has a known source-clock mismatch, so this is an unverified sparse review aid, not a dense persistent reconstruction or a navigation-quality map. It must not be saved as 3D map evidence.

The saved-map panel can inspect an ASCII or little-endian binary PLY locally with orbit, Shift-drag pan, zoom, and fit/reset. It is an offline viewer only: its PLY vertices must already be timestamp-verified and registered into the paired ROS `map` frame in metres. Loading a PLY does not modify ROS, Nav2, the manifest, or the map files.

The browser viewer filters finite points, applies a range limit to live camera data, and removes spatially isolated samples using voxel-neighborhood support. Filtering is enabled by default for display, has a raw-view toggle, and does not change server buffers, ROS topics, the Nav2 costmap, or saved occupancy maps. The live camera range control is disabled in stitched map coordinates because the original sensor origin is not retained for each accumulated point.

The local `scripts/filter_map_candidate.py` utility writes a new PGM/YAML candidate by changing isolated occupied cells to unknown. It preserves known-free and unknown cells and map geometry, refuses to overwrite input or existing candidate files, and is not a navigation-safety filter.

The ROS package already depends on `tf2_ros`. Any more advanced 3D pose-graph system needs a Foxy-compatible evaluation; do not install newer ROS distributions onto the vendor image casually.

## Safety and deployment

Version 15 is deployed on the robot. Its **STOP AUTONOMY** action latches goals off, requests the project gate disabled, cancels a planner goal when present, and stops project Nav2. Live status reports `safety_stop.physical_estop_verified=false`; this is not a physical emergency stop. Current runtime has camera/cloud only, with odometry/scan/map/localization unavailable, mapping/Nav2 stopped, goals disabled, and motion output isolated. The 2026-10-18 predeployment backup is `/home/ysc/lite3_autonomy_ws/backups/20261018/v15_predeploy_20261018_01/`.

Mapping is manually guided with the DEEP Robotics controller. Pause does not stop robot motion. Nav2 goal requests remain disabled by default, and the velocity gate remains disconnected from vendor motors. Do not connect `/autonomy/cmd_vel`, alter vendor services/ports, or attempt physical autonomy until command ownership, RC/autonomy interlock, localization, obstacles, and stop-on-loss behavior are commissioned.

The current read-only preflight found no odometry/scan/map publisher, so mapping must remain stopped until the camera/odometry readiness guard succeeds and the owner requests a supervised mapping session. A successful build or HTTP response does not prove safe walking.

The project user cron runs `scripts/start_robot_api.sh` after boot. Before deployment, make a dated backup of the API source and web assets on the robot. Build only `lite3_odom_tf_adapter` in a clean Foxy environment. Restart only the project API after confirming mapping/Nav2 are stopped. Preserve the cron entry and never stop or modify `transfer.service`, `transfer_ros2.service`, `realsense_ros2.service`, vendor workspaces, or UDP 43897 as part of this UI work.
