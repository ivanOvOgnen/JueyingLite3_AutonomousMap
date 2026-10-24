# Jueying Lite3 Pro — New Location Connection & Mapping Guide

> **Current Linux laptop procedure (2026-10-24):** Use the [step-by-step robot test guide](docs/STEP_BY_STEP_ROBOT_TEST_20261024.md). The dated v20 addresses, ports, and planner-only statements below describe earlier checkpoints; discover the current robot address from the active network. The supervised trial package has a bounded, separately armed motion path and a backed-up installer. The 18:38 depth/costmap timing repair was deployed, but the next screenshot still showed invalid costmap source timestamps and unavailable depth-obstacle time/TF. Physical map-click movement is not viable.

## Live session update - 2026-10-18, v20 deployed

API v20 and UI v20 are deployed. The robot API is listening on port 8080 (PID 15939), and the laptop page at `http://192.168.2.39:18082/` serves `app.js?v=20`. A dated rollback copy is at `/home/ysc/lite3_autonomy_ws/backups/20261018/costmap_goaloptin_predeploy_01/`.

The API restart intentionally stopped its owned Nav2 stack and cleared the in-memory map. Current status is fail-closed: only camera and point cloud are ready; odometry, scan, map, localization, transforms, costmaps and Nav2 are not ready; planner requests are disabled; software STOP is latched; the physical stop is unverified; motor output is isolated. No movement command was sent. Reload the browser, reconnect, load the saved map from the robot, and start Localization/Nav2 from the UI. Keep all readiness thresholds. Wait for fresh odometry/scan/map, required transforms, stable AMCL, and both costmaps, then verify the blue pose against the physical robot before planner opt-in. Planner opt-in does not connect motor output or make walking available.

## Previous deployed checkpoint - API/UI v19 (superseded)

The laptop tunnel at `http://192.168.2.39:18082/` serves API v19 and UI v19. The project API source and installed module match SHA-256 `c0b6268a8e5dac7a6826fc5340004ec43cdb23f2e5ac3d6675d571895d59e2f4`; the Nav2 overlay requests periodic full local/global costmaps. The Foxy build succeeded; the installed Python module was synchronized and compiled after the build left the prior installed copy unchanged. A read-only laptop GET confirms API/UI availability and the new `transforms` and `costmaps` status fields. Robot rollback copy: `/home/ysc/lite3_autonomy_ws/backups/20261018/nav_readiness_predeploy_01/`.

Live API status after restarting only the project API: camera and point cloud ready; odometry, scan, map, all required TF checks, costmaps, localization and navigation not ready. The restart cleared the API's in-memory map; saved files remain in the robot webapp maps directory, but none is currently loaded. Mapping and Nav2 are stopped. API is paired; software STOP is latched; goal requests are disabled; `physical_estop_verified=false`; velocity output remains isolated. No bridge, mapping, Nav2, goal, velocity, or vendor process was started or changed. The last preflight identified vendor ROS 1 `/leg_odom2` but no project bridge; ROS 2 data remains absent. This is not ready for a goal or a walk. Next: while the robot is stationary and operator-supervised, restore the narrow odometry bridge and launch the saved-map localization/Nav2 stack, then verify live TF, scan/map, costmaps, AMCL convergence, and pose alignment. Physical motion additionally requires a verified independent stop and exclusive RC/autonomy arbitration.

> **Previous reconnect state, superseded by the deployed checkpoint above (2026-10-18):** tunnel `http://192.168.2.39:18082/` is live and serves API v16/UI v18. Camera/point cloud are fresh; odometry/scan/map are stale; no localization/Nav2. ROS 1 `/leg_odom2` has vendor publisher `/qnx2ros`, but the project ROS 1-to-ROS 2 bridge is stopped (`/topics` unset), so the ROS 2 consumers have no odometry. UI v18 and the updated read-only preflight report this distinction. Re-establish the project bridge through stack startup while stationary, then verify fresh TF/scan/map and AMCL before any goal. Goal requests are disabled and motor output isolated. Software STOP currently reports not latched; physical E-stop is unverified.

> **Earlier navigation checkpoint before battery reconnect (2026-10-18):** API/UI version 16 was served at `http://192.168.2.39:18082/`. The project Nav2 launch was stopped and software STOP latched. Camera, point cloud and odometry were ready; scan/map were not ready while the stack was stopped. During Nav2 startup, scan/map became live and a standalone TF listener resolved relevant transforms, but Nav2 still reported `odom` and `camera_scan_frame` unknown; its nodes were absent from the graph and lifecycle queries returned “Node not found.” A 90-second AMCL search produced an uncertain pose but did not converge. Goals remained disabled and motor output isolated; the last STOP could not confirm gate disable. Do not try a walk. See [STATUS.md](STATUS.md) and [HANDOVER.md](HANDOVER.md).

> **Earlier offline preflight snapshot (before the live navigation check above):** the user reported saving a room map and selecting a point, the point is currently only a preview. The last read-only preflight found zero publishers for `/leg_odom2`, `/scan`, `/map`, and `/amcl_pose`, with `/odom` absent. Mapping/Nav2 are stopped; global localization and planner are not ready; goals are disabled and motion output isolated. Restore vendor odometry and verify the downstream pipeline before trying localization or any goal. See [STATUS.md](STATUS.md) and [HANDOVER.md](HANDOVER.md).

> **Earlier 2026-10-18 deployment checkpoint (before localization/Nav2 start):** API/UI version 15 is deployed and verified over the laptop tunnel at `http://192.168.2.39:18082/`. The software **STOP AUTONOMY** control is available but is not a physical emergency stop (`physical_estop_verified=false`); keep the vendor controller and hardware stop available. At that earlier checkpoint, only camera/point cloud were ready and the ROS topics listed below had no publishers; the live stack state is described in the latest checkpoint above. Restore and verify the vendor odometry stream and downstream map/localization pipeline before using those controls. Do not infer readiness from a saved map or camera feed.

> **Purpose:** reconnect the Lite3 Pro, controller, laptop, perception computer, ROS stack, and browser UI after moving the robot to a different physical location/network environment.
>
> **Current project scope:** manual RC driving + RealSense mapping + map review + planner-only Nav2 checks. **Physical autonomous walking is not commissioned.** The project intentionally does not connect `/autonomy/cmd_vel` to the vendor motor command path.

> **Earlier predeployment checkpoint (superseded):** port 18080 was unavailable and a prior tunnel snapshot showed missing odometry/scan/map data. The current tunnel on port 18082 and version 15 deployment are described above. Always check the live status before using controls. See [README.md](README.md), [STATUS.md](STATUS.md), and [GROUND_TRUTH.md](GROUND_TRUTH.md).

> **Earlier predeployment checkpoint (superseded):** the tunnel on port 18081 served an API without global-localization capability. Version 15 is now deployed and verified on port 18082. See [STATUS.md](STATUS.md) for the latest live state.

---

## 1. What connects to what

```text
                 ┌─────────────────────────────┐
                 │ Retroid Pocket 3 Plus / RC  │
                 │ DEEP Robotics Lite3 app     │
                 │ + Chrome                    │
                 └──────────────┬──────────────┘
                                │ robot Wi-Fi
                                │
┌─────────────────────┐         │          ┌──────────────────────────┐
│ Windows laptop      │─────────┼─────────▶│ Motion computer          │
│ SSH / Chrome        │         │          │ hostname: ysc            │
│ map backup/review   │         │          │ reachable on robot Wi-Fi │
└──────────┬──────────┘         │          └────────────┬─────────────┘
           │                    │                       │ internal robot LAN
           │                    │                       ▼
           │                    │          ┌──────────────────────────┐
           └──── SSH jump ──────┴─────────▶│ Perception computer      │
                                           │ hostname: lite           │
                                           │ ROS 1 Noetic + ROS 2     │
                                           │ Foxy + RealSense + API   │
                                           └──────────────────────────┘
```

The browser/API normally lives on the **perception computer on port 8080**. The current Windows launcher discovers the Wi-Fi gateway and reads the perception IP from YSC's network configuration, then exposes the API through the laptop's local port 18080 tunnel. This laptop path has now been confirmed with read-only HTTP requests. Treat addresses and routes as location-dependent and rediscover them after moving the robot.

---

## 2. Safety boundary before connecting anything

Keep these rules for every setup:

- Keep the **DEEP Robotics Lite3 controller app** available as the manual control path.
- Do **not** stop or reconfigure vendor transfer services just to make the project work.
- Do **not** change UDP ports `43897` or `43893` as a quick fix.
- Do **not** bridge `/autonomy/cmd_vel`, `/nav2/cmd_vel`, or project output into the vendor `/cmd_vel` path.
- Do **not** run a broad ROS 1 `dynamic_bridge`; the project mapping bridge is intentionally limited to `/leg_odom2`.
- Do not use an old saved map for a different room. A new physical location needs a **new map**.
- Navigation goals in this repository are for **planner-path testing only**. They must not be interpreted as authorization to make the robot walk autonomously.

---

## 3. Before powering on

Check the physical setup first:

- [ ] Robot battery is sufficiently charged.
- [ ] RealSense camera is firmly mounted and has not changed angle/position.
- [ ] Room is clear enough for slow manual walking.
- [ ] Retroid/controller is charged and the Lite3 app opens.
- [ ] Laptop is available for the first setup in the new location.
- [ ] You know the robot Wi-Fi credentials.
- [ ] The current project copy is available on the laptop.

The mapping code assumes the vendor camera mount transform is approximately:

```text
base_link -> camera_link
translation: (0.25489, 0, 0.07249) m
RPY:         (0, 0.34907, 0) rad
```

If the camera has been remounted, do not trust a new map until the transform is revalidated.

---

## 4. Power-on order

1. Power on the Lite3 Pro.
2. Let the robot computers finish booting.
3. Connect the Retroid/controller to the robot as you normally do with the vendor Lite3 app.
4. Put/keep the vendor app in **manual mode**.
5. Connect the laptop to the **same robot Wi-Fi**.
6. Give the robot roughly a minute to finish startup before judging the API as unavailable. The project has a user `@reboot` entry that attempts to start the API after a 45-second delay.

Do not start mapping yet.

### Repeatable laptop startup (normal path)

The API is intended to autostart on the perception computer after boot. On the Windows laptop, connect to the robot Wi-Fi, then from the repository root run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Lite3Session.ps1
```

The script detects the laptop's active Wi-Fi IPv4 and gateway for this location, then reads the current perception `ip` value from YSC's `~/jy_exe/conf/network.toml`. It uses that address for the SSH tunnel, so no address worksheet or manual IP selection is needed. SSH may ask for the YSC password during the read-only lookup and for SSH host-key confirmation; later tunnel or first-time UI asset copy steps may ask for SSH authentication again. Passwords are entered into SSH and are never saved by the script. Accept a host key only if you recognize the robot. The script waits up to 100 seconds for a valid read-only API status response. If the served UI lacks the current cloud-filter feature, it creates a dated backup and copies local `index.html`, `app.js`, and `styles.css` into the project web directory; it does not restart the API or change ROS/vendor processes. Then it prints one HTTP URL for laptop and Retroid and opens it in the laptop's default browser. Connect both devices to robot Wi-Fi. Press **Connect & check status** in the page to create that browser's session and refresh all current component statuses. This check only calls GET endpoints; it does not start or stop project processes.

Keep the SSH window open while using the UI. Close it or press Ctrl+C in that window to end the tunnel. If SSH exits, its verbose error remains visible in that window. The startup script reports a clear failure if it cannot authenticate to YSC or if `network.toml` does not contain one unambiguous IPv4 `ip` value; it never guesses or reuses a remembered perception IP. Do not alter firewall or robot networking settings as part of startup.

---

## 5. Network troubleshooting (normally skip)

### After mapping: localize without entering a start pose

Once a new map has been stopped, saved on the robot, and reviewed, select that same saved robot map and start Localization/Nav2. Wait for fresh camera, scan, odometry and map status. In local UI version 14, **Locate robot automatically (no start pose)** requests AMCL's global relocalization. Wait for the fresh pose estimate and confirm the blue marker and heading match the robot's actual position and direction before clicking a destination. If AMCL does not settle or appears in the wrong part of a repetitive map, do not send a goal; use the initial-pose tool only when you know the robot's actual map pose.

The current API defaults to rejecting Nav2 goals, and `/autonomy/cmd_vel` is not connected to Lite3 motors. The browser's STOP AUTONOMY control is software-only; its cancel action is not a physical stop. Global localization is not a movement command. Physical walking requires separately verified motor routing, exclusive manual/autonomy arbitration, obstacle response, watchdog behavior, and an independent stop path. Version 15 is deployed, but those requirements remain unmet.

You do not need to fill out an address worksheet for a normal startup. Connect the laptop and Retroid to the robot Wi-Fi, then run the Section 4 script. It reads the laptop's current Wi-Fi address and gateway, uses the gateway as the current YSC SSH route, reads the perception address from YSC `network.toml`, and prints/opens the resulting browser URL. The backup directory is date-stamped automatically.

Only use the checks below if the startup script cannot establish the tunnel. Do not substitute addresses remembered from an earlier location.

### On the Windows laptop

To inspect the laptop's current network manually, open PowerShell:

```powershell
ipconfig
```

The script normally reads this automatically. The output is useful only when diagnosing a failed connection.

The script treats the current Wi-Fi default gateway as the motion-computer SSH candidate. If its SSH window reports that this is not YSC, inspect current device/network information before retrying; do not guess an old address. To manually test a confirmed address:

```powershell
Test-NetConnection MOTION_WIFI_IP -Port 22
ssh ysc@MOTION_WIFI_IP
```

Expected result: SSH succeeds and the host reports the motion computer (`ysc`).

### From the motion computer

Run read-only checks:

```bash
hostname
ip -br addr
```

The perception destination was previously visible in the motion-side network configuration. You can inspect it without changing it:

```bash
grep -nE 'ip|target_port|local_port' ~/jy_exe/conf/network.toml
```

The normal startup script reads this configuration automatically. Use the override only if the config cannot be read and you have independently verified the current perception address:

```bash
ping -c 3 PERCEPTION_IP
ssh ysc@PERCEPTION_IP
```

Back on the laptop, retry with the address you just verified:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Start-Lite3Session.ps1 -PerceptionHost CURRENT_PERCEPTION_IP
```

Expected perception hostname:

```bash
hostname
# expected: lite
```

Do not edit `network.toml` during this discovery step.

---

## 6. Run the project preflight on the perception computer

Once logged into the perception computer:

```bash
bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh
```

This is the preferred first diagnostic because it is read-only. It checks:

- ROS Foxy/project installation
- relevant ROS 2 topics
- ROS 1 `/leg_odom2` source publisher versus the ROS 2 bridge
- camera/odometry/map/navigation endpoints
- existing bridge/mapping/navigation processes
- local API status on port 8080
- ROS 1 command-route information
- UDP `43897` ownership
- saved robot maps

### Minimum healthy signs for mapping

You want to see the project workspace and RealSense path present. The browser/API additionally needs a fresh camera stream, and mapping startup needs `/leg_odom2` to become fresh through the narrow bridge.

Useful manual checks if something is unclear:

```bash
source /opt/ros/foxy/setup.bash
ros2 topic list
ros2 topic hz /camera/depth/image_rect_raw
```

Do **not** publish to `/cmd_vel` as a connectivity test.

---

## 7. Check whether the robot-hosted API is already running

On perception:

```bash
curl -fsS http://127.0.0.1:8080/api/v1/status
```

If it responds, the API is alive locally.

If it does not respond, check the boot log:

```bash
tail -n 100 ~/lite3_autonomy_ws/logs/robot_api_autostart.log
```

Also confirm the project build exists:

```bash
source /opt/ros/foxy/setup.bash
source ~/lite3_autonomy_ws/install/setup.bash
ros2 pkg prefix lite3_odom_tf_adapter
```

### API start — supervised API-only check

Do not start the API as a routine connectivity probe: it exposes mapping and navigation actions. For the first check after the Foxy rebuild, proceed only in an owner-supervised stationary session. Do not press mapping, localization, or navigation controls, and do not opt in to planner goals.

For the supervised API-only check, use the project launcher, which keeps planner-goal opt-in disabled. Do not run a second API if one is already listening:

```bash
source /opt/ros/foxy/setup.bash
source ~/lite3_autonomy_ws/install/setup.bash
cd ~/lite3_autonomy_ws
bash ~/lite3_autonomy_ws/scripts/start_robot_api.sh
```

Keep this terminal open. Stop it later with `Ctrl+C`.

The supplied `scripts/start_robot_api.sh` leaves planner-goal requests disabled. Keep it that way. In the current session the API is already running and local/tunnel GET checks pass. Do not press mapping or Nav2 controls during UI verification.

---

## 8. Open the browser UI

Prefer the automated laptop flow in Section 4. Its URL is `http://<laptop-current-wifi-ip>:18080/`; it is plain HTTP. The script keeps the SSH tunnel in a separate visible window. If the status check times out, inspect that window, wait for the robot's boot/API startup, and confirm the current perception hostname/address before retrying. Do not start a second API process as a connectivity workaround.

### Legacy/manual alternatives (use only if the launcher path is unavailable)

The current verified browser path is the laptop-bound local tunnel at `http://<laptop-wifi-ip>:18080/`. The direct and hand-built tunnel commands below are older alternatives; do not start another tunnel if the launcher already has one active.

#### Direct perception access

From the laptop or Retroid Chrome:

```text
http://PERCEPTION_IP:8080/
```

If the page opens, save this as the new-location API URL.

#### Manually tunneled access

This was a known topology in the previous setup: laptop → motion computer → perception computer.

On the Windows laptop, create an SSH tunnel through the motion computer:

```powershell
ssh -N -L 18080:PERCEPTION_IP:8080 ysc@MOTION_WIFI_IP
```

Leave that terminal open, then open on the **laptop**:

```text
http://127.0.0.1:18080/
```

This is a useful laptop fallback because the same forwarded connection serves both the page and its API.

For Retroid access through the same temporary tunnel, bind it to the laptop's current robot-Wi-Fi IPv4 address instead of loopback:

```powershell
ssh -N -L LAPTOP_WIFI_IP:18080:PERCEPTION_IP:8080 ysc@MOTION_WIFI_IP
```

Then open `http://LAPTOP_WIFI_IP:18080/` on the laptop or Retroid. Windows Firewall may block access from the Retroid; do not change firewall/network settings as part of this check. Direct access from the Retroid to the perception computer is still unverified.

### Static preview fallback

If you only need to review maps, the laptop can host `webapp/` as a static site. This supports map preview but **not robot mapping controls**.

From the project root on the laptop:

```powershell
cd webapp
python -m http.server 8000 --bind 0.0.0.0
```

Open from the laptop:

```text
http://127.0.0.1:8000/
```

Or from another device on the same Wi-Fi, if the laptop firewall allows it:

```text
http://LAPTOP_WIFI_IP:8000/
```

---

## 9. Connect the browser session

Press **Connect & check status** to request the host-only, 12-hour `HttpOnly` session and refresh the current component states. The check reads status only; it does not start mapping or navigation. Do not copy or paste an API key. Open the page from the robot-hosted URL over HTTP and reload if you see an older API-key field or pairing button.

Every device that can reach the robot API can receive control access. Use only trusted devices on the password-protected robot Wi-Fi. The service uses plain HTTP and is not intended for public, guest, or untrusted networks. The cookie is same-origin protected; do not expose port 8080 or the laptop tunnel to other networks.

Mapping, localization, and Nav2 buttons remain readiness-gated. A pause stops the mapping processes only; it does not stop the dog, so keep the dog stationary with the vendor controller while paused. Planner goal requests stay disabled by default and motor output remains isolated.

### Battery change: prepare the project for power-off

Use **Stop project and disconnect UI** only after the robot is stationary and you are ready to end this project session. Confirm by typing `STOP PROJECT`. The API cancels any accepted planner goal, stops only project-owned mapping, localization/Nav2, and odometry-bridge processes, then closes the project web API. If a process cannot stop, the API reports failure and stays online so you can check status; do not remove the battery until you have confirmed the robot is powered off.

This button does not power off the computer or stop vendor camera/transfer/controller services. After it reports success, close the laptop SSH tunnel from the terminal that started it, then use the robot's normal manufacturer power-off procedure. Wait for the robot/computer to power down before changing the battery. The robot API starts again automatically after the next boot.

---

## 10. Browser readiness check

Before walking the robot:

- [ ] Page loads from the intended device.
- [ ] Saved-map list loads.
- [ ] Robot diagnostics/preflight runs successfully.
- [ ] Depth camera status is fresh.
- [ ] Mapping is not already running.
- [ ] Vendor RC app is still working in manual mode.
- [ ] Clicking a map only selects a point and does not automatically send a goal.
- [ ] No project output is connected to the robot motor command path.

If the camera, odometry, or API status is stale, diagnose that first. Do not begin a room scan merely because the page itself loads.

---

## 11. Make a fresh map for the new location

A new room/location needs a new PGM/YAML map pair.

### Recommended naming

Use a unique, descriptive name such as:

```text
living_room_20261002
lab_room_a_20261002
new_house_ground_floor_20261002
```

### Mapping procedure

1. Start mapping only after the owner explicitly authorizes a mapping session and is present to supervise it.
2. In the browser, press **Start mapping**.
3. Confirm mapping reports running and odometry becomes fresh.
4. Continue controlling the robot **only with the vendor RC app**.
5. Walk slowly through the usable area.
6. Include walls, corners, and door openings from more than one view where practical.
7. If safe, return near the starting area to help loop closure.
8. Watch the map for obvious shearing, duplicated walls, rotation drift, or unstable geometry.
9. If the map visibly breaks, stop the scan rather than continuing to collect bad data.
10. When finished, press **Stop mapping**.
11. Save using a **new unique name**. Do not overwrite an earlier scan.

If the browser/API route is unavailable, the ROS mapping launch can be run manually only after the preflight/frame checks:

```bash
source /opt/ros/foxy/setup.bash
source ~/lite3_autonomy_ws/install/setup.bash
ros2 launch lite3_odom_tf_adapter mapping.launch.py
```

The room scan is still manually driven with the vendor controller.

---

## 12. Save and review the map

The web API can save the retained `/map` directly. A command-line fallback is also installed:

```bash
source /opt/ros/foxy/setup.bash
source ~/lite3_autonomy_ws/install/setup.bash
ros2 run lite3_odom_tf_adapter save_live_map /home/ysc/maps/NEW_MAP_NAME
```

This writes:

```text
NEW_MAP_NAME.pgm
NEW_MAP_NAME.yaml
```

Use a new prefix for every scan.

### Copy the accepted map to the laptop project

If the laptop can SSH directly to perception:

```powershell
scp ysc@PERCEPTION_IP:/home/ysc/lite3_autonomy_ws/webapp/maps/NEW_MAP_NAME.* .\webapp\maps\
```

If perception is only reachable through the motion computer, use an SSH jump:

```powershell
scp -o ProxyJump=ysc@MOTION_WIFI_IP ysc@PERCEPTION_IP:/home/ysc/lite3_autonomy_ws/webapp/maps/NEW_MAP_NAME.* .\webapp\maps\
```

If the map was saved under `/home/ysc/maps/`, substitute that source path.

### Generate the offline quality report

From the laptop project root:

```powershell
python .\scripts\map_quality_report.py .\webapp\maps\NEW_MAP_NAME.pgm .\webapp\maps\NEW_MAP_NAME.yaml
```

Review:

- map shape and wall continuity
- free/occupied/unknown proportions
- disconnected free-space islands
- obvious duplicated or skewed geometry
- whether the useful room area is actually represented

A larger map or lower unknown percentage alone does not prove it is navigation-ready.

### Optional automatic cleanup candidate

The local cleanup script removes only isolated occupied pixels by converting them to unknown. It does not infer rectangular walls, close gaps, or turn unknown space into free space. It writes a new map pair and refuses to overwrite either the source or an existing candidate:

```powershell
python .\scripts\filter_map_candidate.py `
  .\webapp\maps\NEW_MAP_NAME.pgm `
  .\webapp\maps\NEW_MAP_NAME.yaml `
  .\webapp\maps\NEW_MAP_NAME_filtered_candidate
```

The candidate is for comparison and remains unapproved for navigation. The 3D viewer also filters far and isolated points automatically for display; its raw-view toggle restores the unfiltered camera cloud. Neither filter changes robot topics or the source map.

---

## 13. Planner/localization check — only after the map looks coherent

This stage is optional and **does not make the dog walk** in the current project.

Only proceed if:

- the new map is visually coherent;
- camera/odometry are stable;
- the displayed hosted map is the same map selected for Nav2;
- the project velocity output remains isolated from vendor motor control.

The workflow is:

1. Start the API with planner-goal opt-in only if you specifically want planner testing:

   ```bash
   source /opt/ros/foxy/setup.bash
   source ~/lite3_autonomy_ws/install/setup.bash
   ros2 run lite3_odom_tf_adapter robot_api --ros-args -p allow_nav2_goal_requests:=true
   ```

2. In the browser, select the new hosted map and load it.
3. Start **Localization/Nav2**.
4. Switch the map click mode to **Robot starting pose**.
5. Click the robot's approximate position on the map.
6. Enter the robot heading/yaw and press **Set robot start pose**.
7. Wait for AMCL/localization and the Nav2 action server to report ready.
8. For a planner-only check, select a known-free goal and submit it.
9. Cancel the goal after confirming the planner accepts it.
10. Confirm `/autonomy/cmd_vel` is still not relayed into vendor `/cmd_vel`.
11. Stop Localization/Nav2 when finished.

Do **not** create an ad hoc bridge to “make the last step work.” Physical autonomy remains blocked until command ownership, RC/autonomy arbitration, stop-on-loss behavior, localization, and obstacle handling are commissioned.

---

## 14. Shutdown / end-of-session checklist

- [ ] Stop mapping if it is running.
- [ ] Save the map with a unique name.
- [ ] Stop Localization/Nav2 if it was started.
- [ ] Stop a manually started API with `Ctrl+C` if you do not want it left running.
- [ ] Close any laptop SSH tunnel.
- [ ] Confirm the vendor RC app still works normally.
- [ ] Do not stop vendor transfer/RealSense services merely for cleanup.
- [ ] Back up the accepted `.pgm` + `.yaml` pair to the laptop project.
- [ ] Choose a unique map name for this location.
- [ ] Run the map quality report and keep its result with the scan notes.

Suggested session record:

```text
Date:                       __________________________
Physical location/room:     __________________________
Motion Wi-Fi IP:            __________________________
Perception IP:              __________________________
API URL used:               __________________________
Browser/device used:        __________________________
Map name:                   __________________________
Camera mount unchanged?     YES / NO
Mapping completed?          YES / NO
Map visually coherent?      YES / NO / REVIEW
Localization tested?        YES / NO
Planner-only goal tested?   YES / NO
Motor path connected?       NO  (must remain NO for this prototype)
Notes:                      __________________________
```

---

## 15. Recovery if the robot is restored to an older project build

As of 2026-10-06, the automatic browser session and 2D/3D UI are deployed and built on the robot. Use this section only if a later restore or image rollback removes that deployment. Before replacing anything, compare the robot project with the local repository and preserve any owner changes.

The project workspace should remain separate from vendor workspaces:

```text
/home/ysc/lite3_autonomy_ws
```

Do **not** overwrite `~/lite_cog`, `~/lite_cog_ros2`, vendor service files, or motion-side vendor software.

### Rebuild the project package on perception after a restore

After synchronizing only the project source/assets:

```bash
source /opt/ros/foxy/setup.bash
cd ~/lite3_autonomy_ws
colcon build --packages-select lite3_odom_tf_adapter
source install/setup.bash
```

Then run:

```bash
bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh
```

A successful build proves only that the ROS package compiles. It does **not** prove Wi-Fi reachability, map quality, Nav2 correctness, or safe autonomous movement.

### API autostart

The project contains:

```text
scripts/start_robot_api.sh
scripts/install_user_autostart.sh
```

The handoff records that a user-level `@reboot` entry was already installed on the perception computer. The latest check confirms it remains installed, and its launcher now keeps Nav2 goal requests disabled. Check before reinstalling:

```bash
crontab -l
```

Its log is:

```text
~/lite3_autonomy_ws/logs/robot_api_autostart.log
```

Do not create duplicate boot entries.

---

## 16. Troubleshooting by symptom

### “I can SSH to motion, but not perception”

That can be normal for this topology.

From motion:

```bash
ping -c 3 PERCEPTION_IP
ssh ysc@PERCEPTION_IP
```

From the laptop, use the motion computer as a jump host:

```powershell
ssh -J ysc@MOTION_WIFI_IP ysc@PERCEPTION_IP
```

For the laptop browser API, use the port-forward method in section 8.

### “The page does not open”

On perception:

```bash
curl -v http://127.0.0.1:8080/api/v1/status
ps -ef | grep '[r]obot_api'
tail -n 100 ~/lite3_autonomy_ws/logs/robot_api_autostart.log
```

If localhost works but another device cannot open it, the problem is the network route/firewall—not ROS mapping itself.

### “The page opens but mapping is disabled”

Check:

- browser session status (the current UI creates this automatically; no API key should be pasted)
- fresh RealSense camera status
- mapping is not already running
- `/leg_odom2` can be bridged fresh
- no second `parameter_bridge` is already running

Run:

```bash
bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh
```

### “Camera is missing/stale”

Read-only first:

```bash
source /opt/ros/foxy/setup.bash
ros2 topic list | grep camera
ros2 topic hz /camera/depth/image_rect_raw
```

Do not start randomly toggling vendor camera services until you know which service is expected to own the camera.

### “No odometry / mapping will not start”

The project relies on a narrow ROS 1 → ROS 2 bridge for `/leg_odom2` because the recorded Foxy motion receiver conflicts with the Noetic receiver on UDP `43897`.

Do not solve this by stopping Noetic transfer or changing the port. Use the project/API bridge logic and diagnose with preflight.

### “`Address already in use` on UDP 43897”

This is a **known vendor-stack conflict**, not a reason to kill whichever process owns the port. Noetic-dependent services rely on that path. Preserve the vendor stack and use the selected-topic odometry bridge design already in the project.

### “The map is stretched / duplicated / mostly unknown”

Do not move on to localization just because the map saved successfully. Check:

- camera mount has not moved
- depth stream is stable
- `/scan` and odometry are fresh
- robot is being driven slowly
- walls/corners are seen from enough viewpoints
- loop closure is not visibly breaking geometry

Save the bad scan under its own name for debugging; do not overwrite it or promote it to a navigation map.

### “Retroid can view a static map but cannot control mapping”

A laptop-hosted static `webapp/` can display map files but has no robot API at that origin. Mapping controls require the page to be served by `robot_api` (or an equivalent same-origin proxy to it).

### “Can I just bridge Nav2 `/cmd_vel` now?”

No. The current project deliberately stops before this point. The vendor manual command route and exclusive Manual/Autonomous arbitration are not validated, so connecting planner velocity output to the robot could bypass the safety assumptions of the prototype.

---

## 17. New-location success criteria

Consider the relocation/setup successful when all of the following are true:

- [ ] Laptop and controller are on the intended robot network.
- [ ] Startup script found the current Wi-Fi/gateway, opened the tunnel, and verified `/api/v1/status`.
- [ ] Laptop can reach the motion computer.
- [ ] Motion computer can reach the perception computer.
- [ ] Perception preflight completes without unexpected project processes or unsafe changes.
- [ ] RealSense data is fresh.
- [ ] Robot-hosted API is reachable from at least the laptop; Retroid route is separately verified if it is part of the demo.
- [ ] **Connect & check status** creates a browser session and refreshes statuses without starting processes.
- [ ] Vendor manual RC remains functional.
- [ ] Mapping starts and stops cleanly.
- [ ] A new, uniquely named PGM/YAML pair is saved for the new room.
- [ ] The new map is copied to the laptop and reviewed.
- [ ] No vendor port/service was altered as a shortcut.
- [ ] No project velocity topic was connected to the motor path.

At that point the **mapping demo is reconnected in the new location**. Planner/localization testing can be treated as a separate acceptance stage; physical autonomous navigation is still future work.

---

## Quick version — day-of-demo checklist

```text
[ ] Power robot
[ ] Open vendor Lite3 RC app in manual mode
[ ] Connect laptop + Retroid to robot Wi-Fi
[ ] Run .\scripts\Start-Lite3Session.ps1 from the laptop project folder
[ ] Confirm displayed SSID; enter SSH credentials in the visible SSH window if prompted
[ ] Wait for the read-only API status check; open the printed HTTP URL
[ ] Click Connect & check status; verify expected sensor/process states
[ ] Confirm fresh camera + healthy diagnostics
[ ] Start mapping
[ ] Drive manually with vendor RC
[ ] Stop mapping
[ ] Save NEW map name
[ ] Review map quality
[ ] Copy PGM/YAML to laptop
[ ] Stop project processes used for the session
[ ] Leave vendor services and motor command path untouched
```

## Startup tooling update (2026-10-18)

The automated laptop flow and UI check button are implemented locally. The first attempt on 2026-10-18 showed that `lite` did not resolve through YSC. The launcher was changed to read the perception address from YSC's `network.toml`; the user then reported that the connection was established and read-only API GETs from the laptop succeeded. Auto-deployment, new-location/fresh-boot behavior, browser visuals, and Retroid access remain unverified. The script reports a status timeout without launching a duplicate API and does not modify firewall/network settings. The UI check is limited to session/status GETs and does not start or stop robot processes.

## Historical robot-side snapshot (2026-10-06)

Laptop SSH to the YSC motion computer and the nested SSH connection to the lite perception computer both work. The API responds through the existing laptop SSH tunnel at `http://<laptop-wifi-ip>:18080/`; the tunnel is bound to the laptop's current Wi-Fi address. The laptop API URL has been checked from this session; Retroid access through that tunnel is not freshly verified. Use HTTP, never HTTPS, for this port.

The live read-only preflight found active vendor transfer/RealSense services and fresh camera data, but no odometry, scan, or map publishers. Mapping/Nav2 were stopped. Planner requests remained disabled and motor output isolated. This is a historical robot-side snapshot; see the 2026-10-18 API snapshot at the top and in [STATUS.md](STATUS.md).

The battery-change action is now deployed: type `STOP PROJECT` to stop project-owned mapping/Nav2/bridge processes and close the project API. It leaves the vendor services and Linux computer running. Close the laptop tunnel separately, then use the robot's normal power-off procedure and wait until the computer is off before removing the battery.

Every browser that can reach the robot UI can obtain a 12-hour control session. Use only trusted devices on the robot Wi-Fi. A pause freezes mapping processes but does not stop the robot; hold it stationary with the manual controller. The stitched view is a sparse review aid and needs map-to-camera TF; it is not cleared for navigation.

## Latest navigation readiness note - 2026-10-18

If the robot marker appears on the map but **Send selected point to Nav2** is disabled, read the live status reasons instead of relaxing thresholds. The current diagnosis found Nav2's public costmap topics use `nav_msgs/msg/OccupancyGrid` while the deployed API expected `nav2_msgs/msg/Costmap`; the API therefore had no fresh costmaps. AMCL also needs to finish its convergence checks. A point on the map is only a preview until map identity, converged localization, transforms, fresh local/global costmaps, and action server all pass.

Local source v20 corrects the costmap subscription, clears stale AMCL convergence when stable samples are lost, and adds a separate explicit planner-goal opt-in after readiness and matching the active map. It is not deployed yet. The motor output remains isolated, and the UI's software STOP is not a physical E-stop. Do not lower freshness/localization thresholds or attempt a walk based on UI green indicators alone. Schedule API deployment only when ready for the current API-owned Nav2 stack to stop and then be restarted from the UI.


## Current compatibility and reboot note — 2026-10-19

The API and guarded vendor relay have been verified in a clean ROS 2 Foxy Python environment. `geometry_msgs/Twist` and `nav2_msgs/NavigateToPose` load with valid type support. The C++ `ros1_bridge` intentionally uses both Noetic and Foxy because it translates the vendor ROS 1 graph. The startup helper now starts the project relay in a disabled state and the read-only preflight checks the message types automatically; no motion command is enabled at boot.

After connecting the tunnel, use the UI's explicit localization/global-localization action if the status says the localization refresh stopped. Wait for fresh AMCL and re-enable planner requests only after the map marker is checked against the stationary robot. The motion gate and independent-stop requirement remain in force for a walking test.
