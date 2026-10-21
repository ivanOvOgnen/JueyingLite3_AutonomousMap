
## Tomorrow: larger-room mapping validation

The larger-room scan is manual. After saving/reviewing a map, the app can start localization/Nav2 and exercise AMCL pose + planner goal/cancel with the velocity gate still isolated. Do not connect planner output to the dog.

**Current deployment gate (2026-10-04):** pairing source is deployed and built. The owner-supervised stationary API check is running with goal requests disabled; local tunnel GET checks pass, while Chrome rendering and Retroid access remain unverified. The API reports an existing token (`api_paired=true`), so the first-claim Pair button is not expected. Current preflight has `/leg_odom2` with zero publishers and no `/odom`, `/scan`, or `/map`; do not begin a mapping scan until odometry is restored and fresh. Do not press mapping or Nav2 controls during the UI check. Physical mapping requires separate explicit owner authorization with the owner present to supervise.

1. Connect the controller and laptop as needed, check battery, clear the room, confirm the camera mount is unchanged, and confirm the vendor RC app is in manual mode. Do not start if the dog or room is not safe for manual walking.
2. After boot, wait about one minute for the user-level auto-started API, then open the robot-hosted page in Chrome. If the page/API cannot be reached from the controller/laptop Wi-Fi, record the exact URL and error; network routing to the perception host is not yet verified. Do not change vendor services to make it reachable.
3. On the controller's trusted Chrome browser, press **Pair this browser with the robot** once. This creates/stores the API key locally; use the same browser for the presentation. Run the read-only preflight from the page's **Robot diagnostics** card. The SSH fallback is `bash ~/lite3_autonomy_ws/scripts/robot_preflight.sh`. If required camera/odometry data is missing, or the command path/UDP owner is unclear, pause and investigate. Do not start or bridge motor commands.
4. Check live status in the page. API controls need a fresh depth camera; mapping start additionally requires the `/leg_odom2` bridge to become fresh. If either check fails, stop and diagnose before walking.
   - Quick interface check: confirm the saved-map list loads, the map renders, and clicking only selects a point; it never sends a goal automatically. Verify cell percentages against the paired YAML thresholds. If browser access or status is broken, capture the exact URL/error and stop before starting the scan through the page.
5. Keep the vendor RC app in manual mode. Start mapping from the page only when ready, then user guides the dog slowly around the larger room and doorways, returning near the start if safe. Watch for map shearing, duplicated walls, or instability; stop the scan if seen. The assistant does not send movement commands.
6. Stop mapping from the page after the user confirms the scan is done. Save a uniquely named map through the page. Copy the matching PGM/YAML pair from the robot to laptop `webapp/maps/`. Generate a report with `python .\scripts\map_quality_report.py .\webapp\maps\lite3_room_scan_YYYYMMDD.pgm .\webapp\maps\lite3_room_scan_YYYYMMDD.yaml`, substituting the actual name. Review map continuity, coverage, occupied/free/unknown proportions and compare with the saved scan. Large dimensions alone do not mean a good map.
7. If the map is distorted or mostly unknown, preserve evidence and stop; do not use it for localization or click-to-go. If coherent, next run separate stationary localization and obstacle-layer checks, with the velocity gate disconnected from the robot.
8. Stop mapping and save the map. Select the new robot map and click **Load hosted map** so the displayed canvas matches the map used by Nav2. Start Localization/Nav2. Set map-click mode to **Robot starting pose**, click the robot's approximate location, enter its heading, and press **Set robot start pose**. Confirm AMCL and the Nav2 action server become ready.
9. If explicitly enabled for the planner-only check, send a known-free goal and cancel it; confirm `/autonomy/cmd_vel` is not bridged to vendor `/cmd_vel`. Stop Localization/Nav2 and the API afterward.

Map capture was previously done with a QoS-matched one-shot subscriber because Foxy's `map_saver_cli` timed out. The exact host and process state may change after reboot; recheck before reusing temporary commands.

## Closeout criteria

Call the **mapping demo** ready to package only if all of these are observed and recorded:

- [ ] API page loads from both laptop and Retroid Chrome over the intended Wi-Fi route.
- [ ] Status is current, and a wrong API token does not authorize mapping/save actions.
- [ ] Mapping starts once, publishes a growing map, and stops cleanly without interfering with the vendor RC app or its services.
- [ ] A uniquely named PGM/YAML pair saves on the robot, appears in the map list, reloads in the UI, and its dimensions/coverage/visual structure are documented.
- [ ] Copy the accepted PGM/YAML pair into the laptop's `webapp/maps/` folder, then package with `powershell -ExecutionPolicy Bypass -File .\scripts\package_demo.ps1`; confirm the resulting zip contains the accepted pair.
- [ ] API exits cleanly and vendor RealSense/transfer services remain active.

This validates mapping plus the app-to-AMCL/Nav2 planner path only. A planner-only success does not move the robot. Motor-connected navigation remains blocked on map quality, verified localization/obstacle behavior, UDP 43897 ownership, and a tested manual/autonomous command interlock with an independent stop.
