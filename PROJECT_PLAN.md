# Lite3 autonomous-navigation MVP

Last updated: 2026-10-20 — map data and map-viewer assessment after the robot power cycle.

## MVP

From the existing web page: load a saved map, show the localized robot marker,
click a free destination, send a Nav2 goal, and cancel/stop it.  During the
first walk the existing speed limits and vendor stop/manual controls remain in
place.

## What is already implemented

- The robot API and web page provide saved-map loading, initial-pose setting,
  AMCL/Nav2 startup, map-click goals, goal cancellation, and software stop.
- Nav2 commands are routed through the project gate at `/nav2/cmd_vel` and
  bounded to 0.10 m/s and 0.20 rad/s.
- The ROS 1/ROS 2 bridge carries odometry to ROS 2 and `/cmd_vel_corrected` to
  the vendor ROS 1 subscriber.  The guarded relay is disabled by default.
- Previous live checks proved that camera, odometry, scan, map, transforms,
  costmaps, AMCL, and the Nav2 action server can all become ready on map
  `v3_soba_ogi_test2`.

## Actual remaining work

| Order | Task | Completion evidence | Status |
|---|---|---|---|
| 1 | Reconnect and take one read-only live snapshot. | Current API status and ROS topic/service graph recorded; no assumed address or process state. | Complete — API reachable at the live tunnel; camera, odometry, scan, static map, TF, costmaps, and Nav2 are ready. |
| 2 | Restore a trustworthy initial pose if the snapshot shows AMCL is stale or visibly wrong. | AMCL pose is fresh, stable, and agrees with the robot in the room. | Blocked by map quality — after the power cycle no map/localization is loaded, and all saved maps have insufficient coverage for a trustworthy visual match. Do not seed AMCL with guessed coordinates. |
| 3 | Exercise the existing planner with motor output still disabled. | A nearby free-cell goal is accepted (or its precise Nav2 rejection is fixed); cancel works. | Complete — an initial candidate was correctly rejected for insufficient clearance; a 0.30 m-clear, 0.45 m candidate at `(0.014, -0.009)` was accepted by Nav2 and then canceled. Motion output stayed isolated. |
| 4 | Close the one command-path gap: publish the existing navigation readiness signal and start the existing guarded relay only for the test profile. | Gate, arbiter, and relay accept their documented readiness/stop/manual conditions while output remains disabled until the walk. | Complete — root cause was a DDS implementation mismatch: Nav2/gate use `rmw_cyclonedds_cpp`; the clean-environment relay/arbiter defaulted to Fast DDS, so service RPCs never completed. `start_vendor_cmd_vel_bridge.sh` now explicitly uses Cyclone DDS. With both nodes launch-locked false, arbiter and relay disable calls returned immediately; enable calls returned the expected physical-stop refusal. Backup: `/home/ysc/lite3_autonomy_ws/backups/20261020/rmw_alignment_predeploy_154406/`. |
| 5 | First supervised walking test. | With the operator beside the robot and explicit approval at that moment, enable the guarded path and send a very short free-space goal; cancel/stop is immediately available. | Pending — power-cycle state is manual with Nav2/mapping/localization off and software STOP latched. No earlier walking authorization carries over. |
| 6 | Final UI verification. | Map click, pose marker, navigate, cancel/stop work end to end. | In progress — map viewer v24 is deployed; it awaits a visually usable map and browser check. |

## Rules for this execution

## Latest milestone - 2026-10-20

- Localization investigation: `/leg_odom2` is current but stationary at `(0,0)`
  with blank source frame IDs (the adapter supplies `odom` and `base_link`).
  The 2D scan is current, has 380 valid ranges, and has a complete TF chain.
  The raw RealSense point cloud has a non-ROS timestamp, explaining its local
  costmap freshness warnings but not AMCL's scan input. AMCL global
  relocalization, without an initial-pose guess, settled at a third unverified
  pose near `(0.324, -0.173, 163 degrees)`; it cannot uniquely resolve the
  stationary robot from the current view.
- After manual repositioning, the vendor odometry was verified directly on ROS
  1 and ROS 2 at approximately `(0.562, 0.000, 0.326)`; blank vendor frame IDs
  are normalized by the adapter. Global AMCL still did not converge within its
  90-second no-motion window, so the remaining localization blocker is map/scan
  alignment ambiguity rather than a dead odometry or TF path.
- The arbiter/relay timeout was caused by a DDS mismatch, not an executor
  deadlock, blocking callback, or safety shared-memory read. The Nav2 gate uses
  Cyclone DDS while the clean relay startup defaulted to Fast DDS.
- The smallest deployed fix is the existing relay startup script exporting
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- Live test-profile results: relay enable returned `success=true`; relay
  disable returned `success=true` and sent zero; arbiter disable returned
  `success=true` and sent isolated zero. The gate remains disabled.
- The first armed goal at `(0.014, -0.009)` was accepted, then Nav2 reported
  result status `6` (aborted) before the planned API cancellation. Gate,
  relay, and arbiter were immediately disabled again. Nav2 logged that its
  behavior tree could not send its internal path-planning goal. A direct
  `ComputePathToPose` call succeeds, so the planner endpoint itself is live.
- The direct planner path and current AMCL now place the robot near `(0.35,
  0.60)`, substantially different from the earlier claimed `(0.464, -0.209)`
  starting pose. This is a demonstrated localization discrepancy, so no
  additional motor-enabled goal will be sent until the displayed marker is
  confirmed again at the current live position.
- The next physical action is to recheck fresh navigation, enable arbiter and
  relay, enable the gate last, and submit the verified short goal `(0.014,
  -0.009)`. Cancel promptly after observed motion, then disable all three.

- Fix only a demonstrated blocker in the row being worked.
- No physical command or motion-capable goal is sent without the operator's
  explicit confirmation immediately before it.
- Do not change vendor services, networking, UDP ownership, or the manual RC
  path.  Project-owned changes receive a dated robot backup before deployment.
- A failed or noncritical check is documented here and the next applicable
  task continues.

## Next action

Create one visually usable saved map by a slow, manually driven closed loop of
the room with the existing mapping pipeline. Then load that exact map in the
viewer, start localization, and validate map-to-scan/TF alignment before any
planner request. Keep the gate, relay, and planner requests disabled.

## Map visualization milestone — 2026-10-20

- Read-only live status after the power cycle: API and camera are ready; mapping,
  odometry, scan, TF, localization, Nav2, and costmaps are stopped/not ready;
  software STOP is latched and control mode is manual. No robot control route
  was called.
- The saved-map catalog was read without starting any robot process. Its maps
  are too sparse to be recognizable floor plans: `lite3_room` 63.3% unknown,
  `lite3_room_scan_20260930` 74.8%, `room_scan_review_20261004_01` 50.0%,
  `v2_test1` 77.1%, `v3_soba_Ogi_2026` 51.0%, and the best available
  `v3_soba_ogi_test2` 47.1%. The underlying map data is therefore a real
  blocker, not only a rendering problem.
- UI version 24 is deployed after dated backup
  `/home/ysc/lite3_autonomy_ws/backups/20261020/map_viewer_v24_predeploy_20261020_220000/`.
  It corrects inverted live-map occupancy colors, consistently renders free
  space light, occupied cells dark, and unknown cells gray, adds a legend,
  meter grid, wheel/button zoom, drag-pan, fit reset, and a clear verified vs.
  unverified AMCL pose marker. Map-click coordinates retain the saved YAML's
  resolution, origin, and yaw transform. No API restart, mapping/Nav2 launch,
  goal, velocity, gate, relay, or vendor setting changed.

## Mapping and spatial-alignment preparation — 2026-10-20

- Read-only live verification after the power cycle confirms `control_mode=manual`,
  mapping/Nav2/localization off, software STOP latched, and motor output isolated.
  The RealSense depth camera is live (`/camera/depth/image_rect_raw`,
  `/camera/depth/camera_info`, and `/camera/depth/color/points` from `/camera/camera`).
- The active 2D mapping path is: RealSense depth image → `scan_info_adjuster`
  (20-degree pitch row compensation) → `depthimage_to_laserscan` → delayed `/scan`
  → `slam_toolbox` OccupancyGrid. Vendor `/leg_odom2` is bridged only while the
  project mapping stack runs, then normalized/restamped as `/odom` and supplies
  `odom → base_link`. During mapping, static project transforms define
  `base_link → camera_link`, `camera_link → camera_depth_optical_frame`, and the
  level `camera_scan_frame`; SLAM publishes `map → odom`. The web viewer uses
  the saved map YAML's resolution, origin, and yaw to invert raster pixels and
  convert map clicks, matching the grid metadata exposed by the API.
- `/rslidar_points` is only a named `PointCloud2` topic at present: the read-only
  graph showed zero publishers, so it is not evidence of an available LiDAR and
  is not part of the current mapping path.
- No alignment claim is justified yet. With mapping off, `/leg_odom2`, `/odom`,
  `/scan`, `map`, and the complete `base_link` TF tree are intentionally absent.
  The project source also records that the camera mount and pitch-compensated
  scan plane need a live validation. Timestamp freshness, scan-to-TF lookup at
  scan time, scale, handedness, and physical alignment are therefore explicitly
  **unverified**, not treated as defects or calibrated by guesswork.
- Fixed the viewer deployment cache key to `app.js?v=24` and HTTP-verified it.
  The cache-fix backup is
  `/home/ysc/lite3_autonomy_ws/backups/20261020/map_viewer_v24_cachefix_predeploy_20261020_223000/`.
  This copied only `index.html`; no project process or robot-control route changed.

### Next controlled mapping validation (manual motion only)

1. With the dog stationary, use the page's **Start mapping** button. Confirm the
   page reports fresh odometry, scan, map, and required TF; do not drive yet.
2. Record three fixed references: the starting wall corner, a second wall corner,
   and one straight wall segment measured with a tape measure. Photograph or note
   the measurements and the robot's initial heading.
3. Only after the stationary checks pass, manually guide one slow closed loop:
   start at the first corner, travel along the measured wall, visit the second
   corner, and return near the start. Keep the RealSense facing unobstructed walls;
   avoid turns in place and abrupt motion.
4. Pause at the start/end locations and compare the map's wall direction,
   corner-to-corner distance, and robot heading to the recorded references. Note
   duplicated walls, drift, scale error, or rotation/translation offset rather
   than estimating accuracy by appearance.
5. Stop mapping, save a uniquely named map, and provide that name plus the three
   measurements. Evaluate it before localization/Nav2. Gate, relay, and planner
   requests remain disabled throughout.

## Stationary live spatial-validation result — 2026-10-20

- Mapping was started by the operator while the robot remained stationary. Live
  status verifies manual control, software STOP latched, mapping running, fresh
  odometry/scan/map/point-cloud input, complete required TF, Nav2 off, planner
  requests off, and motor output isolated.
- Verified mapping inputs: `/camera/camera` publishes a 424×240 RealSense depth
  image and calibrated `CameraInfo` in `camera_depth_optical_frame`
  (`fx=fy=216.1861`, `cx=213.0693`, `cy=120.4023`), plus depth point cloud and
  metadata. It does not currently publish RGB, stereo/infrared, or camera IMU
  streams. The ROS 1 vendor interface additionally publishes leg odometry, IMU,
  joint states, and a single ultrasound range. Only leg odometry is deliberately
  bridged to ROS 2 for mapping. `/rslidar_points` has zero current publishers in
  both ROS graphs, so it is not treated as available hardware.
- Verified current 2D path: depth image → pitch-adjusted camera info →
  `depthimage_to_laserscan` → delayed `/scan` → `slam_toolbox`. The scan had
  374 valid of 424 ranges, `camera_scan_frame`, 0.45–6.0 m limits, and 0.5745 s
  age, consistent with the configured 0.5 s TF-wait delay. `/leg_odom2` and the
  normalized `/odom` are current; the vendor header frame IDs are blank, while
  the adapter correctly publishes `odom → base_link`.
- TF and map metadata are internally consistent at rest: map resolution is
  0.05 m; `map → odom` is identity at startup; `odom → base_link` yaw was about
  -1.28°; static `base_link → camera_link` is `(0.25489, 0, 0.07249)` m with
  20° pitch; and `base_link → camera_scan_frame` shares that translation and is
  level. This confirms frame connectivity and software coordinate conventions,
  but cannot establish physical mount accuracy, odometry scale, or map
  handedness without the planned measured manual loop.
- Confirmed defect: `/camera/depth/color/points` has a non-ROS header timestamp
  (`3,225,308,419,000 ns`, appearing ~1.726 billion seconds stale) while the
  depth image, camera info, odometry, scan, and map are current ROS time. This
  makes map-time TF lookup fail for the raw cloud, explaining the empty stitched
  point-cloud view. It does not feed the present 2D SLAM scan, so it does not
  block the controlled mapping loop. Do not use this raw cloud in Nav2 costmaps
  or map-frame visualization until a minimal project-owned timestamp-normalizing
  relay has been justified and tested.

### Prioritized improvement path

1. Proceed with the measured manual 2D mapping loop unchanged; it is the
   shortest test of scan-plane geometry, wall scale, odometry scale, and loop
   closure. Do not infer those properties from this stationary check.
2. Compare the saved map to measured wall/corner references and evaluate the
   return-to-start error before changing SLAM parameters or transforms.
3. If the 2D map passes geometry checks, separately normalize the raw depth
   point-cloud timestamp before enabling its existing costmap/stitching uses.
   That is a contained compatibility repair, not a new SLAM framework.
4. Consider bridging/calibrating the vendor IMU only if measured loop drift or
   yaw error remains material after the scan-plane check. Its zero covariance and
   unverified frame calibration mean it is not currently suitable as a drop-in
   fusion input. Joint states and a single unframed ultrasound range are not
   useful for room geometry today.
5. Do not add 3D SLAM, visual-inertial fusion, or a new navigation stack now.
   A timestamp-correct depth cloud can later improve obstacle representation, but
   it will not substitute for a measured, geometrically sound 2D map required by
   AMCL/Nav2 and the existing web viewer.

## Manual mapping procedure preparation — 2026-10-20

- Live configuration supports a conservative first loop: the depth image was
  measured at 29.59 Hz over five seconds; scan geometry is 424 samples across
  about 89° horizontal field of view with 0.45–6.0 m limits; scan messages carry
  the intentional 0.5 s delay; and SLAM's configured keyframe thresholds are
  0.5 m translation or 0.5 rad (28.6°) rotation. The direct Foxy `ros2 topic hz`
  command did not return scan/odom samples in this graph, so those two rates are
  not claimed as independently measured.
- Use a conservative **estimated** manual speed of 0.05–0.10 m/s and 10–15°/s
  turns. These are not vendor-controller calibrated values; they are chosen to
  provide roughly 5–10 seconds and many depth frames per SLAM translation
  threshold. Keep 0.8–1.5 m from walls, approach corners obliquely, and make
  gradual curved turns. Avoid reverse/sideways travel and repeated full in-place
  turns; a single slow 30–45° look-around with a 2-second pause is acceptable at
  a corner when needed for coverage. Pause 2–3 seconds at reference corners;
  wait up to 5 seconds only when checking a displayed map update.
- Existing process logging is at
  `/home/ysc/lite3_autonomy_ws/logs/robot_api_mapping.log`. A full Foxy rosbag
  attempt could not subscribe reliably to the best-effort scan graph and the
  clean Foxy environment lacks the vendor metadata message package, so it was
  stopped without changing mapping. A session-local evidence recorder is instead
  running as PID `10013`, writing
  `/home/ysc/lite3_autonomy_ws/records/mapping_alignment_20261020_session4.jsonl`.
  It records full `/scan` ranges, decimated vendor and normalized odometry, map
  grids, map/odom/base/camera TF samples and failures, camera intrinsics/depth
  headers, and raw point-cloud timestamp headers. It had written 1,069 records
  (249 KB) during its stationary verification. This recording publishes nothing.
- Before manual movement, watch for fresh green camera/odometry/scan/map/TF and a
  running Mapping state. Stop immediately for stale/frozen states, process
  failure, persistent TF errors, a marker jump unrelated to motion, doubled or
  rotating walls, or geometry that changes while stationary. Small local wall
  adjustments after revisiting a corner are normal SLAM correction; a coherent
  previously mapped region shifting or rotating is not.

## Battery-replacement reference experiment — 2026-10-20

- New runtime verified read-only after battery replacement: API and UI v24 are
  reachable; control mode is manual; camera is fresh; mapping, odometry, scan,
  map, localization, and Nav2 are off; planner requests are disabled; software
  STOP is latched; motor output remains isolated. The previous stationary record
  `/home/ysc/lite3_autonomy_ws/records/mapping_alignment_20261020_session4.jsonl`
  is preserved (3.3 MB); 58 GB remains free.
- The reference shape is an observed L, not a rectangle: A–B = 1.685 m, B–C =
  1.030 m, and angle ABC = 90°. D is open space and must never be treated as a
  wall or measured corner. Absolute map orientation is irrelevant; measurements
  use only segment lengths and their included angle.
- A new passive recorder is running as PID `5410` at
  `/home/ysc/lite3_autonomy_ws/records/mapping_reference_20261020_battery2.jsonl`.
  It is a unique file, publishes nothing, and will capture the mapping startup,
  both stationary observations, and movement between them. Vendor Move/Pose state
  has no verified ROS observation, so `OBSERVATION_AB_*` and `OBSERVATION_BC_*`
  boundaries must be logged from explicit operator confirmations; their timing is
  approximate to that confirmation, not asserted to match a button press exactly.

## Physical-posture prerequisite — 2026-10-20

- Mapping must not start until the operator has visually confirmed the Lite3 is
  in its normal, stable standing posture with neutral body orientation and no
  intentional controller input. `control_mode=manual` establishes command mode,
  not standing posture.
- Read-only inspection verified that `/joint_states`, `/imu/data`, and
  `/leg_odom2` are available, along with a vendor
  `/emotion_bot/hardware/robot_state` topic. The running diagnostic ROS
  environment cannot load that vendor message definition, and no documented
  posture field has therefore been verified. The handle state is only a zero
  `geometry_msgs/Twist`. Raw joint angles, a level IMU, and planar odometry do
  not reliably distinguish standing from a low/belly posture without a
  vendor-calibrated posture definition. They must not be used to infer stand.
- Because this is the first Stand transition after the battery replacement, if
  the robot is not already visibly upright, the operator must use the official
  vendor app's **Stand** procedure once, with clearance and supervision. Do not
  use Move/Pose controls and do not start mapping yet. No project process will
  command posture or movement.
- After the operator confirms stable standing, repeat the stationary
  camera/odometry/scan/TF validation in that actual posture, then begin the
  manual mapping procedure. The passive recorder remains running and autonomy,
  planner requests, and motor output remain disabled.

### Standing-posture stationary baseline — 2026-10-20

- Operator confirmed normal stable standing. The existing mapping pipeline was
  then started without any project motion command. Live status is manual mode,
  camera/odometry/scan/map/required TF ready, Mapping running, software STOP
  latched, planner requests disabled, and motor output isolated.
- The stationary baseline is healthy: `/odom` measured about 144 Hz and held
  `odom→base_link` at `(-0.161, -0.142, 0.326)` m with yaw about `-0.9°` during
  the four-second TF observation; `/scan` measured about 19 Hz; depth measured
  about 28 Hz. Static `base_link→camera_scan_frame` is `(0.255, 0, 0.072)` m
  level, and `base_link→camera_link` has the configured 20° pitch. These verify
  live frame continuity and a stationary input baseline, not physical extrinsic
  accuracy or odometry scale.
- The passive evidence recorder remains active (PID `5410`) and has recorded
  19,739 lines. It still publishes nothing.

### Corrected reference-observation procedure — 2026-10-20

- **Correction:** A–B (`1.685 m`) and B–C (`1.030 m`) are physical wall
  dimensions for later map validation, not robot travel distances or waypoints.
  The robot must not be directed to drive to A, B, or C.
- `OBSERVATION_AB`: operator uses vendor **Move** mode only to place/orient the
  robot safely in open space with both physical A and B corners visible to the
  forward RealSense; switches to vendor **Pose** mode; makes no joystick input
  for 20 seconds; then explicitly confirms the observation start and end.
- `OBSERVATION_BC`: operator returns to **Move** mode and only repositions or
  rotates enough to see B and C safely; switches to **Pose**; remains still for
  20 seconds; then explicitly confirms the observation start and end.
- Sequence: **MOVE → observe A/B → POSE 20 s → MOVE → observe B/C → POSE 20 s**.
  The recorder labels each interval only from the operator's explicit
  confirmations. Mapping, SLAM parameters, autonomy, and project motion output
  are not changed for either observation.
- `OBSERVATION_AB` was completed from explicit operator start/end confirmations.
  Append-only `OBSERVATION_AB_START` and `OBSERVATION_AB_END` markers are in the
  active evidence record. The next observation is B–C; no map-quality conclusion
  is drawn until both reference observations and the later geometry comparison
  are available.
- `OBSERVATION_BC` was completed from explicit operator start/end confirmations.
  Its append-only `OBSERVATION_BC_START` and `OBSERVATION_BC_END` markers are
  also recorded. Post-observation health remains camera/odometry/scan/map/TF
  ready with Mapping running and motor output isolated. The local map grew to
  `1.20 × 1.80 m`, 73.0% observed, with 96 occupied cells; this is sufficient
  evidence that both stationary views reached SLAM, but not yet a geometry pass
  against the physical wall dimensions.

### Reference-observation analysis — 2026-10-20

- The append-only marker intervals contain 1,624 scans / 1,502 odometry samples
  for AB and 1,550 scans / 1,409 odometry samples for BC. Their logged
  start-to-end spans are 84.4 s and 78.9 s, respectively; operator confirmation
  delay means that is **not** proof of an exact 20-second Pose duration. Both
  recorded base poses are stationary within logged precision (zero x/y range;
  yaw standard deviation 0.031° and 0.029°).
- The scan-derived dominant AB plane has 337 inliers, a 1.679 m fitted span
  (physical reference 1.685 m, -0.006 m), 5.4 mm residual, and 1.065 m
  perpendicular camera-to-wall distance. BC has 229 inliers, a 1.022 m span
  (reference 1.030 m, -0.008 m), 5.9 mm residual, and 1.002 m perpendicular
  distance. These are independent distances; neither view was rescaled.
- Valid scan coverage was AB 376/424 beams from -44.16° to +42.28° and BC
  372/424 beams from -44.16° to +41.44°. Median temporal range scatter was
  3.4 mm and 2.4 mm. No adjacent median-range jump exceeded 12 cm, so endpoint
  identity comes from the fitted visible line extent and operator-confirmed
  visual scene, not an automatic corner discontinuity.
- Each dominant line is distinguishable and its length is consistent with the
  stated physical wall, showing no evidence that 5 cm map cells or the synthetic
  scan fundamentally prevent this measurement. The `1.20 × 1.80 m` map extent
  remains only the current observed bounding box, never a wall-length measure.
- Applying the contemporaneous transforms (AB map→odom identity; BC
  map→odom translation `(-0.192,-0.161) m`, yaw `+4.199°`) yields an 85.13°
  acute wall angle (4.87° from 90°). The nearest candidate AB/BC endpoints,
  presumed B, are 0.114 m apart in map frame. This is larger than grid
  resolution and raw fit residuals, so shared-corner consistency is
  **unverified**. It is more consistent with registration/odometry/SLAM
  correction or partly visible endpoint ambiguity than depth scale error; the
  observations do not isolate those causes or prove a faulty static transform.
- Smallest next evidence: one additional stationary Pose observation from a safe
  open, bisector-facing view with B centered and visibly including a short
  portion of both walls. It supplies a single common scan-frame observation of
  the corner before any full-room mapping; do not change SLAM, projection, or TF
  parameters first.
- `OBSERVATION_B_COMMON` start/end markers were appended after the operator
  obtained that bisector-facing view. Its 1,132 scans have 372 valid beams over
  -44.2° to +41.4° and 2.2 mm median temporal scatter. Two independent planes
  are directly visible in one scan frame: 0.840 m and 0.709 m supported spans,
  with 6.0 mm and 2.3 mm residuals. Their normals differ by 85.41°. This
  confirms the shared corner is observable locally; it does not by itself erase
  the earlier 0.114 m cross-view map-frame mismatch. Preserve this record and
  analyze the common-view map-frame registration before any full-room mapping.

### Common-corner map-frame comparison — 2026-10-20

- Applying each observation's recorded map→odom and odom→base transforms at its
  scan interval gives B candidates: AB `(1.135,-1.069) m`, BC
  `(1.246,-1.096) m`, and direct common-view intersection `(1.225,-1.081) m`.
  Thus BC agrees with the direct common view to 2.6 cm; AB differs by 9.0 cm.
  The earlier AB-to-BC endpoint separation was 11.4 cm.
- Map→odom changed from identity during AB to `(-0.192,-0.161) m, +4.199°` in BC
  and `(-0.224,-0.238) m, +2.600°` at the common view. That time-varying SLAM
  correction/registration is the demonstrated source class for the global
  discrepancy; static camera extrinsics remain unproven, not implicated by a
  specific defect. AB/BC wall spans remain within 8 mm of physical dimensions,
  while local temporal scatter (2.2–3.4 mm) and line residuals (2.3–6.0 mm) are
  much smaller than the map-frame offset.
- The common-view 85.40° angle repeats the earlier 85.13° result (0.27°
  difference), so it is a repeatable scan-plane measurement rather than random
  point noise. It may be physical non-perpendicularity or a systematic
  projection/extrinsic bias; current data cannot distinguish those. Do not force
  it to 90° or apply a correction.
- No responsible calibration defect is demonstrated. Keep the architecture
  unchanged. One safe, slow manual loop that revisits the common B view after
  observing another nonparallel feature is the next informative registration
  test before full-room mapping.

### Live mapping and vertical-projection review — 2026-10-20

- The manual session remains live: camera, odometry, scan, map, and required TF
  are fresh; Mapping is running; Nav2, planner requests, and motor output remain
  disabled. The expanded map is 107 × 85 cells at 0.05 m (5.35 × 4.25 m), with
  36.5% observed. Its extent is coverage, never a wall measurement.
- The active converter uses a 10-pixel horizontal depth band, 0.45–6.0 m range,
  and 0.033 s scan time. With `fy≈216 px`, `cy≈120 px`, and configured 20°
  downward camera pitch, the project scan-info adjuster selects row `cy≈42` so
  the sampled physical ray is level in `base_link`; the output is in the static
  level `camera_scan_frame`. This is geometric row selection, not a claim that
  all depth rows are transformed into a level scan.
- No data demonstrates a vertical-projection defect. The current structural
  scan can legitimately include an elevated object only where it intersects its
  narrow selected slice. It cannot represent floor support, overhead clearance,
  or the whole collision envelope; a 5 cm occupancy display may make such cells
  appear visually larger. Keep it for 2D SLAM. A later separate height-filtered
  3D navigation-obstacle layer is the appropriate option if a demonstrated
  collision-envelope limitation appears; do not replace SLAM or alter TF now.

## Mapping-quality decision and MVP readiness — 2026-10-20

### Evidence completed

- The reference scans resolve AB at `1.679 m` (known `1.685 m`) and BC at
  `1.022 m` (known `1.030 m`) with 2–6 mm straight-line residuals and 2–3 mm
  temporal scatter. The common B view repeats an `85.40°` angle rather than the
  physical nominal 90°. It is repeatable but its cause (true room geometry vs.
  systematic slice/extrinsic bias) is unverified; no correction is justified.
- Timestamp-matched transforms show dynamic SLAM registration: AB used identity
  map→odom; BC used `(-0.192,-0.161) m, +4.199°`; the common view used
  `(-0.224,-0.238) m, +2.600°`; later expanded mapping showed another large
  correction. BC-to-common B agreement is 2.6 cm, but AB-to-common is 9.0 cm.
  The map is therefore locally precise but not yet globally reference-verified.
- `OBSERVATION_AB_OBLIQUE` start/end markers were appended while the robot was
  stationary. The current stationary pose and map→odom correction are fresh,
  but a complete oblique line-to-reference fit was not produced before the
  active session expanded further; it cannot retrospectively establish a fixed
  global map frame after subsequent SLAM corrections.

### Verdict

- **Do not use the live, still-changing map for localization/Nav2 yet.** It is a
  useful experimental mapping record, but present evidence does not verify a
  stable global frame, duplicate-wall absence, or a map-frame B error below the
  5 cm grid resolution. Preserve it; do not discard it or change SLAM/TF.
- No demonstrated project-owned scan-projection, camera-pitch, or Nav2 software
  defect requires a code change. Existing map save/load, AMCL, Nav2 gating,
  readiness heartbeat, cancel/stop, and isolated relay architecture are
  implemented. Live localization/Nav2 remains correctly off while mapping runs.

### Genuine remaining checkpoint

- The smallest necessary operator action is to finish manual coverage with one
  slow loop that revisits the already observed B/common view, then hold Pose so
  the recorder can test loop-closure consistency. After that, the operator must
  decide whether to save the resulting map under a unique experimental name;
  saving is an external state change and must not overwrite an existing map.

## Offline saved-map infrastructure — 2026-10-20

- Local source now writes a backward-compatible `<map>.lite3-map.json` manifest
  alongside every newly saved PGM/YAML pair. It records the ROS `map` frame,
  metric origin/yaw, resolution, authoritative 2D files, and explicitly marks
  3D reconstruction, global registration, and localization as unverified until
  actual evidence is supplied. It does not change Nav2's PGM/YAML inputs and it
  does not fabricate a point cloud.
- The web review page now includes an offline interactive PLY viewer (orbit,
  Shift-drag pan, zoom, fit/reset and point-count status) for a separately
  registered map-frame cloud. It is deliberately local-file only at this stage:
  no valid persistent 3D cloud exists in this workspace. The viewer tells the
  operator that its points must be ROS `map` coordinates in metres and labels
  the view review-only.
- The raw RealSense point-cloud timestamps remain incompatible with ROS time.
  No timestamp adapter or 3D map publisher was enabled because the local source
  tree contains no synchronized moving-session evidence from which to estimate
  source-clock offset and uncertainty. The existing live stitched preview is
  not accepted as saved-map data. The next evidence requirement remains the
  recorded slow manual loop, followed by offline timestamp/TF comparison.

## Motion-validation capture preparation — 2026-10-20

- Added project-owned `mapping_recorder`, a passive ROS 2 capture node. Each
  non-overwriting session stores full depth-image payloads plus their original
  headers and recorder reception time, CameraInfo intrinsics, complete scans,
  vendor/normalized odometry, IMU, map grids, dynamic/static TF, and raw-cloud
  headers. It records writer-queue and disk-cap drops and writes a final summary
  only after graceful shutdown. It never publishes, sends commands, or changes
  robot state.
- The capture procedure and stationary acceptance conditions are in
  `docs/MAPPING_CAPTURE_PROCEDURE.md`. It is source-only and has not been
  deployed, started, or tested against the post-battery live ROS graph. A live
  stationary capture is therefore the remaining prerequisite before any manual
  movement evidence can be claimed.
- A local static isolation audit found exactly ten subscriptions and no
  publisher, client/service, action, subprocess, or command-path calls in the
  recorder. It is not referenced by existing mapping/navigation launches. The
  local Windows workspace has no ROS runtime or active ROS process, so it
  cannot establish the post-battery robot graph, QoS compatibility, rates, disk
  availability, or timestamp behavior; those remain Stage 1/2 read-only checks
  after explicit deployment approval.

### Deployment-readiness check — 2026-10-20

- Operator confirmed normal standing posture and authorized deployment of the
  passive recorder only. The laptop Wi-Fi is on `192.168.2.0/24` with gateway
  `192.168.2.1`; no existing local API/SSH forward is listening. A single
  batch-mode, read-only SSH identity/configuration query reached the gateway
  but authentication was refused (`Permission denied (publickey,password)`).
  It could not run a remote command or prompt for a credential, so no robot
  state, file, process, service, map, or control path changed.
- Before deployment, establish an operator-authenticated SSH tunnel/session to
  the perception computer. Then take the prescribed read-only ROS/API/disk
  snapshot, create a dated backup of only the project package files required by
  the recorder, copy them, build only `lite3_odom_tf_adapter` in the verified
  clean Foxy environment, and run the recorder separately. Do not restart the
  API, mapping, Nav2, bridge, vendor services, relay, or any motor path.

## Passive recorder deployment and stationary acceptance — 2026-10-21

- Operator confirmed normal standing posture. Before deployment, runtime was
  manual control with STOP latched and motor output isolated; camera/raw cloud
  were fresh while mapping and Nav2 were off. `/leg_odom2`, `/odom`,
  `/imu/data`, `/scan`, and `/map` had no active publisher, as expected with
  the mapping stack stopped.
- Created backup
  `/home/ysc/lite3_autonomy_ws/backups/20261021/mapping_recorder_predeploy_1726125849`,
  then deployed only `mapping_recorder.py`, `setup.py`, and `package.xml`.
  SHA-256 matched local source for all three. A clean Foxy-only build of only
  `lite3_odom_tf_adapter` succeeded and registered `mapping_recorder`; no
  vendor/project runtime process was restarted.
- Session `/home/ysc/lite3_autonomy_ws/records/motion_validation_20261021_stationary1`
  contains 424 complete 424×240 `16UC1` depth frames (86,292,480 bytes), 432
  CameraInfo samples, 431 raw-cloud headers, static camera TF and heartbeats.
  It stopped cleanly with no writer/disk drops. All 424 retained depth hashes
  verify against the JSONL ledger.
- Depth/CameraInfo/raw-cloud rates were 29.509/29.997/30.022 Hz. Depth and
  nearest CameraInfo acquisition stamps matched exactly; depth reception delay
  was 39.84 ms median (35.88–206.82 ms). Recorded intrinsics reconstruct a
  valid camera-frame point `(-0.005,-0.002,1.053) m`. The raw cloud remains
  invalid for registration: its nearest depth stamp differed by about
  `1.726×10^18 ns`.
- This passes passive recorder and source-depth acceptance. It cannot establish
  map-frame 3D: mapping remained deliberately off, leaving scan/odom/IMU/map
  and dynamic/base/map TF unavailable; only camera-link static transforms were
  captured. Final read-only status confirms the recorder exited, mapping/Nav2
  remain off, STOP is latched, and motor output remains isolated.

## Full mapping-pipeline stationary acceptance — 2026-10-21

- Existing mapping was started only by the operator through the approved UI;
  Nav2 remained off, STOP latched, and motor output isolated. The first
  full-pipeline capture is preserved as
  `/home/ysc/lite3_autonomy_ws/records/motion_validation_20261021_fullpipeline_stationary1`.
  It confirmed all mapping streams but retained only 345 depth frames after
  endpoint discovery, so it is evidence, not the final timed acceptance set.
- The 28-second second attempt, preserved as
  `/home/ysc/lite3_autonomy_ws/records/motion_validation_20261021_fullpipeline_stationary2`,
  retained 620 depth frames, 633 CameraInfo, 414 scans, 4,501 vendor odometry,
  3,693 normalized odometry, 4,963 TF, three static-TF messages, and six map
  grids with no recorder queue/disk drops. No `/imu/data` publisher is active
  in this runtime, so IMU cannot be asserted as captured.
- All 620 depth hashes validate. Depth/CameraInfo nearest stamps match exactly.
  The observed rates are depth 26.152 Hz, CameraInfo 26.626 Hz, scan 17.425
  Hz, vendor odometry 190.112 Hz, and normalized odometry 156.105 Hz. These
  are measured capture rates, not assumed sensor specifications.
- Dynamic TF had a short startup warm-up: discard the first 20 depth frames and
  first 29 scans. Thereafter 600 depth frames and 385 scans had both required
  dynamic transforms within 50 ms of their acquisition stamps (median about
  5.6 ms). The early scan maximum was 1.398 s and must not be used for motion
  registration. Stationary normalized odometry had zero x/y span and 0.051°
  yaw span.
- Scan headers match nearest source-depth headers at 0 ns median; scan receipt
  followed the matched depth receipt by 503.3 ms median. This is consistent
  with the intentional scan delay while preserving acquisition timestamps.
  Timestamp-correct depth transformation through
  `map→odom→base_link→camera_link→camera_depth_optical_frame` succeeds after
  warm-up. Raw PointCloud2 remains excluded due its incompatible clock.
- Continuous passive capture for the manual loop is running as PID `8477`,
  writing the non-overwriting directory
  `/home/ysc/lite3_autonomy_ws/records/motion_validation_20261021_manual_loop1`
  with an 8 GiB payload cap. Startup was verified with 573 events after five
  seconds. It publishes nothing and does not affect mapping or robot control.
