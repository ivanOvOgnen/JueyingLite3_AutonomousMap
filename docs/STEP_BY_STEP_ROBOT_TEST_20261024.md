# Lite3 map, target, and attended Auto Mode test

Updated 2026-10-24. **Current status: physical map-click movement is not viable.** The 18:38 deployment preserved manual joystick operation, but a subsequent screenshot still shows both costmaps and depth obstacle observations unavailable. The steps below describe the intended test sequence after the timestamp/TF fault is identified and repaired; they are not a claim that the goal can move the robot now. Follow this guide from the Linux laptop in this repository. The laptop must be on the robot Wi-Fi before SSH commands; the scripts do not change Wi-Fi. The DEEP controller app is the manual drive and STOP interface. Keep it connected and within reach.

## What this update changes

The earlier Nav2 local obstacle layer expected a new depth cloud every 0.15 seconds, even though a saved robot probe showed longer gaps. The depth projector allowed only 0.35 seconds of source age and the API only 0.5 seconds, while the robot's depth-to-scan pipeline showed roughly 1.25 seconds of scan age including its intentional 0.5-second scan delay. The deployed update permits depth acquisition age up to 1.0 second and a 0.5-second arrival gap, while still rejecting stale or future observations. Nav2's depth buffer expects an update within 0.5 seconds. The source timestamp and acquisition-time TF checks remain in place. **The robot's post-deployment screenshot still fails the costmap and depth checks.** The numeric header ages and TF result were not captured; do not increase limits again without measuring the fault.

## Install once, with backup and manual-control check

1. Power on the robot. In the DEEP app, confirm **Manual + Move**, test a tiny joystick motion in a clear area, and release the sticks to neutral. Keep the app STOP reachable.
2. Connect the laptop to the robot Wi-Fi yourself. From this repository run:

   ```bash
   python3 scripts/nav2_one_shot.py
   ```

3. Type `YES` for the two manual-joystick checks when they are true. The script runs local tests, checks the robot is idle, installs the project package and UI, copies the robot backup under `log/nav2_robot_backup_*`, and restores Manual + Move. It then opens a local Room Mapper URL. Do not close the terminal while using its SSH tunnel.
4. If installation or the second joystick check fails, the wrapper attempts rollback. Read `log/nav2_one_shot_*.json`; a `rollback_error` means the robot needs inspection before another trial. The explicit rollback command is `python3 scripts/rollback_supervised_nav_trial.py --help`. To reassert manual mode after Nav2 is stopped, use `python3 scripts/restore_vendor_mode.py --apply`, then verify the DEEP joystick physically.

The backup and joystick checks protect the previously working vendor controls. The installer does not start Nav2, arm a goal, or publish velocity.

## Map and localization button order

1. In the Room Mapper, press **Connect & check status**. Verify the robot API and live camera/odometry/scan states.
2. For a new room, keep the DEEP app in Manual + Move, press **Start mapping**, manually guide one slow loop and return near the start, then press **Finish scan and save** while stationary. Select that saved map. A prior saved map is usable only in the same unchanged room and coordinate frame; choose its matching PGM/YAML pair from the robot's saved-map list. The map files live in `/home/ysc/lite3_autonomy_ws/webapp/maps/` on the perception computer. Backups and local trial reports live under this repository's `log/`.
3. Press **Start localization / Nav2**. Wait for the map to load. On the map, choose **Robot is here**, click the robot's actual location, set its heading, and press **Use clicked point as robot location**. Check that the blue marker matches the physical robot. AMCL's estimate is an estimate; a selected point does not teleport the robot.
4. Select **Navigation target** and click a nearby observed free point. Target selection can be previewed even if navigation is not yet ready. Use **Preview route** to inspect the actual path. Unknown or occupied cells remain blocked for physical walking; selecting a target is not permission to drive there.
5. Wait for **local and global costmaps**, **depth obstacle observations**, and localization to be ready. The page should then allow **Enable planner goal requests** and **Send selected point to Nav2**. The goal is an attended, short trial: select a target within 0.5 m, explicitly arm for at most 10 seconds, and keep the DEEP app STOP ready. If any readiness item remains unavailable, do not force the motor output. Run the read-only probe below and inspect its named blocker.
6. After the trial press **STOP AUTONOMY**, stop localization/Nav2, close the tunnel with Ctrl+C, and confirm Manual + Move and joystick response in the DEEP app.

## If costmaps or depth are still unavailable

With localization/Nav2 running, save a read-only report:

```bash
python3 scripts/diagnose_localization.py
```

It prints a diagnosis and saves `log/localization_probe_*.json`. Check `api.components.obstacle_observations.reason`, `source_age_seconds`, `api.components.costmaps` (each age), `api.components.transforms`, and `navigation_error_events`. A missing cloud publisher, future/stale source stamp, missing acquisition-time TF, or an inactive Nav2 lifecycle node are different failures. The one-second depth limit is a cutoff, not a guarantee that an obstacle is safe at any speed. The saved map and route still need physical review.

## Separate, bounded Auto Mode movement check

This checks whether the vendor Auto Mode command path moves the standing robot; it **does not** follow a map target or certify Nav2. Stop localization/Nav2 and resolve any goal first. Use a clear area, stand beside the robot, keep the DEEP app STOP open, and verify Manual + Move works first. Then switch Auto Mode on in the DEEP app for the attended test. Run the read-only preflight, then the test:

```bash
python3 scripts/test_auto_mode_move.py
python3 scripts/test_auto_mode_move.py --apply
```

The second command requires an on-robot `ARM` confirmation, sends at most 0.05 m/s forward for at most three seconds, sends repeated zero commands, and asks whether the robot moved and whether app STOP halted it. **Press app STOP during the trial.** The report is saved as `log/supervised_vendor_trial_*.json`. If the preflight rejects another `/cmd_vel` publisher, a missing `/ros2qnx` subscriber, stale STOP state, or non-neutral controller axes, stop there and restore the manual baseline; do not use a direct `rostopic pub` workaround. After the test, close the session and verify DEEP joystick control again.

## Evidence needed to call map-click walking complete

- The updated depth cloud and both costmaps stay ready during a stationary and short moving check, with a current source timestamp and map/odom TF.
- The selected map, blue robot marker, nearby target, and previewed route match the real room; the route avoids unknown and occupied cells.
- A submitted and armed Nav2 goal produces measured command output and observed motion toward that target, then stops within the trial limit or on app STOP.
- Manual + Move and the DEEP joystick still work afterward; save the deployment, trial, and localization reports.

The local tests confirm code behavior only. The latest robot screenshot confirms the repair has not yet achieved navigation readiness. Physical map-click movement remains unverified and currently blocked.
