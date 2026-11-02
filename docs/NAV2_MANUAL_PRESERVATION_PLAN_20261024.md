# Nav2 map-click walking without losing DEEP manual control

**2026-10-24 progress:** A backed-up one-shot install and post-install manual joystick check were recorded in `log/nav2_one_shot_20261024_180343.json`; the DEEP joystick passed. The timing repair raises the depth acquisition limit to 1.0 second and Nav2 observation interval to 0.5 seconds based on saved pipeline latency. It passed 226 local Python tests and a UI wiring check, then was deployed in the backed-up 18:38 run (`log/nav2_one_shot_20261024_183816.json`), again with manual joystick verified. The next screenshot still shows both costmaps with invalid source timestamps and depth acquisition timestamp or map/odom TF not current. **Map-click movement is not viable.** The actual numeric ages and TF result must be captured before another sensor change. The current execution instructions are in [the step-by-step guide](STEP_BY_STEP_ROBOT_TEST_20261024.md). Keep the phase checkboxes below open until live evidence satisfies them.

Revised 2026-10-24 for the next session. This is a work plan, not permission to run an unbounded robot walk. Check a box only after saving the named evidence. Do not rerun `scripts/deploy_supervised_nav_trial.py --apply` from the archived experiment unchanged.

The laptop implementation now lets the operator click **Robot is here**, send that point and heading as a tighter AMCL initial-pose prior, and then switch to **Navigation target** for a separate click and route preview. It waits for an AMCL subscriber before publishing the prior and still requires fresh localization before a goal. The one-shot rollback now names and verifies the exact backup from its own install; it cannot silently select an older snapshot. These changes passed 216 local tests on 2026-10-24. The laptop was on CaciNet, so this revision has **not** been deployed or verified on the robot. A legacy map without an integrity manifest or an experimental map without measured review will remain unavailable for Nav2 walking; the UI now shows that reason before starting Nav2.

The follow-up localization-only repair lets a legacy saved PGM/YAML map load as a transient experimental bundle so AMCL can be started and its position estimate tested. It does not create an integrity manifest or approve walking. Planner opt-in, route preview, and target motion still require a reviewed map. The browser now separates this diagnosis from the Start localization button; the locally revised suite passed 220 tests. Redeployment and live AMCL observations remain pending.

## One-command execution path prepared on 2026-10-24

The local trial bundle and `scripts/nav2_one_shot.py` now package the phases that can be automated. After the operator manually joins robot Wi-Fi, run `python3 scripts/nav2_one_shot.py` once from this repository. It runs local tests; checks that mapping, Nav2, and goals are idle; latches and verifies the project software STOP; creates and verifies a robot-side backup of every overwritten source, installed module, launch/config, API launcher, and web asset; builds and restarts the project API; copies the backup to `log/nav2_robot_backup_*` on the laptop; reasserts Manual + Move; asks for a short joystick check; and opens the app through an SSH tunnel. A failed post-install joystick check invokes the project rollback and mode recovery in that same run. It does not switch Wi-Fi, modify vendor services, or arm a motor command.

An additional local archive of the accepted Git checkpoint already exists at `log/accepted_passive_checkpoint_51cb1f3_20261024.tar.gz` (SHA-256 `5cd0746c4acdea1d0ab8850cc7feb5f545b2a4aebd326a0f5c2618cb76f7f3a8`). The robot-side backup is created only when the installer runs on robot Wi-Fi.

This is **one install command**, not an automatic walk. The physical map-click trial still needs an operator at the robot, a reviewed map/pose/obstacle view, and explicit in-app arming. After a trial, press **STOP AUTONOMY**, close the tunnel with Ctrl+C, and the wrapper attempts Manual + Move restoration. Because the vendor mode packets have no acknowledgement, the joystick must still be tested. If the robot cannot be reached or a Nav2 command route is still active, the wrapper refuses the mode change and prints the recovery command. The operator should not treat package installation or a sent UDP packet as proof of manual control.

## Target and known starting point

The target workflow is: manually scan and save a usable room map; localize the robot against it; click and preview a reachable point; explicitly hand control to Nav2 for a short goal; stop or finish; return the robot to DEEP **Manual + Move** so its joystick works again. The app must report planner acceptance, actual command output, and observed robot movement as separate facts.

The local checkout is the accepted 2026-10-21 commit `51cb1f3`. Its API can run mapping, localization, and planner-only Nav2 goals; `components.motion_output` is intentionally isolated. The old trial bundle is recoverable from `log/checkout_before_original_revert_20261023_221825.tar.gz`, but it had failed installs/API restarts and did not preserve manual control reliably. It is reference material, not a deployable baseline.

On 2026-10-23, restoring project files alone did not restore joystick driving. The user's successful repair followed a two-command vendor mode reassertion: `0x21010C02` (Manual ownership), then `0x21010D06` (Move interpretation), recorded in `log/vendor_mode_recovery_20261023_224200.json`. Those UDP sends had no acknowledgement; the user's subsequent joystick test is the functional evidence. A separate `log/supervised_vendor_trial_20261023_180931.json` records a bounded 0.05 m/s command trial in which the robot moved and the DEEP app STOP halted it. Neither result proves the complete Nav2 route or an automatic return to manual control.

## Rules for every phase

- Keep the DEEP app, vendor firmware, `transfer.service`, motion-host services, and their UDP ports unchanged. Use the existing vendor ROS 1 `/ros2qnx` command consumer. Never start a second motion transport to work around UDP `43897` contention.
- Normal boot means **Manual + Move**, with all project velocity publishers disabled. An API start, browser reload, Wi-Fi loss, or robot reboot must not acquire motion ownership.
- Exactly one autonomous command route may publish to the vendor command subscriber during a trial. Planner goal acceptance alone must never enable that route.
- Before and after **each** project deployment or robot-side test, demonstrate a short manual joystick response and neutral-stick stop. If manual driving changes, end the phase, isolate project processes, restore Manual + Move, and do not continue to navigation.
- Each change needs a dated robot-side backup, source/installed-file hashes, a bounded health check, and one executable rollback path. Preserve logs. Do not edit the robot until the local package, tests, and rollback script are ready.
- The user controls the laptop Wi-Fi connection. Robot access can use the documented SSH jump through `ysc@192.168.2.1` to `ysc@192.168.1.103` after the user joins the robot network. No automatic Wi-Fi switching.
- Only the DEEP app STOP has been physically observed in a short trial. The software STOP and UDP mode commands are not an independent physical emergency stop. Until a separate stop/interlock is verified, keep any walking trial attended, short, and speed-bounded; do not label unattended navigation ready or set `physical_stop_verified=true` without measured evidence.

## Phase 0 — Freeze and record the working manual baseline

- [ ] Leave the current robot configuration alone at the start of the session. Record battery, DEEP app connection, selected mode, robot posture, STOP state, and a short joystick/neutral-stick result in a clear area. Capture the app screen so missing controls can be compared later.
- [ ] Collect a read-only snapshot from both robot computers: project/vendor process lists; vendor service states; ROS 1 `/cmd_vel` publishers and `/ros2qnx` subscriber; ROS 2 command topics and bridge parameters; odometry/camera/scan freshness; API version/status; currently selected map and hashes. Use `scripts/robot_preflight.sh` and the existing read-only diagnostic, supplemented only where a fact is missing.
- [ ] Record whether the vendor exposes a readable ownership/Move-vs-Pose state or acknowledgement. If no such interface exists, mark it **unverified**; do not infer mode from a sent UDP packet or app label alone. The observed joystick test remains the return-to-manual criterion.
- [x] Create the `nav2-movement` work branch from `51cb1f3`; preserve the current robot package and web files, and keep the existing checkpoint recovery script available. Record the exact commands and backup locations before altering the robot.

**Gate:** baseline manual drive works, no project publisher feeds the vendor ROS 1 `/cmd_vel` input, and a rollback/recovery route is available. If this is false, repair manual control first.

## Phase 1 — Make map, pose, and planner dependable while motion stays isolated

- [ ] Confirm the app/API works from the user's laptop without modifying vendor control. Test mapping start/pause/stop/save and reload; trace camera, `/leg_odom2`, `/odom`, `/scan`, `/map`, and TF if a control is disabled. Keep mapping manual with the DEEP joystick.
- [ ] Review or rebuild the saved map. The previous `lite3_scan_niina` map was about 62% unknown; assess the actual intended route, wall continuity, obstacle clearances, and robot footprint rather than accepting a whole-map percentage alone. Save the chosen PGM/YAML pair and integrity evidence.
- [ ] Start localization/Nav2 with command output disconnected. Verify the map-frame robot marker against its real pose, stable AMCL updates while stationary and after a small manual movement, fresh scan/odometry/TF, and live global/local costmaps including the depth obstacle layer.
- [ ] Click a known-free goal, check map-frame coordinates and route preview, submit/cancel a **planner-only** goal, and confirm that ROS 1 `/cmd_vel` still has no project publisher. UI wording must say “accepted by Nav2, motion isolated” at this stage.

**Gate:** mapping and localization work repeatedly, a reviewed route has adequate observed space, and planner-only tests cannot move the robot. If a map or sensor goes stale, fix that specific input before command integration.

## Phase 2 — Build an explicit, reversible vendor handoff

- [ ] Add one small robot-side ownership controller with states such as `MANUAL`, `PREPARING`, `AUTONOMY_TRIAL`, and `RETURNING_TO_MANUAL`. No autonomous velocity node may independently change vendor mode. Define one lease with a short expiry, one active goal ID, and a single owner.
- [ ] Acquire autonomy only when Nav2/localization/route/sensor checks pass, the DEEP controller is connected and neutral, STOP is available, the ROS graph has the expected publisher/subscriber counts, and the operator explicitly arms the short trial. Send the documented vendor Navigation ownership command only at the final handoff step. Check for positive mode feedback if available; otherwise do not claim hardware-verified ownership.
- [ ] Keep lease expiry and return-to-manual handling in a robot-side supervisor independent of the browser and API process. On normal completion, cancel, STOP, timeout, API crash/restart, graph change, sensor/pose loss, or operator takeover: block new commands, publish zero through the active route while it exists, disable the bridge, release the lease, and reassert **Manual + Move** (`0x21010C02`, then `0x21010D06`). Log each transition and its evidence. A full perception-host outage may prevent the supervisor from sending mode commands, so require the DEEP controller recovery path for that case. If return cannot be confirmed, show a prominent “manual control unverified” state and use the already successful manual recovery helper under operator supervision.
- [ ] Add a visible **Return to manual** action and live ownership/status text to the Room Mapper. Do not remove or modify DEEP app controls. The Room Mapper must not display “ready to walk” based solely on Nav2 goal acceptance or a sent mode command.
- [ ] Unit-test state transitions and failure injection: duplicate arming, stale lease, simultaneous joystick/manual override, STOP mid-goal, missing vendor mode feedback, API death, bridge death, timeout, lost browser/Wi-Fi, and restart. The expected result in each case is zero autonomous output and manual-return attempt with honest status.

**Gate:** handoff and return-to-manual pass on the bench with motion isolated; a fault cannot leave a project command publisher active. Then verify the DEEP joystick still works after starting and stopping the new non-moving project processes.

## Phase 3 — Connect one bounded Nav2 command path

- [ ] Trace the **short supervised trial** path precisely: Nav2 `/nav2/cmd_vel` → `supervised_nav_relay.py` (10-second lease, 0.05 m/s clamp, freshness/STOP checks) → one ROS 2 `/cmd_vel` bridge publisher → ROS 1 `/cmd_vel` → existing vendor `/ros2qnx` subscriber. Confirm that remaps include recovery behaviors and that `/cmd_vel_corrected` does not create a second owner. This limited trial route is separate from the not-yet-commissioned permanent arbiter path.
- [ ] Keep the existing `motion_arbiter.py` and `vendor_cmd_vel_bridge.py` disabled/isolated during the short trial. For sustained autonomy, replace the direct trial relay with an ownership-aware guarded route only after independent stop, timeout, footprint, and vendor handoff evidence exists. A lone status flag or UI click must never enable multiple relay stages.
- [ ] Keep command output disabled by default and absent from API autostart where feasible. Clamp speed/turn rate, reject unsupported/non-finite axes, require fresh velocity/readiness/STOP observation, send zero on loss, and require a new explicit lease after every stop. Verify one bridge instance and the expected topic direction before any physical trial.
- [ ] Test the graph with a simulated consumer and recorded Nav2 output: correct route, one publisher, bounded commands, zero on each fault, no output while Manual owns control. Do not claim a successful motor handoff from these software tests.

**Gate:** the guarded route is deterministic in tests, disabled on boot, and leaves manual joystick behavior intact on the robot before any armed trial.

## Phase 4 — Deploy transactionally

- [ ] Replace the old supervised installer with a narrow deployment script for the exact changed files. Dry-run first: compare hashes, assert mapping/Nav2/goal/relay are idle, verify the original project API responds, and check disk space/dependencies. Refuse an active session rather than killing unknown processes.
- [ ] On `--apply`, back up source, installed Python modules, launch/config assets, and UI to a dated directory with a manifest; stage and build one Foxy package; verify installed hashes and import/launch resources; restart **only** the project API; wait for a health response with the expected contract/version. Leave vendor services and app untouched.
- [ ] If any step fails, automatically restore the exact backup, restart the prior API, remove any newly started project command publisher, and leave Manual + Move requested. Report both install and rollback results clearly. A rollback script must also work when the new API cannot start. Do not silently continue after an incomplete rollback.
- [ ] After deployment, compare the DEEP app screen with Phase 0 and perform the same short manual joystick/neutral test. Record a before/after process and ROS graph diff. If manual control is degraded, invoke rollback immediately and stop.

**Gate:** new API is healthy, hashes match, no project velocity is active, and the original manual controls work exactly as before.

## Phase 5 — Commission one supervised map-click trial

- [ ] Put the robot in a clear area with a trusted operator beside it and the DEEP app STOP within reach. Verify battery, connection, fresh map/pose/sensors, an unobstructed goal, and the actual neutral manual state. The operator must be able to stop it; do not run an unattended goal.
- [ ] First test handoff and return with **no nonzero velocity**. Confirm project status transitions, zero command graph, Manual + Move recovery, and working joystick afterward.
- [ ] Arm a single known-free goal no farther than 0.5 m, with at most 0.05 m/s and a 10-second lease. Record Nav2 feedback, commanded velocity, odometry displacement, physical displacement, stop/timeout result, and final manual joystick test. A goal “accepted” without measured motion is a failed end-to-end trial.
- [ ] In separate short trials, validate cancel, app STOP, stale command/sensor, lost API/browser, and operator manual takeover. In each case confirm motion ceases, the autonomous publisher disappears/disables, and Manual + Move returns. Only then consider a longer route with obstacle-response review.

**Gate:** the map click results in bounded, measured movement toward the goal **and** manual joystick works after every success/failure path. Do not expand the trial if any return-to-manual check fails.

## Completion checklist

- [ ] A saved, reviewed map loads after restart; map click previews coordinates and a route in the same map frame.
- [ ] AMCL pose matches the physical robot and remains fresh; depth-derived obstacle costmaps and TF remain valid during the trial.
- [ ] Nav2 goal acceptance is reported separately from bridge enable, actual velocity, physical movement, and final result.
- [ ] One bounded trial reaches a known-free goal or stops within its limit, with command/odometry/operator evidence saved.
- [ ] DEEP manual joystick, neutral stop, visible controls, and app STOP work before and after deploy, each goal, cancel, fault, reboot, and rollback.
- [ ] Project motion starts disabled; loss of readiness/ownership produces zero output and a visible manual-return status.
- [ ] No vendor service, app package, firmware, or UDP-port setting was altered. The deployment backup and one-command rollback are tested.

The next robot session starts at **Phase 0** inside the one-command wrapper. The local trial bundle is staged and locally tested; robot-side installation, mode handoff, mapping quality, and physical Nav2 movement remain unverified until the operator runs it and the acceptance checks pass.
