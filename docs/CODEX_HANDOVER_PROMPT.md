# Prompt for the next Codex session

Copy and send the following prompt to your friend's Codex after giving it this repository:

```text
Continue the Lite3 Pro autonomy project from this repository. Before acting, read HANDOVER.md, STATUS.md, GROUND_TRUTH.md, docs/IMPLEMENTATION_PLAN.md, and the recent end of docs/DEBUG_LOG.md. HANDOVER.md is the source of truth for the current handoff; verify live robot state rather than assuming old network or service state still applies.

The goal is a browser workflow for mapping with a Jueying Lite3 Pro and RealSense depth camera, then eventually selecting a destination in Chrome on a Retroid Pocket 3 Plus/YZX or laptop over robot Wi-Fi. Existing maps are sparse and unfit for navigation. Nav2, the manual RC path, command arbitration, and motor routing are not commissioned. As of 2026-10-04, first-browser pairing is deployed and the API is running for an owner-supervised stationary UI check with goals disabled. The status and page/assets/map-list GET checks pass through a temporary laptop SSH tunnel. The API reports an existing token (`api_paired=true`); Chrome rendering and Retroid access are still being checked. Fresh preflight has `/leg_odom2` with zero publishers and no `/odom`, `/scan`, or `/map`.

Safety is a hard constraint: do not physically move the robot, publish velocity/cmd_vel, send a navigation goal, enable the autonomy gate, launch a motion-capable path, or change/stop vendor services or networking/UDP ports unless the robot owner explicitly authorizes that specific test and is present to supervise it. Preserve the DEEP Robotics Lite3 manual controller path. Read-only checks and local software/documentation work are okay. A successful build does not mean the robot is safe to walk.

First report verified current software, network, and robot state, separating observations from assumptions. Use the read-only preflight when connected. Preserve existing robot/workspace changes and credentials. Then continue the highest-value work toward a reliable presentation workflow, updating README.md, HANDOVER.md, STATUS.md, GROUND_TRUTH.md, and DEBUG_LOG.md when facts change. Do not push to GitHub unless explicitly asked. Never put passwords, tokens, or pairing keys in the repository. If blocked from the robot, continue independent local engineering and state precisely what remains unverified.
```
