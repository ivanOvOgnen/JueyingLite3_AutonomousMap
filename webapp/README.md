# Lite3 Room Mapper

The browser app has two modes:

- **Laptop preview:** run a static web server from this directory. It loads local or hosted PGM/PNG + YAML maps and previews clicked map coordinates. Mapping controls remain disabled because there is no robot API at this origin.
- **Robot-hosted mapping/planner:** when served by `robot_api` on the perception computer, it shows ROS status, authenticated mapping actions, and can submit a selected point to Nav2 after explicit API opt-in and readiness checks. The closed velocity gate remains disconnected from the robot, so this tests planner requests only. The user continues to drive manually with the vendor RC.

The map picker offers the original scan and the separate Sep 30 scan. The latter is marked review-only because it is sparse and distorted. Cell percentages use the map YAML's `negate`, `free_thresh`, and `occupied_thresh` settings; they help compare scans but cannot establish SLAM accuracy or safe navigation.

The app supports P2/P5 PGM with an 8-bit maximum and PNG. YAML must contain `image`, `resolution`, and `origin`; if the YAML names a different image the pair is rejected. Map clicks transform pixel centers through the YAML origin and yaw. The workflow is to map manually with the vendor RC, save and review a map, start localization/Nav2 from a map saved on the robot, either set a map-click initial pose or request AMCL global relocalization, verify the resulting robot marker, then click a known-free destination and submit it to the planner. Depth obstacle avoidance requires the live sensor costmap to be ready. The robot-hosted API can start/stop Nav2, publish an AMCL initial pose, and submit/cancel planner goals when explicitly opted in and readiness checks pass. Planner output remains isolated from the vendor motor command path, so these controls do not currently make the dog walk.

For robot API endpoints and the manual robot-hosted workflow, see [WEB_APP_API.md](../docs/WEB_APP_API.md). The API currently uses plain HTTP on the robot's private Wi-Fi; use it only on that trusted network. Navigation remains unavailable until localization, RC arbitration, and the motion route are verified.

For an offline PGM/YAML coverage and free-space-connectivity summary, run `python scripts/map_quality_report.py <map.pgm> <map.yaml>` from the project root. The report is a review aid, not a navigation readiness check.
