# Motion-mapping capture procedure

`mapping_recorder` is passive: it only subscribes and writes a new local
session directory. It never publishes, calls a service/action, changes TF, or
controls the robot.

## Before a supervised session

After the project source has been deliberately deployed and built on the robot,
start the recorder in a separate terminal before mapping:

```bash
ros2 run lite3_odom_tf_adapter mapping_recorder \
  /home/ysc/lite3_autonomy_ws/records/motion_validation_YYYYMMDD_sessionN
```

The destination must be new; the node refuses to overwrite prior evidence. It
stores full source depth payloads under `depth/`, and `events.jsonl` records
their image header and recorder reception time, CameraInfo intrinsics, full
scan ranges, vendor and normalized odometry, IMU, map grids, dynamic/static TF,
and raw-cloud headers. `summary.json` is written only after graceful Ctrl-C.

Keep at least 25 GiB free for the default 20 GiB depth cap. The node reports
depth-writer queue drops and disk-limit drops in heartbeats and the final
summary; a running PID is not proof of a usable capture.

## Required stationary acceptance check

With the robot visibly standing but no manual controller input, record at least
15 seconds. Stop the recorder, then verify `summary.json` reports nonzero
counts for `depth`, `camera_info`, `scan`, `vendor_odom`, `normalized_odom`,
`tf`, and `raw_cloud_header`, with no depth queue or disk-limit drops. Verify
the events contain current timestamps and at least one retained `/map` sample
after mapping starts. If any required stream is absent, preserve the directory
and fix the capture environment before manual movement.

### Verified stopped-mapping baseline — 2026-10-21

The deployed recorder captured source depth, CameraInfo, raw-cloud headers and
camera static TF at approximately 30 Hz with no recorder drops while mapping
was intentionally stopped. Therefore absent scan, map, odometry, IMU and
map-frame TF are expected publisher inactivity, not a recorder failure. It
proves depth/CameraInfo pairing and independent camera-frame reconstruction,
but not map-frame registration. For a movement dataset, start the existing
mapping stack only after separate approval and require nonzero scan,
vendor/normalized odometry, dynamic TF, map and, where offered, IMU counts
before manual movement.

### Full-pipeline warm-up rule — 2026-10-21

In the verified mapping session, dynamic TF required a short startup period.
For all later captures, keep the robot stationary for at least five seconds
after starting a recorder and mapping, then reject any scan/depth observation
whose nearest required dynamic transform is more than 50 ms from its header
stamp. This is an offline acceptance rule; it does not alter SLAM or TF.

The existing scan publisher intentionally delays delivery by about 0.5 seconds
but preserves scan acquisition stamps. Use each scan/depth header timestamp for
TF lookup, never its recorder reception time or the robot's latest pose.

## Minimal supervised movement

1. Start a fresh recorder and mapping only after the stationary check passes.
2. Record a 15-second stationary reference.
3. The operator uses only vendor manual controls for a slow, clear-open-space
   path (estimated 0.05–0.10 m/s; gradual 10–15°/s turns). Do not seek fast or
   aggressive motion.
4. Revisit a recognizable wall/corner, switch to Pose/no input, and collect a
   further 15 seconds.
5. Stop the recorder gracefully; preserve its complete directory before saving
   any experimental 2D map under a unique name.

Depth reconstruction must pair each saved depth image with CameraInfo and a
timestamp-matched TF/trajectory pose. The raw PointCloud2 is recorded only for
its header-clock diagnosis and is not acceptable registration evidence.
