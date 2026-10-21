# Map review: lite3_room_scan_20260930.pgm

- YAML: `lite3_room_scan_20260930.yaml`
- Grid: 131 x 181 cells at 0.05 m/cell
- Physical dimensions: 6.55 m x 9.05 m
- Origin: x=-2.224 m, y=-5.435 m, yaw=0.0000 rad
- Trinary thresholds: free < 0.196; occupied > 0.65; negate=0

## Cell classification

- Free: 5,113 (21.6%)
- Occupied: 864 (3.6%)
- Unknown: 17,734 (74.8%)
- 4-connected free regions: 71
- Largest free region: 4,828 cells (94.4% of free cells)

## Review notes

- More than half the cells are unknown; map coverage is sparse.
- Compare the rendered map with the room: check continuous walls, duplicated walls, openings, and return-to-start alignment.
- These image-derived metrics do not measure odometry drift, obstacle clearance, localization quality, or safe navigation.
- Treat the report as a comparison aid; a person must review the map before any navigation trial.
