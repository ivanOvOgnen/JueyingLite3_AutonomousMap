# Map review: lite3_room.pgm

- YAML: `lite3_room.yaml`
- Grid: 89 x 99 cells at 0.05 m/cell
- Physical dimensions: 4.45 m x 4.95 m
- Origin: x=-2.540 m, y=-3.030 m, yaw=0.0000 rad
- Trinary thresholds: free < 0.196; occupied > 0.65; negate=0

## Cell classification

- Free: 2,520 (28.6%)
- Occupied: 712 (8.1%)
- Unknown: 5,579 (63.3%)
- 4-connected free regions: 44
- Largest free region: 2,348 cells (93.2% of free cells)

## Review notes

- More than half the cells are unknown; map coverage is sparse.
- Compare the rendered map with the room: check continuous walls, duplicated walls, openings, and return-to-start alignment.
- These image-derived metrics do not measure odometry drift, obstacle clearance, localization quality, or safe navigation.
- Treat the report as a comparison aid; a person must review the map before any navigation trial.
