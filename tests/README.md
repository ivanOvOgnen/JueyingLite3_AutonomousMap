# Local lifecycle regression tests

Run from the repository root:

```bash
python3 -m pytest -q tests
```

Requires Python 3.8+ with `pytest` and PyYAML. The test fixtures supply inert ROS
imports and construct only the API's plain lifecycle state. They forbid real
ROS initialization, subprocess launch, and process-group signals unless a test
explicitly replaces the call with a fake. No robot or ROS graph is contacted.

The tests exercise real API methods, the authenticated HTTP handler path,
and all three command-node constructors/callbacks with inert publishers. They
cover concurrent startup/STOP behavior, stale/missing manual state, expired
commands/readiness, non-finite inputs, explicit re-arm, and fault propagation
through a chained fake output. Passing them does not verify Foxy
type support, DDS, live service responses, vendor ownership, or physical stop
behavior. Those remain robot-side acceptance gates in the implementation plan.

The expanded suite also covers real asynchronous goal futures, pending/late
acceptance, cancellation UUID matching, result transport failures, recorder
queues/writer failures, synthetic capture corruption, temporary map bundle
publication/no-overwrite/rollback, and explicit UI deployment compatibility,
backup and served-hash rollback. All filesystem writes use disposable test
folders. Synthetic fixtures never establish robot acceptance.

JavaScript checks require Node.js and Acorn (ECMAScript 2022 parser):

```bash
node tests/check_map_geometry.js
```

This extracts the real coordinate functions and click/submit handlers through
the parser and tests rotated origins, half-cell centers, scaled clicks,
preview-only selection and heading dispatch without starting the app or ROS.
It does not replace browser rendering/touch-device or live API acceptance.
