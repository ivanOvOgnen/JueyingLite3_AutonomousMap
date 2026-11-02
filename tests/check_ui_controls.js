// Exercise the real readiness function without opening a robot connection.
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const acorn = require('acorn');

const html = fs.readFileSync(path.join(__dirname, '../webapp/index.html'), 'utf8');
const source = fs.readFileSync(path.join(__dirname, '../webapp/app.js'), 'utf8');
const styles = fs.readFileSync(path.join(__dirname, '../webapp/styles.css'), 'utf8');
const ids = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map((match) => match[1]));
const refs = [...source.matchAll(/getElementById\("([^"]+)"\)/g)].map((match) => match[1]);
assert.deepStrictEqual(refs.filter((id) => !ids.has(id)), [], 'Every referenced UI element must exist');
assert.ok(html.includes('id="pointCloudCanvas"'), 'Keep the live 3D viewer');
assert.ok(!html.includes('savedCloudCanvas') && !source.includes('savedCloudCanvas'), 'Remove only saved PLY review');
assert.ok(html.includes('1. Control session') && html.includes('5. Arm movement'),
  'The visible navigation controls must show their required order');
assert.ok(styles.includes('.map-instructions:not([id])'),
  'Static help text should stay hidden while live readiness messages remain visible');

const tree = acorn.parse(source, { ecmaVersion: 2022 });
assert.ok(source.includes('mapCatalog.set(name, { ...mapCatalog.get(name), ...result })'),
  'A save response must retain map_id so localization can enable immediately');
assert.ok(source.includes('if (mapCatalogRefreshPromise) return mapCatalogRefreshPromise'),
  'Concurrent catalogue refresh callers must await the active refresh');
const functionNode = tree.body.find((node) => node.type === 'FunctionDeclaration' && node.id.name === 'updateControls');
assert.ok(functionNode);
const element = () => ({ disabled: false, hidden: false, textContent: '', title: '', removeAttribute(name) { delete this[name]; } });
const context = {
  apiConnected: false, browserSessionReady: false, shutdownRequested: false,
  displayedSessionEpoch: null, selectedGoal: null, selectedTargetFree: false, selectedPixel: null,
  previewRoute: null, mapOnlyRoute: null, controlLease: null, loadedMapName: null, loadedMapId: null,
  loadedPixels: null, loadedMetadata: null, goalFeedbackHoldUntil: 0, robotMapNames: new Set(),
  mapCatalog: new Map(),
};
for (const match of source.matchAll(/const (\w+) = document\.getElementById\("([^"]+)"\)/g)) {
  context[match[1]] = element();
}
context.savedMapPreset.value = 'sample';
context.mapClickMode.value = 'initial';
const sandbox = vm.createContext(context);
vm.runInContext(source.slice(functionNode.start, functionNode.end), sandbox);

sandbox.updateControls(null);
assert.strictEqual(context.startMappingButton.disabled, true);
assert.match(context.mappingReadinessStatus.textContent, /Connect to the robot API/);
assert.strictEqual(context.mapZoomInButton.disabled, true);

const status = {
  api_contract_version: 2,
  capabilities: ['priority_software_stop', 'tracked_goal_lifecycle', 'map_identity', 'path_preview', 'operator_lease'],
  components: {
    camera: { state: 'ready' }, mapping: { process_running: false, paused: false },
    navigation_stack: { process_running: false },
    navigation: { session_epoch: 1, active_goal_state: 'idle', planner_ready: false },
    safety_stop: { state: 'latched' }, motion_output: { bridge_profile: 'telemetry' },
    localization: { state: 'not_ready' },
  },
};
context.apiConnected = true;
context.browserSessionReady = true;
context.robotMapNames.add('sample');
context.mapCatalog.set('sample', { map_id: 'legacy-id', integrity: 'legacy_unverified', approval_state: 'experimental' });
sandbox.updateControls(status);
assert.strictEqual(context.startMappingButton.disabled, false, 'Fresh camera can start manual mapping');
assert.strictEqual(context.startNavigationButton.disabled, false, 'Legacy map can start localization testing');
assert.strictEqual(context.previewPathButton.disabled, true, 'Legacy map cannot preview a walking route');

status.components.camera.state = 'not_ready';
status.components.depth_sensor = { state: 'ready', source: 'pointcloud' };
sandbox.updateControls(status);
assert.strictEqual(context.startNavigationButton.disabled, false,
  'A fresh depth point cloud can start localization without the optional depth image');

status.components.mapping.process_running = true;
status.components.map = { state: 'ready' };
sandbox.updateControls(status);
assert.strictEqual(context.startMappingButton.disabled, true);
assert.strictEqual(context.finishScanButton.disabled, false);
assert.strictEqual(context.saveMapButton.disabled, false);

status.components.mapping.process_running = false;
status.components.navigation_stack.process_running = true;
status.components.navigation = {
  session_epoch: 1, active_goal_state: 'idle', active_map_id: 'reviewed-id',
  active_map_name: 'sample', planner_ready: true, state: 'ready',
  goal_requests_enabled: true, path_preview: { preview_id: 'preview-id' },
};
status.components.localization.state = 'ready';
context.mapCatalog.set('sample', { map_id: 'reviewed-id', integrity: 'verified', approval_state: 'reviewed' });
context.loadedMapName = 'sample';
context.loadedMapId = 'reviewed-id';
context.loadedPixels = { width: 10, height: 10 };
context.loadedMetadata = { resolution: 0.05 };
context.selectedGoal = { x: 1, y: 1 };
context.selectedTargetFree = true;
context.mapClickMode.value = 'goal';
context.controlLease = { lease_token: 'test' };
context.previewRoute = { preview_id: 'preview-id', session_epoch: 1 };
sandbox.updateControls(status);
assert.strictEqual(context.previewPathButton.disabled, false);
assert.strictEqual(context.sendGoalButton.disabled, false);
assert.strictEqual(context.mapZoomInButton.disabled, false);

context.mapCatalog.set('sample', { map_id: 'reviewed-id', integrity: 'verified', approval_state: 'experimental' });
status.components.motion_output.bridge_profile = 'supervised_trial';
status.components.navigation.active_map_target_eligible = true;
status.components.localization.pose = { x: 1, y: 1, yaw: 0 };
context.selectedGoal = { x: 1.4, y: 1 };
sandbox.updateControls(status);
assert.strictEqual(context.previewPathButton.disabled, false, 'Verified map permits a short supervised route preview');
context.selectedGoal = { x: 2, y: 1 };
sandbox.updateControls(status);
assert.strictEqual(context.previewPathButton.disabled, false, 'Distant target can still be inspected passively');
assert.strictEqual(context.previewPathButton.textContent, '2. Check saved-map line');
assert.strictEqual(context.sendGoalButton.disabled, true, 'Distant supervised target cannot be submitted');
assert.match(context.goalReadinessStatus.textContent, /within 0.50 m/);

status.components.navigation.planner_ready = false;
status.components.navigation.state = 'not_ready';
status.components.navigation.reason = 'Fresh local and global costmaps are required.';
context.previewRoute = null;
context.selectedGoal = { x: 1.4, y: 1 };
sandbox.updateControls(status);
assert.strictEqual(context.previewPathButton.disabled, false, 'Map-only inspection remains available without Nav2 readiness');
assert.strictEqual(context.previewPathButton.textContent, '2. Check saved-map line');
assert.strictEqual(context.sendGoalButton.disabled, true, 'Map-only inspection never permits a robot goal');
assert.match(context.goalReadinessStatus.textContent, /Click Check saved-map line/);
context.selectedTargetFree = false;
sandbox.updateControls(status);
assert.strictEqual(context.previewPathButton.disabled, false, 'Unknown target remains selectable for passive inspection');
assert.strictEqual(context.sendGoalButton.disabled, true, 'Unknown target cannot be submitted');
assert.match(context.goalReadinessStatus.textContent, /unknown or occupied/);
context.selectedTargetFree = true;
status.components.safety_stop.state = 'not_latched';
status.components.navigation.active_goal_state = 'stopped';
status.components.navigation.stop_latched = false;
sandbox.updateControls(status);
assert.match(context.safetyStopStatus.textContent, /STOP is clear/);
assert.match(context.goalActionStatus.textContent, /No active goal/);

for (const name of ['mapToPixel', 'knownFreeWithClearance', 'inspectSavedMapLine']) {
  const declaration = tree.body.find((node) => node.type === 'FunctionDeclaration' && node.id.name === name);
  assert.ok(declaration, `${name} must exist`);
  vm.runInContext(source.slice(declaration.start, declaration.end), sandbox);
}
context.loadedPixels = { width: 40, height: 40, data: new Uint8ClampedArray(40 * 40 * 4).fill(255) };
context.loadedMetadata = { resolution: 0.1, originX: 0, originY: 0, yaw: 0,
  negate: 0, freeThreshold: 0.2 };
context.latestRobotStatus = { components: { localization: { pose: { x: 1.5, y: 1.5 } } } };
context.selectedGoal = { x: 2, y: 2 };
assert.match(sandbox.inspectSavedMapLine(), /not a Nav2 route or permission to move/);
assert.ok(context.mapOnlyRoute);
assert.strictEqual(context.previewRoute, null, 'Passive line check must never set a Nav2 preview token');
context.loadedPixels.data[(21 * 40 + 18) * 4] = 0;
assert.match(sandbox.inspectSavedMapLine(), /crosses unknown\/occupied/);
assert.strictEqual(context.mapOnlyRoute, null);
assert.ok(source.includes('if (!response.ok) controlLease = null;'),
  'An authoritative heartbeat rejection must clear the lease');
assert.ok(source.includes('Transient network/AbortController failure'),
  'Transient heartbeat failures must be retried');
console.log('UI wiring and control readiness checks passed');
