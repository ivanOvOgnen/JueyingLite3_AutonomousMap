// Evaluate real pure functions and event handlers without starting the app/ROS.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const acorn = require('acorn');
const source = fs.readFileSync(path.join(__dirname, '../webapp/app.js'), 'utf8');
const tree = acorn.parse(source, { ecmaVersion: 2022 });
const names = ['headingRadians', 'updateGoalCoordinates', 'pixelToMap', 'mapToPixel'];
const code = tree.body.filter(node => node.type === 'FunctionDeclaration' && names.includes(node.id.name))
  .map(node => source.slice(node.start, node.end)).join('\n');
const context = vm.createContext({ Math, Number, String, Error });
vm.runInContext(code, context);
const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`);
const metadata = { resolution: 0.5, originX: -2, originY: 3, yaw: Math.PI / 2, negate: 0, freeThreshold: 0.196 };
const corner = context.pixelToMap(0, 0, 4, metadata);
near(corner.x, -3.75);
near(corner.y, 3.25);
for (const yaw of [0, Math.PI / 2, -Math.PI / 2, Math.PI, 0.731]) {
  const map = { ...metadata, yaw };
  for (const pixel of [[0, 0], [9, 3], [4, 1]]) {
    const world = context.pixelToMap(pixel[0], pixel[1], 4, map);
    const back = context.mapToPixel(world.x, world.y, 4, map);
    near(back.x, pixel[0]);
    near(back.y, pixel[1]);
  }
}
near(context.headingRadians('90'), Math.PI / 2);
near(context.headingRadians('-180'), -Math.PI);
for (const value of ['', ' ', 'NaN', 'Infinity', '181', '-181']) {
  assert.throws(() => context.headingRadians(value));
}
function handler(target, event) {
  const node = tree.body.find(item => item.type === 'ExpressionStatement' &&
    item.expression.type === 'CallExpression' && item.expression.callee.type === 'MemberExpression' &&
    item.expression.callee.object.name === target && item.expression.callee.property.name === 'addEventListener' &&
    item.expression.arguments[0].value === event);
  assert.ok(node, `Missing ${target}.${event} handler`);
  const callback = node.expression.arguments[1];
  return vm.runInContext('(' + source.slice(callback.start, callback.end) + ')', context);
}
let requests = [];
Object.assign(context, {
  mapView: { suppressClick: false }, loadedPixels: { width: 10, height: 6, data: new Uint8ClampedArray(240).fill(254) }, loadedMetadata: metadata,
  mapCanvas: { getBoundingClientRect: () => ({ left: 20, top: 30, width: 200, height: 120 }) },
  goalCoordinates: {}, goalPreview: {}, clickedPositionHint: {}, mapClickMode: { value: 'goal' }, latestRobotStatus: null,
  updateControls: () => {}, paintMap: () => {}, initialYaw: { value: '90' },
  sendGoalButton: {}, goalActionStatus: {}, loadedMapId: 'map-id', previewRoute: null, savedMapPreset: { value: 'room' },
  showGoalFeedback: (message) => { context.goalActionStatus.textContent = message; },
  callMappingApi: async (url, body) => { requests.push({ url, body }); return { message: 'accepted' }; },
  refreshRobotStatus: async () => {},
});
handler('mapCanvas', 'click')({ clientX: 120, clientY: 90 });
near(context.selectedGoal.x, -3.25);
near(context.selectedGoal.y, 5.75);
assert.deepStrictEqual(JSON.parse(JSON.stringify(context.selectedPixel)), { x: 5, y: 3 });
assert.strictEqual(requests.length, 0, 'Click must preview only');
(async () => {
  context.previewRoute = {preview_id: 'test-preview'};
  await handler('sendGoalButton', 'click')();
  assert.strictEqual(requests.length, 1);
  assert.strictEqual(requests[0].url, '/api/v1/navigation/goal');
  near(requests[0].body.yaw, Math.PI / 2);
  assert.strictEqual(requests[0].body.map_name, 'room');
  context.initialYaw.value = '';
  await handler('sendGoalButton', 'click')();
  assert.strictEqual(requests.length, 1, 'Invalid heading must not dispatch');
  console.log('Map transforms, scaled click preview, heading and dispatch checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
