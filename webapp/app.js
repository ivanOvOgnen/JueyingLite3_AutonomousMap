"use strict";

// Same-origin robot API for mapping and explicitly gated Nav2 planner requests.
const UI_DEPLOY_VERSION = 30;
const imageInput = document.getElementById("mapImageInput");
const yamlInput = document.getElementById("mapYamlInput");
const loadButton = document.getElementById("loadMapButton");
const loadHostedMapButton = document.getElementById("loadHostedMapButton");
const savedMapPreset = document.getElementById("savedMapPreset");
const selectedMapReviewStatus = document.getElementById("selectedMapReviewStatus");
const hostedImageUrl = document.getElementById("hostedImageUrl");
const hostedYamlUrl = document.getElementById("hostedYamlUrl");
const mapCanvas = document.getElementById("mapCanvas");
const mapContext = mapCanvas.getContext("2d", { alpha: false });
const mapSurface = document.getElementById("mapSurface");
const mapZoomInButton = document.getElementById("mapZoomInButton");
const mapZoomOutButton = document.getElementById("mapZoomOutButton");
const mapResetViewButton = document.getElementById("mapResetViewButton");
const mapZoomLabel = document.getElementById("mapZoomLabel");
const mapPositionStatus = document.getElementById("mapPositionStatus");
const emptyMap = document.getElementById("emptyMap");
const mapState = document.getElementById("mapState");
const mapMeta = document.getElementById("mapMeta");
const mapSize = document.getElementById("mapSize");
const mapQuality = document.getElementById("mapQuality");
const mapCellCounts = document.getElementById("mapCellCounts");
const mapQualityNote = document.getElementById("mapQualityNote");
const goalPreview = document.getElementById("goalPreview");
const goalCoordinates = document.getElementById("goalCoordinates");
const clickedPositionHint = document.getElementById("clickedPositionHint");
const sendGoalButton = document.getElementById("sendGoalButton");
const plannerOptInButton = document.getElementById("plannerOptInButton");
const cancelGoalButton = document.getElementById("cancelGoalButton");
const goalActionStatus = document.getElementById("goalActionStatus");
const mapClickMode = document.getElementById("mapClickMode");
const initialYaw = document.getElementById("initialYaw");
const setInitialPoseButton = document.getElementById("setInitialPoseButton");
const globalLocalizationButton = document.getElementById("globalLocalizationButton");
const globalLocalizationStatus = document.getElementById("globalLocalizationStatus");
const connectionDot = document.getElementById("connectionDot");
const connectionText = document.getElementById("connectionText");
const robotConnectionTitle = document.getElementById("robotConnectionTitle");
const robotConnectionDetail = document.getElementById("robotConnectionDetail");
const connectCheckButton = document.getElementById("connectCheckButton");
const connectCheckStatus = document.getElementById("connectCheckStatus");
const cameraStatus = document.getElementById("cameraStatus");
const odometryStatus = document.getElementById("odometryStatus");
const scanStatus = document.getElementById("scanStatus");
const mapRobotStatus = document.getElementById("mapRobotStatus");
const pointCloudRobotStatus = document.getElementById("pointCloudRobotStatus");
const stitchedCloudRobotStatus = document.getElementById("stitchedCloudRobotStatus");
const mappingStatus = document.getElementById("mappingStatus");
const localizationStatus = document.getElementById("localizationStatus");
const navigationStackStatus = document.getElementById("navigationStackStatus");
const navigationStatus = document.getElementById("navigationStatus");
const transformsStatus = document.getElementById("transformsStatus");
const costmapsStatus = document.getElementById("costmapsStatus");
const motionOutputStatus = document.getElementById("motionOutputStatus");
const startMappingButton = document.getElementById("startMappingButton");
const stopMappingButton = document.getElementById("stopMappingButton");
const startNavigationButton = document.getElementById("startNavigationButton");
const returnMappingButton = document.getElementById("returnMappingButton");
const prepareTrialButton = document.getElementById("prepareTrialButton");
const stopNavigationButton = document.getElementById("stopNavigationButton");
const stopAutonomyButton = document.getElementById("stopAutonomyButton");
const safetyStopStatus = document.getElementById("safetyStopStatus");
const saveMapButton = document.getElementById("saveMapButton");
const mapSaveStatus = document.getElementById("mapSaveStatus");
const saveMapName = document.getElementById("saveMapName");
const pairStatus = document.getElementById("pairStatus");
const apiActionStatus = document.getElementById("apiActionStatus");
const mappingReadinessStatus = document.getElementById("mappingReadinessStatus");
const goalReadinessStatus = document.getElementById("goalReadinessStatus");
const pauseMappingButton = document.getElementById("pauseMappingButton");
const resumeMappingButton = document.getElementById("resumeMappingButton");
const runPreflightButton = document.getElementById("runPreflightButton");
const preflightStatus = document.getElementById("preflightStatus");
const preflightOutput = document.getElementById("preflightOutput");
const preparePoweroffButton = document.getElementById("preparePoweroffButton");
const shutdownStatus = document.getElementById("shutdownStatus");
const liveMapCanvas = document.getElementById("liveMapCanvas");
const liveMapContext = liveMapCanvas.getContext("2d", { alpha: false });
const liveMapEmpty = document.getElementById("liveMapEmpty");
const liveMapState = document.getElementById("liveMapState");
const liveMapMeta = document.getElementById("liveMapMeta");
const liveMapExtent = document.getElementById("liveMapExtent");
const pointCloudCanvas = document.getElementById("pointCloudCanvas");
const pointCloudContext = pointCloudCanvas.getContext("2d", { alpha: false });
const pointCloudEmpty = document.getElementById("pointCloudEmpty");
const pointCloudMeta = document.getElementById("pointCloudMeta");
const cloudViewMode = document.getElementById("cloudViewMode");
const cloudAutoFilter = document.getElementById("cloudAutoFilter");
const cloudMaxRange = document.getElementById("cloudMaxRange");
const cloudRangeValue = document.getElementById("cloudRangeValue");
const modeBadge = document.getElementById("modeBadge");
const controlLeaseButton = document.getElementById("controlLeaseButton");
const previewPathButton = document.getElementById("previewPathButton");
const armMotionButton = document.getElementById("armMotionButton");
const finishScanButton = document.getElementById("finishScanButton");
let controlLease = null;
let leaseRenewing = false;
let loadedMapId = null;
let previewRoute = null;
let mapOnlyRoute = null;
let displayedSessionEpoch = null;
const mapCatalog = new Map();
let apiConnected = false;
let browserSessionReady = false;
let shutdownRequested = false;
let latestRobotStatus = null;
let refreshingStatus = false;
let refreshingMapCatalog = false;
let refreshingLiveMap = false;
let refreshingPointCloud = false;
let goalFeedbackHoldUntil = 0;
let selectedGoal = null;
let selectedTargetFree = false;
let globalLocalizationPending = false;
let loadedMapName = null;
let robotMapNames = new Set();
let pointCloudPoints = null;
let renderedPointCloudPoints = null;
let pointCloudFrame = "";
let cloudAzimuth = 0.8;
let cloudElevation = 0.55;
let cloudZoom = 1;
let cloudDrag = null;
let lastPointCloudPoll = 0;

for (const option of savedMapPreset.options) {
  if (option.value !== "custom") {
    mapCatalog.set(option.value, {
      image_url: `/maps/${option.value}.pgm`,
      yaml_url: `/maps/${option.value}.yaml`,
    });
  }
}

savedMapPreset.addEventListener("change", () => {
  selectedGoal = null; selectedTargetFree = false; selectedPixel = null; previewRoute = null; mapOnlyRoute = null; goalPreview.hidden = true;
  updateControls(latestRobotStatus);
  if (savedMapPreset.value === "custom") return;
  const pair = mapCatalog.get(savedMapPreset.value);
  if (!pair) return;
  hostedImageUrl.value = pair.image_url;
  hostedYamlUrl.value = pair.yaml_url;
});

for (const field of [hostedImageUrl, hostedYamlUrl]) {
  field.addEventListener("input", () => {
    savedMapPreset.value = "custom";
  });
}

function setStatusValue(element, component) {
  const state = component?.state || "unknown";
  element.textContent = state.replaceAll("_", " ");
  element.className = state === "ready" ? "status-ready" :
    state === "unknown" || state === "starting" ? "unknown" : "status-error";
}

function showGoalFeedback(message) {
  goalActionStatus.textContent = message;
  goalFeedbackHoldUntil = Date.now() + 12000;
}

function updateControls(status) {
  const components = status?.components || {};
  const cameraReady = components.camera?.state === "ready";
  const mappingRunning = components.mapping?.process_running === true;
  const hasMap = Boolean(status?.map);
  const requiredCapabilities = ["priority_software_stop", "tracked_goal_lifecycle",
    "map_identity", "path_preview", "operator_lease"];
  const backendCompatible = status?.api_contract_version === 2 &&
    requiredCapabilities.every((name) => status?.capabilities?.includes(name));
  const canManageMapping = apiConnected && browserSessionReady &&
    backendCompatible && !shutdownRequested;
  for (const button of [mapZoomInButton, mapZoomOutButton, mapResetViewButton]) {
    button.disabled = !loadedPixels;
    button.title = loadedPixels ? "" : "Load a 2D map before changing its view.";
  }
  pairStatus.textContent = !apiConnected ? "Robot API disconnected." :
    !backendCompatible ? "This page and the robot API are different versions. Open a fresh tab from the robot after restoring matching UI files." :
    browserSessionReady ? "This browser is connected locally. Mapping controls are available." :
      "Connected for status only; automatic browser sessions need the updated robot API.";
  runPreflightButton.disabled = !canManageMapping;
  preparePoweroffButton.disabled = !canManageMapping;
  const nav = components.navigation || {};
  modeBadge.textContent = status?.simulation === true ? "DEMO MODE" : "PLANNER GATED";
  if (displayedSessionEpoch !== null && nav.session_epoch !== displayedSessionEpoch) {
    selectedGoal = null; selectedTargetFree = false; selectedPixel = null; previewRoute = null; mapOnlyRoute = null; controlLease = null;
    goalPreview.hidden = true;
  }
  displayedSessionEpoch = nav.session_epoch ?? null;
  if (previewRoute && nav.path_preview?.preview_id !== previewRoute.preview_id) previewRoute = null;
  if (mapOnlyRoute && components.localization?.pose &&
      Math.hypot(mapOnlyRoute.start[0] - components.localization.pose.x,
                 mapOnlyRoute.start[1] - components.localization.pose.y) > 0.10) mapOnlyRoute = null;
  const navStack = components.navigation_stack || {};
  const mapEntry = mapCatalog.get(savedMapPreset.value);
  const mapReviewed = mapEntry?.integrity === "verified" && mapEntry?.approval_state === "reviewed";
  const trialMapEligible = components.motion_output?.bridge_profile === "supervised_trial" &&
    mapEntry?.integrity === "verified" && nav.active_map_target_eligible === true;
  const targetMapEligible = mapReviewed || trialMapEligible;
  const localizedPose = components.localization?.pose;
  const trialTargetTooFar = trialMapEligible && selectedGoal && localizedPose &&
    Math.hypot(selectedGoal.x - localizedPose.x, selectedGoal.y - localizedPose.y) > 0.50;
  selectedMapReviewStatus.textContent = !robotMapNames.has(savedMapPreset.value) ?
    "This map is not in the robot catalogue. Save or select a robot-hosted map." :
    mapEntry?.integrity === "legacy_unverified" ?
      "This legacy map can be used to test localization. It needs a new verified scan and measured review before target walking." :
      mapEntry?.integrity !== "verified" ?
        "This map failed integrity checks and cannot be used for localization or walking." :
      !mapReviewed && trialMapEligible ?
        "This unreviewed map allows only an attended 0.5 m trial through known-free cells after route preview and live obstacle checks." :
      !mapReviewed ? "This saved map can be used to test localization. Its geometry and route must be reviewed before target walking." :
      "This map has a matching integrity manifest and recorded review.";
  const bridgeProfile = components.motion_output?.bridge_profile;
  startMappingButton.disabled = !canManageMapping || !cameraReady || mappingRunning ||
    navStack.process_running || bridgeProfile !== "telemetry";
  stopMappingButton.disabled = !canManageMapping || !mappingRunning;
  pauseMappingButton.disabled = !canManageMapping || !mappingRunning || components.mapping?.paused === true;
  resumeMappingButton.disabled = !canManageMapping || !mappingRunning || components.mapping?.paused !== true;
  startNavigationButton.disabled = !canManageMapping || !cameraReady || mappingRunning ||
    navStack.process_running || !robotMapNames.has(savedMapPreset.value) || !mapEntry?.map_id ||
    mapEntry.integrity === "failed_or_legacy" || (bridgeProfile === "command" && !mapReviewed);
  prepareTrialButton.disabled = !canManageMapping || mappingRunning || navStack.process_running ||
    components.safety_stop?.state !== "latched" || bridgeProfile === "supervised_trial";
  returnMappingButton.disabled = !canManageMapping || mappingRunning || navStack.process_running ||
    components.safety_stop?.state !== "latched" || bridgeProfile === "telemetry";
  prepareTrialButton.textContent = components.motion_output?.bridge_profile === "supervised_trial" ?
    "Supervised walking trial prepared" : "Prepare supervised walking trial";
  stopNavigationButton.disabled = !canManageMapping || !navStack.process_running;
  stopAutonomyButton.disabled = !canManageMapping;
  stopAutonomyButton.textContent = components.safety_stop?.state === "latched"
    ? "STOP AUTONOMY · LATCHED" : "STOP AUTONOMY";
  const stop = components.safety_stop || {};
  if (stop.cleanup_error) {
    safetyStopStatus.textContent = `Software STOP is latched, but cleanup failed: ${stop.cleanup_error}. Use the vendor stop control and check the robot.`;
  } else if (stop.cleanup_pending) {
    safetyStopStatus.textContent = "Software STOP is latched. Output disable was requested; navigation cleanup is pending.";
  } else if (stop.state === "latched" && Object.keys(stop.disable_results || {}).length) {
    const results = Object.entries(stop.disable_results)
      .map(([name, result]) => `${name.replaceAll("_", " ")}: ${result}`).join(" · ");
    safetyStopStatus.textContent = `Software STOP is latched; navigation cleanup completed. ${results}. This does not confirm a physical stop.`;
  } else if (stop.state === "not_latched") {
    safetyStopStatus.textContent = "Project software STOP is clear. Keep the DEEP app STOP within reach during a supervised trial.";
  }
  const liveMapReady = mappingRunning && components.mapping?.paused !== true &&
    components.map?.state === "ready";
  saveMapButton.disabled = !canManageMapping || !liveMapReady;
  saveMapButton.title = liveMapReady ? "Save the current live SLAM map" :
    components.mapping?.paused === true ? "Resume mapping and wait for a fresh live map before saving." :
    "Start mapping and wait for a fresh live map. A saved localization map cannot be saved as a new scan.";
  finishScanButton.title = components.mapping?.paused === true ?
    "Resume mapping, then hold still and finish the scan." :
    "Finish requires a running scan and two seconds of stationary odometry.";
  const goalMode = mapClickMode.value === "goal";
  const goalPending = (nav.unresolved_goal_count || 0) > 0 ||
    ["accepted", "executing", "canceling", "requesting", "acceptance_pending",
      "cancel_unconfirmed", "unresolved"].includes(nav.active_goal_state);
  sendGoalButton.disabled = !canManageMapping || !selectedGoal || !goalMode || !selectedTargetFree ||
    !targetMapEligible || trialTargetTooFar ||
    nav.state !== "ready" || goalPending;
  sendGoalButton.disabled = sendGoalButton.disabled || loadedMapName !== savedMapPreset.value ||
    !loadedMapId || loadedMapId !== nav.active_map_id || !controlLease || !previewRoute ||
    previewRoute.session_epoch !== nav.session_epoch;
  controlLeaseButton.disabled = !canManageMapping || !components.navigation_stack?.process_running || nav.stop_latched === true;
  controlLeaseButton.textContent = controlLease && nav.operator_lease_valid ? "Session control active" : "Control this session";
  const canPreviewWithNav2 = canManageMapping && selectedTargetFree && targetMapEligible && !trialTargetTooFar &&
    !goalPending && loadedMapId && loadedMapName === savedMapPreset.value &&
    loadedMapId === nav.active_map_id && nav.planner_ready === true;
  previewPathButton.disabled = !loadedPixels || !loadedMetadata || !selectedGoal || !goalMode;
  previewPathButton.textContent = canPreviewWithNav2 ? "Preview Nav2 route" : "Check saved-map line";
  armMotionButton.disabled = !canManageMapping || !controlLease ||
    !["command", "supervised_trial"].includes(components.motion_output?.bridge_profile) ||
    !["accepted", "executing"].includes(nav.active_goal_state);
  armMotionButton.textContent = components.motion_output?.bridge_profile === "supervised_trial" ?
    "Arm supervised goal · 10 s" : "Arm commissioned motion";
  finishScanButton.disabled = !canManageMapping || !mappingRunning || components.mapping?.paused === true;
  plannerOptInButton.disabled = !canManageMapping ||
    (!nav.goal_requests_enabled && (
      !targetMapEligible || nav.planner_ready !== true || loadedMapName !== savedMapPreset.value ||
      nav.active_map_name !== savedMapPreset.value || !controlLease));
  plannerOptInButton.textContent = nav.goal_requests_enabled
    ? "Disable planner goal requests" : "Enable planner goal requests";
  setInitialPoseButton.disabled = !canManageMapping || !selectedGoal || goalMode || goalPending ||
    loadedMapName !== savedMapPreset.value || !components.navigation_stack?.process_running ||
    !loadedMapId || loadedMapId !== nav.active_map_id;
  globalLocalizationButton.disabled = !canManageMapping ||
    components.global_localization?.state !== "ready" || mappingRunning ||
    components.global_localization?.search_active === true ||
    ["map", "camera", "scan", "odometry"].some((name) => components[name]?.state !== "ready") ||
    goalPending;
  cancelGoalButton.disabled = !canManageMapping || !goalPending;
  const localization = components.localization || {};
  const sessionReason = !apiConnected ? "Connect to the robot API." :
    !backendCompatible ? "Robot API and page versions differ; deploy matching files." :
    !browserSessionReady ? "Click Connect & check status to create a browser session." :
    shutdownRequested ? "The project API was shut down." : "";
  const mappingReason = sessionReason || (mappingRunning ?
    components.mapping?.paused === true ? "Mapping is paused. Resume before saving or finishing." :
      "Mapping is running. Guide the robot manually, then finish and save." :
    navStack.process_running ? "Stop localization/Nav2 before mapping." :
    bridgeProfile !== "telemetry" ? "Latch project STOP, then choose Return to mapping mode." :
    !cameraReady ? "Wait for fresh depth camera data; run the read-only preflight if it stays unavailable." :
    "Ready to start a manual scan.");
  const localizationReason = sessionReason || (navStack.process_running ?
    "Localization/Nav2 is running." : mappingRunning ? "Finish or stop mapping first." :
    !cameraReady ? "Wait for fresh depth camera data." :
    !robotMapNames.has(savedMapPreset.value) || !mapEntry?.map_id ? "Select a map saved on the robot; refresh status if it was just saved." :
    mapEntry.integrity === "failed_or_legacy" ? "This robot map failed its integrity check." :
    bridgeProfile === "command" && !mapReviewed ? "Command profile requires a reviewed map; return to telemetry for localization testing." :
    "Ready to start localization on the selected robot map.");
  const trialReason = sessionReason || (mappingRunning || navStack.process_running ?
    "Stop mapping and localization/Nav2 before changing bridge mode." :
    components.safety_stop?.state !== "latched" ? "Press STOP AUTONOMY before changing bridge mode." :
    bridgeProfile === "supervised_trial" ? "Supervised trial mode is prepared." :
    "Supervised trial mode can be prepared now.");
  mappingReadinessStatus.textContent = `Mapping: ${mappingReason} Localization: ${localizationReason} Trial mode: ${trialReason}`;
  startMappingButton.title = mappingReason;
  startNavigationButton.title = localizationReason;
  prepareTrialButton.title = trialReason;
  returnMappingButton.title = trialReason;
  const goalReason = sessionReason || (!navStack.process_running ? "Start localization/Nav2." :
    nav.stop_latched === true ? "Project STOP is latched; restart localization/Nav2 before requesting a goal." :
    !loadedMapId || loadedMapName !== savedMapPreset.value ? "Load the selected robot map using Load selected map." :
    loadedMapId !== nav.active_map_id ? "The displayed map differs from the active localization map; reload the selected map." :
    !selectedGoal ? "Choose Robot is here or Navigation target, then click a known-free cell." :
    !goalMode ? "Use clicked point as robot location to seed AMCL, then wait for a stable estimate." :
    !selectedTargetFree ? "This target is unknown or occupied on the saved map. Select a known-free cell before sending a robot goal." :
    !targetMapEligible ? "Target walking needs a reviewed map or a verified map in supervised trial mode." :
    localization.state !== "ready" ? "Wait for a stable AMCL estimate; verify its marker against the room." :
    trialTargetTooFar ? "Supervised trial targets must be within 0.50 m of the robot. Select a closer known-free cell." :
    nav.planner_ready !== true ? (nav.reason || "Wait for fresh transforms, costmaps, and obstacle observations.") :
    !previewRoute ? "Preview the route before sending the target." :
    !controlLease ? "Click Control this session to acquire a control lease." :
    !nav.goal_requests_enabled ? "Enable planner goal requests, then send the target." :
    "Target request is ready; physical motion still needs supervised arming.");
  const mapOnlyStep = selectedGoal && goalMode && loadedPixels && loadedMetadata && !canPreviewWithNav2;
  goalReadinessStatus.textContent = mapOnlyStep ?
    `Next step: ${mapOnlyRoute ? "Saved-map straight line checked; wait for live Nav2 checks" :
      "Click Check saved-map line to inspect the selected target"}. This is not a Nav2 route. Goal submission is blocked: ${goalReason}` :
    `Next step: ${goalReason}`;
  for (const button of [setInitialPoseButton, globalLocalizationButton, controlLeaseButton,
    previewPathButton, plannerOptInButton, sendGoalButton, armMotionButton]) {
    if (button.disabled) button.title = goalReason;
    else button.removeAttribute("title");
  }
  if (Date.now() < goalFeedbackHoldUntil) return;
  if (loadedMapName && loadedMapName !== savedMapPreset.value) {
    goalActionStatus.textContent =
      `The displayed map (${loadedMapName}) differs from the selected robot map (${savedMapPreset.value}). Load the selected map before setting a pose or goal.`;
  } else if (!navStack.process_running) {
    goalActionStatus.textContent = "Start localization / Nav2 with the selected saved map before setting a pose or goal.";
  } else if (localization.state !== "ready") {
    goalActionStatus.textContent =
      `${localization.reason || "AMCL has not established a stable pose."} ` +
      "Check the saved map against the room. If you know the robot's actual position, choose “Robot starting pose” under “Map click selects”, click that position, and set its heading. Goal requests remain disabled until localization stabilizes.";
  } else {
    goalActionStatus.textContent = nav.reason ||
      "Planner action is disabled by default. The velocity gate has no robot output connection.";
  }
  if (previewRoute && !goalPending && ["idle", "stopped", "rejected", "succeeded", "canceled", "aborted"].includes(nav.active_goal_state)) {
    goalActionStatus.textContent = "Route preview ready. Review the displayed route; submitting still requires planner opt-in and session control.";
  } else if (nav.active_goal_state === "stopped" && nav.stop_latched !== true) {
    goalActionStatus.textContent = "No active goal in this localization session. Select a target and use the Next step guidance.";
  } else if (nav.active_goal_detail && nav.active_goal_state !== "idle") {
    if (status?.simulation && nav.active_goal_state === "succeeded") {
      goalActionStatus.textContent = "Simulation: virtual target reached. No robot moved.";
    } else if (["succeeded", "finished_4"].includes(nav.active_goal_state)) {
      goalActionStatus.textContent =
        "Nav2 reports SUCCEEDED (status 4). Check the robot's physical position before confirming arrival.";
    } else {
      goalActionStatus.textContent = nav.active_goal_detail;
    }
  }
}

async function refreshMapCatalog() {
  if (refreshingMapCatalog) return;
  refreshingMapCatalog = true;
  try {
    const response = await fetch("/api/v1/maps", { cache: "no-store" });
    if (!response.ok) return;
    const result = await response.json();
    robotMapNames = new Set((result.maps || []).map((map) => map.name).filter((name) => typeof name === "string"));
    for (const map of result.maps || []) {
      if (typeof map.name !== "string" || typeof map.image_url !== "string" || typeof map.yaml_url !== "string") continue;
      const previous = mapCatalog.get(map.name);
      if (loadedMapName === map.name && previous?.map_id && previous.map_id !== map.map_id) {
        loadedMapId = null; selectedGoal = null; selectedTargetFree = false; selectedPixel = null; previewRoute = null; mapOnlyRoute = null;
      }
      mapCatalog.set(map.name, { ...map });
      if (![...savedMapPreset.options].some((option) => option.value === map.name)) {
        const option = new Option(map.name, map.name);
        savedMapPreset.insertBefore(option, savedMapPreset.options[savedMapPreset.options.length - 1]);
      }
    }
    if (!robotMapNames.has(savedMapPreset.value) && savedMapPreset.value !== "custom" && result.maps?.length) {
      savedMapPreset.value = result.maps[0].name;
      hostedImageUrl.value = result.maps[0].image_url;
      hostedYamlUrl.value = result.maps[0].yaml_url;
    }
    updateControls(latestRobotStatus);
  } catch (_error) {
    // A status-only gateway can still be used when its saved-map catalog is unavailable.
  } finally {
    refreshingMapCatalog = false;
  }
}

startNavigationButton.addEventListener("click", async () => {
  startNavigationButton.disabled = true;
  try {
    const mapName = savedMapPreset.value;
    const result = await callMappingApi("/api/v1/navigation/start", { name: mapName });
    apiActionStatus.textContent = result.message;
  } catch (error) {
    apiActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

prepareTrialButton.addEventListener("click", async () => {
  prepareTrialButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/bridge/profile", {profile: "supervised_trial"});
    apiActionStatus.textContent = `${result.message} Start localization/Nav2 next.`;
  } catch (error) { apiActionStatus.textContent = error.message; }
  await refreshRobotStatus();
});

returnMappingButton.addEventListener("click", async () => {
  returnMappingButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/bridge/profile", {profile: "telemetry"});
    apiActionStatus.textContent = `${result.message} Mapping mode is ready; guide the robot with the RC in manual mode.`;
  } catch (error) { apiActionStatus.textContent = error.message; }
  await refreshRobotStatus();
});

stopNavigationButton.addEventListener("click", async () => {
  stopNavigationButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/navigation/stop");
    apiActionStatus.textContent = result.message;
  } catch (error) {
    apiActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

stopAutonomyButton.addEventListener("click", async () => {
  stopAutonomyButton.disabled = true;
  safetyStopStatus.textContent = "Sending software stop: cancel goal, disable the autonomy gate, and stop Nav2.";
  try {
    const result = await callMappingApi("/api/v1/safety/stop");
    safetyStopStatus.textContent = result.message;
    goalActionStatus.textContent = result.message;
  } catch (error) {
    safetyStopStatus.textContent = `STOP could not be confirmed: ${error.message}. Use the vendor controller or hardware stop immediately.`;
  } finally {
    await refreshRobotStatus();
  }
});

async function refreshRobotStatus() {
  if (refreshingStatus) return;
  refreshingStatus = true;
  try {
    const response = await fetch("/api/v1/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`/api/v1/status returned HTTP ${response.status}; check that the robot runs the matching API`);
    const status = await response.json();
    if (!status.ok || !status.components) throw new Error("API returned an invalid status response");
    latestRobotStatus = status;
    const justConnected = !apiConnected;
    apiConnected = true;
    connectionText.textContent = status.simulation === true ? "Demo server connected" : "Robot API connected · controls gated";
    connectionDot.classList.add("connected");
    robotConnectionTitle.textContent = status.simulation === true ? "Demo server connected" : "Robot API connected";
    robotConnectionDetail.textContent = status.components.navigation?.reason ||
      "Mapping can be managed here. Autonomous navigation is not commissioned.";
    setStatusValue(cameraStatus, status.components.camera);
    setStatusValue(odometryStatus, status.components.odometry);
    setStatusValue(scanStatus, status.components.scan);
    setStatusValue(mapRobotStatus, status.components.map);
    setStatusValue(pointCloudRobotStatus, status.components.pointcloud);
    setStatusValue(stitchedCloudRobotStatus, status.components.stitched_pointcloud);
    setStatusValue(mappingStatus, status.components.mapping);
    setStatusValue(localizationStatus, status.components.localization);
    setStatusValue(navigationStackStatus, status.components.navigation_stack);
    setStatusValue(navigationStatus, status.components.navigation);
    setStatusValue(transformsStatus, status.components.transforms);
    setStatusValue(costmapsStatus, status.components.costmaps);
    setStatusValue(motionOutputStatus, status.components.motion_output);
    updateControls(status);
    const localization = status.components.localization || {};
    if (localization.pose) {
      globalLocalizationPending = localization.converged !== true;
      const pose = localization.pose;
      const estimate = `AMCL estimate: x ${pose.x.toFixed(2)} m, y ${pose.y.toFixed(2)} m, heading ${(pose.yaw * 180 / Math.PI).toFixed(0)}°.`;
      globalLocalizationStatus.textContent = localization.converged === true
        ? `${estimate} Stable estimate. Check the blue robot marker against the room before sending a goal.`
        : `${estimate} ${localization.reason || "Checking pose stability against live scans."}`;
    } else if (globalLocalizationPending) {
      globalLocalizationStatus.textContent = status.components.global_localization?.reason ||
        "Global localization requested. Waiting for repeated AMCL updates from live scans.";
    } else if (!status.components.navigation_stack?.process_running) {
      globalLocalizationStatus.textContent = "Start localization/Nav2, then request global localization; this requires a saved map and live scan/odometry.";
    }
    paintMap();
    if (justConnected) refreshMapCatalog();
    refreshLiveMap();
    if (Date.now() - lastPointCloudPoll > (cloudViewMode.value === "live" ? 500 : 1800)) refreshPointCloud();
  } catch (error) {
    apiConnected = false;
    latestRobotStatus = null;
    connectionText.textContent = shutdownRequested ? "Project API shut down" : "Robot API not connected";
    connectionDot.classList.remove("connected");
    robotConnectionTitle.textContent = shutdownRequested ? "Shutdown prepared" : "Not connected";
    robotConnectionDetail.textContent = shutdownRequested ?
      "Project processes and web API are stopped. Close the laptop SSH tunnel, then power off the robot normally." :
      `Status check failed: ${error instanceof Error ? error.message : String(error)}. Offline map preview still works.`;
    for (const element of [cameraStatus, odometryStatus, scanStatus, mapRobotStatus, pointCloudRobotStatus, stitchedCloudRobotStatus,
      mappingStatus, localizationStatus, navigationStackStatus, navigationStatus, transformsStatus, costmapsStatus, motionOutputStatus]) {
      element.textContent = "unknown";
      element.className = "unknown";
    }
    updateControls(null);
    globalLocalizationPending = false;
    globalLocalizationStatus.textContent = "Start localization/Nav2, then request global localization; this requires a saved map and live scan/odometry.";
    liveMapState.textContent = "DISCONNECTED";
    liveMapMeta.textContent = "Waiting for map data";
    pointCloudMeta.textContent = "Waiting for camera points";
  } finally {
    refreshingStatus = false;
  }
}

async function callMappingApi(path, body) {
  if (!browserSessionReady) throw new Error("This browser has no active robot session. Click Connect & check status to reconnect.");
  apiActionStatus.textContent = "Sending request…";
  if (body !== undefined) body = {
    session_epoch: latestRobotStatus?.components?.navigation?.session_epoch,
    map_id: loadedMapId, lease_token: controlLease?.lease_token, ...body,
  };
  const response = await fetch(path, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : {},
    body: body ? JSON.stringify(body) : undefined,
    credentials: "same-origin",
    cache: "no-store",
  });
  const result = await response.json().catch(() => ({ok: false, error: `Robot API returned HTTP ${response.status} without JSON.`}));
  if (response.status === 401 || response.status === 403) {
    browserSessionReady = false;
    updateControls(latestRobotStatus);
    throw new Error("Browser session expired. Click Connect & check status, then retry the action.");
  }
  if (!response.ok || !result.ok) throw new Error(result.error || `Request failed (${response.status})`);
  apiActionStatus.textContent = result.message || "Request completed.";
  await refreshRobotStatus();
  return result;
}

async function establishBrowserSession() {
  browserSessionReady = false;
  try {
    const response = await fetch("/api/v1/session", {
      cache: "no-store", credentials: "same-origin",
    });
    const result = await response.json();
    if (!response.ok || !result.ok || result.authorized !== true) {
      throw new Error(result.error || `Session endpoint returned HTTP ${response.status}`);
    }
    browserSessionReady = true;
  } catch (error) {
    browserSessionReady = false;
    connectCheckStatus.textContent = `Browser session unavailable: ${error instanceof Error ? error.message : String(error)}. Check the robot API version or tunnel.`;
  }
  updateControls(latestRobotStatus);
  return browserSessionReady;
}

connectCheckButton.addEventListener("click", async () => {
  connectCheckButton.disabled = true;
  connectCheckStatus.textContent = "Creating browser session and reading current robot status…";
  const sessionReady = await establishBrowserSession();
  if (!sessionReady) {
    connectCheckButton.disabled = false;
    return;
  }
  await refreshRobotStatus();
  if (apiConnected && latestRobotStatus) {
    const states = Object.entries(latestRobotStatus.components || {})
      .map(([name, component]) => `${name}: ${(component?.state || "unknown").replaceAll("_", " ")}`)
      .join(" · ");
    connectCheckStatus.textContent = `Connected; read-only status checked at ${new Date().toLocaleTimeString()}. ${states || "No component details returned."}`;
  } else {
    connectCheckStatus.textContent = "Browser session created, but the status endpoint did not answer. No robot action was sent.";
  }
  connectCheckButton.disabled = false;
});

sendGoalButton.addEventListener("click", async () => {
  if (!selectedGoal) return;
  sendGoalButton.disabled = true;
  let requestError = null;
  try {
    const result = await callMappingApi("/api/v1/navigation/goal", {
      ...selectedGoal,
      yaw: headingRadians(initialYaw.value),
      map_name: savedMapPreset.value,
      map_id: loadedMapId,
      preview_id: previewRoute && previewRoute.preview_id,
    });
    goalActionStatus.textContent = result.planner_only === false ? `${result.message} Commissioned motion still requires explicit arming.` :
      latestRobotStatus?.components?.motion_output?.bridge_profile === "supervised_trial" ?
      `${result.message} Arm the short supervised trial to move.` : `${result.message} Planner only; no motor permission granted.`;
  } catch (error) {
    requestError = error.message;
    showGoalFeedback(requestError);
  } finally {
    await refreshRobotStatus();
    // refreshRobotStatus normally reflects the server's active-goal detail. If
    // the API rejected this request before creating a goal, preserve the
    // actionable HTTP error instead of replacing it with "No navigation goal
    // submitted.".
    if (requestError) showGoalFeedback(requestError);
  }
});

plannerOptInButton.addEventListener("click", async () => {
  const nav = latestRobotStatus?.components?.navigation || {};
  const enabled = nav.goal_requests_enabled !== true;
  if (enabled && !await confirmAction(
    "Enable Nav2 planner goal requests? This only enables explicit planner actions. " +
    "It does not connect motor output; do not use it as an emergency stop.")) return;
  plannerOptInButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/navigation/goal-requests", {
      enabled,
      map_name: savedMapPreset.value,
    });
    showGoalFeedback(result.message);
  } catch (error) {
    showGoalFeedback(error.message);
  } finally {
    await refreshRobotStatus();
  }
});

globalLocalizationButton.addEventListener("click", async () => {
  globalLocalizationButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/localization/global");
    globalLocalizationPending = true;
    showGoalFeedback(result.message);
    globalLocalizationStatus.textContent = result.message;
  } catch (error) {
    showGoalFeedback(error.message);
  } finally {
    await refreshRobotStatus();
  }
});

cancelGoalButton.addEventListener("click", async () => {
  cancelGoalButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/navigation/cancel");
    showGoalFeedback(result.message);
  } catch (error) {
    showGoalFeedback(error.message);
  } finally {
    await refreshRobotStatus();
  }
});

mapClickMode.addEventListener("change", () => {
  // A pose click must never silently become a goal after changing the mode.
  selectedGoal = null;
  selectedTargetFree = false;
  selectedPixel = null;
  previewRoute = null;
  mapOnlyRoute = null;
  goalPreview.hidden = true;
  updateControls(latestRobotStatus);
  paintMap();
});

setInitialPoseButton.addEventListener("click", async () => {
  if (!selectedGoal) return;
  setInitialPoseButton.disabled = true;
  try {
    const pose = { ...selectedGoal, yaw: headingRadians(initialYaw.value) };
    const result = await callMappingApi("/api/v1/localization/initial_pose", pose);
    showGoalFeedback(result.message);
    mapClickMode.value = "goal";
    selectedGoal = null;
    selectedTargetFree = false;
    selectedPixel = null;
    previewRoute = null;
    mapOnlyRoute = null;
    goalPreview.hidden = true;
    paintMap();
  } catch (error) {
    showGoalFeedback(error.message);
  } finally {
    await refreshRobotStatus();
  }
});

runPreflightButton.addEventListener("click", async () => {
  runPreflightButton.disabled = true;
  preflightStatus.textContent = "Collecting read-only robot diagnostics…";
  preflightOutput.hidden = true;
  try {
    const response = await fetch("/api/v1/diagnostics/preflight", {
      method: "POST", credentials: "same-origin", cache: "no-store",
    });
    const result = await response.json();
    if (typeof result.output === "string") {
      preflightOutput.textContent = result.output;
      preflightOutput.hidden = false;
    }
    if (!response.ok || !result.ok) {
      throw new Error(result.error || `Preflight returned HTTP ${response.status} (exit ${result.exit_code ?? "unknown"}).`);
    }
    const missingPublishers = ["/leg_odom2", "/odom", "/scan", "/map", "/amcl_pose"]
      .filter((topic) => {
        const escaped = topic.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
        const section = result.output.match(new RegExp(`--- ${escaped} ---([\\s\\S]*?)(?=\\n--- |$)`));
        const publisherCount = section?.[1].match(/Publisher count:\s*(\d+)/);
        return !publisherCount || Number(publisherCount[1]) === 0;
      });
    if (missingPublishers.length) {
      const components = latestRobotStatus?.components || {};
      const directReadiness = {
        "/leg_odom2": components.odometry?.state === "ready",
        "/odom": components.transforms?.state === "ready" && components.navigation_stack?.state === "ready",
        "/scan": components.scan?.state === "ready",
        "/map": components.map?.state === "ready",
        "/amcl_pose": components.localization?.state === "ready",
      };
      const probeOnlyMisses = missingPublishers.every((topic) => directReadiness[topic]);
      if (probeOnlyMisses) {
        preflightStatus.textContent = `Snapshot complete. The short-lived ROS 2 graph probe did not discover ${missingPublishers.join(", ")}, but the live API simultaneously reports fresh dependent data and ready TF/Nav2 state. Treat the graph subsection as incomplete DDS discovery, not proof that those publishers are absent. Confirm freshness, pose alignment, costmaps, and stop/arbitration readiness before any motion.`;
        return;
      }
      const ros1Odom = result.output.match(/ROS 1 \/leg_odom2 telemetry source graph:([\s\S]*?)(?=\nROS 1 \/cmd_vel graph:|$)/);
      const ros1OdomHasPublisher = /Publishers:\s*\r?\n\s+\* /.test(ros1Odom?.[1] || "");
      const odomDetail = ros1OdomHasPublisher
        ? "ROS 1 /leg_odom2 has a source publisher, but its project ROS 1→ROS 2 bridge is not publishing. Starting the project mapping or localization/Nav2 stack creates that narrow bridge."
        : "The ROS 1 /leg_odom2 source is also missing; restore vendor odometry before starting the project stack.";
      preflightStatus.textContent = `Snapshot complete. No ROS 2 publisher was found for ${missingPublishers.join(", ")}. ${odomDetail} Localization/Nav2 needs fresh odometry, scan, and map data. This check does not certify motion readiness; goals remain separately gated.`;
    } else {
      preflightStatus.textContent = "Snapshot complete. Required ROS publishers are visible; confirm fresh messages, localization, costmaps, and stop/arbitration readiness before any motion. This check does not certify motion readiness.";
    }
  } catch (error) {
    preflightStatus.textContent = error.message;
  } finally {
    updateControls(latestRobotStatus);
  }
});

preparePoweroffButton.addEventListener("click", async () => {
  const confirmed = window.prompt(
    "This stops project mapping/localization and closes the project API. It does not power off the robot or stop vendor services. Type STOP PROJECT to continue.");
  if (confirmed !== "STOP PROJECT") return;
  preparePoweroffButton.disabled = true;
  shutdownStatus.textContent = "Stopping project processes. Keep the robot stationary; wait for confirmation.";
  try {
    const response = await fetch("/api/v1/shutdown/prepare", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ confirm: confirmed }),
      credentials: "same-origin",
      cache: "no-store",
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || `Shutdown request failed (${response.status}).`);
    shutdownRequested = true;
    shutdownStatus.textContent = `${result.message} Close the laptop SSH tunnel separately; this page cannot terminate a tunnel running on the laptop.`;
    apiActionStatus.textContent = "Project shutdown confirmed.";
    updateControls(latestRobotStatus);
  } catch (error) {
    shutdownStatus.textContent = `Shutdown was not confirmed: ${error.message} Check robot status before powering off.`;
    preparePoweroffButton.disabled = false;
  }
});

startMappingButton.addEventListener("click", async () => {
  startMappingButton.disabled = true;
  try {
    await callMappingApi("/api/v1/mapping/start");
    apiActionStatus.textContent = "Mapping started. Guide the dog manually with the RC.";
  } catch (error) {
    apiActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

stopMappingButton.addEventListener("click", async () => {
  stopMappingButton.disabled = true;
  try {
    await callMappingApi("/api/v1/mapping/stop");
  } catch (error) {
    apiActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

pauseMappingButton.addEventListener("click", async () => {
  try {
    const result = await callMappingApi("/api/v1/mapping/pause");
    apiActionStatus.textContent = result.message;
  } catch (error) { apiActionStatus.textContent = error.message; }
  await refreshRobotStatus();
});

resumeMappingButton.addEventListener("click", async () => {
  try {
    const result = await callMappingApi("/api/v1/mapping/resume");
    apiActionStatus.textContent = result.message;
  } catch (error) { apiActionStatus.textContent = error.message; }
  await refreshRobotStatus();
});

saveMapButton.addEventListener("click", async () => {
  saveMapButton.disabled = true;
  mapSaveStatus.textContent = "Saving the live map on the robot…";
  try {
    const result = await callMappingApi("/api/v1/maps/save", { name: saveMapName.value.trim() });
    hostedImageUrl.value = result.image_url;
    hostedYamlUrl.value = result.yaml_url;
    const name = result.name;
    let option = [...savedMapPreset.options].find((item) => item.value === name);
    if (!option) {
      option = new Option(name, name);
      savedMapPreset.insertBefore(option, savedMapPreset.options[savedMapPreset.options.length - 1]);
    }
    mapCatalog.set(name, { image_url: result.image_url, yaml_url: result.yaml_url });
    savedMapPreset.value = name;
    await refreshMapCatalog();
    mapSaveStatus.textContent = `Saved ${name}.pgm and ${name}.yaml on the robot. Select “Load selected map” to inspect them.`;
  } catch (error) {
    mapSaveStatus.textContent = `Map was not saved: ${error.message}`;
  } finally {
    await refreshRobotStatus();
  }
});

const now = new Date();
const two = (value) => String(value).padStart(2, "0");
saveMapName.value = `lite3_scan_${now.getFullYear()}${two(now.getMonth() + 1)}${two(now.getDate())}_${two(now.getHours())}${two(now.getMinutes())}`;
establishBrowserSession().then(refreshRobotStatus);
window.setInterval(refreshRobotStatus, 2000);
window.setInterval(() => { if (apiConnected) refreshLiveMap(); }, 1000);
window.setInterval(() => { if (apiConnected) refreshMapCatalog(); }, 10000);
window.setInterval(() => {
  if (!apiConnected || refreshingPointCloud) return;
  const interval = cloudViewMode.value === "live" ? 500 : 1800;
  if (Date.now() - lastPointCloudPoll >= interval) refreshPointCloud();
}, 250);

async function refreshLiveMap() {
  if (refreshingLiveMap) return;
  refreshingLiveMap = true;
  try {
    const response = await fetch("/api/v1/map/grid", { cache: "no-store" });
    if (!response.ok) throw new Error("Live map is not available yet");
    const grid = await response.json();
    if (!grid.ok || !Number.isInteger(grid.width) || !Number.isInteger(grid.height) ||
        grid.width <= 0 || grid.height <= 0 || grid.data.length !== grid.width * grid.height) {
      throw new Error("Map grid response is invalid");
    }
    liveMapCanvas.width = grid.width;
    liveMapCanvas.height = grid.height;
    const image = liveMapContext.createImageData(grid.width, grid.height);
    let known = 0;
    for (let y = 0; y < grid.height; y += 1) {
      for (let x = 0; x < grid.width; x += 1) {
        const value = grid.data[(grid.height - y - 1) * grid.width + x];
        const offset = (y * grid.width + x) * 4;
        if (value < 0) {
          image.data.set([112, 128, 141, 255], offset);
        } else if (value >= 65) {
          image.data.set([29, 40, 48, 255], offset);
          known += 1;
        } else if (value <= 20) {
          image.data.set([229, 236, 234, 255], offset);
          known += 1;
        } else {
          image.data.set([148, 160, 170, 255], offset);
          known += 1;
        }
      }
    }
    liveMapContext.putImageData(image, 0, 0);
    liveMapCanvas.hidden = false;
    liveMapEmpty.hidden = true;
    liveMapState.textContent = latestRobotStatus?.components?.mapping?.paused ? "PAUSED" : "LIVE";
    liveMapMeta.textContent = `${grid.width} × ${grid.height} cells · ${(100 * known / grid.data.length).toFixed(1)}% observed`;
    liveMapExtent.textContent = `${(grid.width * grid.resolution).toFixed(2)} × ${(grid.height * grid.resolution).toFixed(2)} m`;
  } catch (_error) {
    liveMapCanvas.hidden = true;
    liveMapEmpty.hidden = false;
    liveMapState.textContent = "WAITING";
    liveMapMeta.textContent = "Live map appears after SLAM publishes fresh cells";
  } finally {
    refreshingLiveMap = false;
  }
}

async function refreshPointCloud() {
  if (refreshingPointCloud) return;
  refreshingPointCloud = true;
  lastPointCloudPoll = Date.now();
  const endpoint = cloudViewMode.value === "live" ? "/api/v1/pointcloud" : "/api/v1/pointcloud/map";
  try {
    const response = await fetch(endpoint, { cache: "no-store" });
    if (!response.ok) throw new Error("No point cloud available");
    const buffer = await response.arrayBuffer();
    if (buffer.byteLength % 12 !== 0 || buffer.byteLength === 0) throw new Error("Point cloud data is invalid");
    pointCloudPoints = new Float32Array(buffer);
    pointCloudFrame = response.headers.get("X-Point-Frame") || (cloudViewMode.value === "live" ? "camera" : "map");
    cloudMaxRange.disabled = cloudViewMode.value !== "live";
    cloudRangeValue.textContent = `${Number(cloudMaxRange.value).toFixed(1)} m`;
    renderedPointCloudPoints = filterPointCloud(pointCloudPoints, cloudViewMode.value === "live");
    pointCloudCanvas.hidden = false;
    pointCloudEmpty.hidden = true;
    const count = Number(response.headers.get("X-Point-Count")) || pointCloudPoints.length / 3;
    const age = Number(response.headers.get("X-Point-Age"));
    const visibleCount = renderedPointCloudPoints.length / 3;
    const filterNote = cloudAutoFilter.checked ? ` · ${visibleCount.toLocaleString()} shown after filter` : " · raw view";
    pointCloudMeta.textContent = `${count.toLocaleString()} points · ${pointCloudFrame}${Number.isFinite(age) ? ` · ${age.toFixed(2)} s old` : " · accumulated"}${filterNote}`;
    drawPointCloud();
  } catch (_error) {
    pointCloudPoints = null;
    renderedPointCloudPoints = null;
    pointCloudCanvas.hidden = true;
    pointCloudEmpty.hidden = false;
    pointCloudEmpty.textContent = cloudViewMode.value === "live"
      ? "Waiting for fresh depth points."
      : "The stitched view appears while mapping has map-to-camera TF.";
    pointCloudMeta.textContent = "No registered points available";
  } finally {
    refreshingPointCloud = false;
  }
}

function filterPointCloud(points, isLiveCamera) {
  if (!cloudAutoFilter.checked) return points;
  const maxRange = Number(cloudMaxRange.value);
  const voxelSize = 0.16;
  const stride = Math.max(1, Math.ceil(points.length / 180000));
  const eligible = [];
  let eligibleCount = 0;
  const voxelCounts = new Map();
  const voxelKey = (x, y, z) => `${x},${y},${z}`;
  for (let i = 0; i < points.length; i += 3 * stride) {
    const x = points[i], y = points[i + 1], z = points[i + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if (isLiveCamera && (z <= 0.10 || Math.hypot(x, y, z) > maxRange)) continue;
    const bx = Math.floor(x / voxelSize), by = Math.floor(y / voxelSize), bz = Math.floor(z / voxelSize);
    const key = voxelKey(bx, by, bz);
    voxelCounts.set(key, (voxelCounts.get(key) || 0) + 1);
    eligible.push(i, bx, by, bz);
    eligibleCount += 1;
  }
  const kept = [];
  for (let n = 0; n < eligible.length; n += 4) {
    const index = eligible[n], bx = eligible[n + 1], by = eligible[n + 2], bz = eligible[n + 3];
    let support = 0;
    for (let dx = -1; dx <= 1 && support < 3; dx += 1) {
      for (let dy = -1; dy <= 1 && support < 3; dy += 1) {
        for (let dz = -1; dz <= 1 && support < 3; dz += 1) {
          support += voxelCounts.get(voxelKey(bx + dx, by + dy, bz + dz)) || 0;
        }
      }
    }
    if (support >= 3) kept.push(points[index], points[index + 1], points[index + 2]);
  }
  // Keep a sparse but otherwise valid scene visible if the sampling density is low.
  if (eligibleCount && kept.length / 3 < eligibleCount * 0.25) {
    const fallback = [];
    for (let n = 0; n < eligible.length; n += 4) {
      const i = eligible[n];
      fallback.push(points[i], points[i + 1], points[i + 2]);
    }
    return new Float32Array(fallback);
  }
  return new Float32Array(kept);
}

function drawPointCloud() {
  if (!renderedPointCloudPoints?.length) return;
  const rect = pointCloudCanvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * Math.min(devicePixelRatio || 1, 1.5)));
  const height = Math.max(1, Math.round(rect.height * Math.min(devicePixelRatio || 1, 1.5)));
  if (pointCloudCanvas.width !== width || pointCloudCanvas.height !== height) {
    pointCloudCanvas.width = width;
    pointCloudCanvas.height = height;
  }
  const ctx = pointCloudContext;
  ctx.fillStyle = "#07101b";
  ctx.fillRect(0, 0, width, height);
  const p = renderedPointCloudPoints;
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < p.length; i += 3) {
    minX = Math.min(minX, p[i]); maxX = Math.max(maxX, p[i]);
    minY = Math.min(minY, p[i + 1]); maxY = Math.max(maxY, p[i + 1]);
    minZ = Math.min(minZ, p[i + 2]); maxZ = Math.max(maxZ, p[i + 2]);
  }
  const center = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2];
  const span = Math.max(0.25, Math.hypot(maxX - minX, maxY - minY, maxZ - minZ));
  const radius = span * 1.25 / cloudZoom;
  const ce = Math.cos(cloudElevation), se = Math.sin(cloudElevation);
  const ca = Math.cos(cloudAzimuth), sa = Math.sin(cloudAzimuth);
  const camera = [center[0] + radius * ce * ca, center[1] + radius * ce * sa, center[2] + radius * se];
  const forward = center.map((value, i) => value - camera[i]);
  const fn = Math.hypot(...forward) || 1;
  for (let i = 0; i < 3; i += 1) forward[i] /= fn;
  const worldUp = pointCloudFrame === "map" ? [0, 0, 1] : [0, -1, 0];
  let right = [forward[1] * worldUp[2] - forward[2] * worldUp[1],
    forward[2] * worldUp[0] - forward[0] * worldUp[2],
    forward[0] * worldUp[1] - forward[1] * worldUp[0]];
  const rn = Math.hypot(...right) || 1;
  right = right.map((value) => value / rn);
  const up = [right[1] * forward[2] - right[2] * forward[1],
    right[2] * forward[0] - right[0] * forward[2],
    right[0] * forward[1] - right[1] * forward[0]];
  const focal = Math.min(width, height) * 0.9;
  const drawn = [];
  for (let i = 0; i < p.length; i += 3) {
    const relative = [p[i] - camera[0], p[i + 1] - camera[1], p[i + 2] - camera[2]];
    const depth = relative[0] * forward[0] + relative[1] * forward[1] + relative[2] * forward[2];
    if (depth <= 0.02) continue;
    const horizontal = relative[0] * right[0] + relative[1] * right[1] + relative[2] * right[2];
    const vertical = relative[0] * up[0] + relative[1] * up[1] + relative[2] * up[2];
    const x = width / 2 + focal * horizontal / depth;
    const y = height / 2 - focal * vertical / depth;
    if (x >= 0 && x < width && y >= 0 && y < height) drawn.push([depth, x, y, p[i + 2]]);
  }
  drawn.sort((a, b) => b[0] - a[0]);
  for (const [depth, x, y, z] of drawn) {
    const brightness = Math.max(0.35, Math.min(1, 1.5 - depth / (radius * 2)));
    ctx.fillStyle = pointCloudFrame === "map"
      ? `rgba(113,224,191,${brightness})`
      : `rgba(${Math.round(90 + 100 * brightness)},${Math.round(155 + 90 * brightness)},${Math.round(180 + 75 * brightness)},${brightness})`;
    ctx.fillRect(x, y, width < 500 ? 2 : 2.5, width < 500 ? 2 : 2.5);
  }
}

pointCloudCanvas.addEventListener("pointerdown", (event) => {
  cloudDrag = [event.clientX, event.clientY];
  pointCloudCanvas.setPointerCapture(event.pointerId);
});
pointCloudCanvas.addEventListener("pointermove", (event) => {
  if (!cloudDrag) return;
  cloudAzimuth += (event.clientX - cloudDrag[0]) * 0.008;
  cloudElevation = Math.max(-1.35, Math.min(1.35, cloudElevation + (event.clientY - cloudDrag[1]) * 0.008));
  cloudDrag = [event.clientX, event.clientY];
  drawPointCloud();
});
pointCloudCanvas.addEventListener("pointerup", () => { cloudDrag = null; });
pointCloudCanvas.addEventListener("pointercancel", () => { cloudDrag = null; });
pointCloudCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  cloudZoom = Math.max(0.35, Math.min(5, cloudZoom * (event.deltaY < 0 ? 1.12 : 0.89)));
  drawPointCloud();
}, { passive: false });
cloudAutoFilter.addEventListener("change", () => {
  if (!pointCloudPoints) return;
  renderedPointCloudPoints = filterPointCloud(pointCloudPoints, cloudViewMode.value === "live");
  const rawCount = pointCloudPoints.length / 3;
  const shownCount = renderedPointCloudPoints.length / 3;
  pointCloudMeta.textContent = `${rawCount.toLocaleString()} points · ${pointCloudFrame} · ${cloudAutoFilter.checked ? `${shownCount.toLocaleString()} shown after filter` : "raw view"}`;
  drawPointCloud();
});
cloudMaxRange.addEventListener("input", () => {
  cloudRangeValue.textContent = `${Number(cloudMaxRange.value).toFixed(1)} m`;
  if (!pointCloudPoints || cloudViewMode.value !== "live") return;
  renderedPointCloudPoints = filterPointCloud(pointCloudPoints, true);
  drawPointCloud();
});
cloudViewMode.addEventListener("change", () => {
  pointCloudPoints = null;
  renderedPointCloudPoints = null;
  pointCloudEmpty.hidden = false;
  pointCloudCanvas.hidden = true;
  lastPointCloudPoll = 0;
  refreshPointCloud();
});
window.addEventListener("resize", drawPointCloud);

const MAX_PIXELS = 16_000_000;
let loadedPixels = null;
let loadedMetadata = null;
let selectedPixel = null;
const mapView = { scale: 1, offsetX: 0, offsetY: 0, drag: null, suppressClick: false };

function updateMapView() {
  mapCanvas.style.transform = `translate(${mapView.offsetX}px, ${mapView.offsetY}px) scale(${mapView.scale})`;
  const label = `${Math.round(mapView.scale * 100)}%`;
  mapZoomLabel.value = label;
  mapZoomLabel.textContent = label;
}

function resetMapView() {
  mapView.scale = 1;
  mapView.offsetX = 0;
  mapView.offsetY = 0;
  updateMapView();
}

function setMapZoom(nextScale) {
  mapView.scale = Math.max(0.75, Math.min(5, nextScale));
  updateMapView();
}

mapZoomInButton.addEventListener("click", () => setMapZoom(mapView.scale * 1.25));
mapZoomOutButton.addEventListener("click", () => setMapZoom(mapView.scale / 1.25));
mapResetViewButton.addEventListener("click", resetMapView);

mapCanvas.addEventListener("wheel", (event) => {
  if (!loadedPixels) return;
  event.preventDefault();
  setMapZoom(mapView.scale * (event.deltaY < 0 ? 1.15 : 1 / 1.15));
}, { passive: false });

mapCanvas.addEventListener("pointerdown", (event) => {
  if (!loadedPixels) return;
  mapView.drag = {
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    offsetX: mapView.offsetX,
    offsetY: mapView.offsetY,
    moved: false,
  };
  mapCanvas.setPointerCapture(event.pointerId);
  mapCanvas.style.cursor = "grabbing";
});

mapCanvas.addEventListener("pointermove", (event) => {
  const drag = mapView.drag;
  if (!drag || event.pointerId !== drag.pointerId) return;
  const dx = event.clientX - drag.startX;
  const dy = event.clientY - drag.startY;
  if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
  mapView.offsetX = drag.offsetX + dx;
  mapView.offsetY = drag.offsetY + dy;
  updateMapView();
});

function finishMapDrag(event) {
  const drag = mapView.drag;
  if (!drag || event.pointerId !== drag.pointerId) return;
  mapView.suppressClick = drag.moved;
  mapView.drag = null;
  mapCanvas.style.cursor = "crosshair";
  if (mapCanvas.hasPointerCapture(event.pointerId)) mapCanvas.releasePointerCapture(event.pointerId);
}

mapCanvas.addEventListener("pointerup", finishMapDrag);
mapCanvas.addEventListener("pointercancel", finishMapDrag);

function updateLoadButton() {
  loadButton.disabled = !(imageInput.files.length && yamlInput.files.length);
}

imageInput.addEventListener("change", updateLoadButton);
yamlInput.addEventListener("change", updateLoadButton);

function parseMapYaml(text) {
  const imageMatch = text.match(/^\s*image\s*:\s*(.+?)\s*$/m);
  const resolutionMatch = text.match(/^\s*resolution\s*:\s*([-+\d.eE]+)\s*$/m);
  const originMatch = text.match(/^\s*origin\s*:\s*\[([^\]]+)\]\s*$/m);
  if (!imageMatch || !resolutionMatch || !originMatch) {
    throw new Error("YAML must contain image, resolution, and origin fields.");
  }

  const imageName = imageMatch[1].split("#", 1)[0].trim().replace(/^['\"]|['\"]$/g, "").split(/[\\/]/).pop();
  const resolution = Number(resolutionMatch[1]);
  const origin = originMatch[1].split(",").map((part) => Number(part.trim()));
  const readYamlNumber = (key, fallback) => {
    const match = text.match(new RegExp(`^\\s*${key}\\s*:\\s*([-+\\d.eE]+)\\s*$`, "m"));
    return match ? Number(match[1]) : fallback;
  };
  const negate = readYamlNumber("negate", 0);
  const occupiedThreshold = readYamlNumber("occupied_thresh", 0.65);
  const freeThreshold = readYamlNumber("free_thresh", 0.25);
  if (!Number.isFinite(resolution) || resolution <= 0 || origin.length < 2 ||
      !origin.slice(0, 3).every((value) => Number.isFinite(value)) ||
      ![negate, occupiedThreshold, freeThreshold].every(Number.isFinite) ||
      ![0, 1].includes(negate) || freeThreshold < 0 || occupiedThreshold > 1 ||
      freeThreshold >= occupiedThreshold) {
    throw new Error("Map YAML has invalid geometry or occupancy thresholds.");
  }

  return {
    imageName, resolution, originX: origin[0], originY: origin[1], yaw: origin[2] || 0,
    negate, occupiedThreshold, freeThreshold,
  };
}

function parsePgm(buffer) {
  const bytes = new Uint8Array(buffer);
  let offset = 0;
  const isWhitespace = (byte) => byte === 9 || byte === 10 || byte === 11 || byte === 12 || byte === 13 || byte === 32;

  function nextToken() {
    while (offset < bytes.length) {
      if (isWhitespace(bytes[offset])) {
        offset += 1;
      } else if (bytes[offset] === 35) {
        while (offset < bytes.length && bytes[offset] !== 10 && bytes[offset] !== 13) offset += 1;
      } else {
        break;
      }
    }
    const start = offset;
    while (offset < bytes.length && !isWhitespace(bytes[offset]) && bytes[offset] !== 35) offset += 1;
    if (start === offset) throw new Error("Invalid PGM header.");
    return new TextDecoder("ascii").decode(bytes.subarray(start, offset));
  }

  const format = nextToken();
  if (format !== "P2" && format !== "P5") throw new Error("Only P2 or P5 PGM map images are supported.");
  const width = Number(nextToken());
  const height = Number(nextToken());
  const maxValue = Number(nextToken());
  const count = width * height;
  if (!Number.isInteger(width) || !Number.isInteger(height) || width <= 0 || height <= 0 ||
      count > MAX_PIXELS || !Number.isInteger(maxValue) || maxValue < 1 || maxValue > 255) {
    throw new Error("PGM dimensions or max value are unsupported.");
  }

  const pixels = new Uint8ClampedArray(count * 4);
  if (format === "P2") {
    for (let index = 0; index < count; index += 1) {
      const sample = Number(nextToken());
      if (!Number.isInteger(sample) || sample < 0 || sample > maxValue) throw new Error("Invalid PGM pixel data.");
      const gray = Math.round(sample * 255 / maxValue);
      const pixelIndex = index * 4;
      pixels[pixelIndex] = gray;
      pixels[pixelIndex + 1] = gray;
      pixels[pixelIndex + 2] = gray;
      pixels[pixelIndex + 3] = 255;
    }
  } else {
    // P5 has one whitespace separator after maxValue; consume CRLF as a pair.
    if (!isWhitespace(bytes[offset])) throw new Error("Invalid P5 raster separator.");
    const separator = bytes[offset++];
    if (separator === 13 && bytes[offset] === 10) offset += 1;
    if (offset + count > bytes.length) throw new Error("PGM raster is incomplete.");
    for (let index = 0; index < count; index += 1) {
      const gray = Math.round(bytes[offset + index] * 255 / maxValue);
      const pixelIndex = index * 4;
      pixels[pixelIndex] = gray;
      pixels[pixelIndex + 1] = gray;
      pixels[pixelIndex + 2] = gray;
      pixels[pixelIndex + 3] = 255;
    }
  }
  return new ImageData(pixels, width, height);
}

async function readMapImage(file) {
  if (file.size === 0) throw new Error("The selected map image is empty.");
  if (file.name.toLowerCase().endsWith(".pgm")) return parsePgm(await file.arrayBuffer());

  if (!file.type.startsWith("image/png")) throw new Error("Choose a PGM or PNG map image.");
  const bitmap = await createImageBitmap(file);
  if (bitmap.width * bitmap.height > MAX_PIXELS) {
    bitmap.close();
    throw new Error("Map image is too large to preview in this browser.");
  }
  const scratch = document.createElement("canvas");
  scratch.width = bitmap.width;
  scratch.height = bitmap.height;
  const context = scratch.getContext("2d", { willReadFrequently: true });
  context.drawImage(bitmap, 0, 0);
  bitmap.close();
  return context.getImageData(0, 0, scratch.width, scratch.height);
}

loadButton.addEventListener("click", async () => {
  const imageFile = imageInput.files[0];
  const yamlFile = yamlInput.files[0];
  if (!imageFile || !yamlFile) return;

  await loadMapFiles(imageFile, yamlFile, false);
});

function updateMapDisplay(imageName, pixels, metadata, source) {
  loadedPixels = pixels;
  loadedMetadata = metadata;
  loadedMapId = null; previewRoute = null; mapOnlyRoute = null;
  loadedMapName = imageName.replace(/\.(pgm|png)$/i, "");
  selectedPixel = null;
  selectedGoal = null;
  selectedTargetFree = false;
  goalPreview.hidden = true;
  mapCanvas.width = pixels.width;
  mapCanvas.height = pixels.height;
  resetMapView();
  paintMap();
  mapCanvas.hidden = false;
  emptyMap.hidden = true;
  mapSurface.classList.add("has-map");
  mapState.textContent = source === "hosted" ? "HOSTED MAP LOADED" : "LOCAL MAP LOADED";
  mapMeta.textContent = `${imageName} · ${pixels.width} × ${pixels.height} px · ${metadata.resolution} m/cell`;
  mapSize.textContent = `${(pixels.width * metadata.resolution).toFixed(2)} m × ${(pixels.height * metadata.resolution).toFixed(2)} m`;
  let free = 0;
  let occupied = 0;
  let unknown = 0;
  for (let index = 0; index < pixels.data.length; index += 4) {
    const gray = (pixels.data[index] + pixels.data[index + 1] + pixels.data[index + 2]) / 3;
    const occupancy = metadata.negate ? gray / 255 : (255 - gray) / 255;
    if (occupancy > metadata.occupiedThreshold) occupied += 1;
    else if (occupancy < metadata.freeThreshold) free += 1;
    else unknown += 1;
  }
  const total = free + occupied + unknown;
  const percent = (value) => `${(100 * value / total).toFixed(1)}%`;
  mapCellCounts.textContent = `Free ${percent(free)} · occupied ${percent(occupied)} · unknown ${percent(unknown)} (YAML thresholds)`;
  mapQualityNote.textContent = unknown / total > 0.4
    ? "Large parts of this map are unknown. You can still select a target in known-free space and inspect a saved-map line; robot motion needs live Nav2 checks."
    : "Cell proportions only. Check wall continuity, alignment, and pose before considering navigation.";
  mapQuality.hidden = false;
}

async function loadMapFiles(imageFile, yamlFile, isHosted) {
  const button = isHosted ? loadHostedMapButton : loadButton;
  button.disabled = true;
  mapState.textContent = "LOADING MAP";
  try {
    const [pixels, yamlText] = await Promise.all([readMapImage(imageFile), yamlFile.text()]);
    const metadata = parseMapYaml(yamlText);
    if (metadata.imageName && metadata.imageName.toLowerCase() !== imageFile.name.toLowerCase()) {
      throw new Error(`YAML expects '${metadata.imageName}', but '${imageFile.name}' was selected.`);
    }
    updateMapDisplay(imageFile.name, pixels, metadata, isHosted ? "hosted" : "local");
  } catch (error) {
    loadedPixels = null;
    loadedMetadata = null;
    loadedMapName = null;
    loadedMapId = null;
    selectedGoal = null;
    selectedTargetFree = false;
    selectedPixel = null;
    previewRoute = null;
    mapOnlyRoute = null;
    goalPreview.hidden = true;
    mapCanvas.hidden = true;
    emptyMap.hidden = false;
    mapQuality.hidden = true;
    mapSurface.classList.remove("has-map");
    mapState.textContent = "MAP NOT LOADED";
    mapMeta.textContent = error instanceof Error ? error.message : "Could not read these map files.";
    mapSize.textContent = "— m × — m";
  } finally {
    button.disabled = false;
    updateControls(latestRobotStatus);
  }
}

function hostedPath(value, label) {
  const url = new URL(value.trim(), window.location.origin);
  if (url.origin !== window.location.origin || !url.pathname.startsWith("/maps/") || url.pathname.includes("..")) {
    throw new Error(`${label} must be a same-site path under /maps/.`);
  }
  return url;
}

loadHostedMapButton.addEventListener("click", async () => {
  loadHostedMapButton.disabled = true;
  mapState.textContent = "LOADING MAP";
  try {
    const robotMap = mapCatalog.get(savedMapPreset.value);
    if (robotMap?.map_id) {
      const response = await fetch(`/api/v1/maps/bundle?name=${encodeURIComponent(savedMapPreset.value)}&map_id=${encodeURIComponent(robotMap.map_id)}`, { cache: "no-store" });
      const bundle = await response.json();
      if (!response.ok || !bundle.ok || bundle.map_id !== robotMap.map_id) throw new Error(bundle.error || "Map identity changed");
      const image = Uint8Array.from(atob(bundle.image_base64), (character) => character.charCodeAt(0));
      await loadMapFiles(new File([image], `${bundle.name}.pgm`), new File([bundle.yaml_text], `${bundle.name}.yaml`), true);
      if (loadedPixels) loadedMapId = bundle.map_id;
      updateControls(latestRobotStatus);
      return;
    }
    const imageUrl = hostedPath(hostedImageUrl.value, "Map image URL");
    const yamlUrl = hostedPath(hostedYamlUrl.value, "Map YAML URL");
    const [imageResponse, yamlResponse] = await Promise.all([
      fetch(imageUrl, { cache: "no-store" }),
      fetch(yamlUrl, { cache: "no-store" }),
    ]);
    if (!imageResponse.ok || !yamlResponse.ok) throw new Error("Could not find both files on the laptop. Check the filenames in webapp/maps/.");
    const imageName = decodeURIComponent(imageUrl.pathname.split("/").pop());
    const imageBlob = await imageResponse.blob();
    const imageFile = new File([imageBlob], imageName, { type: imageName.toLowerCase().endsWith(".png") ? "image/png" : "application/octet-stream" });
    const yamlFile = new File([await yamlResponse.blob()], decodeURIComponent(yamlUrl.pathname.split("/").pop()), { type: "text/plain" });
    await loadMapFiles(imageFile, yamlFile, true);
  } catch (error) {
    loadedPixels = null;
    loadedMetadata = null;
    loadedMapName = null;
    loadedMapId = null;
    selectedGoal = null;
    selectedTargetFree = false;
    selectedPixel = null;
    previewRoute = null;
    mapOnlyRoute = null;
    goalPreview.hidden = true;
    mapCanvas.hidden = true;
    emptyMap.hidden = false;
    mapQuality.hidden = true;
    mapSurface.classList.remove("has-map");
    mapState.textContent = "MAP NOT LOADED";
    mapMeta.textContent = error instanceof Error ? error.message : "Could not load the hosted map.";
    mapSize.textContent = "— m × — m";
    loadHostedMapButton.disabled = false;
    updateControls(latestRobotStatus);
  }
});

function headingRadians(value) {
  const degrees = Number(value);
  if (String(value).trim() === "" || !Number.isFinite(degrees) || degrees < -180 || degrees > 180) {
    throw new Error("Enter a heading from -180 to 180 degrees.");
  }
  return degrees * Math.PI / 180;
}

function updateGoalCoordinates() {
  if (!selectedGoal) return;
  const coordinates = `x: ${selectedGoal.x.toFixed(3)} m, y: ${selectedGoal.y.toFixed(3)} m`;
  try {
    const degrees = headingRadians(initialYaw.value) * 180 / Math.PI;
    goalCoordinates.textContent = `${coordinates} · heading: ${degrees.toFixed(0)}°`;
  } catch (error) {
    goalCoordinates.textContent = `${coordinates} · ${error.message}`;
  }
}

initialYaw.addEventListener("input", () => {
  previewRoute = null;
  updateGoalCoordinates();
  updateControls(latestRobotStatus);
  paintMap();
});

function pixelToMap(pixelX, pixelY, height, metadata) {
  const localX = (pixelX + 0.5) * metadata.resolution;
  const localY = (height - pixelY - 0.5) * metadata.resolution;
  const cosine = Math.cos(metadata.yaw), sine = Math.sin(metadata.yaw);
  return {
    x: metadata.originX + cosine * localX - sine * localY,
    y: metadata.originY + sine * localX + cosine * localY,
  };
}

function mapToPixel(worldX, worldY, height, metadata) {
  const dx = worldX - metadata.originX, dy = worldY - metadata.originY;
  const cosine = Math.cos(metadata.yaw), sine = Math.sin(metadata.yaw);
  return {
    x: (cosine * dx + sine * dy) / metadata.resolution - 0.5,
    y: height - (-sine * dx + cosine * dy) / metadata.resolution - 0.5,
  };
}

function knownFreeWithClearance(worldX, worldY, clearanceMeters = 0.30) {
  const pixel = mapToPixel(worldX, worldY, loadedPixels.height, loadedMetadata);
  const cx = Math.round(pixel.x), cy = Math.round(pixel.y);
  const radius = Math.ceil(clearanceMeters / loadedMetadata.resolution);
  for (let dy = -radius; dy <= radius; dy += 1) {
    for (let dx = -radius; dx <= radius; dx += 1) {
      if (dx * dx + dy * dy > radius * radius) continue;
      const x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= loadedPixels.width || y >= loadedPixels.height) return false;
      const index = (y * loadedPixels.width + x) * 4;
      const gray = (loadedPixels.data[index] + loadedPixels.data[index + 1] +
                    loadedPixels.data[index + 2]) / 3;
      const occupancy = loadedMetadata.negate ? gray / 255 : (255 - gray) / 255;
      if (!(occupancy < loadedMetadata.freeThreshold)) return false;
    }
  }
  return true;
}

function inspectSavedMapLine() {
  mapOnlyRoute = null;
  previewRoute = null;
  const pose = latestRobotStatus?.components?.localization?.pose;
  if (!pose || !Number.isFinite(pose.x) || !Number.isFinite(pose.y)) {
    return "Target selected. A robot pose is needed to draw a saved-map line; this does not block selecting the target.";
  }
  const distance = Math.hypot(selectedGoal.x - pose.x, selectedGoal.y - pose.y);
  const steps = Math.max(1, Math.ceil(distance / (loadedMetadata.resolution / 2)));
  for (let index = 0; index <= steps; index += 1) {
    const fraction = index / steps;
    if (!knownFreeWithClearance(pose.x + (selectedGoal.x - pose.x) * fraction,
                                pose.y + (selectedGoal.y - pose.y) * fraction)) {
      return "Target selected. The direct line crosses unknown/occupied map cells or lacks 0.30 m clearance. Nav2 may find another route when live checks recover.";
    }
  }
  mapOnlyRoute = {start: [pose.x, pose.y], points: [[pose.x, pose.y], [selectedGoal.x, selectedGoal.y]]};
  return `Target selected. Dashed line is clear on the saved map (${distance.toFixed(2)} m); it is not a Nav2 route or permission to move.`;
}

function paintMap() {
  if (!loadedPixels) return;
  const rendered = mapContext.createImageData(loadedPixels.width, loadedPixels.height);
  for (let index = 0; index < loadedPixels.data.length; index += 4) {
    const gray = (loadedPixels.data[index] + loadedPixels.data[index + 1] + loadedPixels.data[index + 2]) / 3;
    const occupancy = loadedMetadata.negate ? gray / 255 : (255 - gray) / 255;
    const color = occupancy > loadedMetadata.occupiedThreshold ? [29, 40, 48] :
      occupancy < loadedMetadata.freeThreshold ? [229, 236, 234] : [112, 128, 141];
    rendered.data.set([...color, 255], index);
  }
  mapContext.putImageData(rendered, 0, 0);
  if (loadedMetadata.resolution > 0) {
    const step = Math.max(1, Math.round(1 / loadedMetadata.resolution));
    mapContext.save();
    mapContext.strokeStyle = "rgba(46, 61, 72, 0.26)";
    mapContext.lineWidth = 1;
    for (let x = step; x < mapCanvas.width; x += step) {
      mapContext.beginPath(); mapContext.moveTo(x + 0.5, 0); mapContext.lineTo(x + 0.5, mapCanvas.height); mapContext.stroke();
    }
    for (let y = step; y < mapCanvas.height; y += step) {
      mapContext.beginPath(); mapContext.moveTo(0, y + 0.5); mapContext.lineTo(mapCanvas.width, y + 0.5); mapContext.stroke();
    }
    mapContext.restore();
  }
  if (previewRoute?.map_id === loadedMapId && previewRoute?.points?.length) {
    mapContext.save(); mapContext.strokeStyle = "#ffcc66"; mapContext.lineWidth = 2;
    mapContext.beginPath();
    previewRoute.points.forEach((point, index) => {
      const pixel = mapToPixel(point[0], point[1], loadedPixels.height, loadedMetadata);
      if (index === 0) mapContext.moveTo(pixel.x+0.5, pixel.y+0.5);
      else mapContext.lineTo(pixel.x+0.5, pixel.y+0.5);
    });
    mapContext.stroke(); mapContext.restore();
  }
  if (mapOnlyRoute?.points?.length) {
    mapContext.save();
    mapContext.strokeStyle = "#ffcc66";
    mapContext.lineWidth = 2;
    mapContext.setLineDash([4, 3]);
    mapContext.beginPath();
    mapOnlyRoute.points.forEach((point, index) => {
      const pixel = mapToPixel(point[0], point[1], loadedPixels.height, loadedMetadata);
      if (index === 0) mapContext.moveTo(pixel.x + 0.5, pixel.y + 0.5);
      else mapContext.lineTo(pixel.x + 0.5, pixel.y + 0.5);
    });
    mapContext.stroke();
    mapContext.restore();
  }
  const pose = latestRobotStatus?.components?.localization?.pose;
  if (pose && loadedMetadata) {
    const robotPixel = mapToPixel(pose.x, pose.y, loadedPixels.height, loadedMetadata);
    const robotX = robotPixel.x, robotY = robotPixel.y;
    const heading = pose.yaw - loadedMetadata.yaw;
    const size = Math.max(5, Math.min(16, Math.min(mapCanvas.width, mapCanvas.height) * 0.018));
    mapContext.save();
    mapContext.translate(robotX, robotY);
    mapContext.rotate(-heading);
    const localization = latestRobotStatus?.components?.localization;
    const verified = localization?.state === "ready" && localization?.converged === true;
    mapContext.fillStyle = verified ? "#32a8ff" : "#ffc66d";
    mapContext.strokeStyle = "#071523";
    mapContext.lineWidth = Math.max(1, size / 5);
    mapContext.beginPath();
    mapContext.moveTo(size, 0);
    mapContext.lineTo(-size * 0.7, -size * 0.65);
    mapContext.lineTo(-size * 0.45, 0);
    mapContext.lineTo(-size * 0.7, size * 0.65);
    mapContext.closePath();
    mapContext.fill();
    mapContext.stroke();
    mapContext.restore();
    const headingDegrees = ((heading * 180 / Math.PI % 360) + 360) % 360;
    mapPositionStatus.textContent = verified ?
      `Localized estimate · x ${pose.x.toFixed(2)} m · y ${pose.y.toFixed(2)} m · heading ${headingDegrees.toFixed(0)}°. Check alignment against the real robot.` :
      `Robot marker is uncertain: ${localization?.reason || "AMCL is not ready"}. You may select a target; Nav2 goal submission waits for a verified pose and live checks.`;
    if (latestRobotStatus?.simulation) mapPositionStatus.textContent = `Virtual robot pose · x ${pose.x.toFixed(2)} m · y ${pose.y.toFixed(2)} m · SIMULATION ONLY`;
    mapPositionStatus.className = `map-position-status ${verified ? "verified" : "unverified"}`;
  } else {
    mapPositionStatus.textContent = "Robot position is unavailable until localization reports a pose.";
    mapPositionStatus.className = "map-position-status";
  }
  if (!selectedPixel) return;

  const radius = Math.max(3, Math.min(12, Math.min(mapCanvas.width, mapCanvas.height) * 0.008));
  mapContext.strokeStyle = selectedTargetFree ? "#20d8ad" : "#ffbf69";
  mapContext.lineWidth = Math.max(1, radius / 3);
  mapContext.beginPath();
  mapContext.arc(selectedPixel.x + 0.5, selectedPixel.y + 0.5, radius, 0, Math.PI * 2);
  mapContext.moveTo(selectedPixel.x + 0.5 - radius * 1.5, selectedPixel.y + 0.5);
  mapContext.lineTo(selectedPixel.x + 0.5 + radius * 1.5, selectedPixel.y + 0.5);
  mapContext.moveTo(selectedPixel.x + 0.5, selectedPixel.y + 0.5 - radius * 1.5);
  mapContext.lineTo(selectedPixel.x + 0.5, selectedPixel.y + 0.5 + radius * 1.5);
  mapContext.stroke();
}

mapCanvas.addEventListener("click", (event) => {
  if (mapView.suppressClick) {
    mapView.suppressClick = false;
    return;
  }
  if (!loadedPixels || !loadedMetadata) return;
  const bounds = mapCanvas.getBoundingClientRect();
  const pixelX = Math.max(0, Math.min(loadedPixels.width - 1,
    Math.floor((event.clientX - bounds.left) * loadedPixels.width / bounds.width)));
  const pixelY = Math.max(0, Math.min(loadedPixels.height - 1,
    Math.floor((event.clientY - bounds.top) * loadedPixels.height / bounds.height)));

  const world = pixelToMap(pixelX, pixelY, loadedPixels.height, loadedMetadata);
  const worldX = world.x, worldY = world.y;

  const offset = (pixelY * loadedPixels.width + pixelX) * 4;
  const gray = (loadedPixels.data[offset] + loadedPixels.data[offset+1] + loadedPixels.data[offset+2]) / 3;
  const occupancy = loadedMetadata.negate ? gray/255 : (255-gray)/255;
  const knownFree = occupancy < loadedMetadata.freeThreshold;
  if (!knownFree && mapClickMode.value === "initial") {
    showGoalFeedback("Choose a known-free map cell; occupied and unknown targets are rejected.");
    return;
  }
  previewRoute = null;
  mapOnlyRoute = null;
  selectedPixel = { x: pixelX, y: pixelY };
  selectedGoal = { x: worldX, y: worldY, yaw: 0.0 };
  selectedTargetFree = knownFree;
  clickedPositionHint.textContent = mapClickMode.value === "initial" ?
    "This is where the robot is now. Set its heading, then use this point as the robot location." :
    knownFree ? "This is the target. Preview its route before sending it to Nav2." :
      "Target saved for inspection. This cell is unknown or occupied; Nav2 cannot drive to it.";
  updateGoalCoordinates();
  goalPreview.hidden = false;
  updateControls(latestRobotStatus);
  paintMap();
});


controlLeaseButton.addEventListener("click", async () => {
  try {
    controlLease = await callMappingApi("/api/v1/control/acquire", {});
    showGoalFeedback("Session control acquired. Keep this page visible; loss of its heartbeat stops autonomy.");
  } catch (error) { controlLease = null; showGoalFeedback(error.message); }
  await refreshRobotStatus();
});
previewPathButton.addEventListener("click", async () => {
  if (!selectedGoal) return;
  previewPathButton.disabled = true;
  const nav = latestRobotStatus?.components?.navigation || {};
  const mapEntry = mapCatalog.get(savedMapPreset.value);
  const targetMapEligible = mapEntry?.integrity === "verified" &&
    (mapEntry.approval_state === "reviewed" || nav.active_map_target_eligible === true);
  const pose = latestRobotStatus?.components?.localization?.pose;
  const tooFar = latestRobotStatus?.components?.motion_output?.bridge_profile === "supervised_trial" &&
    pose && Math.hypot(selectedGoal.x - pose.x, selectedGoal.y - pose.y) > 0.50;
  const canUseNav2 = apiConnected && browserSessionReady && selectedTargetFree && targetMapEligible && !tooFar &&
    loadedMapId && loadedMapName === savedMapPreset.value && loadedMapId === nav.active_map_id &&
    nav.planner_ready === true;
  if (!canUseNav2) {
    showGoalFeedback(inspectSavedMapLine());
    paintMap();
    updateControls(latestRobotStatus);
    return;
  }
  try {
    const response = await callMappingApi("/api/v1/navigation/preview", {
      ...selectedGoal, yaw: headingRadians(initialYaw.value), map_name: savedMapPreset.value, map_id: loadedMapId,
    });
    mapOnlyRoute = null;
    previewRoute = response.path_preview;
    showGoalFeedback("Route preview ready. Review the route before submitting the goal.");
    paintMap();
  } catch (error) { previewRoute = null; showGoalFeedback(error.message); }
  await refreshRobotStatus();
});
armMotionButton.addEventListener("click", async () => {
  const trial = latestRobotStatus?.components?.motion_output?.bridge_profile === "supervised_trial";
  if (!await confirmAction(trial ?
    "Start a 10-second supervised walking trial for this map goal? Confirm the robot is standing, Auto Mode is ON, the area and route are clear, and the vendor app STOP is open and reachable. Stay beside the robot." :
    "Arm the commissioned motor path for the active goal? Confirm the robot pose, clear route and operator stop are ready.")) return;
  try {
    const response = await callMappingApi(trial ? "/api/v1/motion/arm-supervised" : "/api/v1/motion/arm",
      trial ? {auto_mode_confirmed:true, app_stop_ready:true, area_clear:true} : {});
    showGoalFeedback(response.message);
  }
  catch (error) { showGoalFeedback(error.message); }
  await refreshRobotStatus();
});
finishScanButton.addEventListener("click", async () => {
  if (!await confirmAction("Hold the robot stationary. Finish this scan and save a new experimental map?")) return;
  try {
    const response = await callMappingApi("/api/v1/maps/finish", {name: saveMapName.value, stationary_ack: true});
    apiActionStatus.textContent = response.message;
    await refreshMapCatalog();
    if ([...savedMapPreset.options].some((option) => option.value === response.name)) {
      savedMapPreset.value = response.name;
      hostedImageUrl.value = response.image_url;
      hostedYamlUrl.value = response.yaml_url;
    }
    mapSaveStatus.textContent = `Saved ${response.name}.pgm and ${response.name}.yaml on the robot. Load selected map to inspect it.`;
    updateControls(latestRobotStatus);
  } catch (error) { mapSaveStatus.textContent = `Map was not saved: ${error.message}`; }
  await refreshRobotStatus();
});
setInterval(async () => {
  if (!controlLease || leaseRenewing || document.hidden) return;
  leaseRenewing = true;
  const controller = new AbortController();
  const deadline = setTimeout(() => controller.abort(), 400);
  try {
    const response = await fetch("/api/v1/control/heartbeat", {method: "POST", credentials: "same-origin",
      headers: {"Content-Type": "application/json"}, signal: controller.signal,
      body: JSON.stringify({lease_token: controlLease.lease_token, session_epoch: controlLease.session_epoch})});
    if (!response.ok) throw new Error("Control lease expired");
  } catch (_error) { controlLease = null; }
  finally { clearTimeout(deadline); leaseRenewing = false; }
}, 200);
function releasePageControl() {
  if (!controlLease) return;
  controlLease = null;
  fetch("/api/v1/safety/stop", {method:"POST",credentials:"same-origin",keepalive:true}).catch(() => {});
}
document.addEventListener("visibilitychange", () => { if (document.hidden) releasePageControl(); });
window.addEventListener("pagehide", releasePageControl);


function confirmAction(message) {
  return new Promise((resolve) => {
    const dialog = document.createElement("dialog");
    if (typeof dialog.showModal !== "function") {
      goalActionStatus.textContent = "This browser cannot show a nonblocking confirmation. Update the browser before arming.";
      resolve(false); return;
    }
    dialog.className = "action-confirm";
    const text = document.createElement("p"); text.textContent = message;
    const cancel = document.createElement("button"); cancel.textContent = "Cancel"; cancel.type = "button";
    const accept = document.createElement("button"); accept.textContent = "Confirm"; accept.type = "button";
    const finish = (value) => { dialog.close(); dialog.remove(); resolve(value); };
    cancel.addEventListener("click", () => finish(false));
    accept.addEventListener("click", () => finish(true));
    dialog.addEventListener("cancel", (event) => { event.preventDefault(); finish(false); });
    dialog.append(text, cancel, accept); document.body.append(dialog); dialog.showModal();
  });
}
