"use strict";

// Same-origin robot API for mapping and explicitly gated Nav2 planner requests.
const UI_DEPLOY_VERSION = 24;
const imageInput = document.getElementById("mapImageInput");
const yamlInput = document.getElementById("mapYamlInput");
const loadButton = document.getElementById("loadMapButton");
const loadHostedMapButton = document.getElementById("loadHostedMapButton");
const savedMapPreset = document.getElementById("savedMapPreset");
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
const stopNavigationButton = document.getElementById("stopNavigationButton");
const stopAutonomyButton = document.getElementById("stopAutonomyButton");
const safetyStopStatus = document.getElementById("safetyStopStatus");
const saveMapButton = document.getElementById("saveMapButton");
const saveMapName = document.getElementById("saveMapName");
const pairStatus = document.getElementById("pairStatus");
const apiActionStatus = document.getElementById("apiActionStatus");
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
const savedCloudInput = document.getElementById("savedCloudInput");
const savedCloudReset = document.getElementById("savedCloudReset");
const savedCloudCanvas = document.getElementById("savedCloudCanvas");
const savedCloudContext = savedCloudCanvas.getContext("2d", { alpha: false });
const savedCloudEmpty = document.getElementById("savedCloudEmpty");
const savedCloudState = document.getElementById("savedCloudState");
const savedCloudMeta = document.getElementById("savedCloudMeta");
const mapCatalog = new Map();
let apiConnected = false;
let browserSessionReady = false;
let shutdownRequested = false;
let latestRobotStatus = null;
let refreshingStatus = false;
let refreshingMapCatalog = false;
let refreshingLiveMap = false;
let refreshingPointCloud = false;
let selectedGoal = null;
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
let savedCloudPoints = null;
let savedCloudAzimuth = 0.8;
let savedCloudElevation = 0.55;
let savedCloudZoom = 1;
let savedCloudDrag = null;
let savedCloudPan = [0, 0];

for (const option of savedMapPreset.options) {
  if (option.value !== "custom") {
    mapCatalog.set(option.value, {
      image_url: `/maps/${option.value}.pgm`,
      yaml_url: `/maps/${option.value}.yaml`,
    });
  }
}

savedMapPreset.addEventListener("change", () => {
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
  if (element === motionOutputStatus && state === "not_ready") {
    element.textContent = "isolated";
    element.className = "status-ready";
    return;
  }
  element.textContent = state.replaceAll("_", " ");
  element.className = state === "ready" ? "status-ready" :
    state === "unknown" || state === "starting" ? "unknown" : "status-error";
}

function updateControls(status) {
  const components = status?.components || {};
  const cameraReady = components.camera?.state === "ready";
  const mappingRunning = components.mapping?.process_running === true;
  const hasMap = Boolean(status?.map);
  const canManageMapping = apiConnected && browserSessionReady && !shutdownRequested;
  pairStatus.textContent = !apiConnected ? "Robot API disconnected." :
    browserSessionReady ? "This browser is connected locally. Mapping controls are available." :
      "Connected for status only; automatic browser sessions need the updated robot API.";
  runPreflightButton.disabled = !canManageMapping;
  preparePoweroffButton.disabled = !canManageMapping;
  const nav = components.navigation || {};
  const navStack = components.navigation_stack || {};
  startMappingButton.disabled = !canManageMapping || !cameraReady || mappingRunning;
  stopMappingButton.disabled = !canManageMapping || !mappingRunning;
  pauseMappingButton.disabled = !canManageMapping || !mappingRunning || components.mapping?.paused === true;
  resumeMappingButton.disabled = !canManageMapping || !mappingRunning || components.mapping?.paused !== true;
  startNavigationButton.disabled = !canManageMapping || !cameraReady || mappingRunning ||
    navStack.process_running || !robotMapNames.has(savedMapPreset.value);
  stopNavigationButton.disabled = !canManageMapping || !navStack.process_running;
  stopAutonomyButton.disabled = !canManageMapping;
  stopAutonomyButton.textContent = components.safety_stop?.state === "latched"
    ? "STOP AUTONOMY · LATCHED" : "STOP AUTONOMY";
  saveMapButton.disabled = !canManageMapping || !hasMap;
  const goalMode = mapClickMode.value === "goal";
  sendGoalButton.disabled = !canManageMapping || !selectedGoal || !goalMode || nav.state !== "ready";
  sendGoalButton.disabled = sendGoalButton.disabled || loadedMapName !== savedMapPreset.value;
  plannerOptInButton.disabled = !canManageMapping ||
    (!nav.goal_requests_enabled && (
      nav.planner_ready !== true || loadedMapName !== savedMapPreset.value ||
      nav.active_map_name !== savedMapPreset.value));
  plannerOptInButton.textContent = nav.goal_requests_enabled
    ? "Disable planner goal requests" : "Enable planner goal requests";
  setInitialPoseButton.disabled = !canManageMapping || !selectedGoal || goalMode ||
    loadedMapName !== savedMapPreset.value || !components.navigation_stack?.process_running;
  globalLocalizationButton.disabled = !canManageMapping ||
    components.global_localization?.state !== "ready" || mappingRunning ||
    components.global_localization?.search_active === true ||
    ["map", "camera", "scan", "odometry"].some((name) => components[name]?.state !== "ready") ||
    ["accepted", "executing", "canceling", "requesting"].includes(nav.active_goal_state);
  cancelGoalButton.disabled = !canManageMapping || !["accepted", "executing", "canceling"].includes(nav.active_goal_state);
  goalActionStatus.textContent = nav.reason ||
    "Planner action is disabled by default. The velocity gate has no robot output connection.";
  if (nav.active_goal_detail) {
    if (nav.active_goal_state === "finished_4") {
      goalActionStatus.textContent =
        "Nav2 reports SUCCEEDED (status 4). The planner action completed; motor output remains isolated.";
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
      mapCatalog.set(map.name, { image_url: map.image_url, yaml_url: map.yaml_url });
      if (![...savedMapPreset.options].some((option) => option.value === map.name)) {
        const option = new Option(map.name, map.name);
        savedMapPreset.insertBefore(option, savedMapPreset.options[savedMapPreset.options.length - 1]);
      }
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
    if (!response.ok) throw new Error(`API returned HTTP ${response.status}`);
    const status = await response.json();
    if (!status.ok || !status.components) throw new Error("API returned an invalid status response");
    latestRobotStatus = status;
    const justConnected = !apiConnected;
    apiConnected = true;
    connectionText.textContent = "Robot API connected · controls gated";
    connectionDot.classList.add("connected");
    robotConnectionTitle.textContent = "Robot API connected";
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
  } catch (_error) {
    apiConnected = false;
    latestRobotStatus = null;
    connectionText.textContent = shutdownRequested ? "Project API shut down" : "Robot API not connected";
    connectionDot.classList.remove("connected");
    robotConnectionTitle.textContent = shutdownRequested ? "Shutdown prepared" : "Not connected";
    robotConnectionDetail.textContent = shutdownRequested ?
      "Project processes and web API are stopped. Close the laptop SSH tunnel, then power off the robot normally." :
      "Open this page from the robot-hosted server. Offline map preview still works.";
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
  if (!browserSessionReady) throw new Error("This browser has no active robot session. Reload the page to reconnect.");
  apiActionStatus.textContent = "Sending request…";
  const response = await fetch(path, {
    method: "POST",
    headers: body ? { "Content-Type": "application/json" } : {},
    body: body ? JSON.stringify(body) : undefined,
    credentials: "same-origin",
    cache: "no-store",
  });
  const result = await response.json();
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
    if (!response.ok || !result.ok || result.authorized !== true) throw new Error("Session unavailable");
    browserSessionReady = true;
  } catch (_error) {
    browserSessionReady = false;
  }
  updateControls(latestRobotStatus);
  return browserSessionReady;
}

connectCheckButton.addEventListener("click", async () => {
  connectCheckButton.disabled = true;
  connectCheckStatus.textContent = "Creating browser session and reading current robot status…";
  const sessionReady = await establishBrowserSession();
  if (!sessionReady) {
    connectCheckStatus.textContent = "Could not create a browser session. Check that this page is served by the robot API and reload after reconnecting.";
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
      map_name: savedMapPreset.value,
    });
    goalActionStatus.textContent = `${result.message} Planner only; robot motor output is disconnected.`;
  } catch (error) {
    requestError = error.message;
    goalActionStatus.textContent = requestError;
  } finally {
    await refreshRobotStatus();
    // refreshRobotStatus normally reflects the server's active-goal detail. If
    // the API rejected this request before creating a goal, preserve the
    // actionable HTTP error instead of replacing it with "No navigation goal
    // submitted.".
    if (requestError) goalActionStatus.textContent = requestError;
  }
});

plannerOptInButton.addEventListener("click", async () => {
  const nav = latestRobotStatus?.components?.navigation || {};
  const enabled = nav.goal_requests_enabled !== true;
  if (enabled && !window.confirm(
    "Enable Nav2 planner goal requests? This only enables explicit planner actions. " +
    "It does not connect motor output; do not use it as an emergency stop.")) return;
  plannerOptInButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/navigation/goal-requests", {
      enabled,
      map_name: savedMapPreset.value,
    });
    goalActionStatus.textContent = result.message;
  } catch (error) {
    goalActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

globalLocalizationButton.addEventListener("click", async () => {
  globalLocalizationButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/localization/global");
    globalLocalizationPending = true;
    goalActionStatus.textContent = result.message;
    globalLocalizationStatus.textContent = result.message;
  } catch (error) {
    goalActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

cancelGoalButton.addEventListener("click", async () => {
  cancelGoalButton.disabled = true;
  try {
    const result = await callMappingApi("/api/v1/navigation/cancel");
    goalActionStatus.textContent = result.message;
  } catch (error) {
    goalActionStatus.textContent = error.message;
  } finally {
    await refreshRobotStatus();
  }
});

mapClickMode.addEventListener("change", () => updateControls(latestRobotStatus));

setInitialPoseButton.addEventListener("click", async () => {
  if (!selectedGoal) return;
  const yawDegrees = Number(initialYaw.value);
  if (!Number.isFinite(yawDegrees) || yawDegrees < -180 || yawDegrees > 180) {
    goalActionStatus.textContent = "Heading must be between -180 and 180 degrees.";
    return;
  }
  setInitialPoseButton.disabled = true;
  try {
    const pose = { ...selectedGoal, yaw: yawDegrees * Math.PI / 180 };
    const result = await callMappingApi("/api/v1/localization/initial_pose", pose);
    goalActionStatus.textContent = result.message;
  } catch (error) {
    goalActionStatus.textContent = error.message;
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
    loadHostedMapButton.click();
  } catch (error) {
    apiActionStatus.textContent = error.message;
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

function parseSavedPly(buffer) {
  if (buffer.byteLength > 256 * 1024 * 1024) throw new Error("PLY exceeds the 256 MiB offline-review limit.");
  const bytes = new Uint8Array(buffer);
  const decoder = new TextDecoder("ascii");
  const text = decoder.decode(bytes.subarray(0, Math.min(bytes.length, 128 * 1024)));
  const headerMatch = /end_header\r?\n/.exec(text);
  const headerEnd = headerMatch ? headerMatch.index + headerMatch[0].length : -1;
  if (!text.startsWith("ply\n") && !text.startsWith("ply\r\n")) throw new Error("The saved cloud must be a PLY file.");
  if (headerEnd <= 0) throw new Error("PLY header is incomplete or too large.");
  const header = decoder.decode(bytes.subarray(0, headerEnd)).replace(/\r/g, "");
  const format = header.match(/^format\s+(ascii|binary_little_endian)\s+1\.0$/m)?.[1];
  const count = Number(header.match(/^element\s+vertex\s+(\d+)$/m)?.[1]);
  const properties = [];
  let inVertexElement = false;
  for (const line of header.split("\n")) {
    if (line.startsWith("element ")) { inVertexElement = line.startsWith("element vertex "); continue; }
    const match = inVertexElement && /^property\s+(\w+)\s+(\w+)$/.exec(line);
    if (match) properties.push({ type: match[1], name: match[2] });
  }
  if (!format || !Number.isInteger(count) || count < 1 || count > 2_000_000) throw new Error("PLY must have 1–2,000,000 vertices and use ASCII or little-endian binary format.");
  const xyz = ["x", "y", "z"].map((name) => properties.findIndex((property) => property.name === name));
  if (xyz.some((index) => index < 0)) throw new Error("PLY vertex properties must include x, y, and z.");
  const values = [];
  if (format === "ascii") {
    const rows = decoder.decode(bytes.subarray(headerEnd)).trim().split(/\r?\n/);
    if (rows.length < count) throw new Error("PLY vertex data is incomplete.");
    for (let row = 0; row < count; row += 1) {
      const fields = rows[row].trim().split(/\s+/);
      const point = xyz.map((index) => Number(fields[index]));
      if (point.every(Number.isFinite)) values.push(...point);
    }
  } else {
    const sizeByType = { char: 1, uchar: 1, short: 2, ushort: 2, int: 4, uint: 4, float: 4, double: 8 };
    const stride = properties.reduce((total, property) => total + (sizeByType[property.type] || 0), 0);
    if (!stride || headerEnd + count * stride > bytes.length) throw new Error("Unsupported or incomplete binary PLY vertex data.");
    const view = new DataView(buffer, headerEnd);
    const read = (offset, type) => ({ float: view.getFloat32, double: view.getFloat64, char: view.getInt8, uchar: view.getUint8, short: view.getInt16, ushort: view.getUint16, int: view.getInt32, uint: view.getUint32 }[type].call(view, offset, true));
    for (let row = 0; row < count; row += 1) {
      let offset = row * stride;
      const point = [];
      for (let index = 0; index < properties.length; index += 1) {
        const property = properties[index];
        const value = read(offset, property.type);
        if (xyz.includes(index)) point[xyz.indexOf(index)] = value;
        offset += sizeByType[property.type];
      }
      if (point.every(Number.isFinite)) values.push(...point);
    }
  }
  if (!values.length) throw new Error("PLY contains no finite XYZ vertices.");
  const stride = Math.max(1, Math.ceil(values.length / 3 / 150_000));
  return new Float32Array(values.filter((_, index) => Math.floor(index / 3) % stride === 0));
}

function drawSavedCloud() {
  if (!savedCloudPoints) return;
  const rect = savedCloudCanvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * devicePixelRatio));
  const height = Math.max(1, Math.round(rect.height * devicePixelRatio));
  if (savedCloudCanvas.width !== width || savedCloudCanvas.height !== height) { savedCloudCanvas.width = width; savedCloudCanvas.height = height; }
  savedCloudContext.fillStyle = "#07101b"; savedCloudContext.fillRect(0, 0, width, height);
  const points = savedCloudPoints; let cx = 0; let cy = 0; let cz = 0;
  for (let i = 0; i < points.length; i += 3) { cx += points[i]; cy += points[i + 1]; cz += points[i + 2]; }
  const count = points.length / 3; cx /= count; cy /= count; cz /= count;
  let radius = 0.5; for (let i = 0; i < points.length; i += 3) radius = Math.max(radius, Math.hypot(points[i] - cx, points[i + 1] - cy, points[i + 2] - cz));
  const cosA = Math.cos(savedCloudAzimuth), sinA = Math.sin(savedCloudAzimuth), cosE = Math.cos(savedCloudElevation), sinE = Math.sin(savedCloudElevation);
  const scale = Math.min(width, height) * 0.42 * savedCloudZoom / radius;
  const drawn = [];
  for (let i = 0; i < points.length; i += 3) {
    const x = points[i] - cx, y = points[i + 1] - cy, z = points[i + 2] - cz;
    const rx = cosA * x - sinA * y, ry = sinA * x + cosA * y, depth = cosE * ry + sinE * z;
    drawn.push([depth, width / 2 + savedCloudPan[0] + rx * scale, height / 2 + savedCloudPan[1] - (cosE * z - sinE * ry) * scale, z]);
  }
  drawn.sort((a, b) => b[0] - a[0]);
  for (const [depth, x, y, z] of drawn) { const shade = Math.max(80, Math.min(220, 155 + z * 28 - depth * 9)); savedCloudContext.fillStyle = `rgb(${Math.round(shade * .55)},${Math.round(shade)},${Math.round(shade * .85)})`; savedCloudContext.fillRect(x, y, 2 * devicePixelRatio, 2 * devicePixelRatio); }
}

savedCloudInput.addEventListener("change", async () => {
  const file = savedCloudInput.files[0]; if (!file) return;
  try {
    savedCloudState.textContent = "LOADING CLOUD";
    savedCloudPoints = parseSavedPly(await file.arrayBuffer());
    savedCloudCanvas.hidden = false; savedCloudEmpty.hidden = true; savedCloudReset.disabled = false;
    savedCloudState.textContent = "3D REVIEW ONLY";
    savedCloudMeta.textContent = `${file.name} · ${(savedCloudPoints.length / 3).toLocaleString()} displayed vertices · map frame required`;
    drawSavedCloud();
  } catch (error) { savedCloudPoints = null; savedCloudCanvas.hidden = true; savedCloudEmpty.hidden = false; savedCloudState.textContent = "CLOUD NOT LOADED"; savedCloudMeta.textContent = error instanceof Error ? error.message : "Could not read the PLY cloud."; }
});
savedCloudReset.addEventListener("click", () => { savedCloudAzimuth = .8; savedCloudElevation = .55; savedCloudZoom = 1; savedCloudPan = [0, 0]; drawSavedCloud(); });
savedCloudCanvas.addEventListener("pointerdown", (event) => { savedCloudDrag = { x: event.clientX, y: event.clientY, pan: event.shiftKey || event.button === 1 }; savedCloudCanvas.setPointerCapture(event.pointerId); });
savedCloudCanvas.addEventListener("pointermove", (event) => { if (!savedCloudDrag) return; const dx = event.clientX - savedCloudDrag.x, dy = event.clientY - savedCloudDrag.y; if (savedCloudDrag.pan) { savedCloudPan[0] += dx * devicePixelRatio; savedCloudPan[1] += dy * devicePixelRatio; } else { savedCloudAzimuth += dx * .008; savedCloudElevation = Math.max(-1.35, Math.min(1.35, savedCloudElevation + dy * .008)); } savedCloudDrag.x = event.clientX; savedCloudDrag.y = event.clientY; drawSavedCloud(); });
savedCloudCanvas.addEventListener("pointerup", () => { savedCloudDrag = null; });
savedCloudCanvas.addEventListener("pointercancel", () => { savedCloudDrag = null; });
savedCloudCanvas.addEventListener("wheel", (event) => { event.preventDefault(); savedCloudZoom = Math.max(.35, Math.min(5, savedCloudZoom * (event.deltaY < 0 ? 1.12 : .89))); drawSavedCloud(); }, { passive: false });
window.addEventListener("resize", drawSavedCloud);

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
  loadedMapName = imageName.replace(/\.(pgm|png)$/i, "");
  selectedPixel = null;
  selectedGoal = null;
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
    ? "Coverage is too sparse for reliable localization: rebuild this map with one slow, closed manual loop before using it for navigation."
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
    mapCanvas.hidden = true;
    emptyMap.hidden = false;
    mapQuality.hidden = true;
    mapSurface.classList.remove("has-map");
    mapState.textContent = "MAP NOT LOADED";
    mapMeta.textContent = error instanceof Error ? error.message : "Could not read these map files.";
    mapSize.textContent = "— m × — m";
  } finally {
    button.disabled = false;
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
    mapCanvas.hidden = true;
    emptyMap.hidden = false;
    mapQuality.hidden = true;
    mapSurface.classList.remove("has-map");
    mapState.textContent = "MAP NOT LOADED";
    mapMeta.textContent = error instanceof Error ? error.message : "Could not load the hosted map.";
    mapSize.textContent = "— m × — m";
    loadHostedMapButton.disabled = false;
  }
});

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
  const pose = latestRobotStatus?.components?.localization?.pose;
  if (pose && loadedMetadata) {
    const dx = pose.x - loadedMetadata.originX;
    const dy = pose.y - loadedMetadata.originY;
    const cosine = Math.cos(loadedMetadata.yaw);
    const sine = Math.sin(loadedMetadata.yaw);
    const localX = cosine * dx + sine * dy;
    const localY = -sine * dx + cosine * dy;
    const robotX = localX / loadedMetadata.resolution - 0.5;
    const robotY = loadedPixels.height - localY / loadedMetadata.resolution - 0.5;
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
      `Robot position: verified AMCL pose · x ${pose.x.toFixed(2)} m · y ${pose.y.toFixed(2)} m · heading ${headingDegrees.toFixed(0)}°` :
      `Robot marker is unverified: ${localization?.reason || "AMCL is not ready"}. Do not use it to choose a destination.`;
    mapPositionStatus.className = `map-position-status ${verified ? "verified" : "unverified"}`;
  } else {
    mapPositionStatus.textContent = "Robot position is unavailable until localization reports a pose.";
    mapPositionStatus.className = "map-position-status";
  }
  if (!selectedPixel) return;

  const radius = Math.max(3, Math.min(12, Math.min(mapCanvas.width, mapCanvas.height) * 0.008));
  mapContext.strokeStyle = "#20d8ad";
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

  const localX = (pixelX + 0.5) * loadedMetadata.resolution;
  const localY = (loadedPixels.height - pixelY - 0.5) * loadedMetadata.resolution;
  const cosine = Math.cos(loadedMetadata.yaw);
  const sine = Math.sin(loadedMetadata.yaw);
  const worldX = loadedMetadata.originX + cosine * localX - sine * localY;
  const worldY = loadedMetadata.originY + sine * localX + cosine * localY;

  selectedPixel = { x: pixelX, y: pixelY };
  selectedGoal = { x: worldX, y: worldY, yaw: 0.0 };
  goalCoordinates.textContent = `x: ${worldX.toFixed(3)} m, y: ${worldY.toFixed(3)} m`;
  goalPreview.hidden = false;
  updateControls(latestRobotStatus);
  paintMap();
});
