import { physicsFailureMessage } from "./physics-error.js";
import {
  createPerformanceSessionRecorder,
  serializePerformanceSession,
} from "./performance-log.mjs";
import { createWebGlRenderer } from "./webgl-renderer.js";
import { createWebGpuRenderer } from "./webgpu-renderer.js";

const canvas = document.querySelector("#scene");
const status = document.querySelector("#status");
const debug = document.querySelector("#debug");
const resetButton = document.querySelector("#reset");
const pauseButton = document.querySelector("#pause");
const stepButton = document.querySelector("#single-step");
const startPerformanceLogButton = document.querySelector("#start-performance-log");
const downloadPerformanceLogButton = document.querySelector("#download-performance-log");
const performanceLogStatus = document.querySelector("#performance-log-status");
const viewportShell = document.querySelector(".viewport-shell");

const FIXED_STEP_MS = 1000 / 60;
const MAX_CATCH_UP_STEPS = 8;
const MAX_CATCH_UP_WALL_MS = 40;
const MOVE_SPEED = 7 * 60;
const PROJECTILE_SPEED = 96;
const LOOK_SENSITIVITY = 0.0022;
const KEYBOARD_LOOK_SPEED = 1.8;
const FOV_RADIANS = (70 * Math.PI) / 180;
const NEAR_PLANE = 0.8;
const FAR_PLANE = 1400;
const ORIENTATION_SCALE = 1 << 30;
const RENDER_SNAPSHOT_STRIDE = 11;
const keys = new Set();
const characterModeControl = document.querySelector("#character-mode");
const uprightCratesControl = document.querySelector("#upright-crates");
const fixedGeometryControl = document.querySelector("#fixed-geometry-mode");
const characterParameters = new URLSearchParams(window.location.search);
characterModeControl.value = characterParameters.get("character") === "physical" ? "0" : "1";
uprightCratesControl.checked = characterParameters.get("crates") !== "free";
fixedGeometryControl.value = characterParameters.get("bake") === "runtime" ? "0" : "1";
function resetInteractionOptions() {
  const url = new URL(window.location.href);
  url.searchParams.set("character", characterModeControl.value === "0" ? "physical" : "linear");
  url.searchParams.set("crates", uprightCratesControl.checked ? "upright" : "free");
  url.searchParams.set("bake", fixedGeometryControl.value === "1" ? "load" : "runtime");
  window.history.replaceState(null, "", url);
  if (engine) reset();
}
characterModeControl.addEventListener("change", resetInteractionOptions);
uprightCratesControl.addEventListener("change", resetInteractionOptions);
fixedGeometryControl.addEventListener("change", resetInteractionOptions);

let engine = null;
let renderer = null;
let yaw = 0;
let pitch = 0;
let paused = false;
let quiescent = false;
let renderDirty = true;
let jumpQueued = false;
let previousTimestamp = null;
let accumulator = 0;
let dragLook = false;
let dragDistance = 0;
let lastPointer = null;
let renderWidth = 0;
let renderHeight = 0;
let buildProvenance = null;
let pendingPhysicsStepMs = [];
let pendingPhysicsStepStats = [];

function performanceEnvironment() {
  return {
    build: buildProvenance,
    browser: navigator.userAgent,
    platform: navigator.userAgentData?.platform ?? navigator.platform ?? null,
    hardware_concurrency: navigator.hardwareConcurrency ?? null,
    device_memory_gib: navigator.deviceMemory ?? null,
    device_pixel_ratio: window.devicePixelRatio ?? 1,
    renderer: renderer?.backend ?? null,
    viewport_css_pixels: {
      width: Math.round(canvas.getBoundingClientRect().width),
      height: Math.round(canvas.getBoundingClientRect().height),
    },
  };
}

function performanceScenario() {
  const query = new URLSearchParams(window.location.search);
  return {
    character_response: query.get("character") ?? "linear",
    crate_motion: query.get("crates") ?? "upright",
    fixed_geometry: query.get("bake") ?? "load",
    collision_pairs: query.get("collisions") ?? "all",
  };
}

const performanceRecorder = createPerformanceSessionRecorder({
  environment: performanceEnvironment,
  scenario: performanceScenario,
});

const BOX_FACES = [
  { indices: [0, 2, 3, 1], normal: [0, 0, -1], axes: [0, 1] },
  { indices: [4, 5, 7, 6], normal: [0, 0, 1], axes: [0, 1] },
  { indices: [0, 4, 6, 2], normal: [-1, 0, 0], axes: [2, 1] },
  { indices: [1, 3, 7, 5], normal: [1, 0, 0], axes: [2, 1] },
  { indices: [0, 1, 5, 4], normal: [0, -1, 0], axes: [0, 2] },
  { indices: [2, 6, 7, 3], normal: [0, 1, 0], axes: [0, 2] },
];

const FACE_UVS = [
  [0, 0],
  [0, 1],
  [1, 1],
  [1, 0],
];

async function loadEngine() {
  const response = await fetch("physics_engine_demo.wasm");
  if (!response.ok) throw new Error(`WASM fetch failed with ${response.status}`);
  try {
    return (await WebAssembly.instantiateStreaming(response.clone(), {})).instance.exports;
  } catch {
    const bytes = await response.arrayBuffer();
    return (await WebAssembly.instantiate(bytes, {})).instance.exports;
  }
}

async function loadBuildProvenance() {
  try {
    const response = await fetch("build-provenance.json", { cache: "no-store" });
    return response.ok ? await response.json() : null;
  } catch {
    return null;
  }
}

async function createRenderer() {
  const options = {
    fovRadians: FOV_RADIANS,
    nearPlane: NEAR_PLANE,
    farPlane: FAR_PLANE,
  };

  if (navigator.gpu) {
    try {
      const webGpu = await createWebGpuRenderer(canvas, options);
      if (webGpu) return webGpu;
    } catch (error) {
      console.warn("WebGPU renderer initialization failed; trying WebGL2 fallback", error);
    }
  }

  const webGl = createWebGlRenderer(canvas, options);
  if (webGl) return webGl;
  throw new Error("Neither WebGPU nor WebGL2 is available in this browser");
}

function reset() {
  if (
    engine.sandbox_reset_with_baking_options(
      Number(characterModeControl.value),
      Number(uprightCratesControl.checked),
      Number(fixedGeometryControl.value),
    ) !== 0
  ) {
    throw new Error("Unable to initialize the selected physics comparison options");
  }
  yaw = 0;
  pitch = 0;
  paused = false;
  quiescent = false;
  renderDirty = true;
  jumpQueued = false;
  accumulator = 0;
  pendingPhysicsStepMs = [];
  pendingPhysicsStepStats = [];
  pauseButton.textContent = "Pause";
  performanceRecorder.recordMarker("reset", performanceScenario());
  status.textContent = `Click the world to capture the mouse. WASD moves, Space jumps, mouse or arrows look, and click or F shoots. Rendering with ${renderer.backend}.`;
}

function movementVelocity() {
  const forward = Number(keys.has("KeyW")) - Number(keys.has("KeyS"));
  const strafe = Number(keys.has("KeyD")) - Number(keys.has("KeyA"));
  const length = Math.hypot(forward, strafe);
  if (length === 0) return [0, 0];

  const localForward = forward / length;
  const localStrafe = strafe / length;
  const moveX = (localForward * Math.sin(yaw) + localStrafe * Math.cos(yaw)) * MOVE_SPEED;
  const moveZ = (-localForward * Math.cos(yaw) + localStrafe * Math.sin(yaw)) * MOVE_SPEED;
  return [Math.round(moveX), Math.round(moveZ)];
}

function simulationInputActive() {
  return (
    jumpQueued ||
    keys.has("KeyW") ||
    keys.has("KeyA") ||
    keys.has("KeyS") ||
    keys.has("KeyD")
  );
}

function readPhysicsCounter(name) {
  const read = engine?.[name];
  return typeof read === "function" ? read() : null;
}

function lastPhysicsStepStats() {
  return {
    sampled_events: readPhysicsCounter("sandbox_last_sampled_events"),
    tail_contacts: readPhysicsCounter("sandbox_last_tail_contacts"),
    tail_slices: readPhysicsCounter("sandbox_last_tail_slices"),
    tail_replays: readPhysicsCounter("sandbox_last_tail_replays"),
    tail_candidate_pairs: readPhysicsCounter("sandbox_last_tail_candidate_pairs"),
    tail_broad_phase_queries: readPhysicsCounter("sandbox_last_tail_broad_phase_queries"),
    tail_broad_phase_rebuilds: readPhysicsCounter("sandbox_last_tail_broad_phase_rebuilds"),
    tail_broad_phase_reuses: readPhysicsCounter("sandbox_last_tail_broad_phase_reuses"),
    broad_phase_queries: readPhysicsCounter("sandbox_last_broad_phase_queries"),
    broad_phase_rebuilds: readPhysicsCounter("sandbox_last_broad_phase_rebuilds"),
    broad_phase_reuses: readPhysicsCounter("sandbox_last_broad_phase_reuses"),
  };
}

function simulationStep() {
  const [velocityX, velocityZ] = movementVelocity();
  const started = performance.now();
  const error = engine.sandbox_step_velocity(velocityX, velocityZ, jumpQueued ? 1 : 0);
  const durationMs = performance.now() - started;
  pendingPhysicsStepMs.push(durationMs);
  pendingPhysicsStepStats.push(error === 0 ? lastPhysicsStepStats() : {});
  jumpQueued = false;
  renderDirty = true;
  if (error !== 0) {
    paused = true;
    pauseButton.textContent = "Resume";
    const detail =
      error === 6 && typeof engine.sandbox_error_detail === "function"
        ? engine.sandbox_error_detail()
        : 0;
    status.textContent = physicsFailureMessage(error, detail);
    performanceRecorder.recordMarker("physics-error", { error, detail });
    return;
  }
  quiescent =
    typeof engine.sandbox_is_quiescent === "function" && engine.sandbox_is_quiescent() === 1;
}

function shoot() {
  const cosPitch = Math.cos(pitch);
  const velocityX = Math.round(Math.sin(yaw) * cosPitch * PROJECTILE_SPEED);
  const velocityY = Math.round(-Math.sin(pitch) * PROJECTILE_SPEED);
  const velocityZ = Math.round(-Math.cos(yaw) * cosPitch * PROJECTILE_SPEED);
  if (engine.sandbox_shoot(velocityX, velocityY, velocityZ) < 0) {
    status.textContent = "The engine rejected projectile creation.";
    return;
  }
  quiescent = false;
  renderDirty = true;
  performanceRecorder.recordMarker("shoot");
}

function normalizeQuaternion(raw) {
  const quaternion = raw.map((value) => value / ORIENTATION_SCALE);
  const length = Math.hypot(...quaternion);
  if (!Number.isFinite(length) || length === 0) return [0, 0, 0, 1];
  return quaternion.map((value) => value / length);
}

function readBodies() {
  const stride = engine.sandbox_render_snapshot_stride();
  if (stride !== RENDER_SNAPSHOT_STRIDE) {
    throw new Error(`Unexpected render snapshot stride ${stride}`);
  }
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  if (length % stride !== 0) {
    throw new Error(`Malformed render snapshot length ${length}`);
  }

  const values = new Int32Array(engine.memory.buffer, pointer, length);
  const bodies = new Array(length / stride);
  for (let index = 0; index < bodies.length; index += 1) {
    const offset = index * stride;
    bodies[index] = {
      role: values[offset],
      position: [values[offset + 1], values[offset + 2], values[offset + 3]],
      half: [values[offset + 4], values[offset + 5], values[offset + 6]],
      orientation: normalizeQuaternion([
        values[offset + 7],
        values[offset + 8],
        values[offset + 9],
        values[offset + 10],
      ]),
    };
  }
  return bodies;
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (width === renderWidth && height === renderHeight) return false;

  renderWidth = width;
  renderHeight = height;
  if (canvas.width !== width) canvas.width = width;
  if (canvas.height !== height) canvas.height = height;
  renderer.resize(width, height);
  return true;
}

function cameraSpace(point, camera) {
  const dx = point[0] - camera[0];
  const dy = point[1] - camera[1];
  const dz = point[2] - camera[2];
  const sinYaw = Math.sin(yaw);
  const cosYaw = Math.cos(yaw);
  const cameraX = dx * cosYaw + dz * sinYaw;
  const forward = dx * sinYaw - dz * cosYaw;
  const sinPitch = Math.sin(pitch);
  const cosPitch = Math.cos(pitch);
  return [
    cameraX,
    dy * cosPitch + forward * sinPitch,
    -dy * sinPitch + forward * cosPitch,
  ];
}

function rotateVector(vector, quaternion) {
  const [x, y, z] = vector;
  const [qx, qy, qz, qw] = quaternion;
  const tx = 2 * (qy * z - qz * y);
  const ty = 2 * (qz * x - qx * z);
  const tz = 2 * (qx * y - qy * x);
  return [
    x + qw * tx + (qy * tz - qz * ty),
    y + qw * ty + (qz * tx - qx * tz),
    z + qw * tz + (qx * ty - qy * tx),
  ];
}

function boxVertices(body) {
  const [x, y, z] = body.position;
  const [hx, hy, hz] = body.half;
  const local = [
    [-hx, -hy, -hz],
    [hx, -hy, -hz],
    [-hx, hy, -hz],
    [hx, hy, -hz],
    [-hx, -hy, hz],
    [hx, -hy, hz],
    [-hx, hy, hz],
    [hx, hy, hz],
  ];
  return local.map((point) => {
    const rotated = rotateVector(point, body.orientation);
    return [x + rotated[0], y + rotated[1], z + rotated[2]];
  });
}

function materialFor(body) {
  if (body.role === 3) return 4;
  if (body.role === 2) return 3;
  if (body.role !== 0) return 2;
  const [hx, hy, hz] = body.half;
  if (body.position[1] < 0 && hx >= 400 && hz >= 400) return 0;
  if (hy >= 60) return 1;
  return 2;
}

function textureScale(material, body, face) {
  if (material === 3 || material === 4) return [1, 1];
  const full = body.half.map((value) => Math.max(1, value * 2));
  const u = full[face.axes[0]];
  const v = full[face.axes[1]];
  if (material === 1) return [Math.max(1, u / 42), Math.max(1, v / 22)];
  if (material === 0) return [Math.max(1, u / 64), Math.max(1, v / 64)];
  return [Math.max(1, u / 52), Math.max(1, v / 52)];
}

function faceCenter(vertices, indices) {
  const result = [0, 0, 0];
  for (const index of indices) {
    result[0] += vertices[index][0];
    result[1] += vertices[index][1];
    result[2] += vertices[index][2];
  }
  return result.map((value) => value / indices.length);
}

function faceVisible(vertices, face, body, camera) {
  const center = faceCenter(vertices, face.indices);
  const normal = rotateVector(face.normal, body.orientation);
  const towardCamera = [camera[0] - center[0], camera[1] - center[1], camera[2] - center[2]];
  return (
    towardCamera[0] * normal[0] +
      towardCamera[1] * normal[1] +
      towardCamera[2] * normal[2] >
    0
  );
}

function appendVertex(data, position, normal, uv, material, camera) {
  const cameraPosition = cameraSpace(position, camera);
  data.push(
    cameraPosition[0],
    cameraPosition[1],
    cameraPosition[2],
    normal[0],
    normal[1],
    normal[2],
    uv[0],
    uv[1],
    material,
  );
}

function buildSceneVertices(bodies, camera) {
  const data = [];
  for (const body of bodies) {
    if (body.role === 1) continue;
    const material = materialFor(body);
    const vertices = boxVertices(body);

    for (const face of BOX_FACES) {
      if (!faceVisible(vertices, face, body, camera)) continue;
      const [scaleU, scaleV] = textureScale(material, body, face);
      const uv = FACE_UVS.map(([u, v]) => [u * scaleU, v * scaleV]);
      const corners = face.indices.map((index) => vertices[index]);
      const normal = rotateVector(face.normal, body.orientation);
      const triangles = [
        [0, 1, 2],
        [0, 2, 3],
      ];
      for (const triangle of triangles) {
        for (const corner of triangle) {
          appendVertex(data, corners[corner], normal, uv[corner], material, camera);
        }
      }
    }
  }
  return new Float32Array(data);
}

function ensureCrosshair() {
  if (viewportShell.querySelector(".crosshair")) return;
  const crosshair = document.createElement("div");
  crosshair.className = "crosshair";
  crosshair.setAttribute("aria-hidden", "true");
  viewportShell.append(crosshair);
}

function render() {
  const resized = resizeCanvas();
  if (!renderDirty && !resized) return false;

  const bodies = readBodies();
  const player = bodies.find((body) => body.role === 1);
  if (!player) return false;
  const camera = [player.position[0], player.position[1] + 13, player.position[2]];
  renderer.render(buildSceneVertices(bodies, camera));

  const grounded = engine.sandbox_grounded() === 1 ? "grounded" : "airborne";
  const mouse = document.pointerLockElement === canvas ? "mouse captured" : "mouse free";
  const yawDegrees = Math.round((yaw * 180) / Math.PI);
  const pitchDegrees = Math.round((pitch * 180) / Math.PI);
  const sleep = quiescent ? " · asleep" : "";
  const fixedGeometry =
    engine.sandbox_fixed_geometry_mode() === 1
      ? ` · fixed prepared ${engine.sandbox_fixed_geometry_prepared_count()} (${engine.sandbox_fixed_geometry_retained_bytes()} B)`
      : " · fixed runtime";
  const tailDiagnostics =
    typeof engine.sandbox_last_tail_slices === "function"
      ? ` · tail ${engine.sandbox_last_tail_slices()} slices / ${engine.sandbox_last_tail_candidate_pairs()} candidates`
      : "";
  debug.textContent = `${renderer.backend} · ${bodies.length} bodies · ${grounded}${sleep}${fixedGeometry} · yaw ${yawDegrees}° · pitch ${pitchDegrees}° · ${mouse} · ${engine.sandbox_last_collision_events()} collision contacts this tick${tailDiagnostics} · ${engine.sandbox_total_collisions()} total${paused ? " · paused" : ""}`;
  renderDirty = false;
  return true;
}

function updateKeyboardLook(elapsedSeconds) {
  const yawInput = Number(keys.has("ArrowRight")) - Number(keys.has("ArrowLeft"));
  const pitchInput = Number(keys.has("ArrowDown")) - Number(keys.has("ArrowUp"));
  if (yawInput === 0 && pitchInput === 0) return false;

  yaw += yawInput * KEYBOARD_LOOK_SPEED * elapsedSeconds;
  pitch = Math.max(-1.25, Math.min(1.25, pitch + pitchInput * KEYBOARD_LOOK_SPEED * elapsedSeconds));
  return true;
}

function frame(timestamp) {
  const callbackStarted = performance.now();
  if (previousTimestamp === null) previousTimestamp = timestamp;
  const frameInterval = timestamp - previousTimestamp;
  const elapsed = Math.min(frameInterval, 250);
  previousTimestamp = timestamp;
  if (updateKeyboardLook(elapsed / 1000)) renderDirty = true;

  let droppedAccumulatorMs = 0;
  if (!paused && (!quiescent || simulationInputActive())) {
    accumulator += elapsed;
    let steps = 0;
    let workBudgetExceeded = false;
    const physicsWorkStarted = performance.now();
    while (accumulator >= FIXED_STEP_MS && steps < MAX_CATCH_UP_STEPS && !paused) {
      simulationStep();
      accumulator -= FIXED_STEP_MS;
      steps += 1;
      if (performance.now() - physicsWorkStarted >= MAX_CATCH_UP_WALL_MS) {
        workBudgetExceeded = true;
        break;
      }
    }
    if ((steps === MAX_CATCH_UP_STEPS || workBudgetExceeded) && accumulator >= FIXED_STEP_MS) {
      droppedAccumulatorMs = accumulator;
      accumulator = 0;
    }
  } else if (quiescent) {
    accumulator = 0;
  }

  const renderStarted = performance.now();
  const renderPerformed = render();
  const renderMs = performance.now() - renderStarted;
  performanceRecorder.recordFrame({
    frame_interval_ms: Math.max(0, frameInterval),
    callback_ms: performance.now() - callbackStarted,
    render_performed: renderPerformed,
    render_ms: renderPerformed ? renderMs : null,
    physics_steps_ms: pendingPhysicsStepMs,
    physics_step_stats: pendingPhysicsStepStats,
    dropped_accumulator_ms: droppedAccumulatorMs,
    body_count: typeof engine?.sandbox_body_count === "function" ? engine.sandbox_body_count() : null,
    collision_contacts:
      typeof engine?.sandbox_last_collision_events === "function"
        ? engine.sandbox_last_collision_events()
        : null,
    paused,
  });
  pendingPhysicsStepMs = [];
  pendingPhysicsStepStats = [];
  requestAnimationFrame(frame);
}

function downloadPerformanceSession(session) {
  const blob = new Blob([serializePerformanceSession(session)], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `physics-browser-session-${session.started_at.replaceAll(":", "-")}.json`;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(link.href), 0);
}

function applyLookDelta(deltaX, deltaY) {
  yaw += deltaX * LOOK_SENSITIVITY;
  pitch = Math.max(-1.25, Math.min(1.25, pitch + deltaY * LOOK_SENSITIVITY));
  renderDirty = true;
}

canvas.addEventListener("pointerdown", (event) => {
  canvas.focus();
  dragLook = true;
  dragDistance = 0;
  lastPointer = [event.clientX, event.clientY];
  canvas.setPointerCapture?.(event.pointerId);

  if (document.pointerLockElement !== canvas) {
    canvas.requestPointerLock?.();
  } else if (event.button === 0) {
    shoot();
  }
});

canvas.addEventListener("pointermove", (event) => {
  if (!dragLook || document.pointerLockElement === canvas || !lastPointer) return;
  const deltaX = event.clientX - lastPointer[0];
  const deltaY = event.clientY - lastPointer[1];
  dragDistance += Math.abs(deltaX) + Math.abs(deltaY);
  lastPointer = [event.clientX, event.clientY];
  applyLookDelta(deltaX, deltaY);
});

canvas.addEventListener("pointerup", (event) => {
  if (document.pointerLockElement !== canvas && event.button === 0 && dragDistance < 6) shoot();
  dragLook = false;
  lastPointer = null;
  canvas.releasePointerCapture?.(event.pointerId);
});

canvas.addEventListener("pointercancel", () => {
  dragLook = false;
  lastPointer = null;
});

document.addEventListener("mousemove", (event) => {
  if (document.pointerLockElement !== canvas) return;
  applyLookDelta(event.movementX, event.movementY);
});

document.addEventListener("pointerlockchange", () => {
  status.textContent =
    document.pointerLockElement === canvas
      ? `Mouse captured. Press Esc to release it. Rendering with ${renderer.backend}.`
      : `Mouse free. Click the world to capture it, or drag / use arrow keys to look. Rendering with ${renderer.backend}.`;
  renderDirty = true;
});

document.addEventListener("keydown", (event) => {
  keys.add(event.code);
  if (["Space", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(event.code)) {
    event.preventDefault();
  }
  if (event.code === "Space" && !event.repeat) jumpQueued = true;
  if (event.code === "KeyF" && !event.repeat) shoot();
  if (event.code === "KeyR" && !event.repeat) reset();
  if (event.code === "KeyP" && !event.repeat) {
    paused = !paused;
    pauseButton.textContent = paused ? "Resume" : "Pause";
    renderDirty = true;
  }
  if (event.code === "KeyN" && paused && !event.repeat) simulationStep();
});

document.addEventListener("keyup", (event) => keys.delete(event.code));
window.addEventListener("blur", () => keys.clear());

resetButton.addEventListener("click", reset);
pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
  renderDirty = true;
});
stepButton.addEventListener("click", () => {
  if (!paused) {
    paused = true;
    pauseButton.textContent = "Resume";
  }
  simulationStep();
});
startPerformanceLogButton.addEventListener("click", () => {
  pendingPhysicsStepMs = [];
  pendingPhysicsStepStats = [];
  performanceRecorder.start();
  startPerformanceLogButton.disabled = true;
  downloadPerformanceLogButton.disabled = false;
  performanceLogStatus.textContent = "Recording locally. Exercise the behavior you want to analyze.";
});
downloadPerformanceLogButton.addEventListener("click", () => {
  const session = performanceRecorder.finish();
  downloadPerformanceSession(session);
  startPerformanceLogButton.disabled = false;
  downloadPerformanceLogButton.disabled = true;
  performanceLogStatus.textContent = `${session.summary.recorded_frames} frames captured and downloaded.`;
});

try {
  renderer = await createRenderer();
  [engine, buildProvenance] = await Promise.all([loadEngine(), loadBuildProvenance()]);
  if (typeof engine.sandbox_step_velocity !== "function") {
    throw new Error("WASM sandbox does not expose canonical controller velocity input");
  }
  if (
    typeof engine.sandbox_reset_with_baking_options !== "function" ||
    typeof engine.sandbox_fixed_geometry_prepared_count !== "function" ||
    typeof engine.sandbox_fixed_geometry_retained_bytes !== "function"
  ) {
    throw new Error("WASM sandbox does not expose fixed geometry comparison controls");
  }
  ensureCrosshair();
  characterModeControl.disabled = false;
  uprightCratesControl.disabled = false;
  fixedGeometryControl.disabled = false;
  startPerformanceLogButton.disabled = false;
  reset();
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `Unable to load the Rust physics sandbox: ${error.message}`;
  console.error(error);
}
