import { createWebGlRenderer } from "./webgl-renderer.js";
import { createWebGpuRenderer } from "./webgpu-renderer.js";

const canvas = document.querySelector("#scene");
const status = document.querySelector("#status");
const debug = document.querySelector("#debug");
const resetButton = document.querySelector("#reset");
const pauseButton = document.querySelector("#pause");
const stepButton = document.querySelector("#single-step");
const viewportShell = document.querySelector(".viewport-shell");

const FIXED_STEP_MS = 1000 / 60;
const MOVE_SPEED = 7;
const PROJECTILE_SPEED = 96;
const LOOK_SENSITIVITY = 0.0022;
const KEYBOARD_LOOK_SPEED = 1.8;
const FOV_RADIANS = (70 * Math.PI) / 180;
const NEAR_PLANE = 0.8;
const FAR_PLANE = 1400;
const ORIENTATION_SCALE = 1 << 30;
const keys = new Set();

let engine = null;
let renderer = null;
let yaw = 0;
let pitch = 0;
let paused = false;
let jumpQueued = false;
let previousTimestamp = null;
let accumulator = 0;
let dragLook = false;
let dragDistance = 0;
let lastPointer = null;

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
  engine.sandbox_reset();
  yaw = 0;
  pitch = 0;
  paused = false;
  jumpQueued = false;
  accumulator = 0;
  pauseButton.textContent = "Pause";
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

function simulationStep() {
  const [moveX, moveZ] = movementVelocity();
  const error = engine.sandbox_step(moveX, moveZ, jumpQueued ? 1 : 0);
  jumpQueued = false;
  if (error !== 0) {
    paused = true;
    pauseButton.textContent = "Resume";
    status.textContent = `Physics stopped fail-closed with sandbox error ${error}. Reset to start from the deterministic fixture again.`;
  }
}

function shoot() {
  const cosPitch = Math.cos(pitch);
  const velocityX = Math.round(Math.sin(yaw) * cosPitch * PROJECTILE_SPEED);
  const velocityY = Math.round(-Math.sin(pitch) * PROJECTILE_SPEED);
  const velocityZ = Math.round(-Math.cos(yaw) * cosPitch * PROJECTILE_SPEED);
  if (engine.sandbox_shoot(velocityX, velocityY, velocityZ) < 0) {
    status.textContent = "The engine rejected projectile creation.";
  }
}

function normalizeQuaternion(raw) {
  const quaternion = raw.map((value) => value / ORIENTATION_SCALE);
  const length = Math.hypot(...quaternion);
  if (!Number.isFinite(length) || length === 0) return [0, 0, 0, 1];
  return quaternion.map((value) => value / length);
}

function readBodies() {
  const bodies = [];
  const count = engine.sandbox_body_count();
  for (let index = 0; index < count; index += 1) {
    bodies.push({
      role: engine.sandbox_body_role(index),
      position: [
        engine.sandbox_body_x(index),
        engine.sandbox_body_y(index),
        engine.sandbox_body_z(index),
      ],
      half: [
        engine.sandbox_body_half_x(index),
        engine.sandbox_body_half_y(index),
        engine.sandbox_body_half_z(index),
      ],
      orientation: normalizeQuaternion([
        engine.sandbox_body_orientation_x(index),
        engine.sandbox_body_orientation_y(index),
        engine.sandbox_body_orientation_z(index),
        engine.sandbox_body_orientation_w(index),
      ]),
    });
  }
  return bodies;
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  renderer.resize(canvas.width, canvas.height);
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
  resizeCanvas();
  const bodies = readBodies();
  const player = bodies.find((body) => body.role === 1);
  if (!player) return;
  const camera = [player.position[0], player.position[1] + 13, player.position[2]];
  renderer.render(buildSceneVertices(bodies, camera));

  const grounded = engine.sandbox_grounded() === 1 ? "grounded" : "airborne";
  const mouse = document.pointerLockElement === canvas ? "mouse captured" : "mouse free";
  const yawDegrees = Math.round((yaw * 180) / Math.PI);
  const pitchDegrees = Math.round((pitch * 180) / Math.PI);
  debug.textContent = `${renderer.backend} · ${bodies.length} bodies · ${grounded} · yaw ${yawDegrees}° · pitch ${pitchDegrees}° · ${mouse} · ${engine.sandbox_last_collision_events()} collision contacts this tick · ${engine.sandbox_total_collisions()} total${paused ? " · paused" : ""}`;
}

function updateKeyboardLook(elapsedSeconds) {
  const yawInput = Number(keys.has("ArrowRight")) - Number(keys.has("ArrowLeft"));
  const pitchInput = Number(keys.has("ArrowDown")) - Number(keys.has("ArrowUp"));
  yaw += yawInput * KEYBOARD_LOOK_SPEED * elapsedSeconds;
  pitch = Math.max(-1.25, Math.min(1.25, pitch + pitchInput * KEYBOARD_LOOK_SPEED * elapsedSeconds));
}

function frame(timestamp) {
  if (previousTimestamp === null) previousTimestamp = timestamp;
  const elapsed = Math.min(timestamp - previousTimestamp, 250);
  previousTimestamp = timestamp;
  updateKeyboardLook(elapsed / 1000);

  if (!paused) {
    accumulator += elapsed;
    let steps = 0;
    while (accumulator >= FIXED_STEP_MS && steps < 8 && !paused) {
      simulationStep();
      accumulator -= FIXED_STEP_MS;
      steps += 1;
    }
    if (steps === 8 && accumulator >= FIXED_STEP_MS) accumulator = 0;
  }

  render();
  requestAnimationFrame(frame);
}

function applyLookDelta(deltaX, deltaY) {
  yaw += deltaX * LOOK_SENSITIVITY;
  pitch = Math.max(-1.25, Math.min(1.25, pitch + deltaY * LOOK_SENSITIVITY));
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
  status.textContent = document.pointerLockElement === canvas
    ? `Mouse captured. Press Esc to release it. Rendering with ${renderer.backend}.`
    : `Mouse free. Click the world to capture it, or drag / use arrow keys to look. Rendering with ${renderer.backend}.`;
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
  }
  if (event.code === "KeyN" && paused && !event.repeat) simulationStep();
});

document.addEventListener("keyup", (event) => keys.delete(event.code));
window.addEventListener("blur", () => keys.clear());

resetButton.addEventListener("click", reset);
pauseButton.addEventListener("click", () => {
  paused = !paused;
  pauseButton.textContent = paused ? "Resume" : "Pause";
});
stepButton.addEventListener("click", () => {
  if (!paused) {
    paused = true;
    pauseButton.textContent = "Resume";
  }
  simulationStep();
});

try {
  renderer = await createRenderer();
  engine = await loadEngine();
  ensureCrosshair();
  reset();
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `Unable to load the Rust physics sandbox: ${error.message}`;
  console.error(error);
}
