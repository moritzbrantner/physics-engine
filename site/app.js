const canvas = document.querySelector("#scene");
const context = canvas.getContext("2d");
const status = document.querySelector("#status");
const debug = document.querySelector("#debug");
const resetButton = document.querySelector("#reset");
const pauseButton = document.querySelector("#pause");
const stepButton = document.querySelector("#single-step");

const FIXED_STEP_MS = 1000 / 60;
const MOVE_SPEED = 7;
const PROJECTILE_SPEED = 96;
const keys = new Set();

let engine = null;
let yaw = 0;
let pitch = 0;
let paused = false;
let jumpQueued = false;
let previousTimestamp = null;
let accumulator = 0;

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

function reset() {
  engine.sandbox_reset();
  yaw = 0;
  pitch = 0;
  paused = false;
  jumpQueued = false;
  accumulator = 0;
  pauseButton.textContent = "Pause";
  status.textContent = "Click the world to capture the mouse. WASD moves, Space jumps, and click fires a CCD projectile.";
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
}

function themeColor(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
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

function project(point, camera) {
  const [x, y, depth] = cameraSpace(point, camera);
  if (depth <= 1) return null;
  const focal = canvas.height * 0.9;
  return [canvas.width / 2 + (x * focal) / depth, canvas.height / 2 - (y * focal) / depth, depth];
}

function drawLine(from, to, camera, strokeStyle, alpha = 1) {
  const a = project(from, camera);
  const b = project(to, camera);
  if (!a || !b) return;
  context.globalAlpha = alpha;
  context.strokeStyle = strokeStyle;
  context.beginPath();
  context.moveTo(a[0], a[1]);
  context.lineTo(b[0], b[1]);
  context.stroke();
  context.globalAlpha = 1;
}

function drawGrid(camera) {
  const grid = themeColor("--grid");
  for (let coordinate = -480; coordinate <= 480; coordinate += 80) {
    drawLine([coordinate, 0, -480], [coordinate, 0, 480], camera, grid, 0.55);
    drawLine([-480, 0, coordinate], [480, 0, coordinate], camera, grid, 0.55);
  }
}

const BOX_EDGES = [
  [0, 1], [1, 3], [3, 2], [2, 0],
  [4, 5], [5, 7], [7, 6], [6, 4],
  [0, 4], [1, 5], [2, 6], [3, 7],
];

function boxVertices(body) {
  const [x, y, z] = body.position;
  const [hx, hy, hz] = body.half;
  return [
    [x - hx, y - hy, z - hz],
    [x + hx, y - hy, z - hz],
    [x - hx, y + hy, z - hz],
    [x + hx, y + hy, z - hz],
    [x - hx, y - hy, z + hz],
    [x + hx, y - hy, z + hz],
    [x - hx, y + hy, z + hz],
    [x + hx, y + hy, z + hz],
  ];
}

function roleColor(role) {
  if (role === 2) return themeColor("--dynamic");
  if (role === 3) return themeColor("--projectile");
  return themeColor("--fixed");
}

function drawBody(body, camera) {
  if (body.role === 1) return;
  const vertices = boxVertices(body);
  context.lineWidth = body.role === 3 ? 2.5 : 1.5;
  const stroke = roleColor(body.role);
  for (const [from, to] of BOX_EDGES) {
    drawLine(vertices[from], vertices[to], camera, stroke, body.role === 0 ? 0.7 : 1);
  }

  if (body.role === 3) {
    const center = project(body.position, camera);
    if (center) {
      context.fillStyle = stroke;
      context.beginPath();
      context.arc(center[0], center[1], Math.max(2, 10 / Math.sqrt(center[2])), 0, Math.PI * 2);
      context.fill();
    }
  }
}

function drawCrosshair() {
  const x = canvas.width / 2;
  const y = canvas.height / 2;
  const size = Math.max(7, canvas.height * 0.012);
  context.strokeStyle = themeColor("--crosshair");
  context.lineWidth = 1.5;
  context.beginPath();
  context.moveTo(x - size, y);
  context.lineTo(x + size, y);
  context.moveTo(x, y - size);
  context.lineTo(x, y + size);
  context.stroke();
}

function render() {
  resizeCanvas();
  context.fillStyle = themeColor("--canvas");
  context.fillRect(0, 0, canvas.width, canvas.height);

  const bodies = readBodies();
  const player = bodies.find((body) => body.role === 1);
  if (!player) return;
  const camera = [player.position[0], player.position[1] + 13, player.position[2]];

  drawGrid(camera);
  bodies
    .filter((body) => body.role !== 1)
    .sort((left, right) => cameraSpace(right.position, camera)[2] - cameraSpace(left.position, camera)[2])
    .forEach((body) => drawBody(body, camera));
  drawCrosshair();

  const grounded = engine.sandbox_grounded() === 1 ? "grounded" : "airborne";
  debug.textContent = `${bodies.length} bodies · ${grounded} · ${engine.sandbox_last_pair_checks()} pair checks · ${engine.sandbox_last_collision_events()} collision events this tick · ${engine.sandbox_total_collisions()} total${paused ? " · paused" : ""}`;
}

function frame(timestamp) {
  if (previousTimestamp === null) previousTimestamp = timestamp;
  const elapsed = Math.min(timestamp - previousTimestamp, 250);
  previousTimestamp = timestamp;

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

canvas.addEventListener("pointerdown", (event) => {
  if (document.pointerLockElement !== canvas) {
    canvas.requestPointerLock();
    return;
  }
  if (event.button === 0) shoot();
});

document.addEventListener("mousemove", (event) => {
  if (document.pointerLockElement !== canvas) return;
  yaw += event.movementX * 0.0022;
  pitch = Math.max(-1.35, Math.min(1.35, pitch + event.movementY * 0.0022));
});

document.addEventListener("keydown", (event) => {
  keys.add(event.code);
  if (event.code === "Space") {
    event.preventDefault();
    if (!event.repeat) jumpQueued = true;
  }
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
  engine = await loadEngine();
  reset();
  requestAnimationFrame(frame);
} catch (error) {
  status.textContent = `Unable to load the Rust physics sandbox: ${error.message}`;
  console.error(error);
}
