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
const LOOK_SENSITIVITY = 0.0022;
const KEYBOARD_LOOK_SPEED = 1.8;
const NEAR_PLANE = 2;
const keys = new Set();
const textureCache = new Map();

let engine = null;
let yaw = 0;
let pitch = 0;
let paused = false;
let jumpQueued = false;
let previousTimestamp = null;
let accumulator = 0;
let dragLook = false;
let dragDistance = 0;
let lastPointer = null;

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
  status.textContent = "Click the world to capture the mouse. WASD moves, Space jumps, mouse or arrow keys look, and click or F fires a CCD projectile.";
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

function projectCamera(point) {
  const [x, y, depth] = point;
  const focal = canvas.height * 0.9;
  return [canvas.width / 2 + (x * focal) / depth, canvas.height / 2 - (y * focal) / depth];
}

function clipSegmentToNearPlane(a, b) {
  const aInside = a[2] >= NEAR_PLANE;
  const bInside = b[2] >= NEAR_PLANE;
  if (!aInside && !bInside) return null;
  if (aInside && bInside) return [a, b];

  const from = aInside ? a : b;
  const to = aInside ? b : a;
  const t = (NEAR_PLANE - from[2]) / (to[2] - from[2]);
  const clipped = [
    from[0] + (to[0] - from[0]) * t,
    from[1] + (to[1] - from[1]) * t,
    NEAR_PLANE,
  ];
  return aInside ? [from, clipped] : [clipped, from];
}

function clipPolygonToNearPlane(points) {
  if (points.length === 0) return [];
  const clipped = [];
  for (let index = 0; index < points.length; index += 1) {
    const current = points[index];
    const previous = points[(index + points.length - 1) % points.length];
    const currentInside = current[2] >= NEAR_PLANE;
    const previousInside = previous[2] >= NEAR_PLANE;

    if (currentInside !== previousInside) {
      const t = (NEAR_PLANE - previous[2]) / (current[2] - previous[2]);
      clipped.push([
        previous[0] + (current[0] - previous[0]) * t,
        previous[1] + (current[1] - previous[1]) * t,
        NEAR_PLANE,
      ]);
    }
    if (currentInside) clipped.push(current);
  }
  return clipped;
}

function drawLine(from, to, camera, strokeStyle, alpha = 1) {
  const clipped = clipSegmentToNearPlane(cameraSpace(from, camera), cameraSpace(to, camera));
  if (!clipped) return;
  const a = projectCamera(clipped[0]);
  const b = projectCamera(clipped[1]);
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
  context.lineWidth = 1;
  for (let coordinate = -480; coordinate <= 480; coordinate += 80) {
    drawLine([coordinate, 0.2, -480], [coordinate, 0.2, 480], camera, grid, 0.22);
    drawLine([-480, 0.2, coordinate], [480, 0.2, coordinate], camera, grid, 0.22);
  }
}

const BOX_FACES = [
  [0, 2, 3, 1],
  [4, 5, 7, 6],
  [0, 4, 6, 2],
  [1, 3, 7, 5],
  [0, 1, 5, 4],
  [2, 6, 7, 3],
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

function polygonArea(points) {
  let area = 0;
  for (let index = 0; index < points.length; index += 1) {
    const current = points[index];
    const next = points[(index + 1) % points.length];
    area += current[0] * next[1] - next[0] * current[1];
  }
  return area / 2;
}

function projectedFace(vertices, face, camera) {
  const cameraPoints = face.map((index) => cameraSpace(vertices[index], camera));
  const clipped = clipPolygonToNearPlane(cameraPoints);
  if (clipped.length < 3) return null;
  const screen = clipped.map(projectCamera);
  const area = polygonArea(screen);
  if (area >= -0.1) return null;
  return {
    screen,
    depth: clipped.reduce((sum, point) => sum + point[2], 0) / clipped.length,
  };
}

function texturePattern(kind) {
  if (textureCache.has(kind)) return textureCache.get(kind);
  const tile = document.createElement("canvas");
  tile.width = 32;
  tile.height = 32;
  const tileContext = tile.getContext("2d");

  if (kind === "wall") {
    tileContext.fillStyle = "#2b3139";
    tileContext.fillRect(0, 0, 32, 32);
    tileContext.strokeStyle = "#59636f";
    tileContext.lineWidth = 2;
    for (const y of [0, 16, 32]) {
      tileContext.beginPath();
      tileContext.moveTo(0, y);
      tileContext.lineTo(32, y);
      tileContext.stroke();
    }
    tileContext.beginPath();
    tileContext.moveTo(8, 0);
    tileContext.lineTo(8, 16);
    tileContext.moveTo(24, 0);
    tileContext.lineTo(24, 16);
    tileContext.moveTo(0, 16);
    tileContext.lineTo(0, 32);
    tileContext.moveTo(16, 16);
    tileContext.lineTo(16, 32);
    tileContext.moveTo(32, 16);
    tileContext.lineTo(32, 32);
    tileContext.stroke();
  } else if (kind === "floor") {
    tileContext.fillStyle = "#171d24";
    tileContext.fillRect(0, 0, 32, 32);
    tileContext.fillStyle = "#1f2730";
    tileContext.fillRect(0, 0, 16, 16);
    tileContext.fillRect(16, 16, 16, 16);
    tileContext.strokeStyle = "#394653";
    tileContext.lineWidth = 1;
    tileContext.strokeRect(0.5, 0.5, 31, 31);
  } else if (kind === "crate") {
    tileContext.fillStyle = "#5a3b1f";
    tileContext.fillRect(0, 0, 32, 32);
    tileContext.strokeStyle = "#d29922";
    tileContext.lineWidth = 2;
    tileContext.strokeRect(1, 1, 30, 30);
    tileContext.beginPath();
    tileContext.moveTo(2, 2);
    tileContext.lineTo(30, 30);
    tileContext.moveTo(30, 2);
    tileContext.lineTo(2, 30);
    tileContext.stroke();
  } else {
    tileContext.fillStyle = "#343b44";
    tileContext.fillRect(0, 0, 32, 32);
    tileContext.fillStyle = "#505965";
    for (const [x, y] of [[5, 7], [19, 4], [26, 19], [11, 25]]) {
      tileContext.fillRect(x, y, 2, 2);
    }
  }

  const pattern = context.createPattern(tile, "repeat");
  textureCache.set(kind, pattern);
  return pattern;
}

function textureKind(body) {
  if (body.role === 2) return "crate";
  if (body.role !== 0) return "concrete";
  const [hx, hy, hz] = body.half;
  if (body.position[1] < 0 && hx >= 400 && hz >= 400) return "floor";
  if (hy >= 60) return "wall";
  return "concrete";
}

function roleColor(role) {
  if (role === 2) return themeColor("--dynamic");
  if (role === 3) return themeColor("--projectile");
  return themeColor("--fixed");
}

function drawFace(face, fillStyle, strokeStyle, alpha) {
  const [first, ...rest] = face.screen;
  context.beginPath();
  context.moveTo(first[0], first[1]);
  for (const point of rest) context.lineTo(point[0], point[1]);
  context.closePath();
  context.globalAlpha = alpha;
  context.fillStyle = fillStyle;
  context.fill();
  context.globalAlpha = Math.min(1, alpha + 0.15);
  context.strokeStyle = strokeStyle;
  context.lineWidth = 1.3;
  context.stroke();
  context.globalAlpha = 1;
}

function drawBody(body, camera) {
  if (body.role === 1) return;
  const vertices = boxVertices(body);
  const stroke = roleColor(body.role);
  const faces = BOX_FACES
    .map((face) => projectedFace(vertices, face, camera))
    .filter(Boolean)
    .sort((left, right) => right.depth - left.depth);

  const fill = body.role === 3 ? stroke : texturePattern(textureKind(body));
  const alpha = body.role === 0 ? 0.92 : 0.96;
  for (const face of faces) drawFace(face, fill, stroke, alpha);

  if (body.role === 3) {
    const centerCamera = cameraSpace(body.position, camera);
    if (centerCamera[2] >= NEAR_PLANE) {
      const center = projectCamera(centerCamera);
      context.fillStyle = stroke;
      context.beginPath();
      context.arc(center[0], center[1], Math.max(2.5, 12 / Math.sqrt(centerCamera[2])), 0, Math.PI * 2);
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

function drawBackground() {
  const gradient = context.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, "#0b1a27");
  gradient.addColorStop(0.55, "#111820");
  gradient.addColorStop(1, themeColor("--canvas"));
  context.fillStyle = gradient;
  context.fillRect(0, 0, canvas.width, canvas.height);
}

function render() {
  resizeCanvas();
  drawBackground();

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
  const mouse = document.pointerLockElement === canvas ? "mouse captured" : "mouse free";
  const yawDegrees = Math.round((yaw * 180) / Math.PI);
  const pitchDegrees = Math.round((pitch * 180) / Math.PI);
  debug.textContent = `${bodies.length} bodies · ${grounded} · yaw ${yawDegrees}° · pitch ${pitchDegrees}° · ${mouse} · ${engine.sandbox_last_pair_checks()} pair checks · ${engine.sandbox_last_collision_events()} collision events this tick · ${engine.sandbox_total_collisions()} total${paused ? " · paused" : ""}`;
}

function applyLook(deltaX, deltaY) {
  yaw += deltaX * LOOK_SENSITIVITY;
  if (yaw > Math.PI) yaw -= Math.PI * 2;
  if (yaw < -Math.PI) yaw += Math.PI * 2;
  pitch = Math.max(-1.35, Math.min(1.35, pitch + deltaY * LOOK_SENSITIVITY));
}

function applyKeyboardLook(elapsedMs) {
  const seconds = elapsedMs / 1000;
  const horizontal = Number(keys.has("ArrowRight")) - Number(keys.has("ArrowLeft"));
  const vertical = Number(keys.has("ArrowDown")) - Number(keys.has("ArrowUp"));
  if (horizontal !== 0) yaw += horizontal * KEYBOARD_LOOK_SPEED * seconds;
  if (vertical !== 0) pitch = Math.max(-1.35, Math.min(1.35, pitch + vertical * KEYBOARD_LOOK_SPEED * seconds));
}

function frame(timestamp) {
  if (previousTimestamp === null) previousTimestamp = timestamp;
  const elapsed = Math.min(timestamp - previousTimestamp, 250);
  previousTimestamp = timestamp;
  applyKeyboardLook(elapsed);

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
  canvas.focus({ preventScroll: true });
  if (document.pointerLockElement !== canvas) {
    if (event.button !== 0) return;
    dragLook = true;
    dragDistance = 0;
    lastPointer = [event.clientX, event.clientY];
    try {
      const request = canvas.requestPointerLock?.();
      if (request && typeof request.catch === "function") {
        request.catch(() => {
          status.textContent = "Pointer lock was unavailable; hold and drag on the world to look, or use the arrow keys.";
        });
      }
    } catch {
      status.textContent = "Pointer lock was unavailable; hold and drag on the world to look, or use the arrow keys.";
    }
    return;
  }
  if (event.button === 0) shoot();
});

document.addEventListener("mousemove", (event) => {
  if (document.pointerLockElement === canvas) {
    applyLook(event.movementX, event.movementY);
    return;
  }
  if (!dragLook || (event.buttons & 1) === 0 || !lastPointer) return;
  const deltaX = event.clientX - lastPointer[0];
  const deltaY = event.clientY - lastPointer[1];
  dragDistance += Math.hypot(deltaX, deltaY);
  lastPointer = [event.clientX, event.clientY];
  applyLook(deltaX, deltaY);
});

document.addEventListener("pointerup", (event) => {
  if (event.button !== 0 || !dragLook) return;
  const shouldShoot = document.pointerLockElement !== canvas && dragDistance < 4;
  dragLook = false;
  lastPointer = null;
  if (shouldShoot) shoot();
});

document.addEventListener("pointerlockchange", () => {
  dragLook = false;
  lastPointer = null;
  if (document.pointerLockElement === canvas) {
    status.textContent = "Mouse captured. WASD moves, Space jumps, mouse look rotates the camera, and click fires.";
  } else {
    status.textContent = "Mouse released. Click the world to capture it again; drag or arrow keys can still rotate the camera.";
  }
});

document.addEventListener("pointerlockerror", () => {
  status.textContent = "Pointer lock was unavailable; hold and drag on the world to look, or use the arrow keys.";
});

document.addEventListener("keydown", (event) => {
  keys.add(event.code);
  if (event.code === "Space") {
    event.preventDefault();
    if (!event.repeat) jumpQueued = true;
  }
  if (event.code.startsWith("Arrow")) event.preventDefault();
  if (event.code === "KeyF" && !event.repeat) shoot();
  if (event.code === "KeyR" && !event.repeat) reset();
  if (event.code === "KeyP" && !event.repeat) {
    paused = !paused;
    pauseButton.textContent = paused ? "Resume" : "Pause";
  }
  if (event.code === "KeyN" && paused && !event.repeat) simulationStep();
});

document.addEventListener("keyup", (event) => keys.delete(event.code));
window.addEventListener("blur", () => {
  keys.clear();
  dragLook = false;
  lastPointer = null;
});

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
