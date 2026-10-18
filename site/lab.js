import { createWebGlRenderer } from "./webgl-renderer.js";

const $ = (id) => document.getElementById(id);
const canvas = $("scene"), status = $("status");
const STRIDE = 11;
let e, renderer, paused = true, ticks = 0, accumulator = 0, last = null, error = null, recent = [];

const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a); return a.map((x) => x / l); };
const camera = [235, 195, 425], forward = norm(camera.map((x, i) => [0, 65, 105][i] - x));
const right = norm(cross(forward, [0, 1, 0])), up = cross(right, forward);
const rotate = (p, q) => { const t = cross(q.slice(0, 3), p).map((x) => 2 * x), u = cross(q.slice(0, 3), t); return p.map((x, i) => x + q[3] * t[i] + u[i]); };
const faces = [[[0, 4, 6, 2], [-1, 0, 0]], [[1, 3, 7, 5], [1, 0, 0]], [[0, 1, 5, 4], [0, -1, 0]], [[2, 6, 7, 3], [0, 1, 0]], [[0, 2, 3, 1], [0, 0, -1]], [[4, 5, 7, 6], [0, 0, 1]]];

function rows() {
  const ptr = e.lab_refresh_snapshot(), n = e.lab_snapshot_len();
  const data = new Float64Array(e.memory.buffer, ptr, n);
  return Array.from({ length: n / STRIDE }, (_, i) => Array.from(data.subarray(i * STRIDE, i * STRIDE + STRIDE)));
}

function render() {
  const data = [];
  const vertex = (p, n, uv, mat) => { const d = p.map((v, i) => v - camera[i]); data.push(dot(d, right), dot(d, up), dot(d, forward), ...n, ...uv, mat); };
  for (const r of rows()) {
    const role = r[0], pos = r.slice(1, 4), half = r.slice(4, 7), q = r.slice(7);
    if (role === 1) continue;
    const mat = role >= 3 ? 4 : role === 2 ? 3 : pos[1] < 0 ? 0 : 1;
    if (role === 3) {
      const p = (i, j) => { const a = -Math.PI / 2 + i * Math.PI / 8, b = j * Math.PI / 6; return [Math.cos(a) * Math.cos(b), Math.sin(a), Math.cos(a) * Math.sin(b)]; };
      for (let i = 0; i < 8; i++) for (let j = 0; j < 12; j++) for (const [a, b] of [[i, j], [i + 1, j], [i + 1, j + 1], [i, j], [i + 1, j + 1], [i, j + 1]]) { const n = p(a, b); vertex(n.map((v, k) => pos[k] + v * half[0]), n, [0, 0], mat); }
      continue;
    }
    const v = Array.from({ length: 8 }, (_, i) => rotate(half.map((h, k) => h * ((i & (1 << k)) ? 1 : -1)), q).map((x, k) => x + pos[k]));
    for (const [ids, n] of faces) {
      const normal = rotate(n, q);
      const center = ids.map((i) => v[i]).reduce((a, b) => a.map((x, k) => x + b[k] / 4), [0, 0, 0]);
      if (dot(normal, camera.map((x, k) => x - center[k])) <= 0) continue;
      for (const c of [0, 1, 2, 0, 2, 3]) vertex(v[ids[c]], normal, [[0, 0], [1, 0], [1, 1], [0, 1]][c], mat);
    }
  }
  const w = Math.round(canvas.clientWidth * devicePixelRatio), h = Math.round(canvas.clientHeight * devicePixelRatio);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; renderer.resize(w, h); }
  renderer.render(new Float32Array(data));
  const mean = recent.length ? recent.reduce((a, b) => a + b, 0) / recent.length : 0;
  status.textContent = error ?? `${(ticks / 60).toFixed(2)} s simulated · 32 crates · ${e.lab_awake_crates()} awake · ${mean.toFixed(2)} ms/step${paused ? " · paused" : ""}`;
  window.physicsLabState = { ticks, error };
}

function tick() {
  const start = performance.now(), code = e.lab_step();
  recent.push(performance.now() - start); if (recent.length > 60) recent.shift();
  if (code) { error = `Physics step failed (${code})`; paused = true; return; }
  ticks++;
}

function reset() {
  error = null; ticks = 0; recent = []; paused = true; accumulator = 0;
  if (e.lab_reset($("crates").value === "upright" ? 1 : 0) !== 0) { error = "Reset failed"; render(); return; }
  for (let i = 0; i < 240 && !error; i++) tick();
  recent = []; $("pause").textContent = "Resume"; render();
}

function shoot(miss) {
  if (error) return;
  e.lab_set_projectile(Number($("projectile").value));
  if (e.lab_shoot(...(miss ? [40, 0, -87] : [0, 0, -96])) < 0) { status.textContent = "Projectile rejected"; return; }
  paused = false; accumulator = 0; $("pause").textContent = "Pause";
}

function frame(t) {
  const elapsed = last === null ? 0 : Math.min(100, t - last); last = t;
  if (!paused) { accumulator += elapsed; let steps = 0; while (accumulator >= 1000 / 60 && steps < 4 && !paused) { tick(); accumulator -= 1000 / 60; steps++; } }
  render();
  requestAnimationFrame(frame);
}

try {
  const response = await fetch("physics_engine_demo.wasm");
  if (!response.ok) throw new Error(`WASM ${response.status}`);
  e = (await WebAssembly.instantiate(await response.arrayBuffer(), {})).instance.exports;
  renderer = createWebGlRenderer(canvas, { fovRadians: Math.PI / 3, nearPlane: 1, farPlane: 2000 });
  if (!renderer) throw new Error("WebGL2 is unavailable");
  $("crates").addEventListener("change", reset);
  $("reset").onclick = reset; $("hit").onclick = () => shoot(false); $("miss").onclick = () => shoot(true);
  $("pause").onclick = () => { if (error) return; paused = !paused; accumulator = 0; $("pause").textContent = paused ? "Resume" : "Pause"; };
  for (const b of document.querySelectorAll("button")) b.disabled = false;
  reset();
  requestAnimationFrame(frame);
} catch (err) {
  status.textContent = `Unable to initialize: ${err.message}`;
  window.physicsLabState = { error: String(err) };
}
