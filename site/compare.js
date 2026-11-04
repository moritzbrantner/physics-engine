import { buildCatalogue, observationText, pairedFrame, readScenarios, readTrace, visualMode } from "./oracle-data.mjs";

const $ = (id) => document.getElementById(id);
const COLOR = { engine: "#7de6c4", rapier: "#f5b56c", grid: "#29414f", text: "#b3cad5" };

let wasm, scenarios = [], catalogue = [], capability, scenario, engine, rapier, result = null;
let tick = 0, playing = false, storage = 0, axes = "xy", selectionVersion = 0;

const name = (value) => value.replaceAll("-", " ");
const median = (values) => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
const nextFrame = () => new Promise((resolve) => requestAnimationFrame(resolve));
const byId = (id) => catalogue.find((item) => item.id === id);
const readUrl = () => new URLSearchParams(location.search);

function writeUrl(replace = false) {
  const params = new URLSearchParams();
  params.set("capability", capability.id);
  params.set("scenario", scenario.name);
  if (storage === 3) params.set("storage", "indexed");
  if (axes === "xz") params.set("axes", "xz");
  history[replace ? "replaceState" : "pushState"](null, "", `${location.pathname}?${params}`);
}

function setStatus(message, check = null) {
  const strong = document.createElement("strong");
  strong.textContent = check === true ? "Invariant check passed" : check === false ? "Invariant check failed" : message;
  if (check === false) strong.dataset.result = "failed";
  $("status").replaceChildren(strong);
  if (check !== null) $("status").append(document.createTextNode(` · ${message}`));
}

function renderNavigation() {
  const nav = $("capabilities");
  nav.replaceChildren();
  for (const item of catalogue) {
    const button = document.createElement("button");
    button.type = "button";
    button.setAttribute("aria-current", String(item.id === capability?.id));
    button.title = item.title;
    const id = document.createElement("span"), title = document.createElement("span");
    id.className = "cid"; id.textContent = item.id;
    title.className = "feature-name"; title.textContent = item.title;
    button.append(id, title);
    button.addEventListener("click", () => select(item.id, item.scenarios[0].name));
    nav.append(button);
  }
}

function renderChoices() {
  $("feature-title").textContent = `${capability.id} · ${capability.title}`;
  $("feature-description").textContent = capability.detail === capability.title ? "" : capability.detail;
  $("feature-description").hidden = capability.detail === capability.title;
  $("scenario").replaceChildren(...capability.scenarios.map((item) => {
    const option = document.createElement("option");
    option.value = item.name; option.textContent = name(item.name);
    return option;
  }));
  $("scenario").value = scenario.name;
  $("storage").value = String(storage);
  $("axes").value = axes;
  renderNavigation();
}

function setControls(enabled) {
  for (const id of ["scenario", "storage", "axes", "benchmark", "play", "step", "reset", "tick"]) $(id).disabled = !enabled;
}

async function select(featureId, scenarioName, replaceUrl = false) {
  const currentVersion = ++selectionVersion;
  stop();
  capability = byId(featureId) ?? catalogue[0];
  scenario = capability.scenarios.find((item) => item.name === scenarioName) ?? capability.scenarios[0];
  renderChoices();
  writeUrl(replaceUrl);
  setControls(false);
  engine = null; rapier = null; result = null;
  setStatus(`Running ${name(scenario.name)} on both Rust/WASM engines…`);
  await nextFrame();
  if (currentVersion !== selectionVersion) return;
  try {
    const left = readTrace(wasm, scenario, storage);
    const right = readTrace(wasm, scenario, 1);
    const check = wasm.oracle_check(scenario.index) === 0;
    if (currentVersion !== selectionVersion) return;
    engine = left; rapier = right; result = check; tick = 0;
    $("tick").max = String(Math.max(0, scenario.ticks - 1));
    setControls(true);
    setStatus(`${name(scenario.name)} · ${scenario.ticks} ticks · ${left.total.toLocaleString()} / ${right.total.toLocaleString()} observations (engine / Rapier)`, check);
    render();
  } catch (error) {
    if (currentVersion !== selectionVersion) return;
    setStatus(`Scenario failed to load: ${error.message}`, false);
    window.physicsComparisonState = { scenario: scenario.name, error: String(error) };
  }
}

function stop() {
  playing = false;
  $("play").textContent = "Play";
}

function setTick(value) {
  tick = Math.max(0, Math.min(scenario.ticks - 1, value));
  render();
}

function play() {
  if (playing) { stop(); return; }
  if (tick >= scenario.ticks - 1) tick = 0;
  playing = true;
  $("play").textContent = "Pause";
  let last = 0;
  const advance = (now) => {
    if (!playing) return;
    if (!last || now - last >= 85) {
      last = now;
      setTick(tick + 1);
      if (tick === scenario.ticks - 1) { stop(); return; }
    }
    requestAnimationFrame(advance);
  };
  requestAnimationFrame(advance);
}

function plotContext(canvas) {
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  const w = Math.max(1, canvas.clientWidth), h = Math.max(1, canvas.clientHeight);
  canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio);
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("Canvas 2D is unavailable");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}

function chartFrame(ctx, w, h, caption) {
  ctx.font = "12px system-ui,sans-serif";
  ctx.fillStyle = COLOR.text;
  ctx.fillText(caption, 17, 23);
  ctx.strokeStyle = COLOR.grid;
  ctx.lineWidth = 1;
  for (let i = 0; i < 5; i++) {
    const x = 30 + (w - 54) * i / 4, y = 43 + (h - 70) * i / 4;
    ctx.beginPath(); ctx.moveTo(x, 43); ctx.lineTo(x, h - 27); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(30, y); ctx.lineTo(w - 24, y); ctx.stroke();
  }
  return { left: 30, top: 43, width: Math.max(1, w - 54), height: Math.max(1, h - 70) };
}

function plottedKeys(kind) {
  return [...new Set([...engine.rows, ...rapier.rows]
    .filter((row) => row.kind === kind).map((row) => row.ordinal))].sort((a, b) => a - b).slice(0, 12);
}

function drawLines(ctx, area, trace, kind, keys, value, xValue, domain, color) {
  const paths = new Map(keys.map((key) => [key, []]));
  for (const row of trace.rows) {
    if (row.tick > tick) break;
    if (row.kind === kind) paths.get(row.ordinal)?.push(row);
  }
  keys.forEach((key, i) => {
    const points = paths.get(key);
    if (!points.length) return;
    const x = (row) => area.left + ((xValue(row) - domain.x0) / (domain.x1 - domain.x0)) * area.width;
    const y = (row) => area.top + (1 - (value(row) - domain.y0) / (domain.y1 - domain.y0)) * area.height;
    ctx.strokeStyle = color; ctx.globalAlpha = 0.25 + (i % 4) * 0.15; ctx.lineWidth = 1.7;
    ctx.beginPath(); points.forEach((point, j) => j ? ctx.lineTo(x(point), y(point)) : ctx.moveTo(x(point), y(point))); ctx.stroke();
    const last = points.at(-1);
    ctx.fillStyle = color; ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(x(last), y(last), 3.4, 0, 2 * Math.PI); ctx.fill();
    if (i < 6) { ctx.fillStyle = COLOR.text; ctx.font = "10px ui-monospace,monospace"; ctx.fillText(`#${key}`, x(last) + 5, y(last) - 6); }
  });
  ctx.globalAlpha = 1;
}

function drawPosition(ctx, area, trace, color) {
  const keys = plottedKeys(3);
  const a = axes === "xy" ? 1 : 2;
  const chosen = new Set(keys);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const row of [...engine.rows, ...rapier.rows]) {
    if (row.kind !== 3 || !chosen.has(row.ordinal)) continue;
    minX = Math.min(minX, row.values[0]); maxX = Math.max(maxX, row.values[0]);
    minY = Math.min(minY, row.values[a]); maxY = Math.max(maxY, row.values[a]);
  }
  if (!keys.length) { minX = minY = -1; maxX = maxY = 1; }
  const dx = Math.max(0.1, (maxX - minX) * .08), dy = Math.max(0.1, (maxY - minY) * .08);
  const [x0, x1, y0, y1] = [minX - dx, maxX + dx, minY - dy, maxY + dy];
  drawLines(ctx, area, trace, 3, keys, (row) => row.values[a], (row) => row.values[0], { x0, x1, y0, y1 }, color);
  ctx.fillStyle = COLOR.text; ctx.font = "10px ui-monospace,monospace";
  ctx.fillText(`X ${x0.toFixed(1)}…${x1.toFixed(1)} m`, area.left, area.top + area.height + 17);
  ctx.fillText(`${a === 1 ? "Y" : "Z"} ${y0.toFixed(1)}…${y1.toFixed(1)} m`, area.left, area.top - 7);
}

function drawRotation(ctx, area, trace, color) {
  const keys = plottedKeys(4);
  drawLines(ctx, area, trace, 4, keys, (row) => row.values[3], (row) => row.tick,
    { x0: 0, x1: Math.max(1, scenario.ticks - 1), y0: -1, y1: 1 }, color);
  ctx.fillStyle = COLOR.text; ctx.font = "10px ui-monospace,monospace";
  ctx.fillText("w = −1 … +1", area.left, area.top - 7);
  ctx.fillText(`tick 0…${scenario.ticks - 1}`, area.left, area.top + area.height + 17);
}

function drawEvents(ctx, area, trace, color) {
  const pairs = [...new Set([...engine.rows, ...rapier.rows].filter((row) => row.kind === 10)
    .map((row) => `${row.values[1]} / ${row.values[2]}`))].sort().slice(0, 12);
  ctx.font = "10px ui-monospace,monospace";
  pairs.forEach((pair, i) => {
    const y = area.top + (i + .5) / Math.max(1, pairs.length) * area.height;
    ctx.fillStyle = COLOR.text; ctx.fillText(pair, area.left + 4, y - 5);
  });
  for (const row of trace.rows.filter((item) => item.kind === 10 && item.tick <= tick)) {
    const i = pairs.indexOf(`${row.values[1]} / ${row.values[2]}`);
    if (i === -1) continue;
    const x = area.left + row.tick / Math.max(1, scenario.ticks - 1) * area.width;
    const y = area.top + (i + .5) / Math.max(1, pairs.length) * area.height;
    ctx.fillStyle = row.values[0] ? color : "#e8999b";
    ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
  }
  ctx.fillStyle = COLOR.text; ctx.fillText("● start · red = stop", area.left, area.top + area.height + 17);
}

function drawProbes(ctx, area, trace, color) {
  const rows = trace.frames[tick].slice(0, 12);
  if (!rows.length) {
    ctx.fillStyle = COLOR.text; ctx.fillText("No observations at this tick", area.left + 12, area.top + 36);
    return;
  }
  ctx.font = "11px ui-monospace,monospace";
  rows.forEach((row, i) => {
    const y = area.top + (i + .65) * area.height / rows.length;
    const text = observationText(row);
    ctx.fillStyle = COLOR.text;
    ctx.fillText(`#${row.ordinal} ${text}`, area.left + 12, y, area.width - 15);
    ctx.fillStyle = color;
    ctx.fillRect(area.left + 2, y - 9, 3, 11);
  });
}

function draw(canvas, trace, color, mode) {
  const { ctx, w, h } = plotContext(canvas);
  const captions = {
    position: `Measured trajectories · ${axes.toUpperCase()} projection · first 12 lanes`,
    rotation: "Measured quaternion w · first 12 lanes",
    events: "Recorded contact starts/stops · first 12 pairs",
    probes: "Recorded query / contact results · first 12 rows",
  };
  const area = chartFrame(ctx, w, h, captions[mode]);
  if (mode === "position") drawPosition(ctx, area, trace, color);
  else if (mode === "rotation") drawRotation(ctx, area, trace, color);
  else if (mode === "events") drawEvents(ctx, area, trace, color);
  else drawProbes(ctx, area, trace, color);
}

function renderObservations() {
  const rows = pairedFrame(engine, rapier, tick);
  const tbody = $("observations");
  tbody.replaceChildren();
  if (!rows.length) {
    const row = document.createElement("tr"), cell = document.createElement("td");
    cell.colSpan = 3; cell.textContent = "No observations at this tick";
    row.append(cell); tbody.append(row);
  }
  for (const item of rows) {
    const tr = document.createElement("tr"), index = document.createElement("th");
    index.scope = "row"; index.textContent = `#${item.ordinal}`; tr.append(index);
    for (const side of ["engine", "rapier"]) {
      const cell = document.createElement("td"); cell.textContent = observationText(item[side]); tr.append(cell);
    }
    tbody.append(tr);
  }
  $("sample-info").textContent = `${rows.length} displayed at tick ${tick} · ${engine.displayed.toLocaleString()} / ${engine.total.toLocaleString()} engine and ${rapier.displayed.toLocaleString()} / ${rapier.total.toLocaleString()} Rapier observations in the full run`;
}

function render() {
  if (!engine || !rapier) return;
  $("tick").value = String(tick);
  $("tick-label").textContent = `Tick ${tick + 1} / ${scenario.ticks}`;
  const mode = visualMode(engine, rapier);
  $("axes").disabled = mode !== "position";
  draw($("engine"), engine, COLOR.engine, mode);
  draw($("rapier"), rapier, COLOR.rapier, mode);
  $("engine-summary").textContent = `${engine.frames[tick].length} displayed now · ${engine.total.toLocaleString()} total`;
  $("rapier-summary").textContent = `${rapier.frames[tick].length} displayed now · ${rapier.total.toLocaleString()} total`;
  renderObservations();
  window.physicsComparisonState = { capability: capability.id, scenario: scenario.name, tick, result,
    engineRows: engine.displayed, rapierRows: rapier.displayed };
}

async function benchmark() {
  const currentVersion = selectionVersion;
  stop(); $("benchmark").disabled = true;
  setStatus("Measuring both simulations on this device…", result);
  await nextFrame();
  if (currentVersion !== selectionVersion) return;
  try {
    wasm.oracle_time(scenario.index, storage); wasm.oracle_time(scenario.index, 1);
    const left = [], right = [];
    for (let i = 0; i < 5; i++) {
      left.push(wasm.oracle_time(scenario.index, storage));
      right.push(wasm.oracle_time(scenario.index, 1));
    }
    const a = median(left), b = median(right);
    setStatus(`${name(scenario.name)} · exploratory WASM median of 5 runs: engine ${a.toFixed(2)} µs/tick, Rapier ${b.toFixed(2)} µs/tick (${(a / b).toFixed(2)}×); device-dependent, not CI evidence`, result);
  } catch (error) {
    setStatus(`Browser timing unavailable: ${error.message}`, result);
  } finally { if (currentVersion === selectionVersion) $("benchmark").disabled = false; }
}

$("scenario").addEventListener("change", () => select(capability.id, $("scenario").value));
$("storage").addEventListener("change", () => { storage = Number($("storage").value); select(capability.id, scenario.name); });
$("axes").addEventListener("change", () => { axes = $("axes").value; writeUrl(); render(); });
$("play").addEventListener("click", play);
$("step").addEventListener("click", () => { stop(); setTick(tick + 1); });
$("reset").addEventListener("click", () => { stop(); setTick(0); });
$("tick").addEventListener("input", () => { stop(); setTick(Number($("tick").value)); });
$("benchmark").addEventListener("click", benchmark);
window.addEventListener("resize", () => render());
window.addEventListener("popstate", () => restore(true));

function restore(replace = false) {
  const params = readUrl();
  storage = params.get("storage") === "indexed" ? 3 : 0;
  axes = params.get("axes") === "xz" ? "xz" : "xy";
  select(params.get("capability"), params.get("scenario"), replace);
}

try {
  const [res, metadata] = await Promise.all([fetch("physics_oracle.wasm"), fetch("capabilities.json")]);
  if (!res.ok || !metadata.ok) throw new Error(`Required Pages artifact missing (WASM ${res.status}, catalogue ${metadata.status})`);
  const { instance } = await WebAssembly.instantiate(await res.arrayBuffer(), { env: { oracle_now: () => performance.now() } });
  wasm = instance.exports;
  scenarios = readScenarios(wasm);
  catalogue = buildCatalogue(await metadata.json(), scenarios);
  restore(true);
} catch (error) {
  setStatus(`Unable to initialize comparison: ${error.message}`, false);
  $("feature-title").textContent = "Comparison unavailable";
  window.physicsComparisonState = { error: String(error) };
}
