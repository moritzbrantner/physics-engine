// Exercise the actual Pages exports. A near miss must preserve sleep state, not just its picture.
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { arch, cpus, platform } from "node:os";

const [wasmPath, outputPath, mode] = process.argv.slice(2);
if (!wasmPath || !outputPath || (mode && mode !== "--report-only")) {
  throw new Error(
    "usage: node scripts/benchmark-projectile-wake.mjs <engine.wasm> <results.json> [--report-only]",
  );
}
const bytes = readFileSync(wasmPath);
const module = await WebAssembly.compile(bytes);
const hash = (value) => createHash("sha256").update(value).digest("hex");
const shotTicks = new Set([0, 12, 24]);
const requiredExports = [
  "sandbox_reset_tower_with_baking_options", "sandbox_body_count", "sandbox_set_projectile_type",
  "sandbox_shoot", "sandbox_step_velocity", "sandbox_is_quiescent", "sandbox_error_detail",
  "sandbox_refresh_render_snapshot", "sandbox_render_snapshot_len", "sandbox_render_snapshot_stride",
  "sandbox_last_event_response_passes", "sandbox_last_stabilization_passes",
];

function readCounter(engine, name) {
  return typeof engine[name] === "function" ? engine[name]() : null;
}

function crateSnapshot(engine) {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  assert.equal(engine.sandbox_render_snapshot_stride(), 11);
  const data = new Int32Array(engine.memory.buffer, pointer, length);
  const crates = [];
  const sleeping = [];
  for (let offset = 0; offset < length; offset += 11) {
    if (data[offset] !== 2) continue;
    crates.push(...data.slice(offset, offset + 11));
    if (typeof engine.sandbox_body_sleeping === "function") {
      sleeping.push(engine.sandbox_body_sleeping(offset / 11));
    }
  }
  assert.equal(crates.length, 32 * 11, "the fixture must retain all 32 crates");
  return { poses: hash(JSON.stringify(crates)), sleeping };
}

async function runTrial({ upright, linear, type }, trial) {
  const engine = (await WebAssembly.instantiate(module, {})).exports;
  for (const name of requiredExports) {
    assert.equal(typeof engine[name], "function", `missing Pages export: ${name}`);
  }
  const rules = (1 << 29) | ((1 << 11) - 2) | (1 << 14) | (2 << 12)
    | (upright ? 1 << 11 : 0) | (linear ? 1 : 0);
  assert.equal(engine.sandbox_reset_tower_with_baking_options(rules, 0, 1), 0);
  assert.equal(engine.sandbox_body_count(), 44);
  for (let tick = 0; tick < 240; tick += 1) {
    assert.equal(
      engine.sandbox_step_velocity(0, 0, 0), 0,
      `warmup ${tick}, detail ${engine.sandbox_error_detail()}`,
    );
  }
  assert.equal(engine.sandbox_is_quiescent(), 1);
  const initial = crateSnapshot(engine);
  const trace = createHash("sha256");
  const raw = [];
  const failures = [];
  let changedTicks = 0;
  let awakeCrateTicks = 0;
  let peakAwakeCrates = 0;
  let peakAuthority = 0;
  let woken = 0;
  let retries = 0;
  let probeQueries = 0;
  let probeResponsePasses = 0;
  let responsePasses = 0;
  let stabilizationPasses = 0;
  let completedTicks = 0;
  assert.equal(engine.sandbox_set_projectile_type(type), 0);

  for (let tick = 0; tick < 120; tick += 1) {
    if (shotTicks.has(tick)) assert.ok(engine.sandbox_shoot(40, 0, -87) >= 0);
    const start = performance.now();
    const error = engine.sandbox_step_velocity(0, 0, 0);
    raw.push(performance.now() - start);
    if (error) {
      failures.push({ tick, error, detail: engine.sandbox_error_detail() });
      break;
    }
    completedTicks += 1;
    const state = crateSnapshot(engine);
    trace.update(JSON.stringify(state));
    changedTicks += Number(state.poses !== initial.poses);
    const awake = state.sleeping.filter((value) => value !== 1).length;
    awakeCrateTicks += awake;
    peakAwakeCrates = Math.max(peakAwakeCrates, awake);
    peakAuthority = Math.max(
      peakAuthority, readCounter(engine, "sandbox_last_response_authority_body_count") ?? 0,
    );
    woken += readCounter(engine, "sandbox_last_parked_bodies_woken") ?? 0;
    retries += readCounter(engine, "sandbox_last_parked_wake_retries") ?? 0;
    probeQueries += (readCounter(engine, "sandbox_last_wake_probe_broad_phase_queries") ?? 0)
      + (readCounter(engine, "sandbox_last_wake_probe_tail_broad_phase_queries") ?? 0);
    probeResponsePasses += readCounter(engine, "sandbox_last_wake_probe_response_passes") ?? 0;
    responsePasses += engine.sandbox_last_event_response_passes();
    stabilizationPasses += engine.sandbox_last_stabilization_passes();
  }

  if (changedTicks) failures.push({ message: "near miss changed tower poses", changedTicks });
  if (awakeCrateTicks) failures.push({ message: "near miss woke crates", awakeCrateTicks });
  if (woken || retries || probeQueries || peakAuthority > 2) {
    failures.push({ message: "unexpected tower response work", woken, retries, probeQueries, peakAuthority });
  }
  if (engine.sandbox_body_count() !== 44) {
    failures.push({ message: "projectile lifecycle did not complete" });
  }
  const sleepEvidence = initial.sleeping.length === 32;
  if (!sleepEvidence) failures.push({ message: "build has no per-body sleep observability" });
  const authorityEvidence = typeof engine.sandbox_last_response_authority_body_count === "function";
  if (!authorityEvidence) failures.push({ message: "build has no response-authority observability" });
  return {
    trial,
    passed: failures.length === 0,
    numerical_backend: readCounter(engine, "sandbox_numeric_backend"),
    sleep_evidence_available: sleepEvidence,
    changed_ticks: changedTicks,
    peak_awake_crates: sleepEvidence ? peakAwakeCrates : null,
    awake_crate_ticks: sleepEvidence ? awakeCrateTicks : null,
    peak_response_authority_bodies: authorityEvidence ? peakAuthority : null,
    bodies_woken: typeof engine.sandbox_last_parked_bodies_woken === "function" ? woken : null,
    wake_retries: typeof engine.sandbox_last_parked_wake_retries === "function" ? retries : null,
    wake_probe_queries: typeof engine.sandbox_last_wake_probe_broad_phase_queries === "function" ? probeQueries : null,
    wake_probe_response_passes: typeof engine.sandbox_last_wake_probe_response_passes === "function" ? probeResponsePasses : null,
    event_response_passes: responsePasses,
    stabilization_passes: stabilizationPasses,
    completed_ticks: completedTicks,
    mean_ms: raw.reduce((sum, value) => sum + value, 0) / raw.length,
    max_ms: Math.max(...raw),
    raw_step_ms: raw,
    trace_sha256: trace.digest("hex"),
    failures,
  };
}

const cases = [];
for (const upright of [false, true]) {
  for (const linear of [false, true]) {
    for (const [projectile, type] of [["sphere", 0], ["arrow", 1], ["rigid", 2]]) {
      const trials = [];
      for (let trial = 0; trial < 2; trial += 1) {
        trials.push(await runTrial({ upright, linear, type }, trial));
      }
      const repeatable = trials.every((run) => run.trace_sha256 === trials[0].trace_sha256);
      cases.push({
        projectile, crates: upright ? "upright" : "free", character: linear ? "linear" : "physical",
        repeatable, trials,
      });
      console.log(JSON.stringify({
        projectile, upright, linear, passed: repeatable && trials.every((run) => run.passed),
        changed_ticks: trials[0].changed_ticks, peak_awake: trials[0].peak_awake_crates,
        mean_ms: trials[0].mean_ms, max_ms: trials[0].max_ms,
      }));
    }
  }
}
const report = {
  workload: "pages-tower-near-miss-v1",
  environment: { node: process.version, v8: process.versions.v8, platform: platform(), arch: arch(), cpu: cpus()[0]?.model },
  wasm_sha256: hash(bytes),
  note: "Two same-build replays per case, three near misses then retirement. Timings include all physics attempts but exclude the assertion/snapshot harness; shared host timings are advisory, not laptop FPS. Older builds without sleep/authority exports cannot pass this acceptance test.",
  cases,
  passed: cases.every((entry) => entry.repeatable && entry.trials.every((run) => run.passed)),
};
writeFileSync(outputPath, `${JSON.stringify(report, null, 2)}\n`);
if (!report.passed && mode !== "--report-only") process.exitCode = 1;
