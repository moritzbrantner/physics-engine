import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { arch, cpus, platform } from "node:os";

const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath || !outputPath) {
  throw new Error("usage: node scripts/benchmark-sandbox.mjs <head.wasm> <results.json>");
}
const trials = Number(process.env.TRIALS ?? 1);
if (!Number.isInteger(trials) || trials < 1 || trials > 10) {
  throw new Error("TRIALS must be an integer from 1 through 10");
}
const names = [
  "settled-idle",
  "walking-no-shots",
  "three-shots-idle",
  "three-shots-walking",
  "six-shots-idle",
  "thirty-shots-walking",
];
const cases = process.env.CASE ? names.filter((name) => name === process.env.CASE) : names;
if (cases.length === 0) throw new Error("unknown CASE");
const hash = (value) => createHash("sha256").update(value).digest("hex");
function stats(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const percentile = (p) => sorted[Math.max(0, Math.ceil(sorted.length * p) - 1)];
  return {
    count: values.length,
    mean_ms: values.reduce((sum, value) => sum + value, 0) / values.length,
    p50_ms: percentile(0.5),
    p95_ms: percentile(0.95),
    max_ms: sorted.at(-1),
  };
}
function sumKnown(values, key) {
  const known = values.map((value) => value[key]).filter((value) => Number.isInteger(value));
  return known.length === values.length ? known.reduce((sum, value) => sum + value, 0) : null;
}
function shotTicks(name) {
  if (name === "thirty-shots-walking") {
    return Array.from({ length: 30 }, (_, index) => index * 5);
  }
  if (name === "six-shots-idle") return [0, 30, 60, 90, 120, 150];
  if (name.startsWith("three-shots-")) return [0, 40, 80];
  return [];
}
function caseTicks(name) {
  return name === "thirty-shots-walking" ? 240 : 180;
}
function ratio(head, baseline) {
  return baseline === 0 ? null : head / baseline;
}
async function measure(path) {
  const bytes = readFileSync(path);
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const engine = instance.exports;
  function step(x, z) {
    const error = engine.sandbox_step_velocity(x, z, 0);
    if (error !== 0) throw new Error(`physics error ${error}, detail ${engine.sandbox_error_detail()}`);
  }
  function readCounter(name) {
    return typeof engine[name] === "function" ? engine[name]() : null;
  }
  function stepWork() {
    return {
      sampled_events: readCounter("sandbox_last_sampled_events"),
      tail_contacts: readCounter("sandbox_last_tail_contacts"),
      tail_slices: readCounter("sandbox_last_tail_slices"),
      tail_replays: readCounter("sandbox_last_tail_replays"),
      tail_candidate_pairs: readCounter("sandbox_last_tail_candidate_pairs"),
      tail_broad_phase_queries: readCounter("sandbox_last_tail_broad_phase_queries"),
      tail_broad_phase_rebuilds: readCounter("sandbox_last_tail_broad_phase_rebuilds"),
      tail_broad_phase_reuses: readCounter("sandbox_last_tail_broad_phase_reuses"),
      broad_phase_queries: readCounter("sandbox_last_broad_phase_queries"),
      broad_phase_rebuilds: readCounter("sandbox_last_broad_phase_rebuilds"),
      broad_phase_reuses: readCounter("sandbox_last_broad_phase_reuses"),
      broad_phase_incremental_updates: readCounter("sandbox_last_broad_phase_incremental_updates"),
      broad_phase_reinserts: readCounter("sandbox_last_broad_phase_reinserts"),
      broad_phase_rotations: readCounter("sandbox_last_broad_phase_rotations"),
      broad_phase_partial_queries: readCounter("sandbox_last_broad_phase_partial_queries"),
      broad_phase_partial_body_updates: readCounter("sandbox_last_broad_phase_partial_body_updates"),
      response_scratch_index_rebuilds: readCounter("sandbox_last_response_scratch_index_rebuilds"),
      event_response_passes: readCounter("sandbox_last_event_response_passes"),
      stabilization_passes: readCounter("sandbox_last_stabilization_passes"),
      stabilizations_hitting_limit: readCounter("sandbox_last_stabilizations_hitting_limit"),
      stabilization_candidate_pairs: readCounter("sandbox_last_stabilization_candidate_pairs"),
      stabilization_exact_contacts: readCounter("sandbox_last_stabilization_exact_contacts"),
      stabilization_active_bodies: readCounter("sandbox_last_stabilization_active_bodies"),
    };
  }
  function settle() {
    engine.sandbox_reset();
    for (let tick = 0; tick < 240; tick += 1) step(0, 0);
    if (engine.sandbox_is_quiescent() !== 1) throw new Error("initial scene did not settle");
  }
  settle();
  for (let tick = 0; tick < 120; tick += 1) step(0, -420);
  const result = { wasm_sha256: hash(bytes), cases: [] };
  for (const name of cases) {
    const measurements = [];
    for (let trial = 0; trial < trials; trial += 1) {
      settle();
      if (name === "thirty-shots-walking") {
        if (typeof engine.sandbox_set_projectile_type !== "function") {
          throw new Error("thirty-shots-walking requires explicit projectile type selection");
        }
        if (engine.sandbox_set_projectile_type(0) !== 0) {
          throw new Error("failed to select sphere projectile stress lane");
        }
      }
      const times = [];
      const events = [];
      const work = [];
      const trace = createHash("sha256");
      const shots = shotTicks(name);
      for (let tick = 0; tick < caseTicks(name); tick += 1) {
        const shotIndex = shots.indexOf(tick);
        if (shotIndex >= 0) {
          const projectileX = shotIndex % 2 === 0 ? 38 : -38;
          const projectileY = shotIndex % 3 === 2 ? -7 : 0;
          if (engine.sandbox_shoot(projectileX, projectileY, -88) < 0) {
            throw new Error("projectile creation failed");
          }
        }
        const moving = name === "walking-no-shots" || name.endsWith("-walking");
        const x = moving && tick >= 80 ? Math.round(420 * Math.sin(0.7)) : 0;
        const z = moving ? (tick >= 80 ? -Math.round(420 * Math.cos(0.7)) : -420) : 0;
        const start = performance.now();
        step(x, z);
        times.push(performance.now() - start);
        work.push(stepWork());
        // State evidence is intentionally outside the timed physics call. This is an observable
        // replay fingerprint, not a claim to expose every private solver field.
        const pointer = engine.sandbox_refresh_render_snapshot();
        const length = engine.sandbox_render_snapshot_len();
        const stride = engine.sandbox_render_snapshot_stride();
        if (stride !== 11 || length <= 0 || length % stride !== 0) throw new Error("bad snapshot");
        trace.update(Buffer.from(engine.memory.buffer, pointer, length * Int32Array.BYTES_PER_ELEMENT));
        const contacts = engine.sandbox_last_collision_events();
        events.push(contacts);
        trace.update(JSON.stringify([
          tick, length, engine.sandbox_body_count(), engine.sandbox_grounded(),
          engine.sandbox_is_quiescent(), contacts, engine.sandbox_total_collisions(),
        ]));
      }
      const replay = trace.digest("hex");
      if (measurements.length && measurements[0].replay_sha256 !== replay) {
        throw new Error(`nondeterministic replay for ${name}`);
      }
      measurements.push({
        steps: stats(times),
        replay_sha256: replay,
        body_count: engine.sandbox_body_count(),
        event_sum: events.reduce((a, b) => a + b, 0),
        work: {
          sampled_events: sumKnown(work, "sampled_events"),
          tail_contacts: sumKnown(work, "tail_contacts"),
          tail_slices: sumKnown(work, "tail_slices"),
          tail_replays: sumKnown(work, "tail_replays"),
          tail_candidate_pairs: sumKnown(work, "tail_candidate_pairs"),
          tail_broad_phase_queries: sumKnown(work, "tail_broad_phase_queries"),
          tail_broad_phase_rebuilds: sumKnown(work, "tail_broad_phase_rebuilds"),
          tail_broad_phase_reuses: sumKnown(work, "tail_broad_phase_reuses"),
          broad_phase_queries: sumKnown(work, "broad_phase_queries"),
          broad_phase_rebuilds: sumKnown(work, "broad_phase_rebuilds"),
          broad_phase_reuses: sumKnown(work, "broad_phase_reuses"),
          broad_phase_incremental_updates: sumKnown(work, "broad_phase_incremental_updates"),
          broad_phase_reinserts: sumKnown(work, "broad_phase_reinserts"),
          broad_phase_rotations: sumKnown(work, "broad_phase_rotations"),
          broad_phase_partial_queries: sumKnown(work, "broad_phase_partial_queries"),
          broad_phase_partial_body_updates: sumKnown(work, "broad_phase_partial_body_updates"),
          response_scratch_index_rebuilds: sumKnown(work, "response_scratch_index_rebuilds"),
          event_response_passes: sumKnown(work, "event_response_passes"),
          stabilization_passes: sumKnown(work, "stabilization_passes"),
          stabilizations_hitting_limit: sumKnown(work, "stabilizations_hitting_limit"),
          stabilization_candidate_pairs: sumKnown(work, "stabilization_candidate_pairs"),
          stabilization_exact_contacts: sumKnown(work, "stabilization_exact_contacts"),
          stabilization_active_bodies: sumKnown(work, "stabilization_active_bodies"),
        },
        raw: { steps_ms: times, events, work },
      });
    }
    result.cases.push({ name, trials: measurements });
    console.log(path, name, measurements.map(({ raw, ...summary }) => summary));
  }
  return result;
}
const result = {
  workload: "sandbox-projectiles-v5",
  note: "Warmed Node/V8 WASM physics only; not browser FPS or GPU performance. Timings are advisory. v5 adds a deterministic 30-shot walking stress trace while keeping exact within-build replay determinism and scenario validity. Baseline/head replay and final body-count differences are recorded for the analytic sphere stress lane because projectile representation and out-of-bounds retirement are deliberately different.",
  environment: { node: process.version, v8: process.versions.v8, platform: platform(), arch: arch(), cpu: cpus()[0]?.model },
  head_revision: process.env.HEAD_SHA ?? null,
  baseline_revision: process.env.BASE_SHA ?? null,
};
try {
  if (process.env.BASELINE_WASM) result.baseline = await measure(process.env.BASELINE_WASM);
  result.head = await measure(wasmPath);
  if (result.baseline) {
    result.comparison = result.head.cases.map((entry, index) => ({
      name: entry.name,
      trials: entry.trials.map((trial, trialIndex) => {
        const baseline = result.baseline.cases[index].trials[trialIndex];
        const bodyCountMatches = trial.body_count === baseline.body_count;
        const representationMayDiffer = entry.name === "thirty-shots-walking";
        if (!representationMayDiffer && !bodyCountMatches) {
          throw new Error(`baseline/head body-count mismatch for ${entry.name}`);
        }
        return {
          replay_matches: trial.replay_sha256 === baseline.replay_sha256,
          body_count_matches: bodyCountMatches,
          head_body_count: trial.body_count,
          baseline_body_count: baseline.body_count,
          body_count_delta: trial.body_count - baseline.body_count,
          event_sum_delta: trial.event_sum - baseline.event_sum,
          mean_time_ratio: ratio(trial.steps.mean_ms, baseline.steps.mean_ms),
          p95_time_ratio: ratio(trial.steps.p95_ms, baseline.steps.p95_ms),
          broad_phase_rebuild_delta:
            trial.work.broad_phase_rebuilds === null || baseline.work.broad_phase_rebuilds === null
              ? null
              : trial.work.broad_phase_rebuilds - baseline.work.broad_phase_rebuilds,
          tail_broad_phase_rebuild_delta:
            trial.work.tail_broad_phase_rebuilds === null || baseline.work.tail_broad_phase_rebuilds === null
              ? null
              : trial.work.tail_broad_phase_rebuilds - baseline.work.tail_broad_phase_rebuilds,
        };
      }),
    }));
    result.replay_matches = result.comparison.every((entry) =>
      entry.trials.every((trial) => trial.replay_matches),
    );
  }
  const settled = result.head.cases.find((entry) => entry.name === "settled-idle")?.trials[0];
  if (settled) {
    if (settled.event_sum !== 0) throw new Error("settled-idle produced collision events");
    for (const [key, value] of Object.entries(settled.work)) {
      if (value !== null && value !== 0) throw new Error(`settled-idle reported ${key}=${value}`);
    }
  }
} catch (error) {
  result.failure = String(error);
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
  throw error;
}
writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
