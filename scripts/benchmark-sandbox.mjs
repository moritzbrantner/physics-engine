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
const names = ["settled-idle", "walking-no-shots", "three-shots-idle", "three-shots-walking"];
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
async function measure(path) {
  const bytes = readFileSync(path);
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const engine = instance.exports;
  function step(x, z) {
    const error = engine.sandbox_step_velocity(x, z, 0);
    if (error !== 0) throw new Error(`physics error ${error}, detail ${engine.sandbox_error_detail()}`);
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
      const times = [];
      const events = [];
      const trace = createHash("sha256");
      for (let tick = 0; tick < 180; tick += 1) {
        if (name.startsWith("three-") && [0, 40, 80].includes(tick)) {
          if (engine.sandbox_shoot(tick === 40 ? -38 : 38, tick === 80 ? -7 : 0, -88) < 0) {
            throw new Error("projectile creation failed");
          }
        }
        const moving = name === "walking-no-shots" || name === "three-shots-walking";
        const x = moving && tick >= 80 ? Math.round(420 * Math.sin(0.7)) : 0;
        const z = moving ? (tick >= 80 ? -Math.round(420 * Math.cos(0.7)) : -420) : 0;
        const start = performance.now();
        step(x, z);
        times.push(performance.now() - start);
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
        steps: stats(times), replay_sha256: replay, body_count: engine.sandbox_body_count(),
        event_sum: events.reduce((a, b) => a + b, 0), raw: { steps_ms: times, events },
      });
    }
    result.cases.push({ name, trials: measurements });
    console.log(path, name, measurements.map(({ raw, ...summary }) => summary));
  }
  return result;
}
const result = {
  workload: "sandbox-projectiles-v1",
  note: "Warmed Node/V8 WASM physics only; not browser FPS or GPU performance. Timings are advisory.",
  environment: { node: process.version, v8: process.versions.v8, platform: platform(), arch: arch(), cpu: cpus()[0]?.model },
  head_revision: process.env.HEAD_SHA ?? null,
  baseline_revision: process.env.BASE_SHA ?? null,
};
try {
  if (process.env.BASELINE_WASM) result.baseline = await measure(process.env.BASELINE_WASM);
  result.head = await measure(wasmPath);
  if (result.baseline) {
    result.replay_matches = result.head.cases.every((entry, index) => entry.trials.every(
      (trial, trialIndex) => trial.replay_sha256 === result.baseline.cases[index].trials[trialIndex].replay_sha256,
    ));
    if (!result.replay_matches) throw new Error("baseline/head observable replay mismatch");
  }
} catch (error) {
  result.failure = String(error);
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
  throw error;
}
writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
