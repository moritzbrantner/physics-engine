import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { arch, cpus, platform } from "node:os";
import { bodyMayBeVisible, createCamera } from "../site/scene-visibility.mjs";

const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath || !outputPath) {
  throw new Error("usage: node scripts/benchmark-terrain.mjs <module.wasm> <results.json>");
}
const trials = Number(process.env.TRIALS ?? 2);
if (!Number.isInteger(trials) || trials < 1 || trials > 10) {
  throw new Error("TRIALS must be an integer from 1 through 10");
}
const bytes = readFileSync(wasmPath);
const { instance } = await WebAssembly.instantiate(bytes, {});
const engine = instance.exports;
if (engine.sandbox_numeric_backend() !== 64) throw new Error("expected f64 backend");

function step(x, z) {
  const error = engine.sandbox_step_velocity(x, z, 0);
  if (error !== 0) {
    throw new Error(`physics error ${error}, detail ${engine.sandbox_error_detail()}`);
  }
}

function stats(values) {
  const sorted = [...values].sort((a, b) => a - b);
  return {
    mean_ms: values.reduce((sum, value) => sum + value, 0) / values.length,
    p95_ms: sorted[Math.ceil(sorted.length * 0.95) - 1],
    max_ms: sorted.at(-1),
  };
}

function visibleBodies() {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  const stride = engine.sandbox_render_snapshot_stride();
  if (stride !== 11 || length % stride !== 0) throw new Error("invalid snapshot");
  const data = new Int32Array(engine.memory.buffer, pointer, length);
  let player = null;
  const bodies = [];
  for (let offset = 0; offset < length; offset += stride) {
    const body = {
      role: data[offset],
      position: [data[offset + 1], data[offset + 2], data[offset + 3]],
      half: [data[offset + 4], data[offset + 5], data[offset + 6]],
    };
    if (body.role === 1) player = body;
    else bodies.push(body);
  }
  if (!player) throw new Error("missing player");
  const camera = createCamera(
    [player.position[0], player.position[1] + 13, player.position[2]],
    0, 0, 16 / 9, (70 * Math.PI) / 180, 0.8, 1400,
  );
  return {
    total: bodies.length,
    visible: bodies.filter((body) => bodyMayBeVisible(body, camera)).length,
    player_z: player.position[2],
    bytes: Buffer.from(engine.memory.buffer, pointer, length * Int32Array.BYTES_PER_ELEMENT),
  };
}

function runCase(mode, name) {
  const result = engine.sandbox_reset_terrain_with_baking_options(1, 1, mode);
  if (result !== 0) throw new Error(`terrain reset failed: ${result}`);
  if (engine.sandbox_body_count() !== 151) throw new Error("unexpected terrain body count");
  if (engine.sandbox_set_projectile_type(0) !== 0) throw new Error("sphere selection failed");
  for (let tick = 0; tick < 60; tick += 1) step(0, 0);
  const timings = [];
  const trace = createHash("sha256");
  let visibleTotal = 0;
  let bodyTotal = 0;
  let candidatePairs = 0;
  let broadPhaseQueries = 0;
  let impacts = 0;
  let finalPlayerZ = 440;
  for (let tick = 0; tick < 120; tick += 1) {
    if (name === "projectiles" && tick % 40 === 0) {
      if (engine.sandbox_shoot(0, 0, -96) < 0) throw new Error("shot failed");
    }
    const moving = name === "traverse";
    const started = performance.now();
    step(0, moving ? -420 : 0);
    timings.push(performance.now() - started);
    candidatePairs += engine.sandbox_last_tail_candidate_pairs();
    broadPhaseQueries += engine.sandbox_last_broad_phase_queries()
      + engine.sandbox_last_wake_probe_broad_phase_queries();
    impacts += engine.sandbox_last_ballistic_impacts();
    const snapshot = visibleBodies();
    finalPlayerZ = snapshot.player_z;
    visibleTotal += snapshot.visible;
    bodyTotal += snapshot.total;
    trace.update(snapshot.bytes);
    trace.update(JSON.stringify([
      tick, engine.sandbox_last_collision_events(), engine.sandbox_total_collisions(),
      engine.sandbox_is_quiescent(),
    ]));
  }
  if (name === "traverse" && finalPlayerZ >= 0) {
    throw new Error(`traverse did not cross the terrain: final Z ${finalPlayerZ}`);
  }
  if (name === "projectiles" && impacts === 0) {
    throw new Error("projectile case had no admitted ballistic impacts");
  }
  return {
    timing: stats(timings),
    replay_sha256: trace.digest("hex"),
    prepared_fixed_bodies: engine.sandbox_fixed_geometry_prepared_count(),
    mean_visible_bodies: visibleTotal / 120,
    mean_total_bodies: bodyTotal / 120,
    visible_fraction: visibleTotal / bodyTotal,
    final_player_z: finalPlayerZ,
    work: { tail_candidate_pairs: candidatePairs, broad_phase_queries: broadPhaseQueries, ballistic_impacts: impacts },
  };
}

const names = ["idle", "traverse", "projectiles"];
const results = [];
for (const name of names) {
  const runtime = [];
  const prepared = [];
  for (let trial = 0; trial < trials; trial += 1) {
    runtime.push(runCase(0, name));
    prepared.push(runCase(1, name));
    if (runtime[trial].replay_sha256 !== prepared[trial].replay_sha256) {
      throw new Error(`${name}: fixed geometry modes changed the replay`);
    }
  }
  for (const mode of [runtime, prepared]) {
    if (mode.some((entry) => entry.replay_sha256 !== mode[0].replay_sha256)) {
      throw new Error(`${name}: nondeterministic replay`);
    }
  }
  results.push({ name, runtime, prepared });
}

const output = {
  workload: "terrain-mixed-v1",
  note: "Physics-only Node/V8 WASM timings are advisory; visibility is an untimed conservative camera count, not a browser FPS measurement.",
  wasm_sha256: createHash("sha256").update(bytes).digest("hex"),
  environment: { node: process.version, v8: process.versions.v8, platform: platform(), arch: arch(), cpu: cpus()[0]?.model },
  trials,
  cases: results,
};
writeFileSync(outputPath, `${JSON.stringify(output, null, 2)}\n`);
for (const entry of results) {
  const runtime = entry.runtime.reduce((sum, trial) => sum + trial.timing.mean_ms, 0) / trials;
  const prepared = entry.prepared.reduce((sum, trial) => sum + trial.timing.mean_ms, 0) / trials;
  console.log(`${entry.name}: runtime ${runtime.toFixed(3)} ms, prepared ${prepared.toFixed(3)} ms, ratio ${(prepared / runtime).toFixed(3)}, visible ${(entry.prepared[0].visible_fraction * 100).toFixed(1)}%`);
}
