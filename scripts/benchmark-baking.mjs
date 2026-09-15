import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { arch, cpus, platform } from "node:os";

const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath || !outputPath) {
  throw new Error("usage: node scripts/benchmark-baking.mjs <module.wasm> <results.json>");
}

const bytes = readFileSync(wasmPath);
const { instance } = await WebAssembly.instantiate(bytes, {});
const engine = instance.exports;
const required = [
  "sandbox_reset_with_baking_options",
  "sandbox_fixed_geometry_mode",
  "sandbox_fixed_geometry_prepared_count",
  "sandbox_fixed_geometry_total_preparations",
  "sandbox_fixed_geometry_retained_bytes",
  "sandbox_fixed_geometry_representation_version",
  "sandbox_step_velocity",
  "sandbox_shoot",
  "sandbox_refresh_render_snapshot",
  "sandbox_render_snapshot_len",
  "sandbox_render_snapshot_stride",
];
for (const name of required) {
  assert.equal(typeof engine[name], "function", `missing baking evidence export ${name}`);
}

const workloadCases = [
  "settled-idle",
  "walking-no-shots",
  "three-shots-idle",
  "three-shots-walking",
];
const modes = [
  { name: "runtime", value: 0 },
  { name: "prepare-at-load", value: 1 },
];
const trialsPerMode = 2;

function hash(value) {
  return createHash("sha256").update(value).digest("hex");
}

function timingStats(values) {
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

function step(x, z) {
  const error = engine.sandbox_step_velocity(x, z, 0);
  assert.equal(
    error,
    0,
    `physics error ${error}, detail ${engine.sandbox_error_detail?.() ?? 0}`,
  );
}

function snapshot(trace) {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  const stride = engine.sandbox_render_snapshot_stride();
  assert.equal(stride, 11, "unexpected snapshot stride");
  assert.ok(length > 0 && length % stride === 0, "malformed render snapshot");
  trace.update(
    Buffer.from(
      new Uint8Array(engine.memory.buffer, pointer, length * Int32Array.BYTES_PER_ELEMENT),
    ),
  );
  return length;
}

function preparationEvidence(expectedMode) {
  assert.equal(engine.sandbox_fixed_geometry_mode(), expectedMode);
  return {
    mode: expectedMode,
    prepared_body_count: engine.sandbox_fixed_geometry_prepared_count(),
    total_preparations: engine.sandbox_fixed_geometry_total_preparations(),
    retained_bytes: engine.sandbox_fixed_geometry_retained_bytes(),
    representation_version: engine.sandbox_fixed_geometry_representation_version(),
  };
}

function reset(mode) {
  const started = performance.now();
  assert.equal(
    engine.sandbox_reset_with_baking_options(0, 0, mode),
    0,
    `failed to reset fixed geometry mode ${mode}`,
  );
  const startupMs = performance.now() - started;
  const evidence = preparationEvidence(mode);
  if (mode === 0) {
    assert.deepEqual(
      {
        prepared_body_count: evidence.prepared_body_count,
        total_preparations: evidence.total_preparations,
        retained_bytes: evidence.retained_bytes,
      },
      { prepared_body_count: 0, total_preparations: 0, retained_bytes: 0 },
      "runtime reference unexpectedly retained prepared geometry",
    );
  } else {
    assert.equal(evidence.prepared_body_count, 11, "unexpected fixed preparation count");
    assert.equal(evidence.total_preparations, 11, "fixed geometry was prepared more than once");
    assert.ok(evidence.retained_bytes > 0, "prepared geometry retained no observable bytes");
    assert.equal(evidence.representation_version, 1, "unexpected preparation representation");
  }
  return { startup_ms: startupMs, ...evidence };
}

function settle(mode) {
  const startup = reset(mode);
  for (let tick = 0; tick < 240; tick += 1) step(0, 0);
  assert.equal(engine.sandbox_is_quiescent(), 1, "initial scene did not settle");
  const settledPreparation = preparationEvidence(mode);
  assert.deepEqual(
    settledPreparation,
    {
      mode: startup.mode,
      prepared_body_count: startup.prepared_body_count,
      total_preparations: startup.total_preparations,
      retained_bytes: startup.retained_bytes,
      representation_version: startup.representation_version,
    },
    "sleeping dynamics changed the genuine fixed preparation set",
  );
  return startup;
}

function runCase(mode, name) {
  const startup = settle(mode);
  const trace = createHash("sha256");
  const times = [];
  const events = [];
  for (let tick = 0; tick < 180; tick += 1) {
    if (name.startsWith("three-") && [0, 40, 80].includes(tick)) {
      assert.ok(
        engine.sandbox_shoot(tick === 40 ? -38 : 38, tick === 80 ? -7 : 0, -88) >= 0,
        "projectile creation failed",
      );
    }
    const moving = name === "walking-no-shots" || name === "three-shots-walking";
    const x = moving && tick >= 80 ? Math.round(420 * Math.sin(0.7)) : 0;
    const z = moving ? (tick >= 80 ? -Math.round(420 * Math.cos(0.7)) : -420) : 0;
    const started = performance.now();
    step(x, z);
    times.push(performance.now() - started);

    const length = snapshot(trace);
    const contacts = engine.sandbox_last_collision_events();
    events.push(contacts);
    trace.update(
      JSON.stringify([
        tick,
        length,
        engine.sandbox_body_count(),
        engine.sandbox_grounded(),
        engine.sandbox_is_quiescent(),
        contacts,
        engine.sandbox_total_collisions(),
      ]),
    );
  }
  return {
    startup,
    steps: timingStats(times),
    replay_sha256: trace.digest("hex"),
    body_count: engine.sandbox_body_count(),
    event_sum: events.reduce((sum, value) => sum + value, 0),
    final_preparation: preparationEvidence(mode),
    raw: { steps_ms: times, events },
  };
}

const result = {
  workload: "sandbox-projectiles-v1-fixed-geometry-comparison",
  note: "Same deterministic sandbox-projectiles-v1 inputs in runtime and prepare-at-load modes. Replay equality is required; Node/V8 WASM timings and startup cost are advisory, not browser FPS or a pass/fail performance threshold.",
  wasm_sha256: hash(bytes),
  environment: {
    node: process.version,
    v8: process.versions.v8,
    platform: platform(),
    arch: arch(),
    cpu: cpus()[0]?.model,
  },
  modes: [],
};

try {
  for (const mode of modes) {
    const cases = [];
    for (const name of workloadCases) {
      const trials = [];
      for (let trial = 0; trial < trialsPerMode; trial += 1) {
        trials.push(runCase(mode.value, name));
      }
      assert.equal(
        trials[0].replay_sha256,
        trials[1].replay_sha256,
        `${mode.name} replay changed between identical trials for ${name}`,
      );
      cases.push({ name, trials });
    }
    result.modes.push({ name: mode.name, value: mode.value, cases });
  }

  const runtime = result.modes.find((mode) => mode.value === 0);
  const prepared = result.modes.find((mode) => mode.value === 1);
  for (let index = 0; index < workloadCases.length; index += 1) {
    const runtimeCase = runtime.cases[index];
    const preparedCase = prepared.cases[index];
    for (let trial = 0; trial < trialsPerMode; trial += 1) {
      const reference = runtimeCase.trials[trial];
      const candidate = preparedCase.trials[trial];
      assert.equal(
        candidate.replay_sha256,
        reference.replay_sha256,
        `prepare-at-load changed observable replay for ${runtimeCase.name}`,
      );
      assert.equal(candidate.event_sum, reference.event_sum, `event count changed for ${runtimeCase.name}`);
      assert.equal(candidate.body_count, reference.body_count, `body count changed for ${runtimeCase.name}`);
    }
  }
  result.replay_matches = true;
} catch (error) {
  result.failure = String(error);
  throw error;
} finally {
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
}

for (const mode of result.modes) {
  for (const entry of mode.cases) {
    console.log(
      mode.name,
      entry.name,
      entry.trials.map(({ raw, ...summary }) => summary),
    );
  }
}
