import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { arch, cpus, platform } from "node:os";

const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath || !outputPath) {
  throw new Error(
    "usage: node scripts/benchmark-projectile-policies.mjs <head.wasm> <results.json>",
  );
}

const EXPLICIT_RULES_BIT = 1 << 29;
const PROJECTILE_POLICY_SHIFT = 12;
const PROJECTILE_POLICY_EXPLICIT_BIT = 1 << 14;
const PROJECTILE_PROJECTILE_BIT = 1 << 10;
const ALL_PAIR_BITS = (1 << 11) - 2;
const TOTAL_TICKS = 180;
const TRIALS = 2;

const workCounters = new Map([
  ["sampled_events", "sandbox_last_sampled_events"],
  ["tail_contacts", "sandbox_last_tail_contacts"],
  ["tail_slices", "sandbox_last_tail_slices"],
  ["tail_candidate_pairs", "sandbox_last_tail_candidate_pairs"],
  ["tail_broad_phase_queries", "sandbox_last_tail_broad_phase_queries"],
  ["broad_phase_queries", "sandbox_last_broad_phase_queries"],
  ["broad_phase_partial_queries", "sandbox_last_broad_phase_partial_queries"],
  ["broad_phase_partial_body_updates", "sandbox_last_broad_phase_partial_body_updates"],
  ["event_response_passes", "sandbox_last_event_response_passes"],
  ["stabilization_passes", "sandbox_last_stabilization_passes"],
  ["stabilizations_hitting_limit", "sandbox_last_stabilizations_hitting_limit"],
  ["stabilization_candidate_pairs", "sandbox_last_stabilization_candidate_pairs"],
  ["stabilization_exact_contacts", "sandbox_last_stabilization_exact_contacts"],
  ["stabilization_active_bodies", "sandbox_last_stabilization_active_bodies"],
]);

const requiredExports = [
  "memory",
  "sandbox_reset_with_baking_options",
  "sandbox_simulation_rules",
  "sandbox_step_velocity",
  "sandbox_shoot",
  "sandbox_error_detail",
  "sandbox_is_quiescent",
  "sandbox_refresh_render_snapshot",
  "sandbox_render_snapshot_len",
  "sandbox_render_snapshot_stride",
  "sandbox_body_count",
  "sandbox_projectile_count",
  "sandbox_projectiles_retired_on_contact",
  "sandbox_projectiles_retired_out_of_bounds",
  "sandbox_projectiles_evicted_by_cap",
  "sandbox_last_collision_events",
  ...workCounters.values(),
];

// Keep the dense lifecycle comparison inside the engine's existing 32-event-per-step contract by
// disabling projectile/projectile contacts for every response policy. A separate smaller probe below
// exercises that collision-pair axis without turning the evidence workload into an unsupported swarm.
const scenarios = [
  {
    name: "physical-dense-no-projectile-pair",
    policyCode: 0,
    projectileProjectile: false,
    shotCount: 36,
    shotInterval: 2,
    shotPattern: [[0, 0, -88]],
    group: "dense",
  },
  {
    name: "inelastic-dense-no-projectile-pair",
    policyCode: 1,
    projectileProjectile: false,
    shotCount: 36,
    shotInterval: 2,
    shotPattern: [[0, 0, -88]],
    group: "dense",
  },
  {
    name: "impact-retire-dense-no-projectile-pair",
    policyCode: 2,
    projectileProjectile: false,
    shotCount: 36,
    shotInterval: 2,
    shotPattern: [[0, 0, -88]],
    group: "dense",
  },
  {
    name: "impact-retire-pair-probe-all-pairs",
    policyCode: 2,
    projectileProjectile: true,
    shotCount: 8,
    shotInterval: 1,
    shotPattern: [
      [0, 0, -88],
      [0, 0, -112],
    ],
    group: "pair-probe",
  },
  {
    name: "impact-retire-pair-probe-no-projectile-pair",
    policyCode: 2,
    projectileProjectile: false,
    shotCount: 8,
    shotInterval: 1,
    shotPattern: [
      [0, 0, -88],
      [0, 0, -112],
    ],
    group: "pair-probe",
  },
];

function encodeRules(policyCode, projectileProjectile) {
  let pairs = ALL_PAIR_BITS;
  if (!projectileProjectile) pairs &= ~PROJECTILE_PROJECTILE_BIT;
  return (
    EXPLICIT_RULES_BIT |
    PROJECTILE_POLICY_EXPLICIT_BIT |
    pairs |
    (policyCode << PROJECTILE_POLICY_SHIFT)
  );
}

function stats(values) {
  const sorted = [...values].sort((left, right) => left - right);
  const percentile = (p) => sorted[Math.max(0, Math.ceil(sorted.length * p) - 1)];
  return {
    count: values.length,
    mean_ms: values.reduce((sum, value) => sum + value, 0) / values.length,
    p50_ms: percentile(0.5),
    p95_ms: percentile(0.95),
    max_ms: sorted.at(-1),
  };
}

function addWork(total, current) {
  for (const key of workCounters.keys()) total[key] += current[key];
}

function readWork(engine) {
  return Object.fromEntries(
    [...workCounters].map(([key, exportName]) => [key, Number(engine[exportName]())]),
  );
}

function emptyWork() {
  return Object.fromEntries([...workCounters.keys()].map((key) => [key, 0]));
}

function step(engine, x = 0, z = 0) {
  const error = engine.sandbox_step_velocity(x, z, 0);
  if (error !== 0) {
    throw new Error(`physics error ${error}, detail ${engine.sandbox_error_detail()}`);
  }
}

function settle(engine, encodedRules) {
  const reset = engine.sandbox_reset_with_baking_options(encodedRules, 0, 0);
  if (reset !== 0) throw new Error(`scenario reset failed with ${reset}`);
  if (engine.sandbox_simulation_rules() !== encodedRules) {
    throw new Error(
      `scenario-rules round trip mismatch: expected ${encodedRules}, got ${engine.sandbox_simulation_rules()}`,
    );
  }
  for (let tick = 0; tick < 240; tick += 1) step(engine);
  if (engine.sandbox_is_quiescent() !== 1) throw new Error("scenario did not settle");
}

function updateTrace(engine, trace, tick) {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  const stride = engine.sandbox_render_snapshot_stride();
  if (stride !== 11 || length <= 0 || length % stride !== 0) {
    throw new Error(`invalid render snapshot length=${length} stride=${stride}`);
  }
  trace.update(Buffer.from(engine.memory.buffer, pointer, length * Int32Array.BYTES_PER_ELEMENT));
  trace.update(
    JSON.stringify([
      tick,
      engine.sandbox_body_count(),
      engine.sandbox_projectile_count(),
      engine.sandbox_projectiles_retired_on_contact(),
      engine.sandbox_projectiles_retired_out_of_bounds(),
      engine.sandbox_projectiles_evicted_by_cap(),
      engine.sandbox_last_collision_events(),
    ]),
  );
}

function runTrial(engine, scenario) {
  const encodedRules = encodeRules(scenario.policyCode, scenario.projectileProjectile);
  settle(engine, encodedRules);

  const times = [];
  const work = emptyWork();
  const trace = createHash("sha256");
  let peakProjectileCount = Number(engine.sandbox_projectile_count());
  let peakBodyCount = Number(engine.sandbox_body_count());
  let eventSum = 0;

  for (let tick = 0; tick < TOTAL_TICKS; tick += 1) {
    if (tick < scenario.shotCount * scenario.shotInterval && tick % scenario.shotInterval === 0) {
      const shotIndex = tick / scenario.shotInterval;
      const [x, y, z] = scenario.shotPattern[shotIndex % scenario.shotPattern.length];
      if (engine.sandbox_shoot(x, y, z) < 0) {
        throw new Error(`${scenario.name} projectile creation failed at shot ${shotIndex}`);
      }
      peakProjectileCount = Math.max(
        peakProjectileCount,
        Number(engine.sandbox_projectile_count()),
      );
      peakBodyCount = Math.max(peakBodyCount, Number(engine.sandbox_body_count()));
    }

    const started = performance.now();
    try {
      step(engine);
    } catch (error) {
      throw new Error(`${scenario.name} tick ${tick}: ${error.message}`, { cause: error });
    }
    times.push(performance.now() - started);

    addWork(work, readWork(engine));
    eventSum += Number(engine.sandbox_last_collision_events());
    peakProjectileCount = Math.max(
      peakProjectileCount,
      Number(engine.sandbox_projectile_count()),
    );
    peakBodyCount = Math.max(peakBodyCount, Number(engine.sandbox_body_count()));
    updateTrace(engine, trace, tick);
  }

  return {
    encoded_rules: encodedRules,
    replay_sha256: trace.digest("hex"),
    steps: stats(times),
    event_sum: eventSum,
    peak_body_count: peakBodyCount,
    peak_projectile_count: peakProjectileCount,
    final_body_count: Number(engine.sandbox_body_count()),
    final_projectile_count: Number(engine.sandbox_projectile_count()),
    lifecycle: {
      retired_on_contact: Number(engine.sandbox_projectiles_retired_on_contact()),
      retired_out_of_bounds: Number(engine.sandbox_projectiles_retired_out_of_bounds()),
      evicted_by_cap: Number(engine.sandbox_projectiles_evicted_by_cap()),
    },
    work,
  };
}

function deterministicTrial(scenario, trials) {
  const first = trials[0];
  for (const trial of trials.slice(1)) {
    if (trial.replay_sha256 !== first.replay_sha256) {
      throw new Error(`nondeterministic replay for ${scenario.name}`);
    }
    if (
      JSON.stringify({
        event_sum: trial.event_sum,
        peak_body_count: trial.peak_body_count,
        peak_projectile_count: trial.peak_projectile_count,
        final_body_count: trial.final_body_count,
        final_projectile_count: trial.final_projectile_count,
        lifecycle: trial.lifecycle,
        work: trial.work,
      }) !==
      JSON.stringify({
        event_sum: first.event_sum,
        peak_body_count: first.peak_body_count,
        peak_projectile_count: first.peak_projectile_count,
        final_body_count: first.final_body_count,
        final_projectile_count: first.final_projectile_count,
        lifecycle: first.lifecycle,
        work: first.work,
      })
    ) {
      throw new Error(`nondeterministic work evidence for ${scenario.name}`);
    }
  }
  return first;
}

const bytes = readFileSync(wasmPath);
const { instance } = await WebAssembly.instantiate(bytes, {});
const engine = instance.exports;
for (const exportName of requiredExports) {
  if (!(exportName in engine)) throw new Error(`missing WASM export ${exportName}`);
}

const result = {
  workload: "dense-projectile-policy-matrix-v2",
  note: "Deterministic policy evidence split into a 50+ body lifecycle matrix with projectile/projectile contacts disabled and a smaller bounded collision-pair probe. Work/lifecycle counters are authoritative; Node/V8 WASM timings are advisory and are not CI wall-clock budgets.",
  environment: {
    node: process.version,
    v8: process.versions.v8,
    platform: platform(),
    arch: arch(),
    cpu: cpus()[0]?.model,
  },
  wasm_sha256: createHash("sha256").update(bytes).digest("hex"),
  head_revision: process.env.HEAD_SHA ?? null,
  scenarios: [],
};

try {
  for (const scenario of scenarios) {
    const trials = [];
    for (let trial = 0; trial < TRIALS; trial += 1) {
      trials.push(runTrial(engine, scenario));
    }
    const exact = deterministicTrial(scenario, trials);
    result.scenarios.push({ ...scenario, trials });
    console.log(scenario.name, {
      peak_body_count: exact.peak_body_count,
      final_projectile_count: exact.final_projectile_count,
      lifecycle: exact.lifecycle,
      event_sum: exact.event_sum,
      work: exact.work,
      steps: exact.steps,
    });
  }

  const byName = new Map(
    result.scenarios.map((scenario) => [
      scenario.name,
      deterministicTrial(scenario, scenario.trials),
    ]),
  );
  const physical = byName.get("physical-dense-no-projectile-pair");
  const inelastic = byName.get("inelastic-dense-no-projectile-pair");
  const retire = byName.get("impact-retire-dense-no-projectile-pair");
  const pairAll = byName.get("impact-retire-pair-probe-all-pairs");
  const pairFiltered = byName.get("impact-retire-pair-probe-no-projectile-pair");

  for (const [name, trial] of [
    ["physical", physical],
    ["inelastic", inelastic],
  ]) {
    if (trial.peak_body_count < 50) {
      throw new Error(`dense ${name} workload never reached 50 bodies: ${trial.peak_body_count}`);
    }
    if (trial.lifecycle.retired_on_contact !== 0) {
      throw new Error(`${name} policy retired a projectile on contact`);
    }
    if (trial.lifecycle.evicted_by_cap !== 0) {
      throw new Error(`${name} dense workload reached the projectile cap unexpectedly`);
    }
  }
  if (retire.lifecycle.retired_on_contact <= 0) {
    throw new Error("impact-retire dense workload did not retire any projectile on contact");
  }
  if (retire.final_projectile_count >= physical.final_projectile_count) {
    throw new Error("impact-retire did not reduce retained projectile population");
  }
  if (retire.lifecycle.evicted_by_cap !== 0) {
    throw new Error("impact-retire dense workload reached the projectile cap unexpectedly");
  }
  if ((pairAll.encoded_rules & PROJECTILE_PROJECTILE_BIT) === 0) {
    throw new Error("all-pairs probe did not enable projectile/projectile collisions");
  }
  if ((pairFiltered.encoded_rules & PROJECTILE_PROJECTILE_BIT) !== 0) {
    throw new Error("filtered probe did not disable projectile/projectile collisions");
  }

  result.comparison = {
    impact_retire_vs_physical_dense: {
      final_projectile_delta: retire.final_projectile_count - physical.final_projectile_count,
      event_sum_delta: retire.event_sum - physical.event_sum,
      stabilization_pass_delta:
        retire.work.stabilization_passes - physical.work.stabilization_passes,
      stabilization_active_body_delta:
        retire.work.stabilization_active_bodies - physical.work.stabilization_active_bodies,
      stabilization_exact_contact_delta:
        retire.work.stabilization_exact_contacts - physical.work.stabilization_exact_contacts,
    },
    inelastic_vs_physical_dense: {
      event_sum_delta: inelastic.event_sum - physical.event_sum,
      stabilization_pass_delta:
        inelastic.work.stabilization_passes - physical.work.stabilization_passes,
      stabilization_active_body_delta:
        inelastic.work.stabilization_active_bodies - physical.work.stabilization_active_bodies,
      stabilization_exact_contact_delta:
        inelastic.work.stabilization_exact_contacts - physical.work.stabilization_exact_contacts,
    },
    projectile_pair_probe: {
      event_sum_delta: pairFiltered.event_sum - pairAll.event_sum,
      retired_on_contact_delta:
        pairFiltered.lifecycle.retired_on_contact - pairAll.lifecycle.retired_on_contact,
      stabilization_pass_delta:
        pairFiltered.work.stabilization_passes - pairAll.work.stabilization_passes,
      stabilization_active_body_delta:
        pairFiltered.work.stabilization_active_bodies - pairAll.work.stabilization_active_bodies,
      stabilization_exact_contact_delta:
        pairFiltered.work.stabilization_exact_contacts - pairAll.work.stabilization_exact_contacts,
    },
  };
} catch (error) {
  result.failure = String(error);
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
  throw error;
}

writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
