// Paired, real canonical-tower replay. Physics quality/equality and deterministic work are gates;
// wall-clock measurements are advisory and explicitly exclude observation/assertion work.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import os from 'node:os';
import { createTowerRuntime } from '../site/tower-runtime.mjs';
import { COLLISION_PAIRS, encodeScenarioRules } from '../site/simulation-rules-config.mjs';

const [basePath, candidatePath, output, scope = '--all'] = process.argv.slice(2);
if (!basePath || !candidatePath || !output || !['--all', '--representative'].includes(scope)) {
  throw new Error('Usage: node scripts/benchmark-tower-position.mjs base.wasm candidate.wasm output.json [--all|--representative]');
}
const trials = Number(process.env.TRIALS ?? 2), ticks = Number(process.env.TICKS ?? 1200);
assert(Number.isInteger(trials) && trials >= 1 && trials <= 9, 'TRIALS must be 1..9');
assert(Number.isInteger(ticks) && ticks >= 600 && ticks <= 3600, 'TICKS must be 600..3600');
const bytes = { base: readFileSync(basePath), candidate: readFileSync(candidatePath) };
const modules = Object.fromEntries(await Promise.all(Object.entries(bytes).map(async ([k, b]) => [k, await WebAssembly.compile(b)])));
const sha = b => createHash('sha256').update(b).digest('hex');
const positionNames = ['passes', 'bounds_tests', 'contact_tests', 'corrections', 'max_distance',
  'body_visits', 'fixed_index_rebuilds', 'index_body_scans', 'fixed_frame_preparations',
  'moving_frame_preparations', 'scratch_retained_bytes', 'scratch_growths'];
const phases = ['settling', 'active', 'sleeping'];
function summary(values) {
  const sorted = values.toSorted((a, b) => a - b);
  return { count: values.length, total_ms: values.reduce((a, b) => a + b, 0),
    mean_ms: values.length ? values.reduce((a, b) => a + b, 0) / values.length : null,
    p95_ms: sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * .95))] ?? null,
    max_ms: sorted.at(-1) ?? null };
}
function snapshot(e) {
  const p = e.sandbox_refresh_render_snapshot();
  return new Float64Array(e.memory.buffer, p, e.sandbox_render_snapshot_len()).slice();
}
function bottom(state, k) {
  const [x, y, z, w] = state.slice(k + 7, k + 11);
  return state[k + 2] - Math.abs(2 * (x*y + w*z))*state[k+4]
    - Math.abs(1 - 2*(x*x + z*z))*state[k+5] - Math.abs(2*(y*z - w*x))*state[k+6];
}
async function replay(build, config, measured = true) {
  const raw = (await WebAssembly.instantiate(modules[build], {})).exports;
  const e = createTowerRuntime(raw);
  assert.equal(e.sandbox_reset_tower_with_baking_options(encodeScenarioRules({ ...config,
    enabledPairs: new Set(COLLISION_PAIRS.map(([id]) => id)) })), 0);
  const trace = createHash('sha256');
  const times = Object.fromEntries(phases.map(p => [p, []]));
  const work = Object.fromEntries(phases.map(p => [p, Array(positionNames.length).fill(0)]));
  let peakFloor = 0, peakAwake = 0, shots = 0, changed = false, rotated = false;
  let initial;
  function step(phase) {
    const quiet = e.sandbox_is_quiescent() === 1;
    const start = performance.now();
    const code = e.sandbox_step_velocity(0, 0, 0);
    const duration = performance.now() - start;
    assert.equal(code, 0, `${build} ${JSON.stringify(config)} step failed`);
    if (!measured) return;
    phase = phase === 'settling' ? phase : quiet ? 'sleeping' : 'active';
    times[phase].push(duration);
    const state = snapshot(e), observation = [];
    assert(state.every(Number.isFinite), 'finite poses');
    trace.update(new Uint8Array(state.buffer));
    let crate = 0, awake = 0;
    for (let k = 0; k < state.length; k += 11) {
      const index = k / 11, asleep = e.bodySleeping(index);
      observation.push(asleep);
      for (let axis = 0; axis < 3; axis++) {
        const v = raw.approximate_body_velocity(index, axis);
        assert(Number.isFinite(v)); observation.push(v);
      }
      assert(Math.abs(Math.hypot(...state.slice(k + 7, k + 11)) - 1) < 1e-8);
      if (state[k] === 2) {
        if (asleep !== 1) awake++;
        if (initial) {
          const before = initial[crate];
          for (let j = 1; j < 11; j++) changed ||= Math.abs(state[k+j] - before[j]) > 1e-6;
          for (let j = 7; j < 11; j++) rotated ||= Math.abs(state[k+j] - before[j]) > 1e-5;
          peakFloor = Math.max(peakFloor, -bottom(state, k));
        }
        crate++;
      }
    }
    assert.equal(crate, 32);
    assert.equal(e.sandbox_body_count() - e.sandbox_projectile_count(), 44);
    if (initial) peakAwake = Math.max(peakAwake, awake);
    // ALL pre-existing observations/work/memory remain comparable, including position stat0..4.
    for (let n = 0; n <= 51; n++) {
      const v = raw.approximate_stat(n); assert(Number.isFinite(v)); observation.push(v);
    }
    for (let n = 0; n < positionNames.length; n++) {
      const v = raw.approximate_position_stat(n);
      if (n < 5) { assert(Number.isFinite(v)); observation.push(v); }
      if (Number.isFinite(v)) work[phase][n] = [4, 10].includes(n) ? Math.max(work[phase][n], v) : work[phase][n] + v;
      else assert(build === 'base' && n >= 5, 'candidate must expose actual position bookkeeping');
    }
    observation.push(Number(raw.approximate_total_contacts()), e.error(), e.sandbox_projectile_count(),
      raw.approximate_projectiles_out_of_bounds(), raw.approximate_projectiles_evicted());
    assert.equal(e.error(), 0);
    trace.update(new Uint8Array(new Float64Array(observation).buffer));
    const s = e.stepStats();
    assert(s.fixed_substeps <= 4 && s.fixed_impulse_iterations <= 32 && s.fixed_position_passes <= 8);
  }
  if (config.scenario !== 'immediate-hit') for (let t = 0; t < 240; t++) step('settling');
  const state = snapshot(e);
  initial = Array.from({ length: state.length/11 }, (_, i) => state.slice(i*11, i*11+11)).filter(r => r[0] === 2);
  const before = e.elapsed();
  for (let t = 0; t < (measured ? ticks : 180); t++) {
    const fire = config.scenario === 'immediate-hit' ? t === 0 : config.scenario === 'mixed-burst' ? t < 12 : t < 480 && t % 24 === 0;
    if (fire) {
      assert.equal(e.sandbox_set_projectile_type(config.scenario === 'immediate-hit' ? 1 : shots % 3), 0);
      const v = config.scenario === 'near-misses' ? [40, 0, -87] : config.scenario === 'mixed-burst' ? [((shots % 3)-1)*3, 0, -96] : [0, 0, -96];
      assert(e.sandbox_shoot(...v) > 0); shots++;
    }
    step('active');
  }
  if (!measured) return null;
  assert(Math.abs(e.elapsed()-before-ticks/60) < 1e-7, 'no discarded time');
  assert(peakFloor <= .5, `penetration ${peakFloor}`);
  if (config.scenario === 'near-misses') {
    if (config.projectileImpactPolicy === 'impact-retire') { assert(!changed); assert.equal(peakAwake, 0); }
  } else assert(changed, 'hits must affect the tower');
  if (config.crateMotion === 'upright') assert(!rotated);
  return { trace: trace.digest('hex'), shots, peakFloor, peakAwake, changed, rotated,
    phases: Object.fromEntries(phases.map(p => [p, { timing: summary(times[p]), raw_ms: times[p],
      position: Object.fromEntries(positionNames.map((name, i) => [name, build==='base' && i>=5 ? null : work[p][i]])) }])) };
}
const records = [];
for (const characterResponse of scope === '--all' ? ['physical', 'linear'] : ['physical']) {
  for (const crateMotion of ['free', 'upright']) {
    for (const projectileImpactPolicy of scope === '--all' ? ['impact-retire', 'physical', 'inelastic'] : ['impact-retire']) {
      for (const scenario of ['repeated-hits', 'near-misses', 'immediate-hit', 'mixed-burst']) {
        const config = { characterResponse, crateMotion, projectileImpactPolicy, scenario };
        for (const build of ['base', 'candidate']) await replay(build, config, false);
        const runs = [];
        for (let trial = 0; trial < trials; trial++) {
          const result = {};
          for (const build of trial % 2 ? ['candidate','base'] : ['base','candidate']) result[build] = await replay(build, config);
          assert.equal(result.base.trace, result.candidate.trace, `history mismatch ${JSON.stringify(config)}`);
          if (runs.length) assert.equal(result.base.trace, runs[0].base.trace, 'same-build replay mismatch');
          runs.push({ trial, ...result });
        }
        const timing = Object.fromEntries(phases.map(p => [p, Object.fromEntries(['base','candidate'].map(b => [b,
          summary(runs.flatMap(r => r[b].phases[p].raw_ms))]))]));
        records.push({ config, passed: true, bit_identical_history: true, timing, runs });
        console.log(JSON.stringify({ config, timing, bit_identical_history: true }));
        writeFileSync(output, JSON.stringify({ passed: false, in_progress: true, records }));
      }
    }
  }
}
writeFileSync(output, JSON.stringify({ passed: true, schema: 'tower-position-paired-v1', scope, ticks, trials,
  node: process.version, cpu: os.cpus()[0]?.model,
  hashes: Object.fromEntries(Object.entries(bytes).map(([k,b]) => [k,sha(b)])),
  note: 'Actual canonical tower. All old physics/work counters checked bitwise. Timing excludes observations; settling, active and sleeping phases are separate. Additional scratch measured separately; no timing gate.', records }, null, 2) + '\n');
