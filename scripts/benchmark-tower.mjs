// Exercise the real Pages reset/projectile API, not a reduced hand-built physics fixture.
// Timings are advisory. A replay failure is always recorded and is an exit failure by default.
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";

const [wasmPath, outputPath, option] = process.argv.slice(2);
if (!wasmPath || !outputPath || (option && option !== "--report-only")) {
  throw new Error("usage: node scripts/benchmark-tower.mjs <engine.wasm> <report.json> [--report-only]");
}
const trials = Number(process.env.TRIALS ?? 2);
if (!Number.isInteger(trials) || trials < 1 || trials > 5) {
  throw new Error("TRIALS must be an integer from 1 through 5");
}
const bytes = readFileSync(wasmPath);
const module = await WebAssembly.compile(bytes);
const measuredTicks = 120;
const warmupTicks = 240;
const counterNames = ["sampled_events", "event_response_passes", "stabilization_passes", "stabilizations_hitting_limit"];
const cases = [];
for (const upright of [false, true]) {
  for (const linear of [false, true]) {
    for (const [projectile, type] of [["sphere", 0], ["arrow", 1], ["rigid", 2]]) {
      const runs = [];
      for (let trial = 0; trial < trials; trial += 1) {
        const { exports: engine } = await WebAssembly.instantiate(module, {});
        const rules = (1 << 29) | ((1 << 11) - 2) | (1 << 14) | (2 << 12)
          | (upright ? 1 << 11 : 0) | (linear ? 1 : 0);
        if (engine.sandbox_reset_tower_with_baking_options(rules, 0, 1) !== 0) throw new Error("tower reset rejected");
        if (engine.sandbox_body_count() !== 44) throw new Error("tower fixture must contain 32 crates, player and 11 fixed bodies");
        const snapshot = () => {
          const pointer = engine.sandbox_refresh_render_snapshot();
          const length = engine.sandbox_render_snapshot_len();
          if (engine.sandbox_render_snapshot_stride() !== 11 || length % 11) throw new Error("invalid render snapshot");
          return new Int32Array(engine.memory.buffer, pointer, length).slice();
        };
        const trace = createHash("sha256");
        const steps = [];
        const peakWork = Object.fromEntries(counterNames.map((name) => [name, 0]));
        let failure = null;
        const step = (phase, tick) => {
          const start = performance.now();
          const error = engine.sandbox_step_velocity(0, 0, 0);
          const durationMs = performance.now() - start;
          const detail = error ? engine.sandbox_error_detail() : 0;
          const state = snapshot();
          trace.update(Buffer.from(state.buffer));
          trace.update(JSON.stringify([phase, tick, error, detail, engine.sandbox_is_quiescent()]));
          if (phase === "impact") {
            steps.push(durationMs);
            if (!error) {
              for (const name of counterNames) peakWork[name] = Math.max(peakWork[name], engine[`sandbox_last_${name}`]());
            }
          }
          if (error) failure = { phase, tick, error, detail, duration_ms: durationMs };
          return !error;
        };
        for (let tick = 0; tick < warmupTicks && !failure; tick += 1) step("settle", tick);
        const settledBeforeImpact = engine.sandbox_is_quiescent() === 1;
        const initial = snapshot();
        if (!failure && !settledBeforeImpact) failure = { phase: "settle", message: "tower did not settle" };
        if (!failure) {
          if (engine.sandbox_set_projectile_type(type) !== 0 || engine.sandbox_shoot(0, 0, -96) < 0) {
            throw new Error("projectile creation rejected");
          }
          for (let tick = 0; tick < measuredTicks && !failure; tick += 1) step("impact", tick);
        }
        const final = snapshot();
        // Render poses are an observable trace, not a serialization of private solver state.
        const crates = (state) => {
          const result = [];
          for (let i = 0; i < state.length; i += 11) if (state[i] === 2) result.push(...state.slice(i, i + 11));
          return result;
        };
        const crateResponded = JSON.stringify(crates(initial)) !== JSON.stringify(crates(final));
        if (!failure && !crateResponded) failure = { phase: "impact", message: "projectile did not affect the crates" };
        const run = {
          trial,
          passed: failure === null,
          numerical_backend: engine.sandbox_numeric_backend?.() ?? "legacy-unreported",
          settled_before_impact: settledBeforeImpact,
          completed_impact_ticks: failure?.phase === "impact" && Number.isInteger(failure.tick) ? failure.tick : steps.length,
          crate_responded: crateResponded,
          quiescent_after: engine.sandbox_is_quiescent() === 1,
          max_step_ms: steps.length ? Math.max(...steps) : null,
          total_steps_ms: steps.reduce((sum, time) => sum + time, 0),
          peak_work: peakWork,
          failure,
          replay_sha256: trace.digest("hex"),
          raw_steps_ms: steps,
        };
        runs.push(run);
      }
      const replayRepeatable = runs.every((run) => run.replay_sha256 === runs[0].replay_sha256);
      cases.push({ projectile, crates: upright ? "upright" : "free", character: linear ? "linear" : "physical", replay_repeatable: replayRepeatable, runs });
      console.log(JSON.stringify({ projectile, upright, linear, passed: runs.every((run) => run.passed), replay_repeatable: replayRepeatable, max_ms: Math.max(...runs.map((run) => run.max_step_ms ?? 0)), failure: runs[0].failure }));
    }
  }
}
const report = {
  workload: "pages-tower-projectile-matrix-v1",
  note: "Same-build observable replay checks; host Node/V8 physics-only timings are advisory, not browser FPS. --report-only never turns a failed case into a pass.",
  node: process.version,
  wasm_sha256: createHash("sha256").update(bytes).digest("hex"),
  warmup_ticks: warmupTicks,
  requested_impact_ticks: measuredTicks,
  cases,
  passed: cases.every((entry) => entry.replay_repeatable && entry.runs.every((run) => run.passed)),
};
writeFileSync(outputPath, `${JSON.stringify(report, null, 2)}\n`);
if (!report.passed && option !== "--report-only") process.exitCode = 1;
