// Real Pages exports: keep an entire settled free-rotation tower unchanged through near misses.
// Native tests also compare full body state; this WASM trace covers poses, sleep and induced work.
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";

const [wasmPath, outputPath, option] = process.argv.slice(2);
if (!wasmPath || !outputPath || (option && option !== "--report-only")) {
  throw new Error("usage: node scripts/benchmark-projectile-misses.mjs <engine.wasm> <report.json> [--report-only]");
}
const bytes = readFileSync(wasmPath);
const module = await WebAssembly.compile(bytes);
const names = ["sampled_events", "event_response_passes", "stabilization_passes", "stabilizations_hitting_limit", "response_authority_body_count"];
const cases = [];
for (const linear of [false, true]) {
  for (const [projectile, type] of [["sphere", 0], ["arrow", 1], ["rigid", 2]]) {
    for (const x of [-45, 45]) {
      const runs = [];
      for (let trial = 0; trial < 2; trial += 1) {
        const { exports: e } = await WebAssembly.instantiate(module, {});
        const rules = (1 << 29) | ((1 << 11) - 2) | (1 << 14) | (2 << 12) | Number(linear);
        if (e.sandbox_reset_tower_with_baking_options(rules, 0, 1) !== 0 || e.sandbox_body_count() !== 44) throw new Error("invalid tower reset");
        for (let tick = 0; tick < 240; tick += 1) if (e.sandbox_step_velocity(0, 0, 0) !== 0) throw new Error(`settle error ${e.sandbox_error_detail()}`);
        if (e.sandbox_is_quiescent() !== 1) throw new Error("fixture did not settle");
        const crates = () => {
          const pointer = e.sandbox_refresh_render_snapshot();
          const state = new Int32Array(e.memory.buffer, pointer, e.sandbox_render_snapshot_len());
          if (e.sandbox_render_snapshot_stride() !== 11 || state.length % 11) throw new Error("invalid snapshot");
          const out = [];
          for (let i = 0; i < state.length; i += 11) if (state[i] === 2) out.push(...state.slice(i, i + 11));
          if (out.length !== 32 * 11) throw new Error("crate membership changed");
          return JSON.stringify(out);
        };
        const initial = crates();
        const hash = createHash("sha256");
        const rawMs = [];
        const peak = Object.fromEntries(names.map((name) => [name, 0]));
        const sum = { ...peak };
        const hasSleepEvidence = typeof e.sandbox_sleeping_body_count === "function";
        let firstChangedTick = null;
        let firstWokenTick = null;
        let failure = null;
        let minSleeping = hasSleepEvidence ? e.sandbox_sleeping_body_count() : null;
        if (e.sandbox_set_projectile_type(type) !== 0 || e.sandbox_shoot(x, 0, -85) < 0) throw new Error("shot rejected");
        for (let tick = 0; tick < 120; tick += 1) {
          const started = performance.now();
          const error = e.sandbox_step_velocity(0, 0, 0);
          rawMs.push(performance.now() - started);
          const state = crates();
          const sleep = e.sandbox_sleeping_body_count?.() ?? null;
          if (state !== initial && firstChangedTick === null) firstChangedTick = tick;
          if (sleep !== null) {
            minSleeping = Math.min(minSleeping, sleep);
            if (sleep < 33 && firstWokenTick === null) firstWokenTick = tick;
          }
          hash.update(JSON.stringify([tick, state, sleep, error, e.sandbox_body_count()]));
          if (error) { failure = { tick, error, detail: e.sandbox_error_detail() }; break; }
          for (const name of names) {
            const value = e[`sandbox_last_${name}`]?.() ?? 0;
            sum[name] += value;
            peak[name] = Math.max(peak[name], value);
          }
        }
        const retired = e.sandbox_body_count() === 44 && e.sandbox_is_quiescent() === 1;
        const unchanged = firstChangedTick === null;
        // Missing sleep exports on a historical binary are unknown, not a passing assertion.
        const passed = !failure && unchanged && hasSleepEvidence && firstWokenTick === null && retired;
        runs.push({ trial, passed, failure, unchanged, first_changed_tick: firstChangedTick, first_woken_tick: firstWokenTick,
          sleep_evidence_available: hasSleepEvidence, minimum_sleeping_bodies: minSleeping, projectile_retired: retired,
          work_sum: sum, peak_work: peak, replay_sha256: hash.digest("hex"), raw_steps_ms: rawMs,
          total_ms: rawMs.reduce((a, b) => a + b, 0), max_ms: Math.max(...rawMs) });
      }
      const repeatable = runs[0].replay_sha256 === runs[1].replay_sha256;
      const entry = { character: linear ? "linear" : "physical", crates: "free", projectile, direction: [x, 0, -85], replay_repeatable: repeatable, runs };
      cases.push(entry);
      console.log(JSON.stringify({ ...entry, runs: runs.map(({ raw_steps_ms, ...run }) => run) }));
    }
  }
}
const report = { workload: "pages-tower-projectile-near-misses-v1", numerical_backend_expected: "float64", node: process.version,
  wasm_sha256: createHash("sha256").update(bytes).digest("hex"), warmup_ticks: 240, measured_ticks: 120,
  note: "Host timings are advisory. Missing historical sleep telemetry is unknown; native regressions also compare full body state.",
  cases, passed: cases.every((entry) => entry.replay_repeatable && entry.runs.every((run) => run.passed)) };
writeFileSync(outputPath, `${JSON.stringify(report, null, 2)}\n`);
if (!report.passed && option !== "--report-only") process.exitCode = 1;
