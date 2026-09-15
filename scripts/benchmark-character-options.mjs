import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFileSync, writeFileSync } from "node:fs";
import { cpus } from "node:os";

const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath || !outputPath) throw new Error("usage: node scripts/benchmark-character-options.mjs <module.wasm> <results.json>");
const bytes = readFileSync(wasmPath);
const { instance: { exports: engine } } = await WebAssembly.instantiate(bytes, {});
const result = {
  workload: "character-options-v1",
  note: "Gameplay comparison: these modes intentionally differ, not equivalent solver optimizations. WASM-only timings are advisory, not browser FPS.",
  wasm_sha256: createHash("sha256").update(bytes).digest("hex"),
  environment: { node: process.version, v8: process.versions.v8, cpu: cpus()[0]?.model },
  cases: [],
};
function step(x, z, jump = false) {
  const error = engine.sandbox_step_velocity(x, z, Number(jump));
  assert.equal(error, 0, `physics error ${error}, detail ${engine.sandbox_error_detail()}`);
}
function snapshot() {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  assert.equal(engine.sandbox_render_snapshot_stride(), 11);
  return Buffer.from(new Uint8Array(engine.memory.buffer, pointer, length * 4));
}
function stats(times) {
  const sorted = [...times].sort((a, b) => a - b);
  return { mean_ms: times.reduce((a, b) => a + b, 0) / times.length, p95_ms: sorted[Math.ceil(sorted.length * 0.95) - 1], max_ms: sorted.at(-1) };
}
try {
  for (const character of [0, 1]) {
    for (const upright of [0, 1]) {
      for (const scenario of ["walking", "stack-edge-landing"]) {
        let expected;
        const trials = [];
        for (let trial = 0; trial < 2; trial += 1) {
          assert.equal(engine.sandbox_reset_with_options(character, upright), 0);
          for (let tick = 0; tick < 240; tick += 1) step(0, 0);
          assert.equal(engine.sandbox_is_quiescent(), 1);
          const trace = createHash("sha256");
          const times = [];
          for (let tick = 0; tick < 240; tick += 1) {
            const landing = scenario === "stack-edge-landing";
            const x = landing ? (tick < 13 ? -420 : 0) : (tick >= 80 && tick < 180 ? Math.round(420 * Math.sin(0.7)) : 0);
            const z = landing ? (tick >= 16 && tick < 46 ? -420 : 0) : (tick < 80 ? -420 : tick < 180 ? -Math.round(420 * Math.cos(0.7)) : 0);
            const start = performance.now();
            step(x, z, landing && tick === 27);
            times.push(performance.now() - start);
            trace.update(snapshot());
            trace.update(JSON.stringify([tick, engine.sandbox_grounded(), engine.sandbox_is_quiescent(), engine.sandbox_total_collisions()]));
          }
          const replay_sha256 = trace.digest("hex");
          if (expected) assert.equal(replay_sha256, expected, "same-mode replay changed");
          expected = replay_sha256;
          trials.push({ ...stats(times), replay_sha256, quiescent: engine.sandbox_is_quiescent() === 1, raw_steps_ms: times });
        }
        result.cases.push({ character: character ? "linear" : "physical", crates: upright ? "upright" : "free", scenario, trials });
        console.log(result.cases.at(-1).character, result.cases.at(-1).crates, scenario, trials.map(({ raw_steps_ms, ...summary }) => summary));
      }
    }
  }
} catch (error) {
  result.failure = String(error);
  throw error;
} finally {
  writeFileSync(outputPath, `${JSON.stringify(result, null, 2)}\n`);
}
