import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";

const [wasmPath] = process.argv.slice(2);
assert(wasmPath, "usage: node scripts/test-focused-scenarios.mjs <production.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(wasmPath), {});
const engine = instance.exports;
function snapshot() {
  const pointer = engine.sandbox_refresh_render_snapshot();
  const length = engine.sandbox_render_snapshot_len();
  assert.equal(engine.sandbox_render_snapshot_stride(), 11);
  assert(length > 0 && length % 11 === 0);
  return Buffer.from(engine.memory.buffer, pointer, length * 4);
}
for (let scenario = 1; scenario <= 6; scenario += 1) {
  for (const linear of [0, 1]) {
    let reference;
    for (let trial = 0; trial < 2; trial += 1) {
      assert.equal(engine.sandbox_reset_scenario_with_baking_options(scenario, linear, linear, 1), 0);
      const count = engine.sandbox_body_count();
      const beforeInvalidReset = createHash("sha256").update(snapshot()).digest("hex");
      assert.equal(engine.sandbox_reset_scenario_with_baking_options(7, linear, linear, 1), -1);
      assert.equal(createHash("sha256").update(snapshot()).digest("hex"), beforeInvalidReset);
      if (scenario === 5) {
        assert.equal(engine.sandbox_aim_query(0, 0, -96), 30);
        assert.equal(engine.sandbox_aim_query(0, 0, 0), -1);
      } else {
        assert.equal(engine.sandbox_aim_query(0, 0, -96), -1);
      }
      const trace = createHash("sha256");
      for (let tick = 0; tick < 600; tick += 1) {
        const error = engine.sandbox_step_velocity(0, 0, 0);
        assert.equal(error, 0, `scenario=${scenario} linear=${linear} tick=${tick} detail=${engine.sandbox_error_detail()}`);
        assert.equal(engine.sandbox_body_count(), count);
        if (tick % 60 === 59) {
          trace.update(snapshot());
          trace.update(JSON.stringify([engine.sandbox_is_quiescent(), engine.sandbox_total_collisions()]));
        }
      }
      if (scenario === 4) assert.equal(engine.sandbox_is_quiescent(), 1);
      const replay = trace.digest("hex");
      if (trial === 0) reference = replay;
      else assert.equal(replay, reference, "focused fixture replay differs within this build");
    }
    console.log(JSON.stringify({ scenario, linear, ticks: 600, trials: 2, replay: reference, passed: true }));
  }
}
