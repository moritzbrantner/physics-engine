import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-translational-maintenance-wasm.mjs <contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
assert(Number.isNaN(instance.exports.fixed_bound_metric(0)));
let previous;
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.translational_maintenance_contract(), 0, `retained-storage WASM replay ${replay}`);
  const fields = ["fixed_preparations", "dynamic_preparations", "queries", "staged_bodies", "contact_resolutions", "collision_events", "ticks", "retained_payload_bytes", "all_fixed_preparations", "all_dynamic_preparations", "all_queries", "all_staged_bodies"];
  const metrics = Object.fromEntries(fields.map((field, i) => [field, instance.exports.fixed_bound_metric(i)]));
  assert(Object.values(metrics).every(Number.isFinite));
  assert.equal(metrics.fixed_preparations, 261 * metrics.queries);
  assert.equal(metrics.dynamic_preparations, 8 * metrics.queries);
  assert.equal(metrics.ticks, 24);
  if (previous) assert.deepEqual(metrics, previous);
  previous = metrics;
  assert(Number.isNaN(instance.exports.fixed_bound_metric(12)));
  console.log(`FIXED_BOUND_WASM ${JSON.stringify({ replay, metrics })}`);
}
console.log("Translational maintenance WASM: three lifecycle, release, failure and recovery controls passed.");
