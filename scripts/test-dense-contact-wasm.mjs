import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
const path = process.argv[2];
assert(path, "usage: node scripts/test-dense-contact-wasm.mjs <dense-contact-contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
const fields = [
  "peak_penetration", "peak_speed", "peak_angular_speed", "peak_energy",
  "tail_speed", "tail_angular_speed", "support_departure", "sleep_transitions",
  "wake_transitions", "sleeping", "last_sleep_seconds", "contact_points",
  "constraint_visits", "position_tests", "active_substeps",
];
assert(Number.isNaN(instance.exports.narrow_support_metric(0, 0, 0)));
let previous;
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.dense_contact_contract(), 0, `dense contact WASM replay ${replay}`);
  const rows = ["Balanced", "OffCenter", "Removal"].map((name, index) => {
    const result = { case: name };
    for (const [cadence, key] of ["ticks", "physical_substeps"].entries()) {
      result[key] = Object.fromEntries(fields.map((field, i) => {
        const value = instance.exports.narrow_support_metric(index, cadence, i);
        assert(Number.isFinite(value), `${name}/${key}/${field}`);
        return [field, value];
      }));
      assert(result[key].peak_penetration <= 0.5);
      assert.equal(result[key].sleeping, 1);
      assert.equal(result[key].tail_speed, 0);
      assert.equal(result[key].tail_angular_speed, 0);
    }
    return result;
  });
  if (previous) assert.deepEqual(rows, previous, "same-target measured replay");
  previous = rows;
  assert(Number.isNaN(instance.exports.narrow_support_metric(3, 0, 0)));
  assert(Number.isNaN(instance.exports.narrow_support_metric(0, 2, 0)));
  assert(Number.isNaN(instance.exports.narrow_support_metric(0, 0, fields.length)));
  console.log(`NARROW_SUPPORT_WASM ${JSON.stringify({ replay, rows })}`);
}
console.log("Dense contact WASM: three replays passed, including physical substeps, materials and narrow supports.");
