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
assert(Number.isNaN(instance.exports.primitive_contact_metric(0)));
assert(Number.isNaN(instance.exports.moving_support_metric(0, 0, 0)));
let previous;
let previousPrimitive;
let previousMoving;
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
  const primitive = Object.fromEntries([
    "peak_overlap", "residual_overlap", "normalized_velocity_error",
    "normalized_momentum_response_error", "energy_ratio", "cases",
    "continuous_contact_samples", "constraint_visits", "min_elapsed", "max_elapsed",
  ].map((field, i) => {
    const value = instance.exports.primitive_contact_metric(i);
    assert(Number.isFinite(value), field);
    return [field, value];
  }));
  assert(primitive.peak_overlap <= 0.02 + 1e-10);
  assert(primitive.residual_overlap <= 0.02 + 1e-10);
  assert(primitive.normalized_velocity_error <= 1e-10);
  assert(primitive.normalized_momentum_response_error <= 1e-10);
  assert(primitive.energy_ratio <= 1 + 1e-10);
  assert.equal(primitive.cases, 384);
  assert.equal(primitive.continuous_contact_samples, 8192);
  assert(Math.abs(primitive.min_elapsed - 64 / 240) <= 1e-12);
  assert(Math.abs(primitive.max_elapsed - 64 / 240) <= 1e-12);
  if (previousPrimitive) assert.deepEqual(primitive, previousPrimitive);
  previousPrimitive = primitive;
  assert(Number.isNaN(instance.exports.primitive_contact_metric(10)));
  console.log(`PRIMITIVE_CONTACT_WASM ${JSON.stringify({ replay, primitive })}`);
  const movingFields = [
    "peak_penetration", "peak_speed", "peak_energy", "external_work",
    "max_motion_error", "peak_departure_gap", "max_ballistic_error",
    "support_samples", "departure_samples", "contact_points", "constraint_visits",
    "woken_bodies", "sleep_transitions", "wake_transitions", "final_y", "final_vy",
    "sleeping", "total_constraint_visits", "elapsed",
  ];
  const moving = ["Carry", "Carry", "Departure", "Departure", "Removal", "Removal"].map((name, index) => {
    const row = { case: name, ids: index % 2 === 0 ? [1, 2] : [2, 1] };
    for (const [cadence, key] of ["ticks", "physical_substeps"].entries()) {
      const m = Object.fromEntries(movingFields.map((field, i) => {
        const value = instance.exports.moving_support_metric(index, cadence, i);
        assert(Number.isFinite(value), `${name}/${key}/${field}`);
        return [field, value];
      }));
      assert(m.peak_penetration <= 0.02 + 1e-9);
      assert(m.max_motion_error <= 1e-9);
      assert(m.max_ballistic_error <= 1e-9);
      assert(Math.abs(m.external_work - 8) <= 1e-9);
      assert(Math.abs(m.elapsed - 4) <= 1e-9);
      assert.equal(m.sleeping, 0);
      assert.equal(m.woken_bodies, 0);
      assert.equal(m.sleep_transitions, 0);
      assert.equal(m.wake_transitions, 0);
      if (name === "Carry") {
        assert.equal(m.support_samples, cadence === 0 ? 240 : 960);
        assert.equal(m.departure_samples, 0);
      } else if (name === "Departure") {
        assert(m.departure_samples > 0);
        assert(m.peak_departure_gap > 0.3);
        assert(Math.abs(m.final_y - 1.5) <= 1e-9);
        assert(Math.abs(m.final_vy) <= 1e-9);
        assert(Math.abs(m.peak_energy - 13) <= 1e-9);
      } else {
        assert.equal(m.support_samples, cadence === 0 ? 120 : 480);
        assert.equal(m.departure_samples, cadence === 0 ? 120 : 480);
        assert(Math.abs(m.final_y + 18.541666666666666) <= 1e-9);
        assert(Math.abs(m.final_vy + 20) <= 1e-9);
      }
      row[key] = m;
    }
    return row;
  });
  if (previousMoving) assert.deepEqual(moving, previousMoving);
  previousMoving = moving;
  assert(Number.isNaN(instance.exports.moving_support_metric(6, 0, 0)));
  assert(Number.isNaN(instance.exports.moving_support_metric(0, 2, 0)));
  assert(Number.isNaN(instance.exports.moving_support_metric(0, 0, movingFields.length)));
  console.log(`MOVING_SUPPORT_WASM ${JSON.stringify({ replay, rows: moving })}`);
  assert(Number.isNaN(instance.exports.narrow_support_metric(3, 0, 0)));
  assert(Number.isNaN(instance.exports.narrow_support_metric(0, 2, 0)));
  assert(Number.isNaN(instance.exports.narrow_support_metric(0, 0, fields.length)));
  console.log(`NARROW_SUPPORT_WASM ${JSON.stringify({ replay, rows })}`);
}
console.log("Dense contact WASM: three replays passed, including physical substeps, materials, narrow supports, axial primitive pairs and moving supports.");
