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
assert(Number.isNaN(instance.exports.contact_momentum_metric(0)));
assert(Number.isNaN(instance.exports.fixed_contact_interval_metric(0)));
let previousFixedInterval;
let previousMomentum;
assert(Number.isNaN(instance.exports.primitive_contact_metric(0)));
assert(Number.isNaN(instance.exports.moving_support_metric(0, 0, 0)));
assert(Number.isNaN(instance.exports.driven_parked_metric(0, 0)));
assert(Number.isNaN(instance.exports.contact_wake_force_metric(0, 0)));
assert(Number.isNaN(instance.exports.tangential_contact_metric(0, 0)));
assert(Number.isNaN(instance.exports.stationary_support_metric(0, 0)));
let previousStationary;
let previousTangent;
let previousForces;
let previousDriven;
let previous;
let previousPrimitive;
let previousMoving;
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.dense_contact_contract(), 0, `dense contact WASM replay ${replay}`);
  const fixedInterval = Object.fromEntries([
    "peak_floor", "cases", "contact_points", "sweep_queries", "constraint_visits",
  ].map((field, i) => {
    const value = instance.exports.fixed_contact_interval_metric(i);
    assert(Number.isFinite(value), field);
    return [field, value];
  }));
  assert(fixedInterval.peak_floor <= 0.5);
  assert.equal(fixedInterval.cases, 64);
  assert(fixedInterval.contact_points > 0 && fixedInterval.sweep_queries > 0 && fixedInterval.constraint_visits > 0);
  if (previousFixedInterval) assert.deepEqual(fixedInterval, previousFixedInterval);
  previousFixedInterval = fixedInterval;
  assert(Number.isNaN(instance.exports.fixed_contact_interval_metric(5)));
  console.log(`FIXED_CONTACT_INTERVAL_WASM ${JSON.stringify({ replay, fixedInterval })}`);
  const stationaryFields = [
    "vx", "vy", "vz", "wx", "wy", "wz", "energy", "external_work", "woken_bodies",
    "response_preparations", "inertia_preparations", "inertia_applications", "current_queries",
    "sweep_queries", "contact_points", "constraint_visits", "elapsed", "max_motion_error",
    "sleeping", "independent_sleeping", "parked_at_seconds",
  ];
  const stationary = ["Stationary", "Normal", "Tangent", "DownwardDeparture",
    "HorizontalDeparture", "Removal", "NearMiss", "UnrelatedRemoval", "SharedSideContact"]
    .flatMap((name, caseIndex) => [0, 1, 2, 3].map(variant => {
      const m = Object.fromEntries(stationaryFields.map((field, i) => {
        const value = instance.exports.stationary_support_metric(4 * caseIndex + variant, i);
        assert(Number.isFinite(value), `${name}/${variant}/${field}`);
        return [field, value];
      }));
      const sleeping = name === "Stationary" || name === "NearMiss" || name === "UnrelatedRemoval";
      const substeps = variant % 2 === 0 ? 1 : 4;
      const dt = substeps / 240;
      const horizontal = name === "HorizontalDeparture";
      const fall = horizontal ? substeps - 1 : substeps;
      const falling = horizontal || name === "DownwardDeparture" || name === "Removal";
      const vx = name === "Tangent" ? 6 * dt : horizontal ? 6 / 240 : name === "SharedSideContact" ? 3 : 0;
      const vy = falling ? -10 * fall / 240 : name === "Normal" ? 3 : 0;
      const wz = name === "Tangent" ? 15 * dt : horizontal ? 15 / 240 : name === "SharedSideContact" ? -15 * dt : 0;
      assert(Math.abs(m.vx - vx) <= 1e-10);
      assert(Math.abs(m.vy - vy) <= 1e-10);
      assert(Math.abs(m.wz - wz) <= 1e-10);
      assert.equal(m.vz, 0); assert.equal(m.wx, 0); assert.equal(m.wy, 0);
      assert.equal(m.sleeping, Number(sleeping));
      assert.equal(m.woken_bodies, Number(!sleeping && name !== "Removal"));
      assert.equal(m.response_preparations, sleeping ? 0 : substeps);
      assert.equal(m.inertia_preparations, m.response_preparations);
      assert.equal(m.independent_sleeping, 1);
      assert(m.parked_at_seconds >= 0.5 - 1e-12 && m.parked_at_seconds <= 0.5 + 1 / 60 + 1e-12);
      const dy = falling ? -5 * fall * (fall + 1) / 240 ** 2 : name === "Normal" ? 3 * dt : 0;
      assert(m.energy + 20 * dy <= m.external_work + 1e-10);
      assert(Math.abs(m.elapsed - dt) <= 1e-14);
      assert(m.max_motion_error <= 1e-10);
      assert(m.constraint_visits <= 8 * m.contact_points);
      if (sleeping) { assert.equal(m.contact_points, 0); assert.equal(m.constraint_visits, 0); }
      if (name === "NearMiss") assert(m.sweep_queries > 0);
      return { case: name, ids: variant < 2 ? [1, 10] : [10, 1], substeps, ...m };
    }));
  if (previousStationary) assert.deepEqual(stationary, previousStationary, "same-target stationary support replay");
  previousStationary = stationary;
  assert(Number.isNaN(instance.exports.stationary_support_metric(36, 0)));
  assert(Number.isNaN(instance.exports.stationary_support_metric(0, stationaryFields.length)));
  console.log(`STATIONARY_EXTERNAL_SUPPORT_WASM ${JSON.stringify({ replay, rows: stationary })}`);
  const tangentFields = [
    "vx", "vy", "vz", "wx", "wy", "wz", "energy", "external_work", "woken_bodies",
    "response_preparations", "inertia_preparations", "inertia_applications", "current_queries",
    "sweep_queries", "contact_points", "constraint_visits", "elapsed", "max_motion_error",
    "sleeping", "independent_sleeping",
  ];
  const tangent = ["Tangent", "Loaded", "LoadedFour", "Slow", "Normal", "Stationary",
    "Separating", "SkinMiss", "BroadMiss", "Sensor", "Layers", "SpinCenter"]
    .flatMap((name, caseIndex) => [0, 1].map(order => {
      const m = Object.fromEntries(tangentFields.map((field, i) => {
        const value = instance.exports.tangential_contact_metric(2 * caseIndex + order, i);
        assert(Number.isFinite(value), `${name}/${order}/${field}`);
        return [field, value];
      }));
      const admitted = caseIndex < 5;
      const loaded = name === "Loaded" || name === "LoadedFour";
      const substeps = name === "LoadedFour" ? 4 : 1;
      const dt = substeps / 240;
      assert.equal(m.woken_bodies, Number(admitted));
      assert.equal(m.sleeping, Number(!admitted));
      assert.equal(m.independent_sleeping, 1);
      assert.equal(m.response_preparations, admitted ? substeps : 0);
      assert.equal(m.inertia_preparations, m.response_preparations);
      assert(Math.abs(m.vx - (loaded ? 6 * dt : 0)) <= 1e-10);
      assert(Math.abs(m.vy - (name === "Normal" ? 3 : 0)) <= 1e-10);
      assert.equal(m.vz, 0);
      assert.equal(m.wx, 0);
      assert.equal(m.wy, 0);
      assert(Math.abs(m.wz - (loaded ? 15 * dt : 0)) <= 1e-10);
      assert(Math.abs(m.elapsed - dt) <= 1e-14);
      assert(m.max_motion_error <= 1e-10);
      assert(m.energy + (name === "Normal" ? 60 * dt : 0) <= m.external_work + 1e-10);
      assert(m.constraint_visits <= 8 * m.contact_points);
      if (!admitted) { assert.equal(m.contact_points, 0); assert.equal(m.constraint_visits, 0); }
      if (name === "SkinMiss" || name === "BroadMiss") assert(m.sweep_queries > 0);
      return { case: name, ids: order === 0 ? [1, 10] : [10, 1], substeps, ...m };
    }));
  if (previousTangent) assert.deepEqual(tangent, previousTangent, "same-target tangent contact replay");
  previousTangent = tangent;
  assert(Number.isNaN(instance.exports.tangential_contact_metric(24, 0)));
  assert(Number.isNaN(instance.exports.tangential_contact_metric(0, tangentFields.length)));
  console.log(`TANGENTIAL_CONTACT_WAKE_WASM ${JSON.stringify({ replay, rows: tangent })}`);
  const forceFields = [
    "vx", "vy", "vz", "angular_speed", "energy", "external_work", "woken_bodies",
    "response_preparations", "inertia_preparations", "inertia_applications",
    "current_queries", "sweep_queries", "contact_points", "constraint_visits",
    "elapsed", "max_motion_error", "independent_sleeping",
  ];
  const forces = ["ContactWake", "RepeatedRoots", "AwakeReference", "RemovedSupport"]
    .flatMap((name, caseIndex) => [0, 1, 2, 3].map(variant => {
      const index = 4 * caseIndex + variant;
      const substeps = variant % 2 === 0 ? 1 : 4;
      const m = Object.fromEntries(forceFields.map((field, i) => {
        const value = instance.exports.contact_wake_force_metric(index, i);
        assert(Number.isFinite(value), `${name}/${variant}/${field}`);
        return [field, value];
      }));
      const removed = name === "RemovedSupport";
      assert.equal(m.woken_bodies, Number(name === "ContactWake" || name === "RepeatedRoots"));
      assert.equal(m.response_preparations, substeps);
      assert.equal(m.inertia_preparations, substeps);
      assert.equal(m.independent_sleeping, 1);
      assert(Math.abs(m.vx - 3) <= 1e-10);
      assert(Math.abs(m.vy - (removed ? -10 * substeps / 240 : 0)) <= 1e-10);
      assert(Math.abs(m.vz - (name === "RepeatedRoots" ? 3 : 0)) <= 1e-10);
      assert(Math.abs(m.angular_speed - (removed ? 0 : 15 * substeps / 240)) <= 1e-10);
      assert(Math.abs(m.elapsed - substeps / 240) <= 1e-14);
      assert(m.max_motion_error <= 1e-10);
      const y = removed ? 1 - 5 * substeps * (substeps + 1) / 240 ** 2 : 1;
      assert(m.energy + 20 * (y - 1) <= m.external_work + 1e-10);
      assert(m.constraint_visits <= 8 * m.contact_points);
      return { case: name, ids: variant < 2 ? [1, 10] : [10, 1], substeps, ...m };
    }));
  if (previousForces) assert.deepEqual(forces, previousForces, "same-target first-force replay");
  previousForces = forces;
  assert(Number.isNaN(instance.exports.contact_wake_force_metric(16, 0)));
  assert(Number.isNaN(instance.exports.contact_wake_force_metric(0, forceFields.length)));
  console.log(`CONTACT_WAKE_FORCES_WASM ${JSON.stringify({ replay, rows: forces })}`);
  const drivenFields = [
    "rider_y", "rider_vy", "energy", "external_work", "woken_bodies",
    "response_preparations", "inertia_preparations", "inertia_applications",
    "pair_tests", "narrow_tests", "current_queries", "sweep_queries",
    "contact_points", "constraint_visits", "swept_contacts", "sleeping",
    "independent_sleeping", "elapsed",
  ];
  const driven = ["Current", "SlowCurrent", "Swept", "SweptOutsideSlop", "SkinMiss", "BroadMiss", "Stationary", "Separating", "Sensor", "Layers"]
    .flatMap((name, caseIndex) => [0, 1].map(order => {
      const index = 2 * caseIndex + order;
      const m = Object.fromEntries(drivenFields.map((field, fieldIndex) => {
        const value = instance.exports.driven_parked_metric(index, fieldIndex);
        assert(Number.isFinite(value), `${name}/${order}/${field}`);
        return [field, value];
      }));
      const admitted = caseIndex < 4;
      assert.equal(m.woken_bodies, Number(admitted));
      assert.equal(m.sleeping, Number(!admitted));
      assert.equal(m.independent_sleeping, 1);
      assert.equal(m.response_preparations, Number(admitted));
      assert.equal(m.inertia_preparations, Number(admitted));
      assert(Math.abs(m.elapsed - ((name === "Current" || name === "SweptOutsideSlop") ? 1 / 60 : 1 / 240)) <= 1e-14);
      assert(m.energy + 20 * (m.rider_y - 1) <= m.external_work + 1e-10);
      if (name === "Current" || name === "SlowCurrent") {
        const speed = name === "Current" ? 3 : 0.25;
        assert(Math.abs(m.rider_vy - speed) <= 1e-10);
        assert(Math.abs(m.rider_y - 1 - speed * ((name === "Current" || name === "SweptOutsideSlop") ? 1 / 60 : 1 / 240)) <= 1e-10);
      }
      if (name === "Swept" || name === "SweptOutsideSlop" || name === "SkinMiss" || name === "BroadMiss") assert(m.sweep_queries > 0);
      if (!admitted) {
        assert.equal(m.rider_y, 1);
        assert.equal(m.rider_vy, 0);
        assert.equal(m.contact_points, 0);
        assert.equal(m.constraint_visits, 0);
      }
      return { case: name, ids: order === 0 ? [1, 10] : [10, 1], ...m };
    }));
  if (previousDriven) assert.deepEqual(driven, previousDriven, "same-target driven/parked replay");
  previousDriven = driven;
  assert(Number.isNaN(instance.exports.driven_parked_metric(20, 0)));
  assert(Number.isNaN(instance.exports.driven_parked_metric(0, drivenFields.length)));
  console.log(`DRIVEN_PARKED_WASM ${JSON.stringify({ replay, rows: driven })}`);
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
  const momentum = Object.fromEntries([
    "normalized_linear_error", "normalized_angular_error", "energy_ratio", "cases",
  ].map((field, i) => {
    const value = instance.exports.contact_momentum_metric(i);
    assert(Number.isFinite(value), field);
    return [field, value];
  }));
  assert(momentum.normalized_linear_error <= 1e-10);
  assert(momentum.normalized_angular_error <= 1e-10);
  assert(momentum.energy_ratio <= 1 + 1e-10);
  assert.equal(momentum.cases, 768);
  if (previousMomentum) assert.deepEqual(momentum, previousMomentum);
  previousMomentum = momentum;
  assert(Number.isNaN(instance.exports.contact_momentum_metric(4)));
  console.log(`CONTACT_MOMENTUM_WASM ${JSON.stringify({ replay, momentum })}`);
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
console.log("Dense contact WASM: three replays passed, including physical substeps, materials, narrow supports, axial primitive pairs moving supports driven/parked admission and first-substep forces.");
