import assert from "node:assert/strict";
import { createHash } from "node:crypto";

export const hash = (bytes) => createHash("sha256").update(bytes).digest("hex");
const object = (value) => value !== null && typeof value === "object" && !Array.isArray(value);
const keys = (value) => Object.keys(value).sort();
const same = (a, b, label) => assert.deepEqual(a, b, label);

export function validateRows(rows) {
  assert(object(rows) && keys(rows).length > 0, "missing measurements");
  for (const [id, row] of Object.entries(rows)) {
    assert(/^(native|sandbox)\/[a-z0-9/-]+$/.test(id), `invalid scenario ${id}`);
    same(keys(row), ["correctness", "timing", "work"], `${id}: invalid measurement shape`);
    for (const group of ["work", "correctness", "timing"]) {
      assert(object(row[group]), `${id}: missing ${group}`);
      for (const [name, value] of Object.entries(row[group])) {
        assert(/^[a-z0-9_]+$/.test(name), `${id}: invalid metric ${name}`);
        if (group === "correctness" && name === "replay_sha256") {
          assert(typeof value === "string" && /^[a-f0-9]{64}$/.test(value), `${id}: invalid replay hash`);
        } else {
          assert(typeof value === "number" && Number.isFinite(value) && value >= 0,
            `${id}: invalid ${group}.${name}`);
          if (group !== "timing") assert(Number.isSafeInteger(value), `${id}: non-integer count ${name}`);
        }
      }
    }
    assert(keys(row.correctness).length > 0, `${id}: missing correctness evidence`);
  }
}

// Missing or renamed scenarios/metrics fail closed. Timings are recorded, never used as noisy gates.
export function compareRows(baseline, candidate) {
  validateRows(baseline);
  validateRows(candidate);
  same(keys(candidate), keys(baseline), "scenario set changed; declare a new workload epoch");
  const improvements = [];
  for (const id of keys(baseline)) {
    const before = baseline[id];
    const after = candidate[id];
    same(after.correctness, before.correctness, `${id}: correctness/replay changed`);
    same(keys(after.work), keys(before.work), `${id}: work metric set changed`);
    same(keys(after.timing), keys(before.timing), `${id}: timing metric set changed`);
    for (const [metric, previous] of Object.entries(before.work)) {
      const value = after.work[metric];
      assert(value <= previous, `${id}: ${metric} regressed from ${previous} to ${value}`);
      if (value < previous) improvements.push({ scenario: id, metric, before: previous, after: value });
    }
  }
  return improvements;
}

export function validateNativeTests(stdout, expected) {
  const available = new Set(stdout.split("\n").filter((line) => line.endsWith(": test"))
    .map((line) => line.slice(0, -6)));
  for (const name of expected) assert(available.has(name), `missing native fixture test ${name}`);
}

export function parseNative(stdout, expected) {
  const rows = {};
  const marker = "PERFORMANCE_RATCHET ";
  for (const line of stdout.split("\n")) {
    const start = line.indexOf(marker);
    if (start < 0) continue;
    const { scenario, ...row } = JSON.parse(line.slice(start + marker.length));
    assert(!Object.hasOwn(rows, `native/${scenario}`), `duplicate scenario ${scenario}`);
    rows[`native/${scenario}`] = row;
  }
  same(keys(rows), expected.map((id) => `native/${id}`).sort(), "missing/unexpected native evidence");
  validateRows(rows);
  return rows;
}

export function sandboxRows(results, fixture) {
  assert(!results.failure, `sandbox failed: ${results.failure}`);
  assert.equal(results.workload, fixture.workload, "sandbox workload changed");
  assert(Array.isArray(results.head?.cases), "missing sandbox results");
  same(results.head.cases.map((entry) => entry.name).sort(), [...fixture.scenarios].sort(), "missing/duplicate sandbox scenarios");
  const rows = {};
  for (const entry of results.head.cases) {
    assert.equal(entry.trials.length, fixture.trials, `${entry.name}: missing replay trials`);
    const first = entry.trials[0];
    const correctness = (trial) => ({
      replay_sha256: trial.replay_sha256, bodies: trial.body_count,
      events: trial.event_sum, steps: trial.steps?.count,
    });
    assert(first.body_count > 0 && first.steps?.count > 0, `${entry.name}: empty workload`);
    for (const trial of entry.trials) {
      same(correctness(trial), correctness(first), `${entry.name}: nondeterministic replay`);
      same(trial.work, first.work, `${entry.name}: nondeterministic work`);
      for (const value of [trial.steps?.mean_ms, trial.steps?.p95_ms]) {
        assert(Number.isFinite(value) && value >= 0, `${entry.name}: missing timing`);
      }
    }
    assert(object(first.work) && keys(first.work).length > 0, `${entry.name}: missing work`);
    // Successful cache reuse is not a cost to minimize. Full raw counters remain in sandbox.json.
    const work = Object.fromEntries(Object.entries(first.work).filter(([name]) => !name.endsWith("_reuses")));
    rows[`sandbox/${entry.name}`] = {
      work, correctness: correctness(first), timing: {
        mean_step_ms: entry.trials.reduce((sum, trial) => sum + trial.steps.mean_ms, 0) / entry.trials.length,
        worst_trial_p95_ms: Math.max(...entry.trials.map((trial) => trial.steps.p95_ms)),
      },
    };
  }
  validateRows(rows);
  return rows;
}

export function validateHistory(files) {
  let previous = null;
  for (const [index, file] of files.entries()) {
    const entry = JSON.parse(file.contents);
    assert.equal(entry.schemaVersion, 1, "unsupported history schema");
    assert.equal(entry.ordinal, index + 1, "history must be contiguous");
    assert(typeof entry.id === "string" && /^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(entry.id), "invalid history id");
    assert.equal(file.name, `${String(index + 1).padStart(4, "0")}-${entry.id}.json`, "invalid history filename");
    assert(typeof entry.reason === "string" && entry.reason.trim().length >= 20, "history requires a meaningful reason");
    assert.equal(entry.previousSha256, index === 0 ? null : hash(files[index - 1].contents), "history hash chain changed");
    assert(typeof entry.fixtureSha256 === "string" && /^[a-f0-9]{64}$/.test(entry.fixtureSha256), "missing fixture identity");
    assert(/^[a-f0-9]{40}$/.test(entry.source?.revision), "missing source revision");
    assert(/^[a-f0-9]{64}$/.test(entry.source?.snapshotSha256), "missing source snapshot");
    assert(typeof entry.source?.dirty === "boolean", "missing dirty-state provenance");
    assert(object(entry.environment) && typeof entry.environment.rustc === "string", "missing environment");
    assert(Number.isFinite(Date.parse(entry.recordedAt)), "missing timestamp");
    validateRows(entry.rows);
    if (previous) {
      assert(entry.epoch === previous.epoch || entry.epoch === previous.epoch + 1, "invalid epoch progression");
      if (entry.epoch === previous.epoch) {
        assert.equal(entry.fixtureSha256, previous.fixtureSha256, "fixture changed without a new epoch");
        compareRows(previous.rows, entry.rows);
      }
    } else {
      assert.equal(entry.epoch, 1, "initial epoch must be one");
    }
    previous = entry;
  }
  return previous;
}

export function assertHistoryPrefix(base, current) {
  assert(current.length >= base.length, "previous history entries were removed");
  for (const [index, file] of base.entries()) {
    same(current[index], file, `recorded history is immutable: ${file.name}`);
  }
}
