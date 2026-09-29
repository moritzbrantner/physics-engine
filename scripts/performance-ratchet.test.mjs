import assert from "node:assert/strict";
import { test } from "node:test";
import { assertHistoryPrefix, compareRows, hash, parseNative, sandboxRows, validateHistory, validateNativeTests } from "./performance-ratchet-lib.mjs";

const rows = () => ({ "native/fixture": {
  work: { preparations: 4, snapshots: 0 }, correctness: { contacts: 8 }, timing: { median_ms: 1 },
} });
const copy = (value) => structuredClone(value);

test("count improvements pass, zero budgets cannot drift, and timings are advisory", () => {
  const candidate = rows();
  candidate["native/fixture"].work.preparations = 3;
  candidate["native/fixture"].timing.median_ms = 100;
  assert.equal(compareRows(rows(), candidate).length, 1);
  candidate["native/fixture"].work.snapshots = 1;
  assert.throws(() => compareRows(rows(), candidate), /snapshots regressed/);
});

for (const [label, mutate] of [
  ["missing scenario", (r) => delete r["native/fixture"]],
  ["missing metric", (r) => delete r["native/fixture"].work.preparations],
  ["missing timing", (r) => delete r["native/fixture"].timing.median_ms],
  ["missing correctness", (r) => r["native/fixture"].correctness = {}],
  ["fewer useful contacts", (r) => r["native/fixture"].correctness.contacts = 0],
  ["negative work", (r) => r["native/fixture"].work.preparations = -1],
  ["fractional work", (r) => r["native/fixture"].work.preparations = 0.5],
  ["null work", (r) => r["native/fixture"].work.preparations = null],
  ["NaN work", (r) => r["native/fixture"].work.preparations = NaN],
  ["infinite time", (r) => r["native/fixture"].timing.median_ms = Infinity],
]) test(`fails closed on ${label}`, () => {
  const candidate = rows();
  mutate(candidate);
  assert.throws(() => compareRows(rows(), candidate));
});

test("native collection rejects skipped tests, duplicate rows, and truncated JSON", () => {
  const line = `test x ... PERFORMANCE_RATCHET ${JSON.stringify({ scenario: "fixture", ...rows()["native/fixture"] })}\n`;
  assert.deepEqual(parseNative(line, ["fixture"]), rows());
  assert.throws(() => parseNative("running 0 tests", ["fixture"]));
  assert.throws(() => parseNative(line + line, ["fixture"]), /duplicate/);
  assert.throws(() => parseNative("PERFORMANCE_RATCHET {", ["fixture"]));
});

const sandbox = () => ({ workload: "test-v1", head: { cases: [{ name: "scene", trials: [0, 1].map(() => ({
  replay_sha256: "a".repeat(64), body_count: 2, event_sum: 3,
  steps: { count: 4, mean_ms: 1, p95_ms: 2 }, work: { queries: 4, tree_reuses: 9 },
})) }] } });
const fixture = { workload: "test-v1", scenarios: ["scene"], trials: 2 };

test("sandbox collection requires repeated deterministic useful work", () => {
  const result = sandboxRows(sandbox(), fixture);
  assert.deepEqual(result["sandbox/scene"].work, { queries: 4 });
  for (const mutate of [
    (r) => r.head.cases[0].trials.pop(),
    (r) => r.head.cases.push(copy(r.head.cases[0])),
    (r) => r.head.cases[0].trials[1].work.queries++,
    (r) => r.head.cases[0].trials[1].replay_sha256 = "b".repeat(64),
    (r) => r.failure = "physics error",
    (r) => r.head.cases[0].trials.forEach((t) => t.work.queries = null),
    (r) => r.head.cases[0].trials.forEach((t) => t.body_count = 0),
  ]) {
    const input = sandbox();
    mutate(input);
    assert.throws(() => sandboxRows(input, fixture));
  }
});

function entry(previous = null, overrides = {}) {
  const value = {
    schemaVersion: 1, ordinal: previous ? 2 : 1, id: "measurement", epoch: 1,
    reason: "Measured reduction of unchanged geometry preparations.",
    recordedAt: "2026-09-20T00:00:00.000Z", previousSha256: previous ? hash(previous.contents) : null,
    fixtureSha256: "a".repeat(64), source: { revision: "b".repeat(40), snapshotSha256: "c".repeat(64), dirty: true },
    environment: { rustc: "rustc 1.98.0" }, rows: rows(), ...overrides,
  };
  return { name: `${String(value.ordinal).padStart(4, "0")}-${value.id}.json`, contents: JSON.stringify(value) };
}

test("history is chained, contiguous, and monotone within each epoch", () => {
  const first = entry();
  const improved = rows();
  improved["native/fixture"].work.preparations = 2;
  const second = entry(first, { rows: improved });
  assert.equal(validateHistory([first, second]).ordinal, 2);
  assert.throws(() => validateHistory([first, entry(first, { previousSha256: "d".repeat(64) })]), /hash chain/);
  assert.throws(() => validateHistory([entry(null, { ordinal: 2 })]), /contiguous/);
  const worse = rows();
  worse["native/fixture"].work.preparations = 5;
  assert.throws(() => validateHistory([first, entry(first, { rows: worse })]), /regressed/);
  assert.equal(validateHistory([first, entry(first, { epoch: 2, rows: worse })]).epoch, 2);
  assert.throws(() => validateHistory([first, entry(first, { epoch: 2, reason: "" })]), /reason/);
  assert.throws(() => validateHistory([first, entry(first, { fixtureSha256: "d".repeat(64) })]), /fixture/);
});

test("base history cannot be deleted or rewritten even if its hash chain is regenerated", () => {
  const first = entry();
  const second = entry(first);
  assertHistoryPrefix([first], [first, second]);
  assert.throws(() => assertHistoryPrefix([first, second], [first]), /removed/);
  const rewritten = entry(null, { reason: "Rewritten old history must be rejected." });
  assert.throws(() => assertHistoryPrefix([first], [rewritten, entry(rewritten)]), /immutable/);
});

test("native preflight requires exact test names before measurement", () => {
  const listed = "module::fixture_extra: test\nmodule::fixture: test\nmodule::bench: benchmark\n";
  assert.doesNotThrow(() => validateNativeTests(listed, ["module::fixture"]));
  assert.throws(() => validateNativeTests(listed, ["module::missing"]), /missing native fixture test/);
  assert.throws(() => validateNativeTests("module::fixture_extra: test\n", ["module::fixture"]));
  assert.throws(() => validateNativeTests(listed, ["module::bench"]));
});
