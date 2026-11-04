import { test } from "node:test";
import assert from "node:assert/strict";
import { buildCatalogue, decodeRows, observationText, pairedFrame, readTrace, visualMode } from "../../site/oracle-data.mjs";

const pack = (...rows) => Float64Array.from(rows.flat());
const position = (tick, ordinal, x, y, z) => [tick, ordinal, 3, x, y, z, 0, 0, 0];
const event = (tick, ordinal, started, a, b) => [tick, ordinal, 10, Number(started), a, b, 0, 0, 0];

test("catalogue requires a real scenario for every documented capability", () => {
  const capabilities = [{ id: "C1", title: "Boxes" }, { id: "C2", title: "Plane" }];
  const scenarios = [{ name: "queries", capabilities: ["C1", "C2"] }];
  assert.deepEqual(buildCatalogue(capabilities, scenarios).map((item) => item.scenarios.length), [1, 1]);
  assert.throws(() => buildCatalogue(capabilities, [{ name: "queries", capabilities: ["C1"] }]), /C2/);
});

test("decoder preserves tick, original ordinal and exact observation values", () => {
  const result = decodeRows(pack(position(0, 0, 1, 2, 3), position(0, 7, 4, 5, 6), event(2, 8, true, 10, 11)), 3);
  assert.equal(result.rows.length, 3);
  assert.deepEqual(result.frames.map((frame) => frame.length), [2, 0, 1]);
  assert.equal(result.frames[0][1].ordinal, 7);
  assert.deepEqual(result.frames[2][0].values.slice(0, 3), [1, 10, 11]);
});

test("decoder rejects malformed, non-finite, misordered and unknown observations", () => {
  assert.throws(() => decodeRows(new Float64Array(8), 2), /stride/);
  assert.throws(() => decodeRows(pack(position(2, 0, 0, 0, 0)), 2), /Invalid/);
  assert.throws(() => decodeRows(pack(position(0, 1, 0, 0, 0), position(0, 1, 0, 0, 0)), 1), /Invalid/);
  assert.throws(() => decodeRows(pack([0, 0, 99, 0, 0, 0, 0, 0, 0]), 1), /Invalid/);
  assert.throws(() => decodeRows(pack(position(0, 0, Infinity, 0, 0)), 1), /Invalid/);
});

test("paired frame preserves missing or extra observations instead of fabricating matches", () => {
  const left = decodeRows(pack(position(0, 0, 1, 2, 3), position(0, 5, 4, 5, 6)), 1);
  const right = decodeRows(pack(position(0, 0, 1, 2, 3), event(0, 7, false, 10, 11)), 1);
  assert.deepEqual(pairedFrame(left, right, 0).map((row) => row.ordinal), [0, 5, 7]);
  assert.equal(pairedFrame(left, right, 0)[1].rapier, undefined);
  assert.equal(pairedFrame(left, right, 0)[2].engine, undefined);
  assert.equal(visualMode(left, right), "position");
});

test("display labels retain semantic units and distinguish absent hits", () => {
  assert.equal(observationText({ kind: 3, values: [1, 2.34, 3] }), "Position · [1, 2.34, 3] m");
  assert.equal(observationText({ kind: 1, values: [0, 0, 0, 0, 0, 0] }), "Ray · clear");
  assert.match(observationText({ kind: 10, values: [1, 11, 12] }), /Contact started.*11 \/ 12/);
});

test("trace decoding copies the WASM buffer before the next side overwrites it", () => {
  const memory = { buffer: new ArrayBuffer(9 * 8) };
  const content = new Float64Array(memory.buffer);
  content.set(position(0, 0, 1, 2, 3));
  const e = {
    memory,
    oracle_trace: () => 0,
    oracle_trace_stride: () => 9,
    oracle_trace_ptr: () => 0,
    oracle_trace_len: () => 9,
    oracle_trace_total: () => 1,
  };
  const result = readTrace(e, { index: 0, ticks: 1 }, 0);
  content[3] = 999;
  assert.equal(result.rows[0].values[0], 1);
  assert.equal(result.displayed, 1);
});
