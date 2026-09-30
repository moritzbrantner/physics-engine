import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-rotating-interval-wasm.mjs <rotating-interval-contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
assert.equal(typeof instance.exports.rotating_interval_contract, "function");
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.rotating_interval_contract(), 0, `WASM rotating interval replay ${replay}`);
}
console.log("WASM rotating intervals: three public-contract replays passed, including late-error rollback, sleep deadlines, parked activation, damping and quiet work.");
