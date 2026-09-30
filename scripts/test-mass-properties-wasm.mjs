import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-mass-properties-wasm.mjs <mass-properties-contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
assert.equal(typeof instance.exports.mass_properties_contract, "function");
for (let replay = 0; replay < 3; replay++) {
  assert.equal(instance.exports.mass_properties_contract(), 0, `WASM mass properties replay ${replay}`);
}
console.log("WASM mass properties: five native public fixtures passed across three replays.");
