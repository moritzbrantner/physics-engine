import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-translational-maintenance-wasm.mjs <contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.translational_maintenance_contract(), 0, `retained-storage WASM replay ${replay}`);
}
console.log("Translational maintenance WASM: three lifecycle, release, failure and recovery controls passed.");
