import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
const path = process.argv[2];
assert(path, "usage: node scripts/test-dense-contact-wasm.mjs <dense-contact-contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.dense_contact_contract(), 0, `dense contact WASM replay ${replay}`);
}
console.log("Dense contact WASM: three replays passed, including physical substeps and the material contract.");
