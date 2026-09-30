import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-shape-query-wasm.mjs <shape-query-contract.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
assert.equal(typeof instance.exports.shape_query_contract, "function");
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.shape_query_contract(), 0, `WASM shape query replay ${replay}`);
}
console.log("WASM geometry queries: three native-contract replays passed, including all four primitives, finite rays and shape casts, thin walls, touching/overlap, retained output and failed searches.");
