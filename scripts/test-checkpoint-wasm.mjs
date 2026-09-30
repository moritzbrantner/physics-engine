import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";

const path = process.argv[2];
assert(path, "usage: node scripts/test-checkpoint-wasm.mjs <checkpoint-continuation.wasm>");
const { instance } = await WebAssembly.instantiate(await readFile(path), {});
assert.equal(typeof instance.exports.checkpoint_continuation, "function");
for (let replay = 0; replay < 3; replay += 1) {
  assert.equal(instance.exports.checkpoint_continuation(), 0, `WASM continuation replay ${replay}`);
}
console.log("WASM checkpoints: three exact continuation replays passed, including sleep, pending input, support removal, ID reuse, retirement and failed ticks.");
