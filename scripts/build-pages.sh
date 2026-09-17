#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build \
  --manifest-path demo-wasm/Cargo.toml \
  --target wasm32-unknown-unknown \
  --release \
  --locked

node --input-type=module <<'NODE'
import { readFile } from "node:fs/promises";

const bytes = await readFile(
  "demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm",
);
const { instance } = await WebAssembly.instantiate(bytes, {});
const exports = instance.exports;
const requiredFunctions = [
  "sandbox_reset_with_options",
  "sandbox_reset_with_baking_options",
  "sandbox_fixed_geometry_mode",
  "sandbox_fixed_geometry_prepared_count",
  "sandbox_fixed_geometry_total_preparations",
  "sandbox_fixed_geometry_retained_bytes",
  "sandbox_fixed_geometry_representation_version",
  "sandbox_simulation_rules",
  "sandbox_refresh_render_snapshot",
  "sandbox_render_snapshot_len",
  "sandbox_render_snapshot_stride",
];
for (const name of requiredFunctions) {
  if (typeof exports[name] !== "function") {
    throw new Error(`missing WASM sandbox export: ${name}`);
  }
}
if (!(exports.memory instanceof WebAssembly.Memory)) {
  throw new Error("WASM module does not export linear memory for the render snapshot");
}

if (exports.sandbox_reset_with_baking_options(0, 0, 0) !== 0) {
  throw new Error("runtime fixed-geometry reference mode failed to initialize");
}
if (
  exports.sandbox_fixed_geometry_mode() !== 0 ||
  exports.sandbox_fixed_geometry_prepared_count() !== 0 ||
  exports.sandbox_fixed_geometry_total_preparations() !== 0 ||
  exports.sandbox_fixed_geometry_retained_bytes() !== 0
) {
  throw new Error("runtime fixed-geometry reference unexpectedly retained preparation");
}
if (exports.sandbox_reset_with_baking_options(0, 0, 1) !== 0) {
  throw new Error("prepare-at-load fixed geometry mode failed to initialize");
}
if (
  exports.sandbox_fixed_geometry_mode() !== 1 ||
  exports.sandbox_fixed_geometry_prepared_count() !== 11 ||
  exports.sandbox_fixed_geometry_total_preparations() !== 11 ||
  exports.sandbox_fixed_geometry_retained_bytes() <= 0 ||
  exports.sandbox_fixed_geometry_representation_version() !== 1
) {
  throw new Error("prepare-at-load fixed geometry evidence is invalid");
}

const pointer = exports.sandbox_refresh_render_snapshot();
const length = exports.sandbox_render_snapshot_len();
const stride = exports.sandbox_render_snapshot_stride();
if (stride !== 11 || length <= 0 || length % stride !== 0) {
  throw new Error(`invalid render snapshot layout: pointer=${pointer} length=${length} stride=${stride}`);
}
const byteEnd = pointer + length * Int32Array.BYTES_PER_ELEMENT;
if (pointer < 0 || byteEnd > exports.memory.buffer.byteLength) {
  throw new Error(
    `render snapshot exceeds WASM memory: pointer=${pointer} byteEnd=${byteEnd} memory=${exports.memory.buffer.byteLength}`,
  );
}
new Int32Array(exports.memory.buffer, pointer, length);
NODE

node --input-type=module --check < site/app.js
node --input-type=module --check < site/webgpu-renderer.js
node --input-type=module --check < site/webgl-renderer.js
node --input-type=module --check < site/physics-error.js
node --check site/physics-error.mjs
node --check site/interaction-controls.mjs
node --check site/simulation-rules.mjs
node --check site/simulation-rules-config.mjs
node --check site/performance-log.mjs
node --check scripts/adapt-performance-evidence.mjs
node --check scripts/package-performance-log.mjs
node --test site/physics-error.test.mjs site/interaction-controls.test.mjs site/simulation-rules-config.test.mjs site/performance-log.test.mjs scripts/adapt-performance-evidence.test.mjs scripts/package-performance-log.test.mjs scripts/summarize-cpu-profile.test.mjs

rm -rf pages-dist
mkdir -p pages-dist
cp -R site/. pages-dist/
cp demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm pages-dist/
node --input-type=module <<'NODE'
import { createHash } from "node:crypto";
import { readFile, writeFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";

const wasm = await readFile("pages-dist/physics_engine_demo.wasm");
const provenance = {
  schema_version: 1,
  repository: "moritzbrantner/physics-engine",
  revision: execFileSync("git", ["rev-parse", "HEAD"], { encoding: "utf8" }).trim(),
  wasm_sha256: createHash("sha256").update(wasm).digest("hex"),
};
await writeFile("pages-dist/build-provenance.json", `${JSON.stringify(provenance, null, 2)}\n`);
NODE
test -s pages-dist/index.html
test -s pages-dist/app.js
test -s pages-dist/webgpu-renderer.js
test -s pages-dist/webgl-renderer.js
test -s pages-dist/physics-error.js
test -s pages-dist/physics-error.mjs
test -s pages-dist/interaction-controls.mjs
test -s pages-dist/simulation-rules.mjs
test -s pages-dist/simulation-rules-config.mjs
test -s pages-dist/performance-log.mjs
test -s pages-dist/build-provenance.json
test -s pages-dist/physics_engine_demo.wasm
