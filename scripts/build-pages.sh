#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build -p physics-engine-demo --target wasm32-unknown-unknown --release --locked
WASM=target/wasm32-unknown-unknown/release/physics_engine_demo.wasm
cargo build -p physics-oracle --lib --target wasm32-unknown-unknown --release --locked
ORACLE_WASM=target/wasm32-unknown-unknown/release/physics_oracle.wasm

# Smoke-test the shipped module: reset, settle, shoot, step, snapshot.
node --input-type=module - "$WASM" <<'NODE'
import { readFile } from "node:fs/promises";
const { instance } = await WebAssembly.instantiate(await readFile(process.argv[2]), {});
const e = instance.exports;
if (e.lab_reset(0) !== 0) throw new Error("lab_reset failed");
for (let i = 0; i < 240; i++) if (e.lab_step() !== 0) throw new Error(`settle step ${i} failed`);
if (e.lab_shoot(0, 0, -96) < 0) throw new Error("lab_shoot failed");
for (let i = 0; i < 120; i++) if (e.lab_step() !== 0) throw new Error(`impact step ${i} failed`);
e.lab_refresh_snapshot();
if (e.lab_snapshot_len() % 11 !== 0 || e.lab_snapshot_len() === 0) throw new Error("bad snapshot");
NODE
# Checkpoint continuation on WASM (#452); a separate example, not part of the shipped demo.
bash scripts/checkpoint-wasm.sh
node --input-type=module --check < site/lab.js
node --input-type=module --check < site/webgl-renderer.js
node --input-type=module --check < site/compare.js
node --input-type=module --check < site/oracle-data.mjs
node --test scripts/tests/oracle-data.test.mjs

rm -rf pages-dist
mkdir -p pages-dist
cp -R site/. pages-dist/
cp "$WASM" pages-dist/
cp "$ORACLE_WASM" pages-dist/
node scripts/generate-pages-capabilities.mjs pages-dist/capabilities.json

# Exercise the *shipped* oracle, not a separate test build. The same metadata and traces
# drive the browser, while the native and WASM oracle jobs remain the full correctness gates.
node --input-type=module - "$ORACLE_WASM" <<'NODE'
import { readFile } from "node:fs/promises";
import { buildCatalogue, readScenarios, readTrace } from "./site/oracle-data.mjs";

const env = { oracle_now: () => performance.now() };
const { instance } = await WebAssembly.instantiate(await readFile(process.argv[2]), { env });
const e = instance.exports;
const scenarios = readScenarios(e);
const capabilities = JSON.parse(await readFile("pages-dist/capabilities.json", "utf8"));
buildCatalogue(capabilities, scenarios);
if (!scenarios.length || new Set(scenarios.map((scenario) => scenario.name)).size !== scenarios.length) {
  throw new Error("Oracle scenario registry is incomplete or contains duplicate names");
}

for (const selected of ["static-queries", "props", "debris", "contact-events"]) {
  const scenario = scenarios.find((item) => item.name === selected);
  if (!scenario || e.oracle_check(scenario.index) !== 0) throw new Error(`${selected}: invariants failed`);
  const engine = readTrace(e, scenario, 0), rapier = readTrace(e, scenario, 1);
  if (engine.total === 0 || rapier.total === 0 || !engine.displayed || !rapier.displayed) {
    throw new Error(`${selected}: no usable browser observations`);
  }
  if (engine.displayed > scenario.ticks * 64 || rapier.displayed > scenario.ticks * 64) {
    throw new Error(`${selected}: display budget exceeded`);
  }
}
if (e.oracle_trace(e.oracle_scenarios(), 0) !== -1 || e.oracle_trace(0, 99) !== -1) {
  throw new Error("Invalid oracle replay inputs must be rejected");
}
NODE

test -s pages-dist/index.html
test -s pages-dist/tower.html
test -s pages-dist/lab.js
test -s pages-dist/compare.js
test -s pages-dist/physics_engine_demo.wasm
test -s pages-dist/physics_oracle.wasm
test -s pages-dist/capabilities.json
