#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build -p physics-engine-demo --target wasm32-unknown-unknown --release --locked
WASM=target/wasm32-unknown-unknown/release/physics_engine_demo.wasm

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
node --input-type=module --check < site/lab.js
node --input-type=module --check < site/webgl-renderer.js

rm -rf pages-dist
mkdir -p pages-dist
cp -R site/. pages-dist/
cp "$WASM" pages-dist/
test -s pages-dist/index.html
test -s pages-dist/lab.js
test -s pages-dist/physics_engine_demo.wasm
