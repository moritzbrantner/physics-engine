#!/usr/bin/env bash
# WASM half of the cross-target state hash (#576): the fixture of tests/cross_target_hash.rs,
# built for wasm32-unknown-unknown and run in Node. The example traps unless its hash equals
# the pinned native one, so native and browser builds step identically.
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build --locked --release --example cross_target_hash --target wasm32-unknown-unknown
node --input-type=module - target/wasm32-unknown-unknown/release/examples/cross_target_hash.wasm <<'NODE'
import { readFile } from "node:fs/promises";
const { instance } = await WebAssembly.instantiate(await readFile(process.argv[2]), {});
if (instance.exports.main(0, 0) !== 0) throw new Error("cross-target state hash differs on WASM");
console.log("cross-target state hash: WASM equals the pinned native hash");
NODE
