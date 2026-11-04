#!/usr/bin/env bash
# WASM acceptance for checkpoint continuation (#452): the MMORPG-style zone of
# tests/checkpoint_zone.rs, uninterrupted vs capture -> drop -> restore -> replay on both body
# storages, built for wasm32-unknown-unknown and run in Node. The example traps on any
# difference. Native and WASM checkpoints are tested independently, never exchanged.
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build --locked --release --example checkpoint_zone --features checkpoints \
  --target wasm32-unknown-unknown
node --input-type=module - target/wasm32-unknown-unknown/release/examples/checkpoint_zone.wasm <<'NODE'
import { readFile } from "node:fs/promises";
const { instance } = await WebAssembly.instantiate(await readFile(process.argv[2]), {});
if (instance.exports.main(0, 0) !== 0) throw new Error("checkpoint zone replay failed");
console.log("checkpoint zone: WASM continuation identical on both storages");
NODE
