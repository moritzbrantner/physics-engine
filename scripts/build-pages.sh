#!/usr/bin/env bash
set -euo pipefail

rustup target add wasm32-unknown-unknown
cargo build \
  --manifest-path demo-wasm/Cargo.toml \
  --target wasm32-unknown-unknown \
  --release \
  --locked

rm -rf pages-dist
mkdir -p pages-dist
cp -R site/. pages-dist/
cp demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm pages-dist/
test -s pages-dist/index.html
test -s pages-dist/physics_engine_demo.wasm
