#!/usr/bin/env bash
set -euo pipefail

root="$(pwd)"
actual_head="$(git rev-parse HEAD)"
if [[ -n "${HEAD_SHA:-}" && "$HEAD_SHA" != "$actual_head" ]]; then
  echo "Expected exact head $HEAD_SHA, got $actual_head" >&2
  exit 1
fi
export HEAD_SHA="$actual_head"
output="$root/performance-evidence"
mkdir -p "$output"
{
  printf 'head=%s\nbase=%s\n' "$HEAD_SHA" "${BASE_SHA:-none}"
  rustc -Vv
  cargo --version
  node --version
  uname -a
} > "$output/environment.txt"

rustup target add wasm32-unknown-unknown
cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
cp demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm "$output/head.wasm"
cp scripts/benchmark-sandbox.mjs "$output/benchmark-sandbox.mjs"

if [[ -n "${BASE_SHA:-}" ]]; then
  temporary="$(mktemp -d)"
  cleanup() {
    git worktree remove --force "$temporary/base" 2>/dev/null || true
    rm -rf "$temporary"
  }
  trap cleanup EXIT
  git worktree add --detach "$temporary/base" "$BASE_SHA"
  (
    cd "$temporary/base"
    cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
  )
  cp "$temporary/base/demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm" "$output/base.wasm"
  export BASELINE_WASM="$output/base.wasm"
fi
node scripts/benchmark-sandbox.mjs "$output/head.wasm" "$output/sandbox.json"
