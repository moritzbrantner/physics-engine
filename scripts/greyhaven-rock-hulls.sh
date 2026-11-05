#!/usr/bin/env bash
# Focused native acceptance for physics-engine#180 using Greyhaven's authored rock proxy.
# Usage: bash scripts/greyhaven-rock-hulls.sh [--perf|--stats|--wasm]
set -euo pipefail
cd "$(dirname "$0")/.."

# Keep local debug tests working around occasional zero-sized Rust incremental archives.
export CARGO_INCREMENTAL="${CARGO_INCREMENTAL:-0}"
export CARGO_PROFILE_DEV_DEBUG="${CARGO_PROFILE_DEV_DEBUG:-0}"

cargo test --locked --features checkpoints --test convex_hull
cargo run --locked -p physics-oracle -- --only greyhaven-rock-hulls --no-timing

case "${1:-}" in
  "") ;;
  --perf)
    cargo run --release --locked -p physics-oracle -- --only greyhaven-rock-hulls
    ;;
  --stats)
    PHYSICS_ROCK_WORK=1 cargo run --release --locked -p physics-oracle -- --only greyhaven-rock-hulls --no-timing
    ;;
  --wasm)
    node scripts/oracle-wasm.mjs --only greyhaven-rock-hulls
    ;;
  *)
    echo "usage: $0 [--perf|--stats|--wasm]" >&2
    exit 2
    ;;
esac
