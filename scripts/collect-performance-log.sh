#!/usr/bin/env bash
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
head_sha="$(git -C "$root" rev-parse HEAD)"
short_sha="$(git -C "$root" rev-parse --short=12 HEAD)"
temporary="$(mktemp -d "${TMPDIR:-/tmp}/physics-performance-log-XXXXXX")"
cleanup() {
  rm -rf "$temporary"
}
trap cleanup EXIT

export HEAD_SHA="$head_sha"
export PERFORMANCE_EVIDENCE_DIR="$temporary/evidence"
mkdir -p "$PERFORMANCE_EVIDENCE_DIR"

changed_files="$temporary/all-performance-inputs.txt"
cat > "$changed_files" <<'FILES'
src/query.rs
src/rigid_box_free_flight.rs
src/current_contact_query.rs
src/rotating_broad_phase.rs
src/rotating_contact_search.rs
src/rotating_recontact_search.rs
src/obb_response.rs
FILES

cd "$root"
bash scripts/run-performance-evidence.sh "$changed_files"
bash scripts/run-sandbox-performance.sh
cp scripts/benchmark-character-options.mjs "$PERFORMANCE_EVIDENCE_DIR/"
node scripts/benchmark-character-options.mjs \
  "$PERFORMANCE_EVIDENCE_DIR/head.wasm" \
  "$PERFORMANCE_EVIDENCE_DIR/character-options.json"
cp scripts/benchmark-baking.mjs "$PERFORMANCE_EVIDENCE_DIR/"
node scripts/benchmark-baking.mjs \
  "$PERFORMANCE_EVIDENCE_DIR/head.wasm" \
  "$PERFORMANCE_EVIDENCE_DIR/baking-options.json"

archive="$root/physics-performance-log-${short_sha}.tar.gz"
node scripts/package-performance-log.mjs \
  "$PERFORMANCE_EVIDENCE_DIR" \
  "$archive" \
  "$@"

printf 'Portable performance log: %s\n' "$archive"
