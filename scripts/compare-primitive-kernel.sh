#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${PERFORMANCE_EVIDENCE_DIR:-$root/performance-evidence}"
base_sha="${BASE_SHA:-}"
trials="${TRIALS:-7}"

if [[ -z "$base_sha" ]]; then
  echo "BASE_SHA is not set; skipping paired primitive-kernel comparison."
  exit 0
fi
if ! [[ "$trials" =~ ^[1-9][0-9]*$ ]]; then
  echo "TRIALS must be a positive integer" >&2
  exit 2
fi

mkdir -p "$output"
temporary="$(mktemp -d)"
cleanup() {
  git -C "$root" worktree remove --force "$temporary/base" >/dev/null 2>&1 || true
  rm -rf "$temporary"
}
trap cleanup EXIT

git -C "$root" worktree add --detach "$temporary/base" "$base_sha" >/dev/null

benchmark() {
  local label="$1"
  local directory="$2"
  local log="$output/primitive-kernel-${label}.log"
  : > "$log"

  (
    cd "$directory"
    cargo test --release --locked --lib capsule_and_wedge_query_benchmark --no-run
  ) >>"$log" 2>&1

  for trial in $(seq 1 "$trials"); do
    printf 'trial=%s\n' "$trial" >>"$log"
    (
      cd "$directory"
      cargo test --release --locked --lib capsule_and_wedge_query_benchmark -- \
        --ignored --nocapture --test-threads=1
    ) >>"$log" 2>&1
  done
}

benchmark base "$temporary/base"
benchmark head "$root"

echo "Paired primitive benchmark logs:"
echo "  base: $output/primitive-kernel-base.log"
echo "  head: $output/primitive-kernel-head.log"
