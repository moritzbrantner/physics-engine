#!/usr/bin/env bash
# Collector for the `performance:work` capability: runs tests/work_complexity.rs with the
# test-only work counters and prints one `coding-tooling/work-evidence/v1` document on stdout.
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
cd "$root"
contract=".performance/contract.json"
evidence_dir="$(mktemp -d)"
trap 'rm -rf "$evidence_dir"' EXIT
export PHYSICS_ENGINE_WORK_EVIDENCE_DIR="$evidence_dir"
cargo test --quiet --release --locked --features work-counters --test work_complexity -- --test-threads 1 >&2
operations=(ray-first ray-all overlap-shape cast-shape body-churn body-insert-middle body-churn-indexed move-characters place-character place-static terrain-ray terrain-capsule-sweep warm-tick-allocations)
fragments=()
for operation in "${operations[@]}"; do
  fragment="$evidence_dir/$operation.json"
  if [[ ! -s "$fragment" ]]; then
    printf 'missing work evidence for %s\n' "$operation" >&2
    exit 1
  fi
  fragments+=("$(<"$fragment")")
done
contract_sha="$(sha256sum "$contract" | cut -d' ' -f1)"
joined="$(IFS=,; printf '%s' "${fragments[*]}")"
printf '{"schemaVersion":"coding-tooling/work-evidence/v1","suite":"physics-engine","contractSha256":"%s","operations":[%s]}\n' "$contract_sha" "$joined"
