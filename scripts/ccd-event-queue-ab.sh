#!/usr/bin/env bash
# Same-machine A/B of the #484 CCD safety baseline and the current queue experiment.
# Timings come from the existing 31-sample whole-step fixture, not instrumentation.
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
baseline_ref="${1:-8739c321db022a5a0e0596805a39ac4678c178f2}"
candidate_ref="${2:-HEAD}"
baseline_sha="$(git -C "$repo_root" rev-parse --verify "${baseline_ref}^{commit}")"
candidate_sha="$(git -C "$repo_root" rev-parse --verify "${candidate_ref}^{commit}")"
if [[ "$baseline_sha" == "$candidate_sha" ]]; then
  printf 'Baseline and candidate resolve to the same commit: %s\n' "$baseline_sha" >&2
  exit 2
fi

worktree_root="$(mktemp -d)"
baseline_tree="$worktree_root/baseline"
candidate_tree="$worktree_root/candidate"
cleanup() {
  git -C "$repo_root" worktree remove "$candidate_tree" >/dev/null 2>&1 || true
  git -C "$repo_root" worktree remove "$baseline_tree" >/dev/null 2>&1 || true
  rmdir "$worktree_root" >/dev/null 2>&1 || true
}
trap cleanup EXIT

git -C "$repo_root" worktree add --quiet --detach "$baseline_tree" "$baseline_sha"
git -C "$repo_root" worktree add --quiet --detach "$candidate_tree" "$candidate_sha"

mkdir -p "$repo_root/target/ccd-event-queue-ab"
log_dir="$(mktemp -d "$repo_root/target/ccd-event-queue-ab/run-XXXXXX")"

{
  printf 'baseline=%s\ncandidate=%s\n' "$baseline_sha" "$candidate_sha"
  rustc --version
  uname -srm
  if command -v lscpu >/dev/null 2>&1; then
    lscpu | grep -E '^(Model name|CPU\(s\)|Architecture):' || true
  fi
} | tee "$log_dir/environment.txt"

run_timing() {
  local round="$1" label="$2" tree
  if [[ "$label" == baseline ]]; then
    tree="$baseline_tree"
  else
    tree="$candidate_tree"
  fi
  printf '\n=== %s: %s ===\n' "$round" "$label"
  (
    cd "$tree"
    # Keep each revision's compiled test binary distinct across Cargo worktrees.
    export CARGO_TARGET_DIR="$repo_root/target/ccd-event-queue-ab/build-$label"
    cargo test --release --locked --test ccd_dense_scaling \
      print_dense_first_tick_timings -- --ignored --nocapture
  ) 2>&1 | tee "$log_dir/$round-$label.log"
}

# ABBA limits interpretation errors from one-way thermal or shared-host drift.
run_timing 1 baseline
run_timing 2 candidate
run_timing 3 candidate
run_timing 4 baseline

printf '\n=== Candidate deterministic queue-work counts (timings here are advisory) ===\n'
(
  cd "$candidate_tree"
  export CARGO_TARGET_DIR="$repo_root/target/ccd-event-queue-ab/build-candidate"
  cargo test --release --locked --features work-counters --test ccd_dense_scaling \
    print_dense_first_tick_timings -- --ignored --nocapture
) 2>&1 | tee "$log_dir/5-candidate-work.log"

printf '\n64-sphere and co-moving controls (send these lines or the logs back):\n'
grep -H -E 'ccd-dense case=(converging-true-impacts n=64|co-moving-aabb-overlap n=128)' "$log_dir"/*.log
printf '\nFull logs: %s\n' "$log_dir"
