#!/usr/bin/env bash
set -euo pipefail

changed_file_list="${1:-}"
if [[ -z "$changed_file_list" || ! -f "$changed_file_list" ]]; then
  echo "usage: $0 <changed-file-list>" >&2
  exit 2
fi

changed="$(cat "$changed_file_list")"
ran=0
output="${PERFORMANCE_EVIDENCE_DIR:-$(pwd)/performance-evidence}"
mkdir -p "$output/rust"

run_integration() {
  local target="$1"
  [[ -f "tests/${target}.rs" ]] || return 0
  echo "::group::release performance evidence: integration target ${target}"
  /usr/bin/time -v cargo test --release --locked --test "$target" -- \
    --ignored --nocapture --test-threads=1 2>&1 | tee "$output/rust/${target}.log"
  echo "::endgroup::"
  ran=1
}

run_library_module() {
  local module="$1"
  echo "::group::release performance evidence: library module ${module}"
  /usr/bin/time -v cargo test --release --locked --lib "${module}::tests::" -- \
    --ignored --nocapture --test-threads=1 2>&1 | tee "$output/rust/${module}.log"
  echo "::endgroup::"
  ran=1
}

if grep -Eq '(^|/)(query\.rs|ray_query_performance\.rs)$' <<<"$changed"; then
  run_integration ray_query_performance
fi

if grep -Eq '(^|/)(rigid_box_free_flight\.rs|repeated_rotating_events\.rs|current_contact_broadphase\.rs)$' <<<"$changed"; then
  run_integration current_contact_broadphase
fi

if grep -Eq '(^|/)(current_contact_query\.rs|rotating_world\.rs|stabilized_rotating_world\.rs|relaxed_rotating_world\.rs|support_query\.rs)$' <<<"$changed"; then
  run_library_module current_contact_query
fi

if grep -Eq '(^|/)(stabilized_rotating_world\.rs|relaxed_rotating_world\.rs|sleep_parking_performance\.rs)$' <<<"$changed"; then
  run_integration sleep_parking_performance
fi

if grep -Eq '(^|/)(stabilized_rotating_world\.rs)$' <<<"$changed"; then
  run_library_module stabilized_rotating_world
fi

if grep -Eq '(^|/)(rotating_broad_phase\.rs|rotating_broad_phase_tree\.rs)$' <<<"$changed"; then
  run_library_module rotating_broad_phase
fi

if grep -Eq '(^|/)(rotating_contact_search\.rs|rotating_contact_cache_performance\.rs)$' <<<"$changed"; then
  run_integration rotating_contact_cache_performance
fi

if grep -Eq '(^|/)(rotating_contact_search\.rs|indexed_contact_cache_performance\.rs|bounded_contact_search_performance\.rs)$' <<<"$changed"; then
  run_integration indexed_contact_cache_performance
  run_integration bounded_contact_search_performance
fi

if grep -Eq '(^|/)(rotating_recontact_search\.rs|rotating_recontact_cache_performance\.rs|recontact_scaling_performance\.rs)$' <<<"$changed"; then
  run_integration rotating_recontact_cache_performance
  run_integration recontact_scaling_performance
fi

if grep -Eq '(^|/)(obb_response\.rs|rotating_contact_response\.rs|contact_solver_scratch_performance\.rs)$' <<<"$changed"; then
  run_integration contact_solver_scratch_performance
fi

if [[ "$ran" -eq 0 ]]; then
  echo "No changed file maps to release performance evidence."
fi
