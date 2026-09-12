#!/usr/bin/env bash
set -euo pipefail

changed_file_list="${1:-}"
if [[ -z "$changed_file_list" || ! -f "$changed_file_list" ]]; then
  echo "usage: $0 <changed-file-list>" >&2
  exit 2
fi

changed="$(cat "$changed_file_list")"
ran=0

run_integration() {
  local target="$1"
  [[ -f "tests/${target}.rs" ]] || return 0
  echo "::group::release performance evidence: integration target ${target}"
  /usr/bin/time -v cargo test --release --locked --test "$target" -- \
    --ignored --nocapture --test-threads=1
  echo "::endgroup::"
  ran=1
}

run_library_module() {
  local module="$1"
  echo "::group::release performance evidence: library module ${module}"
  /usr/bin/time -v cargo test --release --locked --lib "${module}::tests::" -- \
    --ignored --nocapture --test-threads=1
  echo "::endgroup::"
  ran=1
}

if grep -Eq '(^|/)(query\.rs|ray_query_performance\.rs)$' <<<"$changed"; then
  run_integration ray_query_performance
fi

if grep -Eq '(^|/)(rigid_box_free_flight\.rs|repeated_rotating_events\.rs|current_contact_broadphase\.rs)$' <<<"$changed"; then
  run_integration current_contact_broadphase
fi

if grep -Eq '(^|/)(rotating_broad_phase\.rs|rotating_broad_phase_tree\.rs)$' <<<"$changed"; then
  run_library_module rotating_broad_phase
fi

if grep -Eq '(^|/)(rotating_contact_search\.rs|rotating_contact_cache_performance\.rs)$' <<<"$changed"; then
  run_integration rotating_contact_cache_performance
fi

if grep -Eq '(^|/)(rotating_recontact_search\.rs|rotating_recontact_cache_performance\.rs)$' <<<"$changed"; then
  run_integration rotating_recontact_cache_performance
fi

if grep -Eq '(^|/)(obb_response\.rs|rotating_contact_response\.rs|contact_solver_scratch_performance\.rs)$' <<<"$changed"; then
  run_integration contact_solver_scratch_performance
fi

if [[ "$ran" -eq 0 ]]; then
  echo "No changed file maps to release performance evidence."
fi
