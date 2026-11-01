#!/usr/bin/env bash
# size:budget: run the WASM oracle (which builds each engine's lone WASM into target/size/), then
# report those files against .performance/size.json. CODING_TOOLING overrides the CLI command
# (CI points it at a pinned checkout).
#
# SIZE_BASE_REF (a commit, set by CI on pull requests) also measures that commit's WASM in the
# same run and passes it as the baseline, so growth is reported against the base branch.
#
# The size budget is advisory (PRINCIPLE-015): a size over maxBytes or growth over
# maxIncreaseBytes prints a warning (plus a GitHub annotation and step summary in CI) so an
# optimization issue can be filed, but never fails. The WASM oracle (parity + 1.10x perf gate)
# stays blocking.
set -euo pipefail
cd "$(dirname "$0")/.."
repo="$PWD"
# shellcheck disable=SC2086 # CODING_TOOLING is a command line, split on purpose.
ct() { ${CODING_TOOLING:-coding-tooling} "$@"; }
baseline=()
if [[ -n "${SIZE_BASE_REF:-}" ]]; then
  base="$repo/target/size-base-src"
  rm -rf "$base" && mkdir -p "$base"
  git archive "$SIZE_BASE_REF" | tar -x -C "$base"
  (cd "$base" && CARGO_TARGET_DIR="$repo/target" node "$repo/scripts/oracle-wasm.mjs" --size-only)
  # A base over its own budget (exit 1) is still a usable baseline.
  ct performance size --root "$base" --report "$repo/target/size-base.json" --json > /dev/null ||
    [[ $? -eq 1 ]]
  baseline=(--baseline target/size-base.json)
fi
node scripts/oracle-wasm.mjs
status=0
table=$(ct performance size --root . "${baseline[@]}") || status=$?
printf '%s\n' "$table"
# 2 (unavailable) and 3 (error) mean the measurement itself broke; only budgets are advisory.
[[ $status -le 1 ]] || exit "$status"
if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
  {
    echo '### WASM size (advisory)'
    echo '```'
    printf '%s\n' "$table"
    echo '```'
  } >> "$GITHUB_STEP_SUMMARY"
fi
if [[ $status -eq 1 ]]; then
  msg="WASM size over its .performance/size.json budget (maxBytes or maxIncreaseBytes);"
  msg+=" advisory per PRINCIPLE-015 - file an optimization issue"
  echo "warning: $msg" >&2
  [[ -z "${GITHUB_ACTIONS:-}" ]] || echo "::warning title=WASM size budget::$msg"
fi
