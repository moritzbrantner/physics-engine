#!/usr/bin/env bash
# size:budget: run the WASM oracle (which builds each engine's lone WASM into target/size/), then
# judge those files against .performance/size.json. CODING_TOOLING overrides the CLI command
# (CI points it at a pinned checkout).
#
# SIZE_BASE_REF (a commit, set by CI on pull requests) also measures that commit's WASM in the
# same run and passes it as the baseline, so maxIncreaseBytes is checked against the base branch.
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
ct performance size --root . "${baseline[@]}"
