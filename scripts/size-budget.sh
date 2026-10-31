#!/usr/bin/env bash
# size:budget: run the WASM oracle (which builds each engine's lone WASM into target/size/), then
# judge those files against .performance/size.json. CODING_TOOLING overrides the CLI command
# (CI points it at a pinned checkout).
set -euo pipefail
cd "$(dirname "$0")/.."
node scripts/oracle-wasm.mjs
# shellcheck disable=SC2086 # CODING_TOOLING is a command line, split on purpose.
${CODING_TOOLING:-coding-tooling} performance size --root .
