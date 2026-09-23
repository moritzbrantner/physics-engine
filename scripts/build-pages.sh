#!/usr/bin/env bash
set -euo pipefail

SETTINGS_BROWSER_DIST_COMMIT="1a268c485380eafb4e233a24c5803db4ff1f9ed0"
SETTINGS_SOURCE_SHA="4aff7dc2dbfcae0e245269bd3fe50f6afb8e19e8"
SETTINGS_RAW_BASE="https://raw.githubusercontent.com/moritzbrantner/settings/${SETTINGS_BROWSER_DIST_COMMIT}"
export SETTINGS_BROWSER_DIST_COMMIT SETTINGS_SOURCE_SHA

rustup target add wasm32-unknown-unknown
cargo build \
  --manifest-path demo-wasm/Cargo.toml \
  --target wasm32-unknown-unknown \
  --release \
  --locked

node --input-type=module <<'NODE'
import { readFile } from "node:fs/promises";

const bytes = await readFile(
  "demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm",
);
const { instance } = await WebAssembly.instantiate(bytes, {});
const exports = instance.exports;
const requiredFunctions = [
  "approximate_reset_tower",
  "approximate_position_stat",
  "sandbox_reset_with_options",
  "sandbox_reset_with_baking_options",
  "sandbox_reset_tower_with_baking_options",
  "sandbox_body_count",
  "sandbox_numeric_backend",
  "sandbox_body_sleeping",
  "sandbox_last_response_authority_body_count",
  "sandbox_last_parked_bodies_woken",
  "sandbox_last_parked_wake_retries",
  "sandbox_fixed_geometry_mode",
  "sandbox_fixed_geometry_prepared_count",
  "sandbox_fixed_geometry_total_preparations",
  "sandbox_fixed_geometry_retained_bytes",
  "sandbox_fixed_geometry_representation_version",
  "sandbox_simulation_rules",
  "sandbox_refresh_render_snapshot",
  "sandbox_render_snapshot_len",
  "sandbox_render_snapshot_stride",
];
for (const name of requiredFunctions) {
  if (typeof exports[name] !== "function") {
    throw new Error(`missing WASM sandbox export: ${name}`);
  }
}
if (exports.sandbox_numeric_backend() !== 64) {
  throw new Error("Pages must use the production f64 numerical backend, not exact-reference");
}
if (!(exports.memory instanceof WebAssembly.Memory)) {
  throw new Error("WASM module does not export linear memory for the render snapshot");
}

if (exports.sandbox_reset_with_baking_options(0, 0, 0) !== 0) {
  throw new Error("runtime fixed-geometry reference mode failed to initialize");
}
if (exports.sandbox_reset_tower_with_baking_options(0, 0, 0) !== 0) {
  throw new Error("tower scenario failed to initialize");
}
if (exports.sandbox_body_count() !== 44) {
  throw new Error(`tower scenario expected 44 bodies, got ${exports.sandbox_body_count()}`);
}
if (
  exports.sandbox_fixed_geometry_mode() !== 0 ||
  exports.sandbox_fixed_geometry_prepared_count() !== 0 ||
  exports.sandbox_fixed_geometry_total_preparations() !== 0 ||
  exports.sandbox_fixed_geometry_retained_bytes() !== 0
) {
  throw new Error("runtime fixed-geometry reference unexpectedly retained preparation");
}
if (exports.sandbox_reset_with_baking_options(0, 0, 1) !== 0) {
  throw new Error("prepare-at-load fixed geometry mode failed to initialize");
}
if (
  exports.sandbox_fixed_geometry_mode() !== 1 ||
  exports.sandbox_fixed_geometry_prepared_count() !== 11 ||
  exports.sandbox_fixed_geometry_total_preparations() !== 11 ||
  exports.sandbox_fixed_geometry_retained_bytes() <= 0 ||
  exports.sandbox_fixed_geometry_representation_version() !== 1
) {
  throw new Error("prepare-at-load fixed geometry evidence is invalid");
}

const explicitRules = (1 << 29) | ((1 << 11) - 2);
const interactionPolicyPack = (1 << 30) | 5 | (1 << 5) | (17 << 10);
if (exports.sandbox_reset_with_baking_options(explicitRules, interactionPolicyPack, 0) !== 0) {
  throw new Error("settings-backed pair-policy payload failed to initialize");
}
// Existing explicit-rule callers still use 0/1 as the compatibility argument.
if (exports.sandbox_reset_with_baking_options(explicitRules, 1, 0) !== 0) {
  throw new Error("legacy explicit-rule compatibility input regressed");
}

const pointer = exports.sandbox_refresh_render_snapshot();
const length = exports.sandbox_render_snapshot_len();
const stride = exports.sandbox_render_snapshot_stride();
if (stride !== 11 || length <= 0 || length % stride !== 0) {
  throw new Error(`invalid render snapshot layout: pointer=${pointer} length=${length} stride=${stride}`);
}
const byteEnd = pointer + length * Int32Array.BYTES_PER_ELEMENT;
if (pointer < 0 || byteEnd > exports.memory.buffer.byteLength) {
  throw new Error(
    `render snapshot exceeds WASM memory: pointer=${pointer} byteEnd=${byteEnd} memory=${exports.memory.buffer.byteLength}`,
  );
}
new Int32Array(exports.memory.buffer, pointer, length);
NODE

# Exercise the real shipped WASM, including sleep/retirement, rather than only counting bodies.
node scripts/benchmark-projectile-wake.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm \
  demo-wasm/target/pages-projectile-wake.json

# The canonical page is a different consumer from the comparison page: exercise its real adapter.
TOWER_TICKS=600 node scripts/test-tower-runtime.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm \
  demo-wasm/target/pages-tower-runtime.json
node --test site/tower-runtime.test.mjs

node --input-type=module --check < site/app.js
node --input-type=module --check < site/webgpu-renderer.js
node --input-type=module --check < site/webgl-renderer.js
node --input-type=module --check < site/physics-error.js
node --check site/bootstrap.mjs
node --check site/physics-settings.mjs
node --check site/physics-error.mjs
node --check site/interaction-controls.mjs
node --check site/simulation-rules.mjs
node --check site/simulation-rules-config.mjs
node --check site/performance-log.mjs
node --check scripts/adapt-performance-evidence.mjs
node --check scripts/package-performance-log.mjs
node --test site/physics-error.test.mjs site/interaction-controls.test.mjs site/simulation-rules-config.test.mjs site/performance-log.test.mjs scripts/adapt-performance-evidence.test.mjs scripts/package-performance-log.test.mjs scripts/summarize-cpu-profile.test.mjs

rm -rf pages-dist
mkdir -p pages-dist/vendor/settings/pkg
cp -R site/. pages-dist/
cp demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm pages-dist/

curl --proto '=https' --tlsv1.2 -fsSL \
  "${SETTINGS_RAW_BASE}/settings-browser.js" \
  -o pages-dist/vendor/settings/settings-browser.js
curl --proto '=https' --tlsv1.2 -fsSL \
  "${SETTINGS_RAW_BASE}/pkg/settings_wasm.js" \
  -o pages-dist/vendor/settings/pkg/settings_wasm.js
curl --proto '=https' --tlsv1.2 -fsSL \
  "${SETTINGS_RAW_BASE}/pkg/settings_wasm_bg.wasm" \
  -o pages-dist/vendor/settings/pkg/settings_wasm_bg.wasm
curl --proto '=https' --tlsv1.2 -fsSL \
  "${SETTINGS_RAW_BASE}/SOURCE_SHA" \
  -o pages-dist/vendor/settings/SOURCE_SHA

if [[ "$(tr -d '\r\n' < pages-dist/vendor/settings/SOURCE_SHA)" != "${SETTINGS_SOURCE_SHA}" ]]; then
  echo "Pinned settings browser distribution does not match expected source revision" >&2
  exit 1
fi

node --input-type=module <<'NODE'
import { createHash } from "node:crypto";
import { readFile, writeFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";

const wasm = await readFile("pages-dist/physics_engine_demo.wasm");
const settingsWasm = await readFile("pages-dist/vendor/settings/pkg/settings_wasm_bg.wasm");
const provenance = {
  schema_version: 1,
  numerical_backend: "float64",
  repository: "moritzbrantner/physics-engine",
  revision: execFileSync("git", ["rev-parse", "HEAD"], { encoding: "utf8" }).trim(),
  wasm_sha256: createHash("sha256").update(wasm).digest("hex"),
  settings: {
    repository: "moritzbrantner/settings",
    browser_dist_commit: process.env.SETTINGS_BROWSER_DIST_COMMIT,
    source_revision: process.env.SETTINGS_SOURCE_SHA,
    wasm_sha256: createHash("sha256").update(settingsWasm).digest("hex"),
  },
};
await writeFile("pages-dist/build-provenance.json", `${JSON.stringify(provenance, null, 2)}\n`);
NODE

test -s pages-dist/index.html
test -s pages-dist/catalog.css
test -s pages-dist/scenarios/sandbox/index.html
test -s pages-dist/scenarios/tower/index.html
test -s pages-dist/app.js
test -s pages-dist/bootstrap.mjs
test -s pages-dist/physics-settings.mjs
test -s pages-dist/webgpu-renderer.js
test -s pages-dist/webgl-renderer.js
test -s pages-dist/physics-error.js
test -s pages-dist/physics-error.mjs
test -s pages-dist/interaction-controls.mjs
test -s pages-dist/simulation-rules.mjs
test -s pages-dist/simulation-rules-config.mjs
test -s pages-dist/performance-log.mjs
test -s pages-dist/build-provenance.json
test -s pages-dist/physics_engine_demo.wasm
test -s pages-dist/vendor/settings/settings-browser.js
test -s pages-dist/vendor/settings/pkg/settings_wasm.js
test -s pages-dist/vendor/settings/pkg/settings_wasm_bg.wasm
test -s pages-dist/vendor/settings/SOURCE_SHA

test -s pages-dist/scenarios/fixed-step/index.html
test -s pages-dist/fixed-step-lab.js
