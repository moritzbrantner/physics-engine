# Sandbox performance evidence

`node scripts/benchmark-sandbox.mjs <head.wasm> <results.json>` runs the stable `sandbox-projectiles-v5` workload. Build the module with `cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked`.

Each case resets and settles the actual Rust sandbox for 240 ticks. A separate 120-tick walking warmup precedes measurement. The ordinary cases measure 180 canonical 1/60-second physics calls: settled idle, walking without shots, three projectiles while stationary, three projectiles while walking, and a six-projectile scaling case. The v5 stress case measures 240 ticks while walking and fires 30 deterministic sphere projectiles at five-tick intervals, specifically to expose population-driven tail and stabilization work. Keep the workload version and inputs stable for historical comparisons.

Set `TRIALS=3` for repeated measurements, `CASE=three-shots-walking` for the representative journey, or `CASE=thirty-shots-walking` for the high-population stress trace. Set `BASELINE_WASM=/path/to/base.wasm` to execute the same workload against an earlier artifact in the same process. Output includes module SHA-256, Node/V8/CPU metadata, raw tick timings, percentiles, explicit solver-work counters, and an observable per-tick replay fingerprint covering body poses, grounding, quiescence, and collision counts. It does not expose every private solver field. Replay must remain deterministic within one build. Head/base replay differences are recorded rather than rejected so deliberate deterministic physics changes can be reviewed together with correctness and performance evidence. Machine-dependent timing remains advisory.

## Canonical Performance Evidence

The raw benchmark JSON remains useful diagnostic evidence, but it is no longer the primary portable record. `scripts/package-performance-log.mjs` adapts the deterministic sandbox result to the canonical contract owned by [`performance-evidence`](https://github.com/moritzbrantner/performance-evidence), pinned by exact revision in the workflow and bundle provenance.

Each sandbox case/trial becomes one canonical Performance Evidence document under `canonical/sandbox/`. The document records:

- exact source revision and dirty-state provenance;
- a deterministic workload fingerprint and scenario parameters;
- a stable environment fingerprint plus collector/toolchain metadata;
- **useful work**, such as completed simulation steps and produced collision events;
- **induced work**, such as sampled events, candidate pairs, broad-phase work, event-response passes, and stabilization work;
- **outcomes**, including step-time distributions and final body count;
- hashed references to the raw benchmark, benchmark source, and physics performance policy;
- an exact hash reference to the corresponding baseline evidence when a base build was collected.

The physics-owned measurement semantics are declared in `.performance/sandbox-evidence-profile.json`. The interchange structure and validation semantics remain owned by `performance-evidence`. `.performance/contract.json` remains separately responsible for physics-engine regression budgets and blocking policy; representation and policy are intentionally not conflated.

Interactive browser captures use the same canonical interchange boundary but remain diagnostic. Their raw frame and marker stream stays attached as a hashed artifact while aggregate browser and physics-work measurements are emitted under `canonical/browser/`. Browser scheduling, rendering, device, and input variability mean these captures do not replace deterministic benchmark evidence.

`bash scripts/collect-performance-log.sh [browser-session.json ...]` creates a portable `physics-performance-evidence-<sha>.tar.gz` bundle. Start analysis with `manifest.json` and the documents under `canonical/`, then inspect raw artifacts or CPU profiles when a counter or timing distribution needs deeper diagnosis. `SHA256SUMS` covers the entire bundle.

## CI boundary

The Performance Evidence workflow checks out the exact PR head, builds both that head and its base with the same runner/toolchain, and retains the modules, script, environment, raw JSON, CPU profile, canonical evidence, profiles, and portable archive. It then checks out the pinned `performance-evidence` revision and validates the generated documents against that repository's canonical schemas and semantic invariants. `bash scripts/run-sandbox-performance.sh` performs the same benchmark build-and-run locally; optional `BASE_SHA` selects the comparison commit.

These are isolated WASM physics timings, not browser FPS, hardware-GPU measurements, or end-to-end input latency. Rendering and replay fingerprinting are outside the timed physics call.

## Optimization boundaries

Fixed bodies use their exact current quantized OBB bounds even for nonzero intervals; their canonical free-flight samples cannot translate or rotate. Dynamic rotational sweeps remain conservative. Re-contact search prepares quantized geometry once for reusable samples and memoizes one exact pair result while both complete shapes remain identical. Cache lifetime is one immutable search invocation, with a maximum of 4,096 coarse entries plus one latest preparation per input body and one pair-result slot. Cache exhaustion falls back to canonical computation. No sampling resolution, refinement count, event limit, contact ordering, impulse arithmetic, sleep policy, or fail-closed stepping rule is relaxed.
