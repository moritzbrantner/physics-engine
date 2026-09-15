# Sandbox performance evidence

`node scripts/benchmark-sandbox.mjs <head.wasm> <results.json>` runs the stable `sandbox-projectiles-v1` workload. Build the module with `cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked`.

Each case resets and settles the actual Rust sandbox for 240 ticks. A separate 120-tick walking warmup precedes measurement. Cases each measure 180 canonical 1/60-second physics calls: settled idle, walking, three projectiles while stationary, and three projectiles while walking. Shots occur at ticks 0, 40, and 80 with the same directions as the September 15 lag investigation. Keep this workload version and inputs stable for historical comparisons.

Set `TRIALS=3` for repeated measurements, or `CASE=three-shots-idle` to isolate one case. Set `BASELINE_WASM=/path/to/base.wasm` to execute the same workload against an earlier artifact in the same process. Output includes module SHA-256, Node/V8/CPU metadata, raw tick timings, percentiles, and an observable per-tick replay fingerprint covering body poses, grounding, quiescence, and collision counts. It does not expose every private solver field. A baseline comparison fails on replay differences, not on machine-dependent timing thresholds. Correctness tests remain separate.

The existing Performance Evidence workflow checks out the exact PR head, builds both that head and its base with the same runner/toolchain, and retains the modules, script, environment, and JSON results. `bash scripts/run-sandbox-performance.sh` performs the same build-and-run locally; optional `BASE_SHA` selects the comparison commit.

These are isolated WASM physics timings, not browser FPS, hardware-GPU measurements, or end-to-end input latency. Rendering and replay fingerprinting are outside the timed physics call.

## Optimization boundaries

Fixed bodies use their exact current quantized OBB bounds even for nonzero intervals; their canonical free-flight samples cannot translate or rotate. Dynamic rotational sweeps remain conservative. Re-contact search prepares quantized geometry once for reusable samples and memoizes one exact pair result while both complete shapes remain identical. Cache lifetime is one immutable search invocation, with a maximum of 4,096 coarse entries plus one latest preparation per input body and one pair-result slot. Cache exhaustion falls back to canonical computation. No sampling resolution, refinement count, event limit, contact ordering, impulse arithmetic, sleep policy, or fail-closed stepping rule is relaxed.
