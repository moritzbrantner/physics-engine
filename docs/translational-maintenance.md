# Translational World maintenance

Issue #190 measures the actual `physics_engine::World` API imported by MMORPG. This API remains a translational integer-state compatibility surface. These changes do not select the rotating, parked, or persistent-f64 solver and do not establish a solver migration.

## Baseline and scope

The production baseline is `f3a5a08ad9c7e4dd15f7be5b5c59034a7012102c`. For a stationary 2048-body grid, 120 complete calls took about 97 ms in the stage probe: one broad-phase query consumed about 41 ms, and quantization plus successful map reconstruction about 12 ms. Ordinary stepping ran another broad-phase query for stabilization, staged every full body, and replaced the authoritative map after success. Times are advisory and compiler/target dependent.

The first repair retains no-op evidence after a successful converged stationary step. A positive-tick call can reuse it only when no dynamic body can receive gravity, all velocities are zero, and quantized poses exactly match the completed scaled poses. Mutations invalidate the evidence only when physical data actually changes. Repeated zero commands, identical replacements, and unsuccessful removals preserve it. New bodies, removal, kind/shape/mass/material/pose/velocity replacement invalidate it. It is disposable derived state, not physical sleep, and rebuilding a world from its bodies does not require serializing this cache.

Outside the retained no-op case, gravity still applies first for the complete requested interval. Zero post-stabilization velocities make the remaining constant-velocity sweep stationary, so its redundant future-impact query is omitted. Moving bodies retain the original full-interval CCD, simultaneous event ordering, remaining-time progression and event limit.

All fallible final position conversions complete before any authoritative write. Successful stepping updates only changed position and velocity fields in the existing BodyId-ordered map. A late quantization or event-limit error leaves every body unchanged. Caller-requested membership/whole-body replacement remains explicit. Tests also verify stable body-map slots across successful moving steps.

## Work and memory

`StepStats.work: TranslationalStepWork` exposes dynamic and initially moving populations, bodies staged/quantized, committed physical deltas, map reconstruction, bound preparation, sort/query invocations, active bound checks, fixed-pair rejection, stabilization/current-contact tests, and temporary vector capacities. Initially moving means after this step's gravity application. Memory fields describe vector capacities; they exclude allocator overhead, standard-library stable-sort scratch, and the existing authoritative body map. Failed calls retain the original error boundary and do not return a success report.

The API has no physical sleeping, layer filtering or rotating motion-authority modes; counters for those capabilities are absent. Its work counters follow the existing always-available translational diagnostics and are separate from the optional rotating/ballistic instrumentation.

After bootstrap, unchanged quiet ticks stage zero bodies, prepare zero bounds, sort/query zero times, allocate no working vector, and rebuild no map. Successful moving or supported-gravity steps also rebuild no body map. Sparse-motion steps still stage all admitted bodies and rebuild/sort broad-phase entries; supported gravity still performs real contact work. This bounded slice leaves retained scratch/index maintenance for those necessary active paths as follow-up work in #190.

## Verification and reproduction

A development-only rebuild-from-authoritative-state oracle uses an independent exhaustive pair traversal. Identical commands cover negative coordinates, movement, teleports, material/shape/mass/kind changes, admission/removal and same-ID reuse, dense spawns, gravity support/removal, zero stabilization, event limits, and late overflow/recovery. Compare ordered events, errors, full physical bodies and unchanged semantic counters, while allowing measured maintenance counters to differ. Existing thin-wall CCD and elastic/gravity controls remain unchanged.

Run the same public-API workload fixture against the pinned baseline and candidate:

```sh
cargo test --release --locked --test translational_maintenance_benchmark -- --ignored --nocapture
cargo test --locked --lib world::maintenance_tests
cargo test --release --locked --lib world::maintenance_tests::translational_necessary_work_ratchet -- --ignored --nocapture
```

The whole-call matrix uses the consumer's 30/90/30 body dimensions, 120 ticks per trial, three replays, and includes bootstrap. Quiet zero-gravity populations are separate from sparse travel, stationary support under gravity and a crowded-spawn control. Every intermediate body/event trace contributes to a same-build diagnostic checksum; differential tests compare full values independently. Timing excludes commands and replay validation. Deterministic work assertions and the existing append-only ratchet are blocking; wall-clock results are advisory.

Local median whole-call totals (120 ticks):

| Workload | Bodies | Moving | Baseline ms | Candidate ms |
| --- | ---: | ---: | ---: | ---: |
| Quiet, zero gravity | 128 | 0 | 1.936 | 0.017 |
| Quiet, zero gravity | 512 | 0 | 10.984 | 0.103 |
| Quiet, zero gravity | 2048 | 0 | 95.811 | 0.844 |
| Sparse travel | 512 | 1 | 11.011 | 10.542 |
| Sparse travel | 512 | 8 | 11.092 | 10.455 |
| Stationary gravity support | 513 | 0 | 19.797 | 13.251 |
| Crowded spawn | 64 | 0 | 5.605 | 5.088 |

All seven physical/event traces match the previous production implementation in all three trials. The crowded case still reaches its configured eight stabilization passes; it is not relabeled a contact-quality repair. These are engine-call measurements, not full-zone performance claims. Actual MMORPG dependency adoption and equivalent zone/canonical/recovery validation remain required by #190/#46.

Resolved conventions sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
