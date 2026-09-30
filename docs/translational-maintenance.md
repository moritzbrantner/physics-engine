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

## Retained body-staging capacity

The focused #226 follow-up retains the `BodyState` vector across necessary active
steps. The authoritative body map remains unchanged as an ownership boundary:
each active call restages current bodies in BodyId order, applies the same solver,
validates every final position and commits the same physical deltas. Staged values
are cleared on every success or returned error; only empty vector capacity survives.
Panics are outside the returned-error contract. No world snapshot or previous staged
physical state is reused for continuation.

`TranslationalStepWork::staged_state_capacity_growths` reports actual staging capacity
increases on successful calls. A warmed active call within the retained capacity
reports zero. Population growth can reserve more capacity; shrink/removal retains
the high-water mark. The existing `staged_state_capacity_bytes` field still measures
active staging payload, so cached stationary calls continue to report zero work and
zero active payload. `World::retained_step_scratch_bytes()` separately reports the
retained staging payload even after a cached call or returned failure. This excludes
allocator overhead, authoritative bodies, event output and broad-phase vectors.
Returned errors still have no success work report; the capacity getter does not
claim to count discarded solver work.

`World::release_step_scratch()` releases that disposable capacity without changing
physical bodies or stationary evidence. An unchanged quiet call still does no work
after release; a later active call allocates again. There is no automatic shrink
policy on ordinary ticks, and no second persistent physics authority.

Native exhaustive/rebuilding controls retain exact body, event and error parity
through mutation, support, dense and failure sequences. A narrower storage test
checks allocation identity and empty logical contents across warmed active calls
and repeated late failures. Shared public native/WASM controls cover growth/shrink,
same-ID replacement, release, early gravity overflow, event exhaustion, late
quantization failure and recovery against a fresh authoritative-state control.
The ordinary fast tier and existing Pages build execute those controls; no workflow
or diagnostic production export is added.

```sh
cargo test --locked --lib world::maintenance_tests
cargo test --locked --test translational_staging
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example translational-maintenance-contract
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release \
  --target wasm32-unknown-unknown --example translational-maintenance-contract
node scripts/test-translational-maintenance-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/translational_maintenance_contract.wasm
```

This removes repeated staging capacity acquisition, not all-N body staging/cloning,
quantization, broad-phase reconstruction or sorting. Broad-phase capacity and
dependency-valid fixed-bound reuse are separate #227/#228 slices. Further consumer
adoption and delta/neighborhood maintenance remain #190 requirements.
