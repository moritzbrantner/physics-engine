# Floating-state failed-step boundary

The `approximate::World::step` returned-error contract is atomic. `InvalidInput` and numerical errors preserve every authoritative body, queued force/torque/impulse, sleep/quiet timer, warm-start point/impulse, membership, substep history and elapsed time. The previous successful `last_report` stays available. An error returns no successful retirement/event report. Zero-duration calls keep their existing successful no-op report behavior. Rust panics/abort, including allocation failure, are outside the returned-error guarantee.

## Transaction mechanics

The world remains the sole mutable authority. Before the first mutation in an attempt, a journal records only the touched body's motion/pending-input fields. Immutable shape, mass, material, layers and authority metadata are not copied for ordinary motion. A retired body moves into the lifecycle journal so its exact metadata can be reinserted if a later substep fails. Motion before-images restore by BodyId after membership restoration; generational slot stamps track same-step index shifts and avoid repeated copies. Epoch rollover clears the stamps safely.

Warm-start contact history is authoritative continuation data. The first update/removal of a pair records its previous ordered points and impulses, or its prior absence. Failure restores that history and the prior substep duration. Bounds, prepared response, geometry caches and indexed adjacency/activity are disposable accelerators and are invalidated against the restored boundary. No complete World or full body vector is cloned for rollback.

All successful integration/contact arithmetic, constraint order, timestep, limits and physical quality remain unchanged. Reports are published after the final fallible stage. `release_transaction_scratch()` frees disposable rollback storage at an explicit call boundary without dropping warm starts, pending inputs or physical sleep history.

## Work and memory

`last_step_transaction()` reports the most recent attempt's journaled bodies/pairs, copied contact points, lifecycle removals, vector growths/capacities and whether rollback occurred. Calls that reject input or need no solver journal nothing. Vector capacities exclude allocator/BTreeMap node overhead and authoritative storage; they are not RSS. Motion/stamp/lifecycle vectors retain their high-water capacity until explicitly released. Pair before-image vectors and map entries live only for the attempt; their copying/allocation remains visible, rather than being presented as free retention.

`last_failed_step_work()` separately exposes discarded numerical-attempt work: attempted substeps, candidates/narrow/contact work, integration attempts, waking, retirement counts, constraint visits, position contact tests, bookkeeping and geometry work. It contains no successful retired-body identities/events. Failed wake/contact work is counted even though its physical changes were rolled back. Input rejection and successful calls expose no failed work.

Steady local steps among 128 or 512 sleeping bodies journal the same two bodies (responding box and fixed floor), one contact pair and four previous contact points per tick. Over 64 completed calls both populations record 128 body before-images and 256 copied points. Explicitly released quiet worlds record zero journaling/growth; stamp memory for active transactions still scales with admitted slots. Full active scenes can legitimately journal every active body. This is a transaction guarantee with measured cost, not a claim to finish all retained-storage work under #197.

## Evidence and reproduction

```sh
cargo test --locked --test floating_step_atomicity
cargo test --features exact-reference,experimental-soft-contact --locked --test floating_step_atomicity
cargo test --features f32-physics --locked --test floating_step_atomicity
cargo test --release --locked --lib approximate::transaction::tests::transaction_work_ratchet -- --ignored --nocapture
cargo test --release --locked --test floating_transaction_benchmark -- --ignored --nocapture
PHYSICS_TRANSACTION_TRACE_DIR=performance-evidence/floating-traces cargo test --release --locked --test floating_transaction_benchmark -- --ignored --nocapture
```

The original one-body regression demonstrated a changed pose and consumed impulse after `NonFiniteState`. The repaired public-API regressions also cover repeated failures, earlier body changes, fixed-position correction, retirement before a later failing substep, waking with existing warm starts, sleep timers, changed timesteps, continuation after recovery, scratch release and optional experimental relaxation. A failed wake's discarded sweep/constraint work remains observable. Existing thin-wall CCD, impact retirement, energy/impulse and settling controls remain unchanged.

The successful whole-call matrix runs eight scenarios in three trials, 120 completed ticks each, with setup/commands/export/trace validation outside timing. Every intermediate body's exact round-trip float representation, private physical fields, canonical order, elapsed time and semantic report values contribute to the trace. All 24 full binary traces match production baseline `0baf3411419fc250273caec24d64654cb30c28ec` byte for byte. Quiet/local controls are prepared separately from free motion, a 48-body dense stack and a retiring CCD projectile. The dense control is not relabeled a quality repair.

Advisory median whole-step totals for 120 ticks:

| Workload | Dynamic bodies | Before ms | Atomic ms |
| --- | ---: | ---: | ---: |
| free | 128 | 5.240 | 5.165 |
| free | 512 | 19.511 | 20.620 |
| quiet | 128 | 0.011 | 0.011 |
| quiet | 512 | 0.016 | 0.016 |
| local | 128 | 2.160 | 2.230 |
| local | 512 | 6.456 | 6.520 |
| stack | 48 | 472.172 | 491.195 |
| impact | 48 | 1.219 | 1.251 |

Free-512 adds about 5.7% and the dense stack about 4.0% in this local observation; quiet/local/impact changes are small. These timings quantify the extra returned-error guarantee, not a speedup or portable throughput gate. Failed-call work is recorded separately with zero completed ticks; a failed/shortened duration is never counted as successful simulation. Five deterministic transaction fences extend the append-only suite without replacing prior physics/quality controls.

Fingerprint: Rust 1.98.0, `x86_64-unknown-linux-gnu`, release profile, empty RUSTFLAGS, Linux, AMD Ryzen 7 5700X, identical public fixture/source and committed dependency graph on both sides. Resolved conventions sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.

## Remaining migration boundary

This completes the returned-error transaction slice for this selected floating world. The separate [physical checkpoint slice](floating-checkpoints.md) now preserves continuation history. General public character/query support, analytic rotational CCD and multiplayer solver adoption remain pending. Those acceptance criteria remain under #198, #194 and #3. MMORPG still uses the translational World adopted in PR #47; changing that authority requires its separate canonical/recovery/gameplay acceptance.

Shared capsule/wedge search failures also return through this rollback boundary.
See [checked primitive searches](checked-primitive-sweeps.md) for ordered pair/reason
errors, unchanged budgets and discarded-work counters.
