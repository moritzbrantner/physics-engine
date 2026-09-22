# Contact-triggered activation of parked bodies

Settled bodies remain collision-testable but do not participate in integration or constraint
response until an admitted collision requires it. Projectile creation, a swept bounding-box
candidate, and unrelated projectile removal are not wake evidence.

## Admission and response

1. Keep the existing stationary collision proxies and retained broad phase. Run the existing
   swept rigid or analytic ballistic narrow phase; do not replace CCD with endpoint overlaps.
2. Before response, `ContactWakeGuard3d` checks the admitted contacts against parked membership,
   interaction policies, and passive linear-actuator support. A near miss never reaches activation.
3. A genuine contact with a parked target interrupts the **uncommitted core attempt**. Restore
   the target's dynamic contact island with its real mass/inertia and retry the requested step.
   Every retry removes at least one parked body, so retries are bounded by initial parked membership.
4. Check subsequent contacts as well, including ballistic impacts, rigid recontacts, stabilization,
   and persistent-tail response. A same-step wall ricochet can therefore activate a target that was
   not on the projectile's original trajectory.

The core already stages rigid bodies in one working buffer. Analytic projectiles are now staged
until the same commit boundary, preventing a discarded wake probe from advancing or retiring a
projectile twice. This does not introduce a second persistent world. Fixed-boundary and sleep
bookkeeping after a successful core commit retain their existing failure semantics; this is not a
claim of arbitrary whole-facade rollback.

Dynamic island traversal uses current contact evidence and the parked bounds index. A genuinely
fixed floor cannot bridge independent dynamic islands. Contact wake propagation still obeys
`WakePropagation3d`; removal is a separate topology authority and wakes contact dependents even
when collision wake propagation is disabled. Removing a remote projectile does not wake the tower.
A dependency-query error falls back conservatively for removal, whose existing API returns `Option`.

## Work accounting

`parked_bodies_woken` and `parked_wake_retries` record activation and discarded probes. Ordinary
step counters describe the committed attempt. Additional `wake_probe_broad_phase_queries`,
`wake_probe_tail_broad_phase_queries`, and `wake_probe_response_passes` expose discarded work.
The sandbox performance comparison includes probe queries in its existing query budgets; probe
response passes include all response stages, not just the chronological event lane. Wall timings
include every attempt. A near-miss run must have zero activation and zero wake probes.

## Acceptance

```sh
cargo test --locked --test projectile_wake_admission
cargo test --manifest-path demo-wasm/Cargo.toml --locked --release --lib projectile_wake_tests
cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
node scripts/benchmark-projectile-wake.mjs demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm near-misses.json
```

The Pages build runs this matrix as a required check on the shipped WASM; its JSON is written
to `demo-wasm/target/pages-projectile-wake.json`. The Pages acceptance matrix runs free/upright crates, physical/linear character modes, and
sphere/arrow/rigid projectiles twice each. Three near misses are followed through retirement.
Each tick verifies that all 32 crates remain asleep and retain their poses. Native coverage
also compares complete body state and compares real impacts against already-awake references.
It covers broad-phase false positives, wall shielding, same-step ricochets, simultaneous independent
hits, collision layers, no-wake policy, and support removal.

This admission change is not a replacement for the floating-state persistent-manifold solver.
The separate direct-impact tower matrix remains an independent acceptance case. Do not label a
near-miss pass as proof that the free-rotating tower survives direct projectile impacts. The default
numerical backend remains f64; no exact-reference fallback or raised event budget is introduced.
