# Prepared fixed-step response coefficients (issue #154)

## Scope

The fixed-step solver still uses f64, four substeps, eight sequential-impulse passes,
the same contact ordering, and the same CCD, wake, friction, support and sleep rules.
This change moves inverse-mass and shape/mass-dependent local inverse-inertia coefficient
construction out of repeated effective-mass and impulse applications.

`approximate::World` owns a reusable, body-indexed `responses` scratch vector. At each
substep it refreshes the vector against the current body layout. Immovable and sleeping
bodies have zero response; rotation-locked bodies retain inverse mass but no angular response.
Newly awakened dynamics are prepared before contact constraints apply their real mass/inertia.
Retirement can change indices only after the final use of that substep's preparation; the
next substep rebuilds the indexed contents. Zero-duration and quiescent steps do not prepare.
External impulses still apply once before the first substep's force integration.

The scratch is derived data, never a second store of body motion. Public body changes happen
through world operations or remove/reinsert; preparation does not survive the next substep
without refresh. Allocated capacity is retained, but no coefficient value is trusted merely
because its old index happens to exist.

## Numerical behavior

The local coefficients use the original formulas. Quaternion inverse rotation, component
multiplication, rotation and final vector additions keep their original evaluation order.
Orientation is read from authoritative body state. It does not change during velocity solving;
it is integrated after the last use of the current substep's response.

This deliberately does **not** collapse inverse inertia to a world-space matrix or pre-scale
normal/tangent impulse vectors. Those transformations regroup floating-point arithmetic and
need separate accuracy/performance evidence. They are not necessary to remove repeated
shape/mass coefficient construction in this slice.

A compile-time false specialization of the private stepping method is instantiated only by
unit tests. It reconstructs response coefficients at each use and provides a full-body replay
oracle. There is no user-visible runtime precision or solver toggle. Independent unit tests
also compare the prepared vector operation against the original inverse-inertia formula.

## Deterministic work evidence

Reports add `response_preparations`, `inertia_preparations`, and `inertia_applications`.
The corresponding existing generic WASM stat indices are 11, 12 and 13. The previous indices
and snapshot layouts are unchanged. Preparations count active bodies; applications count
angular vector evaluations, including force/impulse integration, effective-mass preparation,
warm starting and solver iterations. Locked/fixed/sleeping bodies do not count angular work.
These are source-level work counters, not counts of hardware instructions after optimization.

The focused iteration-count test keeps two angularly responsive bodies and one substep:
1, 8 and 32 iterations all prepare exactly two inertia coefficient sets. Vector applications
increase with iteration count, and each prepared replay exactly matches its unprepared oracle.

## Reproduce

```sh
cargo test --locked --lib approximate::response::tests
cargo test --locked --test fixed_step_approximation
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release --target wasm32-unknown-unknown
node scripts/benchmark-prepared-response.mjs base.wasm candidate.wasm prepared-response.json
```

Build `base.wasm` from the pre-change `090fe521bbfe539deeefc6a5c3741677836ee7c4`
source using the same compiler/options. This benchmark compares the **same approximation**
on both builds, not the old event solver. It uses the real Pages tower and projectile exports,
warms the modules, alternates build order, and repeats free/upright sphere/arrow/rigid hit/miss
cases. The hash checks all visible f64 poses, linear velocities, sleep flags and existing work
counters after every tick. Full-body unit comparisons additionally cover angular state.

The script enforces the existing 0.5-unit floor-penetration bound, actual hit response, locked
rotation, sleeping/unchanged misses, and the 4-substep/32-pass tick ceilings. `TRIALS` defaults
to 2 and `TICKS` to 240 after 240 settle ticks. `CRATE_MODE=free` or `upright` partitions the
same matrix for bounded local execution; `both` runs all twelve cases. Partial checkpoints
are explicitly marked incomplete and are not passing evidence.

Settling, non-quiescent post-shot ticks and fully quiescent post-shot ticks are reported
separately. Timings wrap only physics calls, not setup, snapshots or assertions. Tiny sleeping
and four-tick near-miss samples are noisy; do not convert them to headline speedup claims.
Local paired measurements matched all physical hashes but had mixed elapsed-time results.
That demonstrates preserved behavior and reduced coefficient constructions, **not yet a
consistent end-to-end speedup**. Keep raw repeats and evaluate hosted measurements before
claiming one. Nothing in this change completes the other performance/stability issues.
