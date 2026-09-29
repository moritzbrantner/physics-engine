# Contact-island-local convergence

Issue #166. This extends the bounded f64 solver after `3074f49ca2219b89cd093af6b6f8099bb387b887`.
It does not reduce the default four substeps, eight velocity iterations, contact slop,
convergence tolerances, CCD coverage, materials or sleeping policy. The canonical tower
continues to select two fixed-position correction passes. The optional soft-contact experiment is compile-time gated and uses this same scheduler when explicitly enabled; see `docs/soft-contact-correction.md`. It remains absent from production builds.

## Policy and authority

`Config::convergence_scope` defaults to `ConvergenceScope::ContactIslands`.
`ConvergenceScope::WholeWorld` retains the previous whole-constraint-set stopping policy.
`convergence: None` still selects the original fixed-pass reference, without partitioning.
Every interacting body uses the same substep schedule. This is not per-body time scaling,
a wall-clock quality adjustment, or parallel execution.

The new path first makes the original two-pass whole-world attempt. If its complete-pass
impulse and final-velocity residual checks pass, it finishes without constructing islands.
If not, it constructs fresh response partitions and resumes each from pass three. The next
unchanged early-exit probe is after **four total passes**, followed by 8/16/... below the
configured maximum. A difficult island still receives all eight default passes. No pass is
restarted, and no force, movement integration or simulated time is repeated. Ceilings of
four or less use the existing global kernel directly: after the first two passes there is
no remaining eligible local probe below those ceilings.

The shared prefix intentionally avoids paying for partitions when all contacts are easy.
Consequently an easy island next to a hard one may receive four passes rather than the two
it would need alone. It does not have to follow the hard island through all eight. This is
a performance tradeoff, not a promise of minimum possible work or bit-identical trajectories
between different stopping policies. Every accepted exit uses the existing tolerances.

## Partition correctness

Partitions are derived from the **current prepared response constraints**, after contact
admission and waking. Do not substitute the persistent warm-start/sleep adjacency graph:
that graph can miss a new speculative/swept contact or retain an obsolete connection.
All new contacts take part in the shared prefix before any local stopping decision.

A dynamic body with writable velocity connects every row that reads it, even if that row's
one-way response mask prevents writing it. Omitting such read dependencies can incorrectly
certify a contact while a later island changes one of its inputs. Fixed, externally driven
and still-sleeping velocity anchors do not connect otherwise independent dynamic groups.
They remain participants in their own contacts. In particular, a common fixed floor does
not make every object on it one dynamic island.

Union-find with size-weighted links and path halving builds components. Epoch stamps reset
only involved dynamic nodes during normal use; wrap clears old stamps. Roots are ordered by
minimum mutable BodyId. A stable CSR index view preserves the original row/point order
within each island, without moving or copying the constraint objects. Unrelated islands
may finish sequentially because no writable velocity is shared between them. A one-island
partition uses the existing contiguous row loop instead of indexed traversal.

The node/row data is scratch, not a source of physics state. It is rebuilt on every deferred
partition request and never reused as a convergence certificate. New contacts, waking,
removal, shifted body indices and reused IDs therefore cannot retain an old connection.
World cloning copies matching state and scratch; the next request still rebuilds.

## Work accounting

Existing world-substep counters remain world-substep counters:

- `Report::impulse_iterations` is the sum of the **maximum completed pass count per
  substep**, equivalent to the number of round-robin rounds. It is not a sum of island work.
- `convergence.skipped_iterations` complements those rounds up to the unchanged ceiling.
  A mixed substep is capped if any island is capped; it converges only if all do.
- `convergence.constraint_visits`, residual-row visits, and inertia applications count
  actual executed row work, including the shared prefix. Probe/residual counts can refer
  to multiple islands and must not be interpreted as new timesteps.

`Report::islands` additionally reports partition requests, rows/endpoints examined, dynamic
nodes initialized, union operations, root links, partition sizes, logical island passes,
accepted/capped islands, omitted row visits, buffer growth and retained payload capacity.
`shared_prefix_iterations` and `prefix_converged_substeps` make the bypass visible.

`islands` and `max_island_rows` only describe **partitions actually constructed**. When a
whole-world prefix converges, no graph census is done: a zero count is not evidence that
there were no contacts. Where partitions exist, `island_iterations` includes their shared
prefix logically; do not add prefix rounds to that sum a second time. Actual row visits
plus skipped row visits equal contact rows times the configured ceiling in island mode.

New scratch bytes are reported separately and are not inserted into a previous memory
metric's scope. The node array has capacity proportional to peak world-body count; CSR and
root buffers scale with peak rows/components. Measurements exclude allocator and inline
World/Report overhead. A default caller that never needs a partition allocates no island
heap scratch. This does not make the rest of stepping O(active bodies).

## Mixed-scene acceptance and benchmark

`experiments/contact-islands` is a separate, dependency-free diagnostic crate using the real
engine. It is not a production WASM export, a scene authority inside the engine, or a change
to the Pages UI. Its native test covers reset validation. Its WASM fixtures include:

1. A 32-box four-level tower alongside 32 or 128 **separate** floor-supported boxes, at rest
   and under a genuine swept projectile hit.
2. Only the separate boxes and only the tower as overhead/no-benefit controls.
3. Naturally sleeping tower/props with an aimed near miss.
4. A long swept projectile contacting two initially separate supported boxes; both must
   wake and move, while the new contact participates in the merged response partition.

The first three active scaling scenes intentionally disable sleep. They measure awake
contact workload, not a speedup obtained by putting objects to sleep. Sleeping and bridge
scenes retain normal sleeping. A 32+128 scene is **not** a 160-body dense rubble pile and
does not establish that the existing 128-box dense-impact quality limitation is fixed.

`run.mjs` compares the new island scope and the whole-world scope in the same module. An
optional separately compiled immutable baseline is also checked: the refactored global
scope must match both physical and old work histories exactly. `baseline` is solely an
experiment feature to compile the same fixture against the previous engine API.

Quality is checked outside timing throughout each replay: finite normalized states, body
inventory, actual hit/retirement and both bridge targets, unchanged sleeping near misses,
full requested time, fixed iteration limits, and a 0.5-scene-unit floor bound. Unforced
startup/rest drift is limited to 1.8 units for the 36-unit crates, the previously declared
experimental screening value. Paired scope differences have explicit bounds (1e-3 scene
units for positions, 1e-3 for linear/angular velocity components, 1e-4 for quaternion
components, and no sleep-flag difference). Same-mode histories/work must repeat exactly.
This does not claim cross-target/compiler identity or universal long-term trajectory parity.
Native tests separately cover one-way dependencies, stale bridges, zero/tiny/fixed budgets,
full-edge traversal oracles, lifecycle/epoch changes and unchanged momentum/restitution.

Each run records 240 startup ticks separately, followed by a configurable measured window.
Physics calls include grouping, projectile insertion, collision queries, solving and
integration; they exclude snapshots, quality checks, hashing and rendering. Warmup precedes
measurement and policy order rotates/reverses. Raw samples and each mode's startup, active
and sleeping phases are retained. Nearest-rank p95 is used, not a hard per-frame guarantee.
Failed or partial experiments remain failed; no wall-clock CI threshold is introduced.
Shared-host outliers must be retained. Small/no-benefit controls cannot be omitted to report
only a favorable aggregate.

```sh
cargo test --locked --lib approximate::islands::
cargo test --locked --manifest-path experiments/contact-islands/Cargo.toml
cargo build --locked --manifest-path experiments/contact-islands/Cargo.toml \
  --release --target wasm32-unknown-unknown
TRIALS=4 TICKS=600 node experiments/contact-islands/run.mjs \
  experiments/contact-islands/target/wasm32-unknown-unknown/release/contact_islands_experiment.wasm \
  island-results.json [immutable-baseline-fixture.wasm]
TOWER_TICKS=1200 TOWER_TRIALS=2 node scripts/test-tower-runtime.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm tower-quality.json
```

The mixed fixture is the next acceptance workload, not a Battlefield capacity claim. Jointed
ragdolls, vehicle suspension, varied timesteps between coupled bodies, a general convex/mesh
solver and the dense-overlap repair remain separate work. Within-substep fixed-orientation
CCD and existing nontransactional mid-step-error limitations are unchanged.

The existing Performance Evidence job runs the diagnostic crate test and two full policy
comparisons for 600 post-start ticks, retaining the module and raw results. The ordinary
Pages build continues running the canonical tower matrix. Both are correctness/work gates,
not wall-clock performance thresholds.
