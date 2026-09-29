# Architecture review — 2026-09-20

Baseline: `c796ea382bdcb0276b9309e8a3cca34c8c28313b`.
Shared convention `sourceRevision`: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.

The review followed the recent projectile integration, stabilization, contact-cache, and persistent
response-scratch commits. The existing native suite passed at the baseline. New reproductions exposed
the following problems; the accompanying changes fix them without changing public signatures or
public data fields.

## Correctness findings

1. **Motion authority was duplicated and inconsistent across response lanes.** Ballistic response treated
   every non-fixed target as mutable, unlike box normal response, friction, and projection. An off-center
   sphere impact moved and spun an externally controlled target. `RigidBox3d::receives_physics_response`
   now owns this rule for all those paths, including response admission and effective inverse mass.
   External targets still contribute contact velocity, but receive no impulse or projection.

2. **Existing sphere contacts were misclassified as new impacts.** The point-to-box distance used signed
   `saturating_sub`, which does not clamp negative distances to zero. Interior coordinates incorrectly
   contributed squared distance. A stationary sphere at `(11, 7, 9)`, radius 2, already overlapping a
   box with half extents `(10, 10, 10)`, produced a new impact at one subtick. Clamp each distance to
   zero before squaring. This also removes repeated false impacts in the sandbox stress trace.

3. **A step-long resolved-pair blacklist disabled real re-contact.** An elastic sphere between two walls
   resolved two impacts and then passed through a wall; the interval requires six impacts. Remove that
   history: the corrected geometric initial-contact check suppresses existing contact, while a later
   sweep from separation can hit the same target again. Rigid and ballistic event budgets remain
   independent. Tail slices now have an explicit ballistic event bound as well.

## Scaling and ownership findings

4. **Projectile broad-phase work was projectile count × target count.** The narrow-phase counters looked
   linear even though every projectile scanned every target. Each prepared ballistic interval now owns
   a balanced immutable index of swept target bounds. Queries traverse a compact preorder array without
   allocating traversal scratch. Exact rounded-OBB testing and `(time, BodyId)` selection remain the
   collision authority. The existing world `ballistic_target_bound_checks` counter now reports actual
   branch and leaf overlap tests. Preparation is still paid once per interval; the index is shared by
   all queries for that interval, not cached across changing world state.

5. **Wake propagation repeatedly revisited old sources.** Each newly reached sleeping body caused another
   pass over every previously awake source. Drain the current frontier and queue only newly awakened
   bodies. During propagation, source geometry and policies do not change and the parked set only
   shrinks, so revisiting old sources cannot discover anything new. Stable ID ordering, layer checks,
   passive-support exceptions, and conservative topology wake behavior remain intact.

6. **Search preparation still copied authoritative body collections.** Ballistic target projection built
   a complete `Vec<RigidBox3d>` before preparing derived geometry; current-contact graph construction
   cloned every body merely to satisfy a slice interface. Both now borrow the authoritative bodies.
   Broad-phase preparation accepts borrowed iterators, and ballistic projection retains only derived
   geometry and motion. This removes those two whole-world materializations without adding a parallel
   mutable state store.

7. **Every box SAT axis repeated eight vertex dot products per box.** Profiling the corrected 30-shot
   trace attributed about 28% of sampled self time to SAT and projection alone, ahead of allocation.
   The geometry constructor forms all eight corners from the same three quantized basis vectors.
   Projecting the origin corner and its three adjacent corners therefore determines the exact interval:
   add each negative edge projection to the minimum and each positive one to the maximum. This halves
   projection dot products, from 16 to eight per pair-axis, without changing sampling, candidate axes,
   support masks, tie selection, or integer collision semantics. SAT axes derived from i32 vertices are
   at most 65 bits, so the extra intermediate sums remain representable in i128.

8. **One wake transition rebuilt every remaining sleeper's bounds.** Waking one of 32 parked bodies
   performed 31 unnecessary geometry preparations. The retained bounds tree now supports insertion,
   update, and removal by BodyId, including balancing and arena-slot reuse. Parking prepares only that
   body's stationary bounds; waking removes only its leaf. Small pose updates inside an existing fat
   envelope update the exact bound without changing tree topology.

9. **Wake discovery scanned sleeping bodies to find awake sources.** The wrapper now retains a sorted
   set of active dynamic IDs instead of just their count. Add/remove and wake/sleep transitions maintain
   that set. An active step with one awake body visits one source even with many parked bodies; the
   earlier frontier-draining fix still ensures each propagated source is queried once.

10. **Any geometry change discarded every precise subject-contact result.** Precise queries now share
    a retained stationary bounds index. A geometry delta invalidates the changed body and cached
    subjects overlapping its old or new bounds. Both sides matter: old bounds invalidate departed
    contacts, while new bounds invalidate cached negative results where a contact may have arrived.
    Unrelated stationary contacts survive movement elsewhere. The first query prepares the index;
    subsequent queries and geometry updates reuse it. This trades initial index construction/storage
    for avoiding repeated whole-scene bound preparation. Membership changes still reset this cache.

11. **Velocity-only steps invalidated geometry caches.** Step writeback now tracks observable body
    changes separately from pose changes. A gravity step that changes velocity but not the quantized
    position/orientation still reports the changed BodyId, but retains current-contact evidence.

12. **Fixed-body contact answers depended on previous queries.** The shared contact graph excludes
    fixed/fixed pairs. Once built by a dynamic overlap query, it incorrectly supplied incomplete answers
    for fixed subjects, including parked proxies. Explicit fixed-subject queries now use the complete
    indexed path. Dynamic subjects can still reuse the graph. When both are live, invalidation reaches
    both before the pending changes are discarded.

## Evidence

Native release measurements on Linux x86_64, AMD Ryzen 7 5700X, Rust 1.98.0. The sparse query workload
uses 256 projectiles, one true target per projectile, and 15 timing samples. Each measured batch includes
per-interval preparation and index construction. Timings are advisory; deterministic counts are the gate.

| Targets | Previous bound checks | Indexed bound checks | Previous median | Indexed median |
| --- | ---: | ---: | ---: | ---: |
| 32 | 8,192 | 2,816 | 0.211 ms | 0.201 ms |
| 256 | 65,536 | 4,352 | 0.378 ms | 0.239 ms |
| 2,048 | 524,288 | 5,888 | 2.196 ms | 0.549 ms |

The index cuts bound tests by 98.9% in the largest fixture. Narrow-phase work remains exactly 256 TOI
tests and 6,656 feature tests. The old 18-target projectile benchmark also remains covered; its
`linear_scan_bounds` field explicitly labels the theoretical old scan rather than claiming measured work.

The eight-body wake chain changes from 45 queries to nine. A 32-body chain uses 33 queries. Both leave
an unrelated sleeper parked. Indexed queries are compared with exhaustive single-target queries over
moving, rotated, filtered, and self-excluded targets, multiple horizons, ties, and empty scenes.

Reproduction/verification entry points:

```bash
cargo test --locked --test analytic_ballistic_world
cargo test --locked --lib ballistic_sphere::
cargo test --locked --lib relaxed_rotating_world::tests
cargo test --release --locked --lib ballistic_sphere::indexed_tests::sparse_ballistic_target_benchmark -- --ignored --nocapture
```

The new benchmark is routed through `scripts/run-performance-evidence.sh`. The ordinary fast tier,
the dedicated Pages and character-landing replays, and the 30-shot native sandbox stress trace cover
the combined changes. No event limit, sampling resolution, or collision tolerance was increased.

The fast tier passed 287 engine tests and 46 adapter tests; both explicitly required replay tests also
passed. The release WASM build passed. Two runs of each of the six `sandbox-projectiles-v5` scenes were
deterministic. Five scenes retained their baseline replay hashes; the 30-shot sphere trace changed with
the repaired contact semantics and produced replay hash
`8676f1f1a0f0b1e653e8a1ba79ac2764dca1b4cba140682212c46d55d61894b7` in both runs.

The sparse-index improvement is not an across-the-board frame-time claim. A focused reversed-order
comparison of the unchanged three-shot walking scene put warmed means near 6.23 ms for the candidate
and 6.06 ms for the baseline. For the corrected 30-shot scene, three reversed-order trials averaged
7.15 ms for the candidate versus 4.91 ms for the baseline. Its sampled rigid events increased from
684 to 913 and stabilization passes from 825 to 1,651; exact stabilization contact tests increased
from 637 to 1,990. Re-enabling valid collisions changes the physical workload. The resulting stress
slowdown is a remaining solver-performance cost, not a reason to reinstate the incorrect pair blacklist.

### Follow-up: exact SAT projection work

The follow-up uses the corrected first-round engine as its baseline, including its valid recontacts.
It does not compare the optimized solver against the earlier incorrect projectile behavior.

The prepared-contact benchmark runs 20,480 queries over 512 deterministically generated shapes, with
9,800 contacts. Fifteen samples alternate the optimized and original eight-vertex algorithms within
one warm process. Medians were 55.56 ms and 59.97 ms respectively (7.4% lower kernel time). An earlier
separate-build comparison measured 53.62 ms versus 68.29 ms; the paired measurement is the more useful
estimate because it reduces compilation/order and changing host-load effects. Preparation is excluded
from this kernel benchmark, unlike the ballistic preparation benchmark above.

Three trials of each of the six WASM sandbox scenes preserve every replay hash, body count, event sum,
and work counter from the corrected baseline. Whole-scene timings varied with concurrent unrelated
compilation load and run order, so this follow-up makes no reliable whole-scene speedup claim. The
deterministic improvement is four rather than eight dot products per box-axis.

The ordinary parity tests compare complete public contact seeds and errors with the original
vertex-by-vertex SAT algorithm over 3,216 ordered pairs, including arbitrary rotations, large extents,
invalid geometry, and swapped pairs. Separate tests cover face/edge/corner touching, containment, and
projection equality through the full 65-bit SAT-axis bound. The benchmark is included in the changed-file
performance router:

```bash
cargo test --locked --lib oriented_box::
cargo test --release --locked --lib oriented_box::parity_tests::prepared_contact_benchmark -- --ignored --nocapture
```

The follow-up fast tier passed formatting, lint, build, and all 336 ordinary engine/adapter tests.
Both explicitly required adapter replay tests also passed. The release WASM build and the 18 measured
sandbox trials passed; no public interface or collision limit changed.

### Follow-up: predominantly sleeping scenes

The failing reproductions established three avoidable costs before repair: waking one of 32 sleepers
reprepared the remaining 31 bounds; one distant body's motion rebuilt all 64 stationary subject
results; a velocity-only step rebuilt a contact result despite identical geometry. A fourth reproduction
showed a fixed/fixed contact disappearing solely because an unrelated dynamic overlap query had built
the shared graph.

After repair, waking a body prepares zero neighbor bounds. Reparking it prepares one bound. Fixtures
with 32 and 128 sleepers visit just the one awake source, and repeated wake/repark cycles return to
the exact quiescent state. Ordered index insertion, deletion, updates, complete removal, and ID reuse
are compared against a fresh rebuild and checked for tree balance.

The release cache workload prepares the initial scene once, then changes one body's pose and queries
every subject for each of 64 generations. Deterministic work, rather than time, is the acceptance gate:

| Stationary bodies | Subject queries | Bounds prepared after warmup | Subject results rebuilt |
| --- | ---: | ---: | ---: |
| 32 | 2,048 | 64 | 64 |
| 256 | 16,384 | 64 | 64 |
| 2,048 | 131,072 | 64 | 64 |

The original global invalidation policy would rebuild every queried subject each generation. The new
work stays at one geometry preparation and one result rebuild per changed body in this sparse fixture;
returning the caller-requested results still scales with query count. Native release timings for these
three measured batches were 0.067 ms, 0.423 ms, and 6.973 ms respectively, excluding initial preparation.
These are workload-specific advisory measurements, not whole-engine timing claims.

Arrival/departure tests exercise both a cold graph and a retained graph, including negative-cache
invalidation and unrelated-query reuse. The new benchmark uses the existing current-contact performance
router:

```bash
cargo test --locked --lib relaxed_rotating_world::tests::
cargo test --locked --lib current_contact_query::tests::
cargo test --locked --lib rotating_broad_phase::tests::
cargo test --release --locked --lib current_contact_query::tests::stationary_subject_reuse_benchmark -- --ignored --nocapture
```

The final fast tier passed formatting, lint, builds, and all 345 engine and adapter tests, including the
ground-support regression. Both dedicated adapter replay tests passed. The release WASM build passed.
All six sandbox scenes were deterministic
across two trials, with identical solver-work counters and collision-event counts. Four retained their
previous replay hashes. The three- and six-shot idle scenes changed only their grounded flag: the player
now correctly retains floor support while parked during remote projectile activity. Frame-by-frame
comparison found zero render-snapshot differences; the repaired grounded flag differed in 127 and 103
frames respectively. A further adapter regression asserts unchanged player position and continuous
ground support during the first 40 ticks after firing a remote projectile.

### Follow-up: contact reuse across sleep transitions

Baseline: `a07f1eeb30f4287d25a5944c47b889ab59fbcbec`; shared convention `sourceRevision` remains
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.

Sleep proxies, motion stopping, and fixed-boundary position correction used body removal/reinsertion
to express local state changes. This discarded contact graphs and subject caches. Parking also used
the fixed-geometry insertion path, which scanned the entire scene for affected dynamics.

These operations now commit narrow internal body-state deltas. Identity, material, orientation and
unrelated body state stay in place. Velocity-only stopping preserves geometry evidence; position and
response-kind changes invalidate only their affected contact neighborhoods. The current-contact broad
phase updates eligibility-changing leaves locally, including the dynamic flags used for fixed/fixed
pruning. Parking uses retained contacts to schedule neighboring dynamics for stabilization. Public
interfaces, body ordering, swept collision discovery and solver event limits are unchanged.

Incremental query admission also now matches full query admission: externally controlled bodies retain
observable contacts even when neither participant can receive solver impulses. Tests compare warm
contacts and overlap queries with freshly constructed worlds across motion and proxy transitions.

The added ratchet fixture performs four wake/park cycles among separated touching pairs. Every queried
body must retain its exact partner. Before optimization, subject rebuilds were 256 / 1,024 / 4,096 at
32 / 128 / 512 bodies, with four full graph rebuilds. The candidate needs 12 subject rebuilds and zero
full graph rebuilds at every size. The ledger first records this extended workload before changing the
implementation, then records the reduced ceilings; existing workload ceilings and replay fingerprints
are retained.

An initial candidate also removed the strict facade's sleep-timer invalidation on proxy transitions.
The ratchet rejected this because the sandbox became quiescent earlier. That behavior is preserved:
timer bookkeeping remains separate from geometry-cache invalidation. This slice optimizes derived-state
work without changing when existing scenes sleep.

Validation passed: 349 ordinary tests, both dedicated adapter replays, and all 21 ratchet scenarios.
All six sandbox replay hashes and cost counters match the previous baseline. The new ratchet entry
tightens six work ceilings. Source archives were verified against their file hashes; snapshots now
remain compressed so the repository's check runner cannot discover historical Cargo manifests as
additional live projects.

## Remaining architectural limits

The [performance ratchet](performance-ratchet.md) now preserves these changes in native scaling
fixtures and all six sandbox replays. Its initial ledger entry measures the combined working-tree
implementation; future entries record individual improvements without overwriting earlier evidence.

- `rotating_world::RotatingWorld3d::step` still stages all solid rigid bodies in a working vector before
  committing them back to its map. This remains a gap against `world-state-architecture.md`; fixing it
  requires one canonical solver store plus explicit touched-state error/rollback semantics across both
  rigid and ballistic lanes. The changes here remove search copies, not that step-level working copy.
- Ordered sampled searches recreate their BodyId indices, candidate state, and coarse geometry caches
  per invocation. Reuse needs keys covering membership, complete motion/geometry, gravity, and the exact
  remaining horizon. Existing persistent response scratch does not make sampled-search scratch persistent.
- Ballistic insertion and live ballistic steps conservatively wake every parked body. Replacing this
  with only the initial straight sweep is unsafe: later bounces and target impulses can redirect the
  projectile. Local waking needs event-time ownership or a proven conservative reach bound.
- Proxy transitions preserve contact caches, but still retain the strict facade's conservative
  sleep-timer resets for its remaining sleepers. Changing this sleep policy needs separate correctness
  evidence and an explicit replay-baseline decision. Parked bodies are not part of that strict sleep set.
- Rotating-box CCD remains sampled, and ballistic target orientation remains frozen over each query
  interval. The target index preserves these existing approximation contracts; it does not provide
  analytic CCD for arbitrary rotating or accelerated targets.
- The existing tower tests establish their recorded first-frame behavior, not arbitrary long-running
  tower convergence. No broader tower-convergence claim is made by these changes.

The next structural work remains the canonical solver store and dependency-complete sampled-search
reuse. The follow-up profile justified removing repeated exact projection work first; it does not
resolve those ownership gaps. Keep error-state semantics and changing motion horizons explicit rather
than relying on the presence of a warm cache.
