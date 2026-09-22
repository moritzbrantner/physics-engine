# Contact geometry reuse: conservative first slice of #156

The fixed-step solver remains f64, four substeps and eight velocity iterations. This change
reuses derived contact geometry, not impulses, solver results or time of impact. Warm-start
impulses and contact graph ownership are unchanged. The legacy event solver is untouched.

## Reuse boundaries

`World::manifolds` still discovers every candidate with the existing broad phase, applies
sensor/layer/response filters, and preserves canonical BodyId ordering. Pair caching is used
only when both bodies are fixed, sleeping or rotation-locked. Actively free-rotating pairs
retain no pair result or support-projection entry. They now share orientation frames within the
read-only discovery pass and evaluate SAT axes lazily, without allocating an axis vector. This
avoids the first prototype's repeated frame/projection work without caching moving contact points.
This routing changes computation, not contact admission or response.

The derived cache has two levels:

- Complete current-contact manifolds (including a negative current-contact result) require
  bit-identical body IDs, shapes, orientations, positions and contact margin. Normals, ordered
  points and separation are then identical to fresh generation. Even signed zero participates
  in keys. No tolerance, hash collision or approximate positional key is used.
- Body frames and box separating-axis support projections survive translation while their
  shape/orientation dependencies are unchanged. Translation still evaluates every relevant SAT
  axis with the new centers, selects the current contact feature, and clips fresh face points.
  Sliding, separation, recontact and feature changes therefore cannot reuse stale points.
  Shape/orientation changes rebuild the relevant preparation.

Frames are validated once per read-only manifold-discovery pass and can be reused by other
pairs during that pass. A wake/requery starts a new pass. They are never used after pose
integration without starting another pass. The actual mass and inertia are still prepared
from active body state before response, independently of this geometry cache.

After a cached current-contact miss the solver ALWAYS evaluates the normal CCD eligibility
and, where required, a fresh sweep using current velocities and the requested duration.
Swept results and future contact times are never stored in the cache. Box sweeps reuse the
orientation frames only: velocity, duration, slab entry/exit times and contact generation at the
new impact point are evaluated afresh. The existing translation-only sweep holds orientation fixed,
so these frames remain valid at its impact point. Closest-hit wall shielding and equal-time contacts
retain the original implementation.

The cache retains only pairs queried in the latest discovery pass. Absent candidates are
pruned; removal and projectile retirement remove incident entries immediately. Indexed frame
storage revalidates after lifecycle/index changes. Epoch wrap clears both pair and frame
entries. A shared floor does not become a dynamic-island connection.

## Scope intentionally left open

This is not the more aggressive local-anchor transport experiment: it does not carry old
contact points through changed translations/rotations using drift tolerances. Moving points
still get clipped afresh. Such reuse needs separate feature-validity and quality evidence.
Issue #156 remains open for that decision. This first slice preserves exact same-build
arithmetic/results rather than introducing another approximation to make a benchmark pass.

## Telemetry and cost

`Report::geometry` records current queries, complete-manifold hits (negative hits are a
subset), refreshes, frame preparations/reuses, projection preparations/reuses, SAT queries,
SAT axes tested, clipping passes, fresh sweep queries, pruned pairs and retained payload bytes.
The generic WASM stats API appends these at indices 25 through 39, in that order. Existing
indices 0 through 24 do not change. Current queries/SAT/clipping include the fresh contact
query made at a swept impact, but sweep-axis tests themselves are not counted as current SAT.

Retained geometry bytes cover frame and clipping/point scratch capacities, pair payloads,
support-projection capacity and any boxed cached manifold. Negative entries allocate no manifold.
They exclude BTreeMap node/allocator overhead, body storage, response scratch and warm-start caches. This is not an allocation profiler or whole-engine
memory estimate. Reading this byte count on sleeping/zero ticks is O(1); it is maintained at
cache mutation boundaries, not recomputed by scanning pairs on each idle tick.

A cache hit still copies ordered points into the current solver output. Manifold output stores the
existing maximum four points inline; copying it no longer allocates a point vector. Polygon clipping
and pre-reduction points use reusable world-owned vectors, cleared and overwritten on each fresh
query. This changes storage, not the original four-point selection or clipping order. Pair lookups
and validation still cost time; lower geometry counts do not guarantee lower total runtime.

Inline output increases the stride of the retained manifold-output scratch vector. The generic
WASM statistic 23 measures that scratch payload, so it legitimately differs from the old layout.
The explicit `--geometry-inline` benchmark records the peak of statistic 23 for **both** builds and
excludes only that memory measurement from equality hashing. All other old statistics 0..24,
poses, orientations, velocities and sleeping flags remain in the equality oracle. The original
`--geometry` option retains its stricter old behavior. Neither option changes behavioral acceptance
or any structural-performance threshold. Report memory alongside timing, not as a hidden free gain.

## Verification and paired evidence

```sh
cargo test --locked --lib approximate::geometry::
cargo test --locked --test fixed_step_approximation
cargo test --locked
cargo test --manifest-path demo-wasm/Cargo.toml --locked
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release --target wasm32-unknown-unknown
TRIALS=6 node scripts/benchmark-prepared-response.mjs base.wasm candidate.wasm paired.json --geometry-inline
```

The paired test uses the actual Pages reset/step/projectile exports, 240 settling ticks and
240 post-shot ticks, with at least two alternating-order trials. All 12 free/upright,
sphere/arrow/rigid, direct-hit/near-miss configurations must pass. Every observed pose,
quaternion, linear velocity, sleep flag and pre-existing non-memory report counter is hashed for
exact base/candidate equality. The deliberately changed retained-scratch statistic is reported
separately as described above. Native cached/uncached tests additionally compare full Body state
and warm-start contact impulses. Test-only uncached specialization adds no runtime UI toggle.

Raw durations exclude setup, observations and rendering. Settling, non-quiescent post-shot
work and sleeping ticks are separate; failed simulations are not speedup denominators. The
benchmark adds no wall-clock threshold and relaxes no physical/structural acceptance test.

## Rotating hot-path regressions

Native tests independently compare the original allocating polygon clipper with reused scratch,
check the exact four-point selection and iteration order, and compare transient frame-based
contacts/sweeps against the unprepared formulas over 1,200 deterministic rotated configurations.
Changed velocity/duration, a thin wall, per-pass frame lifetime, same-ID reinsertion, SAT early exit,
and zero/quiescent work are covered. Complete-world comparisons include warm-start impulses and
angular state; the real-WASM matrix preserves observable histories for all 12 hit/miss cases.
