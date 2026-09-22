# Contact geometry reuse: conservative first slice of #156

The fixed-step solver remains f64, four substeps and eight velocity iterations. This change
reuses derived contact geometry, not impulses, solver results or time of impact. Warm-start
impulses and contact graph ownership are unchanged. The legacy event solver is untouched.

## Reuse boundaries

`World::manifolds` still discovers every candidate with the existing broad phase, applies
sensor/layer/response filters, and preserves canonical BodyId ordering. Pair caching is used
only when both bodies are fixed, sleeping or rotation-locked. Actively free-rotating bodies
keep the uncached narrow phase: retaining data which changes every substep was not profitable
in the initial experiment. This routing changes computation, not contact admission or response.

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
Swept results and future contact times are never stored in the cache. Closest-hit wall
shielding and equal-time contacts retain the original implementation.

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

Retained bytes cover frame vector capacity, pair payloads, support-projection capacity and
cached point-vector capacity. They exclude BTreeMap node/allocator overhead, body storage,
response scratch and warm-start caches. This is not an allocation profiler or whole-engine
memory estimate. Reading this byte count on sleeping/zero ticks is O(1); it is maintained at
cache mutation boundaries, not recomputed by scanning pairs on each idle tick.

A cache hit still copies ordered points into the current solver output. Pair lookups and
validation have a cost; lower SAT/clipping counts do not guarantee lower total runtime.

## Verification and paired evidence

```sh
cargo test --locked --lib approximate::geometry::
cargo test --locked --test fixed_step_approximation
cargo test --locked
cargo test --manifest-path demo-wasm/Cargo.toml --locked
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release --target wasm32-unknown-unknown
node scripts/benchmark-prepared-response.mjs base.wasm candidate.wasm paired.json --geometry
```

The paired test uses the actual Pages reset/step/projectile exports, 240 settling ticks and
240 post-shot ticks, with at least two alternating-order trials. All 12 free/upright,
sphere/arrow/rigid, direct-hit/near-miss configurations must pass. Every observed pose,
quaternion, linear velocity, sleep flag and pre-existing report counter is hashed for exact
base/candidate equality. Native cached/uncached tests additionally compare full Body state
and warm-start contact impulses. Test-only uncached specialization adds no runtime UI toggle.

Raw durations exclude setup, observations and rendering. Settling, non-quiescent post-shot
work and sleeping ticks are separate; failed simulations are not speedup denominators. The
benchmark adds no wall-clock threshold and relaxes no physical/structural acceptance test.
