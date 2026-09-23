# Prepared fixed-contact position correction

The canonical Tower Stability runtime opts into `fixed_position_iterations = 2` on the
bounded f64 solver. This stage removes residual overlap with fixed colliders after pose
integration. Its equations, body order, contact order, pass budget and existing slop are
unchanged by this optimization. It remains an approximation, not a substitute for CCD.

## Reused data

The stage uses the existing canonical active-body view, then a BodyId-ordered list of
mass-zero, non-sensor colliders instead of scanning every body for every active body.
The list contains body indices, fixed AABBs and lazily prepared box orientation axes.
All surviving candidate pairs still receive the same layer test and inclusive AABB test.
This is not a new spatial tree: bounds work remains O(active bodies × fixed colliders).

Every positive AABB query runs fresh narrow-phase geometry. Box queries use the existing
lazy separating-axis and clipping implementation with prepared frames. One moving frame
is prepared on demand per eligible body/pass; fixed frames survive between stages until
layout invalidation. Clipping buffers retain capacity, not a previous manifold. Sphere
queries use the unchanged current-contact routines.

No normals, separations, support projections, contact points, collision decisions, swept
bounds or times of impact are cached by this stage. In particular, a correction can move
a body into a later fixed collider. The body's current bounds are refreshed immediately
and each subsequent pair is tested against those new bounds. Building a candidate list
from only the initial bounds would be incorrect.

## Invalidation and ownership

Stored bodies are accessible publicly only by immutable references. Creation, removal,
retirement and same-ID reinsertion invalidate the position index through the existing
central `bookkeeping::Scratch::layout_changed` hook. This includes removing an early-ID
projectile that shifts every later index. A world clone copies its authoritative bodies
and corresponding derived scratch together.

Public mutations of force, impulse and velocity cannot modify fixed collider geometry.
They already update the active view through contact-gated wake admission. Should an API
for shape/pose/mass/sensor mutation be introduced, it must invalidate these dependencies
as well; cached rows must not become another authority over body state.

The position stage does not modify orientation, linear/angular velocity or elapsed time.
It still resets quiet time after material correction, respects external/sleeping/sensor
bodies and collision masks, and does not connect unrelated islands through a fixed floor.
`fixed_position_iterations = 0` allocates no position scratch. Zero and quiescent steps do
no position work; they report retained payload without traversing the index.

## Telemetry

Existing `approximate_position_stat(0..4)` keeps its meanings and values: passes, bounds
tests, contact tests, corrections and maximum correction distance. All old generic stats
`approximate_stat(0..51)` remain unchanged. Appended position stats are:

| Index | Meaning |
|---|---|
| 5 | Eligible moving bodies visited across the position passes |
| 6 | Fixed index rebuilds |
| 7 | All bodies scanned while rebuilding that index |
| 8 | Fixed box frames actually prepared |
| 9 | Moving box frames actually prepared |
| 10 | Retained fixed-index and clipping-buffer payload bytes |
| 11 | Observed capacity increases: fixed-index pushes and end-of-stage clipping storage |

Stat 11 is not a global allocation count and does not count every transient capacity
increase inside clipping. Stat 10 excludes allocator overhead, other solver scratch,
body storage, and existing geometry/warm-start caches. It is separate from generic stat
23; the original memory metric and its scope are not silently changed.

## Verification and measurement

A frozen scalar implementation from `ca394b3f` is instantiated only in tests. Comparisons
cover 1,200 rotated box/sphere arrangements with filters, full-step body and warm-start
histories, two-collider correction in one pass, lifecycle/index shifts, actual retirement,
and zero/quiescent calls. A deterministic 32/128/512-sleeper fixture asserts that warmed
position processing visits one active body and rebuilds neither the index nor fixed frames.
The scene is unchanged; excluded sleepers remain collision-queryable in the normal solver.

```sh
cargo test --locked --lib approximate::position
cargo test --release --locked --lib \
  approximate::position::parity_tests::position_preparation_scaling_benchmark \
  -- --ignored --nocapture
TRIALS=6 TICKS=1200 node scripts/benchmark-tower-position.mjs \
  base.wasm candidate.wasm paired.json --representative
TRIALS=2 TICKS=1200 node scripts/benchmark-tower-position.mjs \
  base.wasm candidate.wasm all-configurations.json --all
```

The paired benchmark uses the **canonical tower reset and browser adapter**, not the
comparison page. It hashes every tick's raw f64 render pose, linear velocities, sleeping
flags, lifecycle counts, old position stats 0..4 and generic stats 0..51. No equality metric
is removed; only the new position bookkeeping measurements have no historical counterpart.
Native tests additionally compare angular velocities and warm-start impulses.

The full matrix has both character modes, both rotation modes, three projectile impact
policies, and four firing sequences: twenty repeated mixed hits, twenty near misses, an
immediate arrow and a twelve-shot rapid burst. Completion, all original bodies, finite
normalized poses, hit response, rotation locks, unchanged retiring misses and the existing
0.5-unit floor-penetration bound are enforced. Post-wall ricochets may legitimately hit.

Build order alternates between trials, following module warmup. Physics-call timing
excludes snapshots and assertions. Settling, non-quiescent post-shot ticks and already
quiescent calls are reported separately, with raw samples. Failed/incomplete simulations
cannot supply a speedup denominator. Times are advisory host measurements, not laptop FPS
or CI wall-clock limits. The ignored native benchmark measures this stage in isolation,
not complete solver throughput. Payload memory and index rebuild work are reported alongside
time savings; the optimization is not claimed to remove every whole-world scan.
