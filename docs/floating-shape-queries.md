# Floating snapshot overlaps and shape casts

`approximate::World::overlap_shape` and `cast_shape` query current authoritative
sphere, box, capsule and wedge poses. Together with
[finite rays](floating-ray-queries.md), they are the current query slices of
[issue 194](https://github.com/moritzbrantner/physics-engine/issues/194). Shared
shape/contact/sweep math stays in `rust-kernels`; this engine owns world traversal,
filters, result ordering, admission and physical velocity evidence.

```rust
use physics_engine::approximate::{QueryFilter, QueryPose, Shape, ShapeCast, Vector};

let pose = QueryPose::new(Shape::capsule(0.6, 0.3), Vector(0.0, 2.0, 0.0));
let cast = ShapeCast::new(pose, Vector(0.0, -4.0, 0.0));
let mut hits = Vec::new();
let work = world.cast_shape(cast, QueryFilter::default(), &mut hits)?;
```

## Contract

Query positions/dimensions are in the same f64 scene units as world bodies. A cast
accepts a **displacement**, rather than seconds or velocity. It examines its entire
interval, holds both orientations fixed, and keeps target positions at their
current snapshot. Target physical velocities do not move targets during the cast.
This is not a prediction of a moving world or analytic angular CCD.

Query dimensions use the existing body limits: radius/box/wedge half extents at
least `1e-6` and below `1e12`; capsule half segment may be zero. Query start and end
positions must be finite with absolute components below `1e12`. Query orientations
must already be unit quaternions (norm-squared tolerance `1e-9`); queries do not
silently normalize supplied poses. Invalid input fails before visiting bodies.

Both calls clear prior caller output. Overlaps include exact touching and are
ordered by `BodyId`. Casts order by fraction, then `BodyId`; distance is fraction
multiplied by the displacement length. Initial penetration and touching produce a
zero-fraction hit even for separating or tangent motion. Callers choose response
policy explicitly. `starts_overlapping` means strictly negative initial signed
separation; touching is not penetration.

Each hit contains query/target witness points, signed separation, an outward target
normal, sensor status and current target velocity at its witness point:
`linear_velocity + angular_velocity × (target_point - target_position)`. Degenerate
contacts use the shared kernel's deterministic normal. Polyhedral witness points
are support witnesses on the selected SAT axis, not a clipped contact manifold or
necessarily a unique pair of nearest points. There are no stable face/edge/vertex
IDs yet; gameplay must not invent them from these witnesses.

For sphere/capsule and capsule/capsule skeleton intersections, the shared kernel
chooses a deterministic direction perpendicular to every nonzero skeleton.
The signed depth and witnesses use that selected direction; translating the query
along the outward normal by the penetration clears these solids, within the
scale-aware roundoff budget. Two collapsed skeletons retain sphere-distance
semantics. Ordinary resolved deltas, including nonnegative separation, retain the
closest-point normal. This does not supply a unique minimum-translation manifold.

Layer admission uses the same symmetric membership/mask rule as physical bodies.
Sensors are excluded by default; `include_sensors` admits them explicitly. `exclude`
skips one current body ID. Sleeping, fixed, dynamic and externally driven targets
are otherwise eligible. Removal and same-ID replacement use current authoritative
state; no query history persists across lifecycle changes.

The per-target sweep budget is explicit (`1..=128`, default 128), bounding shared
conservative advance or swept SAT. Initial-contact queries are separately counted.
Exhaustion and non-finite geometry/search computations are errors, not misses.
Scale-aware sweep convergence may admit a hit with small positive residual
separation, bounded by the shared tolerance `1e-9 * (1 + query_radius + target_radius)`.
Overlap admission itself uses signed separation `<= 0`, without a proximity margin.

On any error, output is empty; `QueryFailure` identifies invalid input or the first
failing target in stable body order and includes all discarded work/admitted hits.
Only the error path allocates a boxed diagnostic result. Queries take `&World` and
never advance time, consume force/impulse input, wake bodies, alter contact/sleep
history, or replace the successful step report. These are queries, not a collision
response or support-state controller.

## Work and verification

Traversal visits N bodies. It reuses authoritative cached target bounds, prepares
one conservative query/swept bound and prunes exact candidates using rounding and
sweep-tolerance padding. This introduces no second spatial index and does not claim
K-only traversal. A future indexed/batched path needs consumer evidence first.
The output vector retains caller-owned capacity. Shared primitive kernels use
bounded inline scratch; ordinary query calls retain no engine storage or world
snapshot. Counters report visits/filtering, bound tests, exact candidates, explicit
contact and total primitive queries, frame adapters, axes/support/vertex work,
sweep iterations, entered segment-distance problems, segment endpoint/slab/partition feature visits and actual output capacity growth. Output capacity bytes exclude
allocator overhead and error-diagnostic storage; no RSS claim is made.

Public controls cover thin obstacles across complete displacements, all four query
shapes, rotated capsules, independent analytic sphere/wall and wedge/ramp results,
target angular/linear support velocity, initial touching/penetration, filters,
sensors, ID reuse, invalid input, output retention and exhausted search cleanup.
Checkpoint bytes verify queries preserve pending commands and physical history.
640 seeded rotated-shape controls compare bounds-pruned hits with an exhaustive
walk through the same current geometry. This is an independent **candidate** oracle,
not a second primitive geometry implementation. Primitive analytic/invariant
controls and the upstream geometry suite cover that separate layer.

Three shared native/WASM capsule regression fixtures cover a sphere on the
skeleton, rotated crossing capsules and a collapsed capsule, in both query/target
orders at 1e-4, 1 and 1e6 dimensional scales. Independent support-plane formulas
certify clearance after moving by the reported depth, and checkpoint bytes protect
pending forces and physical history. They fail on the prior kernel pin. Upstream
coverage additionally includes 1024 seeded intersections and near-parallel/short
skeleton controls. The subsequent [capsule row acceptance](capsule-row-acceptance.md) supplies the complete sphere/box/capsule row reference and work contract, with public native/WASM query and physical-step propagation.

A 512-body sparse fixture ratchets N bound visits but one exact query and zero warmed
output growth. Native and WASM run the same public contract driver, separately from
the shipped demo. Reproduce:

```sh
cargo test --locked --test floating_shape_queries
cargo test --locked --test capsule_contact_normals
cargo test --locked --test capsule_row_work
cargo test --locked --lib approximate::query
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example shape-query-contract
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release \
  --target wasm32-unknown-unknown --example shape-query-contract
node scripts/test-shape-query-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/shape_query_contract.wasm
cargo test --release --locked --lib approximate::query::tests::paired_query_workload_evidence \
  -- --ignored --nocapture
```

Paired timing/work controls use 512 current targets with sparse overlap, sparse cast
and miss queries, each of four query shapes, three trials and 256 warmed calls.
Both paths visit the same N bodies and return identical results; the exhaustive path
runs narrow phase against all eligible targets. Whole-call release timings are
advisory. This is not a before/after solver performance claim: no public floating
query existed before this slice.

[Recorded local query evidence](floating-query-work-2026-09-30.json), measured with
rustc 1.98.0 on `x86_64-unknown-linux-gnu` on 2026-09-30, preserves all 12 control
results exactly. Each sparse call reduces 512 exact candidates to one; misses reduce
them to zero. Both paths still visit 512 bodies. Median pruned durations are
1.19–1.41 ms per 256 warmed calls versus 12.39–145.57 ms for the exhaustive controls,
depending on shape/workload. Timings are advisory and compare query mechanics only;
no gameplay, world-step or large-N dynamic-contact benefit is asserted.

Finite rays with local geometric feature identity are supplied by
[floating ray queries](floating-ray-queries.md). Shape-cast feature IDs,
richer response manifolds, collide-and-slide/corner/crease
handling, slope/step policy, moving supports and a real game adoption remain open
in #194. The existing translational/rotating query contracts are unchanged.
Convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
