# Floating snapshot ray queries

`approximate::World::cast_ray` completes the ray part of the current snapshot
query surface in #194/#173/#176. It uses the same `QueryFilter`, `QueryStats`
and error contracts as [overlaps and translation casts](floating-shape-queries.md).
Sphere, box, capsule and wedge geometry comes from the checked f64 ray kernel
in rust-kernels; this engine owns filtering, ordering, body identity, snapshot
authority and support velocity.

`RayCast { origin, displacement }` tests the complete finite segment
`origin + displacement * [0, 1]`, including its endpoint. There is no direction
normalization parameter or infinite-range query. Origin and endpoint must be
finite and have component magnitudes below `1e12`; displacement must be finite
and nonzero. Dimensions retain the world’s validated range. Targets stay at
their current authoritative poses, irrespective of velocity; this is not
time-dependent or rotational CCD.

Every hit returns `BodyId`, fraction, world distance, point, normal, local
geometric feature, initial-overlap status, sensor status and rigid support
velocity `v + omega × (point - position)`. Results sort by fraction then BodyId,
including exact simultaneous ties. Layers use the existing symmetric policy;
sensors are excluded by default and one body can be excluded. External and
fixed bodies remain queryable. There is no response/one-way/movement policy.

Initial touching returns zero, even for separating movement. Strict interior
origins return the original interior point, zero, `Interior` feature and a
normal opposing displacement. That point is not a surface witness. Other
normals point outward from the target. Box face indices are -X/+X/-Y/+Y/-Z/+Z;
wedge faces are bottom/-X/slope/+Z/-Z. Capsule cap/side seam ties prefer the cap;
a zero-length capsule uses the sphere feature. Feature identity describes
local geometry, not a persistent contact/cache identifier; it expires when a
body’s shape/lifecycle changes. Shape casts continue to expose their existing
witnesses without feature identity.

The shared kernel uses 32-machine-epsilon relative distance rounding for
initial boundary and curved tangency classification, with normalized local
geometry. A nearby rounded witness can therefore differ slightly from the
exact surface. There is no collision skin. Invalid request values return
`InvalidInput`; an exact target failure reports `Search { body, reason }`.
Non-finite or unrepresentable geometry work is an error, never a kernel miss.
Pruned targets incur no exact work. Output clears on any returned error;
diagnostics include prior admitted/discarded hits and failed work.

Calls borrow `&World` and do not consume pending inputs, step time, wake bodies,
alter contact/sleep history or retain query state. Caller-owned output keeps
capacity across calls. One segment bound is prepared; all N bodies are visited
in stable order, and cached conservative bounds prune exact tests with rounding
padding. This is not a spatial-index claim. Each exact candidate prepares one
kernel frame and uses at most one sphere or three capsule quadratics, six box
or five wedge planes. Counters report visited/filtered bodies, bounds, exact
candidates, frame preparations, ray queries/planes/quadratics and output growth.
Reported bytes cover only caller output payload, not authoritative state,
allocator metadata or process memory. No query-owned heap scratch is retained.

Native controls compare 640 seeded pruned queries against exhaustive candidate
traversal over rotated shapes. Geometry correctness independently comes from
the upstream scalar occupancy/bisection and known analytic fixtures. Tests
cover thin walls, ties, support velocity, touching/interior, filters, sensors,
ID reuse, failed partial output and exact unchanged physical checkpoints.
The existing separate native/WASM shape-query driver now exercises rays for
all four primitives and replays three times in the existing Pages build.
There is no new workflow or production diagnostic export.

`cargo test --release --locked --lib
approximate::query::ray::tests::paired_ray_workload_evidence -- --ignored
--nocapture` runs 16 named controls (four shapes × small hit, sparse hit/miss
and crowded hit), three alternating-order trials of 256 warmed calls. It
checks exact pruned/exhaustive equality and retained output separately from
advisory complete-call timing. This compares query mechanics, not a previous
ray API or a game/solver speedup. Character collide-and-slide, moving-platform
policy, shape-cast feature identities and game adoption remain open in #194.
