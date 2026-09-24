# Primitive collision specialization matrix

Issues #171 and #172 define the fixed-step f64 narrow-phase contract used by `approximate::World`.

## Authority and pair ordering

`Shape` remains the sole primitive-shape authority. Narrow-phase dispatch canonicalizes the two
shape kinds before selecting a kernel; reversing caller order reuses the same unordered pair and only
flips contact ownership/normal afterward. Stable `BodyId` ordering remains a world/pair-traversal
contract and is not replaced by shape ordering.

The current dense matrix is:

| unordered pair | production kernel |
| --- | --- |
| sphere × sphere | analytic center distance |
| sphere × box | analytic closest point on oriented box |
| sphere × capsule | analytic point-to-segment distance |
| sphere × wedge | closest fixed wedge feature |
| box × box | fixed 15-axis OBB SAT + clipped manifold |
| box × capsule | segment-to-oriented-box distance |
| box × wedge | bounded fixed-topology SAT |
| capsule × capsule | analytic segment-to-segment distance |
| capsule × wedge | segment-to-fixed-wedge distance |
| wedge × wedge | bounded fixed-topology SAT |

Adding a primitive means adding one `PrimitiveKind`, the new unordered `PrimitivePair` row, and
exhaustively handling that row. The closed Rust enums deliberately make an omitted pair a compile-time
maintenance problem rather than a silent generic dispatch.

## Cost model and counters

Primitive contact cost is driven by narrow-phase queries, support evaluations, SAT axes, fixed-feature
tests, clipping passes and CCD iterations. `GeometryStats` records those signals separately.

- `specialized_pair_dispatches[PrimitivePair::index()]` records canonical kernel selection.
- `generic_fallback_calls` is reserved for deliberately generic convex shapes. It is zero for every
  currently supported fixed-topology primitive pair and tests ratchet that boundary.
- `support_evaluations` counts support-map calls; fixed wedge support additionally records vertex
  evaluations.
- `sat_axes_tested`, `primitive_axes_tested`, `clip_passes`, and
  `primitive_sweep_iterations` remain distinct work signals.
- `manifold_candidates` records fresh narrow-phase manifolds admitted after the contact margin test.

These are deterministic work counters, not wall-clock substitutes. Shared-runner timing remains advisory.

## Reference mechanics

Production uses canonical unordered dispatch. Tests retain the pre-matrix ordered reference dispatcher
so every current pair is differentially checked in both caller orders. This reference path is test-only
and is not a second production physics authority.

The reference dispatcher proves dispatch/orientation equivalence. Shape-specific tests remain responsible
for the underlying geometric oracle cases (analytic segment distances, slope truth, SAT bounds, CCD, and
future generic-convex comparisons).

## Baseline ratchets

Sphere/sphere and sphere/box must perform no SAT, clipping, support mapping, or primitive fallback work.
Box/box performs one SAT query over at most 15 axes, at most four clipping passes, and at most two support
evaluations when a clipped face does not provide a point. All three must record exactly one specialized
pair dispatch and zero generic fallback calls.

These deterministic budgets are the baseline for subsequent matrix rows. Do not weaken them to absorb a
new shape; a new primitive should add its own bounded row and evidence.
