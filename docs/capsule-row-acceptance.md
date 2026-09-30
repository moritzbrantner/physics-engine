# Capsule row acceptance and public work

The existing `approximate::Shape::Capsule` is a local-Y skeleton plus radius. The
sphere/capsule, box/capsule and capsule/capsule paths consume bounded analytic
geometry from rust-kernels; engine queries, physical response, CCD admission,
contact history and failed-step semantics remain engine-owned. This completes the
existing capsule row's acceptance slice without introducing another collider.

Bounds/support, finite rays and translation casts retain their existing contracts.
Checked [uniform mass/COM/inertia products](shape-mass-properties.md) are public.
The [intersection-normal repair](floating-shape-queries.md) handles crossing,
collapsed and sphere-on-skeleton contacts. Wedge COM-centered dynamics, additional
shape rows, character policy and chronological/rotational CCD remain separate work.

## Independent geometry evidence

Merged [rust-kernels #136](https://github.com/moritzbrantner/rust-kernels/pull/136),
adopted at `7049e423ef00f5f65e7276281c0845d431ed6300`, supplies:

- 2304 seeded and 540 edge cases using an independent 80-step convex distance
  minimizer, direct point/segment or point/box formulas and independent support
  planes. Fixtures include rotated/parallel/nearly parallel/antiparallel skeletons,
  zero separation, endpoints, containment, touching, misses, collapsed/tiny
  skeletons and dimensional scales 1e-6, 1 and 1e6. Both pair orders have clearance
  checks; containment depth is not compared to an unsigned box distance.
- Independent-support generic GJK comparisons, with 162 unresolved probes explicitly
  reported and separately checked by the convex-distance/clearance oracle.
- 108 fast translation sweeps with separated frame endpoints, thin boxes, moving
  targets, oblique skeletons and pair reversal; independent static-distance
  bisection supplies first-contact times. Orientations remain fixed within sweeps.
- Actual work ceilings, including a separated skeleton crossing all six box planes
  that reaches the 46-feature bound, and long-thin half lengths through 1e9 with
  radius 1e-6. A 6144-query Cargo-native allocation ratchet reports zero allocation
  and reallocation calls; the box partition uses eight inline f64 values.

[The upstream contract and measurements](https://github.com/moritzbrantner/rust-kernels/blob/7049e423ef00f5f65e7276281c0845d431ed6300/docs/capsule-row-acceptance.md)
record tolerances, all generic statuses and timing limitations. Instrumentation
preserved 6147 sampled rotated contact/witness/sweep and old-work products exactly.
The local paired timing control showed about 1.6% aggregate counter overhead;
this is an observability improvement with a cost, not a solver speedup claim.

## Public reports

`QueryStats::segment_distance_evaluations` and `segment_feature_tests` include
initial/witness queries, every swept contact query and work discarded by failed
searches. `Report::geometry` carries the same products as
`primitive_segment_distance_evaluations` and `primitive_segment_feature_tests`.
`FailedStepWork::geometry` retains them through its existing cloned diagnostic
report. Cached manifold reuse performs no new distance query and does not invent
counter work. Failed queries clear output while preserving physical state and
pending commands.

| Contact pair | Distance problems per fresh query | Segment feature visits | SAT axes |
| --- | ---: | ---: | ---: |
| Sphere/capsule | 1 | <=1 | 0 |
| Capsule/capsule | 1 | <=4 | 0 |
| Box/capsule | <=6 | <=46 | <=6 |

These count geometric problems and defined feature visits, not individual scalar
operations or complete machine instructions. Triangle-only features, normal
construction and numerical validation are excluded. Capsule/wedge segment helpers
also contribute these fields; full wedge feature accounting remains separate.
No generic production fallback or support/vertex scan is introduced for these
three pairs. Query output capacity remains caller-owned; counters are additive
and do not alter physical arithmetic, solver budgets or numerical tolerances.

Four new shared native/WASM fixtures verify public overlap/cast work for all
three pairs, retained hit output, a one-iteration exhausted search (two discarded
segment problems), exact checkpoint preservation including a pending force, and
actual physical-step report propagation and discarded work after an atomic failed
physical step. The existing native/WASM shape-query
acceptance driver runs these alongside the prior three capsule-normal fixtures;
its exports remain separate from the production demo.

```sh
cargo test --locked --test capsule_contact_normals --test capsule_row_work
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example shape-query-contract
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release \
  --target wasm32-unknown-unknown --example shape-query-contract
node scripts/test-shape-query-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/shape_query_contract.wasm
```

The broader ten-pair matrix remains partial. This capsule-row evidence does not
supply unrestricted wedge rotation, additional shape rows or general angular CCD.
Shared convention sourceRevision:
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
