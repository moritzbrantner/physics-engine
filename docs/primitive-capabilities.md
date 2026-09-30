# Floating primitive capability ledger

[`primitive-capabilities.json`](primitive-capabilities.json) is the versioned,
engine-owned inventory for `approximate::World`. Native and WASM consumers can
read the exact compiled artifact through
`physics_engine::approximate::PRIMITIVE_CAPABILITIES_JSON`. It is immutable
metadata; parsing belongs outside physical hot paths. The engine does not use
JSON or consumer presentation to choose collisions, dynamics or motion authority.

The ledger describes the limited floating-world API. It does not upgrade the
separate compatibility `World` or `RotatingWorld3d`, make a new shape authority,
or promise universal dense-contact quality. The [world contract](world-api-contract.md)
remains the authority for selecting a solver.

## Schema version 1

- `schemaVersion`, `world`, `worldApiStatus` and `scalar` identify the contract.
  Consumers must reject versions they do not understand.
- `capabilities` supplies the shared shape-capability cells for every listed kind.
  `internal-*` means an engine/kernel facility, not a new public support, distance
  or bounds API. Mass products, snapshot overlaps, finite rays and translation
  casts are public. Missing shape-cast features, angular CCD and chronological
  secondary-impact response remain explicit.
- `validation` records finite-value and dimension/mass boundaries. Capsule skeleton
  half length may be zero; radius stays positive. Fixed-body zero mass is distinct
  from uniform-solid mass input.
- `shapes` contains unique kind names and overrides: solver-owned dynamic rotation,
  solver COM origin and collapsed-skeleton support. Wedge mass products expose its
  geometric centroid/full tensor while physical body poses and lever arms retain
  the bounding-box origin. Dynamic wedges must remain rotation locked. This
  restriction does not prohibit statically oriented wedges or external authority.
- `pairs` contains exactly N(N+1)/2 canonical unordered cells. `dispatchIndex`
  identifies the public `GeometryStats::specialized_pair_dispatches` slot.
  `classification` is `analytic-specialized`, `fixed-topology-sat`,
  `fixed-topology-feature`, `intentional-generic-convex` or `unsupported`.
  The current ten cells are specialized and have `genericFallback: "none"`.
  Future generic/unsupported rows must be explicit, not silent accidental fallback.
- `manifoldPointLimit` bounds one fresh physical manifold: four points for box/box,
  one for the other current pairs. This does not count all contacts across substeps
  or claim richer multi-point capsule/wedge manifolds.
- `translationSearch` describes physical step admission; `shapeCastSearch`
  describes snapshot casts. Sphere/box physical paths retain their conservative
  analytic baseline, box polyhedra use fixed-axis swept SAT, and shared rounded
  paths use bounded conservative advancement with explicit search failures.
  Snapshot casts use the shared kernel path even for sphere/box pairs.
- `referenceAcceptance` is separate from dispatch. `capsule-row` links the completed
  sphere/capsule, box/capsule and capsule/capsule reference/work slice; `partial`
  is the honest status for the other cells and the global matrix. `evidence` links
  the corresponding repository documents. A green dispatch count does not turn
  a partial cell into full reference acceptance.
- `workCounters` names existing public report fields, including specialized and
  fallback calls, support/SAT/segment/feature work, manifold generation, search
  failures and retained/capacity growth products. Byte gauges exclude allocator
  overhead; counters do not claim complete machine-instruction accounting.
  `limitations` contains human explanations and never configures the solver.

The shape-capability cells are shared because the current four shapes have the
same exposed query and mass APIs. Their motion, origin and degeneracy differences
stay in the shape rows. Consumers can render a rectangular table from these two
parts without copying capability truth into a second repository.

## Verification and current limits

Three shared native/WASM fixtures parse the actual compiled ledger and execute
public mass/COM/inertia, rotated overlaps, finite rays and translation casts. They
preserve checkpoint bytes including a pending force; check validation boundaries
and wedge rotation restrictions; execute all ten physical cells in both orders;
and check the matching dispatch slot, contact presence, manifold ceiling and
zero generic fallback. The shape-name mapping is exhaustive over `Shape`, so a
new engine kind requires revisiting the gate.

Native gates compare all canonical kernel pairs with ledger classification/search
and manifold limits, verify advertised counter names against actual public fields,
and check conservative rotated bounds. The [candidate-bound repair](floating-conservative-bounds.md)
supplies independent surface-extrema and actual swept-endpoint enclosure evidence;
its minimized corner-contact/miss control also runs in the shared WASM driver.

These focused gates catch missing/duplicate cells, wrong dispatch slots, missing
public APIs and stale restrictions. They are not a replacement for independent
geometry/reference tests or a proof that a classification is faster. The global
matrix remains partial; #183 still owns remaining per-cell quality/work evidence
and the consumer-driven catalogue. General angular/chronological CCD remains #3,
character movement #194 and wedge COM-centered response #176.

```sh
cargo test --locked --test primitive_capabilities
cargo test --locked --lib approximate::capabilities
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example shape-query-contract
```

The complete Pages build runs three actual WASM replays of the separate query
acceptance module. JSON parsing uses `serde_json` only as a development dependency;
the engine's production graph, demo exports and physical continuation format do
not acquire a JSON parser. Collision Lab adoption is the following consumer slice.

Shared convention sourceRevision:
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
