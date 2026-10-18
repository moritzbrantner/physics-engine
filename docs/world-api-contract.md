# World API contract

Physics Engine has one world: `physics_engine::approximate::World`, with `f32` physical state
([technical decisions](technical-decisions.md) D3). The translational integer `World` and
`RotatingWorld3d` are being removed without shims; consumers that still use them stay on their
current git pin until they move to Rapier.

The API is designed around how the consuming games use Rapier (D2). It is not a Rapier clone: a
consumer uses a thin adapter, and every capability listed as at parity is checked against Rapier by
the oracle harness (D4). The per-capability parity status is tracked in
[consumer capabilities](consumer-capabilities.md).

## Units and meanings

- Positions and collider dimensions are scene distance units; box dimensions are **half extents**.
- `step(dt)` takes seconds (finite, `0..=0.1`). Elapsed time is kept in `f64`.
- `gravity` is acceleration in distance units per second squared.
- `Body::mass` scales response and force/impulse integration.
- `add_force(id, F)` accumulates force for the next step (`F / mass` per substep) and is cleared
  after a successful step.
- `apply_impulse(id, J, point)` changes linear velocity by `J / mass` before the first substep;
  the off-centre component adds angular impulse through the lever arm.

## Ordering and determinism

- Body ids are stable. Pair traversal, events and query hits are ordered by stable id after the
  primary key (time of impact or distance), so ties are deterministic.
- Results are identical across worker counts on the same build and target. There is no
  cross-target bit-identity claim.

## Queries

- `overlap_shape` and `cast_shape` query current authoritative poses through the shared
  `rust-kernels` primitives and return stable ids, ordered contact witnesses, outward target
  normals and support velocity; see [floating shape queries](floating-shape-queries.md).
  Translation-only casts hold orientations and target positions fixed.
- `cast_ray` queries a finite segment against current poses with local feature identity; see
  [floating ray queries](floating-ray-queries.md).
- `has_support` is a solver-facing support predicate.

## Failure behaviour

A returned step error rolls back touched motion, history, sleep and retirement state; time and the
prior successful report are retained ([step transactions](floating-step-transactions.md)). Panics
are excluded. Non-finite inputs are rejected, never saturated.

## Checkpoints

Versioned physical checkpoints restore only on their own build and target
([floating checkpoints](floating-checkpoints.md)). Moving between providers or targets
reconstructs physics from consumer-owned game state; Rapier state is never deserialized into a
Physics Engine checkpoint.

## Example

`examples/floating_world.rs` is the minimal compiling example; `cargo build --examples --locked`
builds it.
