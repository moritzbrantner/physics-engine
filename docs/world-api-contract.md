# World API contract

Physics Engine has one world: `physics_engine::approximate::World`, with `f32` physical state
([technical decisions](technical-decisions.md) D3). The translational integer `World` and
`RotatingWorld3d` were removed without shims; consumers that still use them stay on their old git
pin until they move to Rapier.

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

## Body ids and keys

- `add_body(body)` returns the body's `BodyId`: a 32-bit slot and a 32-bit generation packed in a
  `u64`. Slots are pooled (the lowest free slot is reused) and every reuse bumps the generation,
  so the id of a removed body is rejected (`MissingBody`) and never aliases a newer body. Lookup
  is a direct slot index. `BodyId::{slot, generation, to_bits, from_bits}` serve adapters.
- `Body::keyed(key, ..)` (or `body.key = Some(key)`) attaches the game's stable entity id.
  Keys are unique among live bodies (`DuplicateKey`); `World::body_id(key)` finds the live body.
- Ids are handles, not an order (`BodyId` has no `Ord`). Engine ids are never meant to be saved:
  games rebuild physics from game state on load.

## Ordering and determinism

- Every traversal, pair orientation and tie (equal time of impact or distance) follows body
  order: keyed bodies first, by key, then keyless bodies by creation serial within the run.
  Results therefore depend on keys and the current contents, not on slot reuse or the insertion
  order of keyed bodies; a world rebuilt from game state answers queries and character moves
  exactly as the uninterrupted one. Give a body a key when its ties must survive save/load;
  keyless bodies (projectiles and other transient objects) are deterministic within a run only.
  Persistent contact warm-start state is not part of a rebuild; use checkpoints to continue
  dynamics bit-exactly.
- Results are identical across worker counts on the same build and target. There is no
  cross-target bit-identity claim.

## Queries

- `overlap_shape` and `cast_shape` query current authoritative poses through the shared
  `rust-kernels` primitives and return stable ids, ordered contact witnesses, outward target
  normals and support velocity; see [floating shape queries](floating-shape-queries.md).
  Translation-only casts hold orientations and target positions fixed.
- `cast_ray` queries a finite segment against current poses with local feature identity; see
  [floating ray queries](floating-ray-queries.md). `cast_ray_first` returns only the earliest
  hit (ties by lower BodyId), the same hit as `cast_ray(..)[0]`, without collecting the rest.
- Queries traverse an incremental AABB tree (`bvh-kernels`) kept current by `add_body`,
  `remove_body` and `step`; it changes which bodies are tested, never a result.
- `has_support` is a solver-facing support predicate.

## Failure behaviour

A returned step error rolls back touched motion, history, sleep and retirement state; time and the
prior successful report are retained ([step transactions](floating-step-transactions.md)). Panics
are excluded. Non-finite inputs are rejected, never saturated.

## Checkpoints

Checkpoints are opt-in (Cargo feature `checkpoints`) for debugging and logging. Versioned physical
checkpoints restore only on their own build and target
([floating checkpoints](floating-checkpoints.md)). Moving between providers or targets
reconstructs physics from consumer-owned game state; Rapier state is never deserialized into a
Physics Engine checkpoint.

## Example

`examples/floating_world.rs` is the minimal compiling example; `cargo build --examples --locked`
builds it.
