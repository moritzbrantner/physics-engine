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

- `Config::length_unit` declares scene units per metre (1 by default, 100 for a centimetre
  game). Built-in length defaults are defined in metres and scaled by it —
  `World::character_settings()` returns the character defaults (2.5 mm offset, 5 cm snap) in
  scene units; values the game passes explicitly stay in scene units. Query and controller
  tolerances are relative (to magnitudes or to the character offset), so a scene scaled by the
  unit moves identically. The rigid-body dynamics defaults (`gravity`, `contact_slop`,
  `sleep_speed`) are still the historical scene-unit values.

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
- `World::bodies()` yields the live bodies in body order; `body_count()` counts them.

## Body storage

`Config::body_storage` picks how a world stores its bodies. It is fixed when the world is
created. Both modes give identical results for the same calls: bodies, ids, step reports, query
results and character moves. CI runs every test suite in both modes, and the Rapier oracle
checks that both produce the same traces. Only the cost of adding and removing bodies differs.

| | `Ordered` (default) | `Indexed` |
| --- | --- | --- |
| Layout | one array in body order | arena in insertion order, plus an order index |
| Add after every body | amortized O(1) | O(log n) |
| Add in the middle of the order | O(n) (later records move) | O(log n) |
| Remove any body | amortized O(1) | O(log n) |
| Next step after adds and removes | O(n) compaction if bodies were removed | O(n + k log k) sort of the k out-of-order adds |
| Steps, queries, characters | contiguous, in body order | the same (the step sorts the arena first) |

A removal leaves a tombstone. The next step compacts the tombstones away, or a removal does it
once tombstones are half the records. Removing a body that touches others also costs
O(contact pairs), because its contacts wake their islands. `set_velocity`, `apply_impulse` and
`add_force` sort `Indexed` storage first when bodies were added out of order, because waking an
island searches storage by body order.

When to pick which:

- **`Ordered`** suits games whose keys mostly grow, such as entity ids handed out in sequence,
  and games that add and remove only a few bodies per tick. Appends and removals are O(1), and
  no index is kept.
- **`Indexed`** suits games that insert many bodies in the middle of the key order between
  steps, such as streaming level chunks with pre-assigned keys or spawning many keyed bodies
  while keyless ones exist (keyless bodies sort after every keyed one). Each add costs O(log n)
  instead of moving the later records.

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

## Terrain

`add_heightfield(HeightfieldDesc { origin, cell_size, columns, rows, heights }, key)` adds
static heightfield terrain (heights row-major, Z rows then X columns, added to `origin.1`) and
returns its body, whose shape is `Shape::Heightfield`. Each cell is split along the same
diagonal as parry/Rapier heightfields. Rays, sphere and capsule overlaps and casts, and
character moves work against it. Box and wedge overlaps (door probes) are tested against the
triangles by the separating-axis test: whether they overlap is exact, and the reported depth and
normal come from the deepest triangle alone. The terrain is a surface, so a box wholly beneath
it does not touch it. Box and wedge casts against terrain return an error. Terrain takes no part
in rigid-body contacts, and worlds with terrain are not checkpointed. `add_body` rejects
heightfield shapes.

## Halfspaces

`Shape::HalfSpace` is the solid side of the plane through the body position; the outward
normal is the body's local +Y, so the identity orientation gives an infinite floor. It is
static only (mass 0, not external). Every ray and shape query tests it with a closed form,
wherever the query is; a box or wedge meets it at its deepest point. Characters stand,
walk and snap on it. Like terrain, it takes no part in rigid-body contacts, and worlds with
a halfspace are not checkpointed.

## Characters

`add_character(id, CharacterSettings)` makes an external (kinematic) upright capsule body a
character. `move_characters(&[CharacterMove], &mut Vec<CharacterMoveResult>)` is the only move
primitive: collide-and-slide that keeps `offset` to every surface it touches, then a grounded
test against walkable static support within `offset + snap_to_ground` (snapping down when the
character was grounded and does not move up). Zero desired motion never moves a character;
penetration is resolved first and reported as `depenetration`, apart from `translation`.
`CharacterSettings::plane: Some(height)` makes a planar character: it moves only horizontally
at its height, needs no floor collider and is always grounded with `support` `None`.
Depenetration leaves static and kinematic bodies, not other characters.
Moves run once each in body order (keyed by key, then keyless by creation), independent of
the order of the slice; each sees the earlier movers at their new positions, and `out[i]` is
the result of `moves[i]`. Characters of a batch that overlap each other are first separated
horizontally, half each (a character overlapping one outside the batch moves the whole way);
that separation is part of `depenetration`. A character blocked head-on by another character
sidesteps to its right instead of locking. `CharacterSettings::max_push: Some(ceiling)` enables
bounded pushing: a mover pushes a blocking character horizontally by at most the smaller of their
two ceilings per move (scene units; `None` on either side: no push) and follows it; the pushed
character keeps its gap to everything else, so walls and other characters stop it, and the part
of the motion it does not yield is handled as above. What a character received from pushes during
the batch is reported as `pushed`, apart from `translation` and `depenetration`. On walkable
surfaces downward intent stops instead of sliding down the slope; horizontal intent follows the
surface.
`set_kinematic_position` places an external body without resolving contact.

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
