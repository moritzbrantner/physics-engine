# Supported world APIs and migration contract

Issue: #192.

This document is the consumer-facing contract for choosing between the three public world surfaces.
It describes what the repository supports today; it is not a promise that a newer world is a drop-in
replacement for an older one.

## Status vocabulary

- **Supported** — intended for production use within the stated contract and covered by repository tests.
- **Limited / experimental** — usable for the documented scenarios, but missing guarantees needed for a
  general consumer migration.
- **Compatibility-only** — supported for existing consumers while they migrate; new engine work should
  not extend this representation unless required to preserve those consumers.
- **Missing** — no public contract exists. Consumers must not emulate it locally and call that an engine
  migration.

## Which world should a consumer use?

| World | Status | Intended use |
| --- | --- | --- |
| `physics_engine::World` | **Compatibility-only** | Existing integer, translational AABB consumers such as MMORPG, ARPG and Battle Royale. |
| `physics_engine::RotatingWorld3d` | **Supported with explicit limits** | Engine-owned rotating cuboids, interaction policies, overlap/contact queries and the existing sampled rotational/ballistic pipeline. |
| `physics_engine::approximate::World` | **Limited / experimental** | Persistent `f64` rigid-body state and the intended migration destination once each consumer-required guarantee is proven. |

A dependency update does not move a consumer between these worlds.

## Capability matrix

| Capability | `World` | `RotatingWorld3d` | `approximate::World` |
| --- | --- | --- | --- |
| Authoritative state | **Compatibility-only:** `Vec3i` position/velocity, integer mass/material boundary | **Supported:** one authoritative rotating-body world; quantized translational/material state plus fixed-point orientation/angular velocity | **Limited / experimental:** persistent `f64` position, velocity, quaternion and angular velocity |
| Shapes | **Compatibility-only:** translational AABBs | **Supported:** rotating cuboids; separate engine-owned ballistic spheres | **Limited / experimental:** box, sphere, capsule and wedge; movable wedges must remain rotation locked |
| Motion authority | Fixed or dynamic | **Supported:** physics-owned or externally driven bodies | **Limited / experimental:** zero-mass fixed, dynamic and external bodies |
| Units / timestep | Integer scene units; velocity per engine tick; `step(i32 ticks)` | Integer/fixed-point compatibility state; rational timestep numerator/denominator | `f64` scene units and seconds; `step(dt)` accepts finite `0..=0.1` seconds |
| Filters / sensors | **Missing:** no layers or sensor body contract | **Supported:** collision layers, interaction categories/policies, overlap-only participation | **Limited / experimental:** collision layers plus sensor/external flags |
| Queries | **Supported:** AABB overlap, AABB cast, ray cast and first ray hit over the current snapshot | **Supported:** oriented-box overlap, per-body current contacts and per-body overlaps | **Missing as a general public query API:** only solver-facing state plus `has_support` is public |
| Contact/event ordering | **Supported:** stable `BodyId` traversal; collision report order follows deterministic earliest-event processing | **Supported within the documented sampled pipeline:** stable body identities and deterministic ordered reports | **Limited / experimental:** stable body ordering and bounded sequential-impulse work; not promised to match event-solver event streams |
| CCD | **Supported:** swept translational AABB CCD | **Limited:** sampled rotating-box search; ballistic spheres have a dedicated continuous feature sweep. This is not analytic general rotational CCD | **Limited:** translational sweeps per substep with orientation held fixed; not analytic rotational CCD |
| Failure behavior | **Supported:** `step` builds next state off to the side and commits only after a successful step | **Supported only to the bounds documented by the rotating solver;** configured limits fail visibly | **Limited:** invalid/non-finite input fails visibly, but mid-step errors do not promise transactional rollback |
| Checkpoint / recovery | **Missing:** no first-class continuation-complete checkpoint API | **Missing:** no public continuation-complete checkpoint API | **Missing:** no public continuation-complete checkpoint API |
| Replay guarantee | Same build/target with stable inputs/order; no universal cross-target bit-identity claim | Same build/target under the documented sampled solver; no universal cross-target bit-identity claim | Same build/target for covered fixtures; cross-solver hashes are expected to differ |
| Bindings | Native Rust; the Pages demo is a narrow adapter, not a general binding | Native Rust; the Pages demo exposes selected scenario operations | Native Rust plus a narrow Pages comparison adapter; general consumer bindings remain missing |

Unsupported query, checkpoint or CCD behavior is deliberately classified as missing/limited here. A
consumer must block on the owning engine issue instead of silently falling back to a second local physics
implementation.

## Current consumer inventory

The current game consumers establish the compatibility surface that must remain usable while migration
work proceeds.

### MMORPG

`moritzbrantner/mmorpg/crates/mmorpg-core/src/zone.rs` currently uses
`physics_engine::World` to:

- add fixed colliders/world-limit bodies and dynamic gameplay bodies;
- set player/body velocity;
- advance the authoritative simulation with `step(1)`;
- read bodies after collision correction;
- remove bodies on lifecycle changes; and
- use `overlap_query` for current collision/occupancy checks.

The migration tracked by `moritzbrantner/mmorpg#46` therefore needs lifecycle, mutation, stepping,
post-collision reads, queries and continuation-safe recovery before it can switch worlds.

### ARPG

`moritzbrantner/arpg/crates/arpg-core/src/lib.rs` uses `World` for fixed dungeon/door geometry,
dynamic players, velocity updates, `step(1)`, post-step body reads and body removal. The migration in
`moritzbrantner/arpg#48` additionally depends on the character/corner/sliding behavior required by that
game; those semantics are not inferred from a dependency bump.

### Battle Royale

`moritzbrantner/battle-royale/crates/battle-royale-core/src/lib.rs` uses `World` for player admission
and removal, velocity updates, `step(1)`, and post-step position/velocity reads. The migration in
`moritzbrantner/battle-royale#7` must preserve match/recovery semantics independently of solver choice.

### Existing floating-state consumer

`moritzbrantner/collision-lab/wasm/src/physics_engine.rs` already imports
`approximate::{Body, Config, Report, Shape, Vector, World}`. It is evidence for the floating-state
surface, not proof that the API already covers the game consumers above.

## Integer-tick to floating-state conversion

A migrating consumer must choose explicit scale constants rather than scattering casts through gameplay
code.

Let:

- `U` = integer position units per chosen floating distance unit;
- `T` = legacy engine ticks per second.

For a legacy `Vec3i` state and an `approximate::World` state:

```text
position_f64       = position_i32 / U
half_extent_f64    = half_extent_i32 / U
velocity_f64       = velocity_i32 * T / U
gravity_f64        = gravity_i32 * T * T / U
dt_seconds         = ticks / T
restitution_f64    = restitution_milli / 1000
friction_f64       = friction_milli / 1000
```

Keep `BodyId` stable. Preserve relative mass ratios; do not invent a mass-unit conversion unless the
consumer has an external physical unit contract.

### Gravity, force and impulse meanings

The compatibility `World` does **not** expose a public force or impulse command. Its `gravity` is an
integer velocity delta per engine tick that is applied before motion. Therefore there is no honest
force/impulse value to mechanically copy from `World`; a consumer that introduces those commands must
define their physical meaning instead of relabeling an old gameplay number.

In `approximate::World` the public quantities have ordinary second-based meanings:

- `gravity` is acceleration in distance units per second squared;
- `Body::mass` is the mass scale used by response and by force/impulse integration;
- `add_force(id, F)` accumulates force for the next step; the solver applies `F / mass` as acceleration
  during that step's substeps and clears accumulated force after a successful step;
- `apply_impulse(id, J, point)` is an instantaneous momentum change applied before the first substep:
  linear velocity changes by `J / mass`, and the off-center component contributes angular impulse through
  the lever arm;
- collider dimensions use the same distance conversion as positions; box dimensions in both APIs are
  **half extents**, not full widths.

Because `World` has no force/impulse API, conversion tests should compare the common position, velocity,
gravity, mass/material and collision boundary first. New force/impulse gameplay needs its own acceptance
fixture with units stated explicitly.

### Query result conventions

Do not normalize different query surfaces in game code by inventing missing data:

- `World::overlap_query` returns matching `BodyId` values in stable `BodyId` order.
- `World::cast_aabb` and `World::ray_cast` query the **current snapshot**: target body velocities are
  intentionally ignored. Hits are ordered by time of impact and then `BodyId`. An initial overlap is
  reported at time zero with `normal: None`; swept hits carry a contact normal.
- `World::ray_cast_first` has the same ordering semantics without allocating the complete hit list.
- `RotatingWorld3d::overlap_query` exposes exact current oriented-box overlap identities.
  `body_contacts` is subject-relative: contacts are ordered by the other `BodyId` and its SAT axis is
  oriented from the queried subject toward that body. `body_overlaps` is the sensor/trigger seam and does
  not promote overlaps into solver contacts.
- `approximate::World` does not yet expose a general ray/cast/overlap query contract. `has_support` is
  a solver-facing support predicate, not a substitute for physical query results. Consumers must wait for
  #194 rather than reconstructing a second query engine from exported body state.

The executable compatibility test in `tests/world_migration_contract.rs` uses `U = 1` and
`T = 60` and proves the no-contact, one-substep integration boundary against the compatibility
`World`. It intentionally does **not** claim cross-solver contact equivalence: contact manifolds,
substepping and numerical representation can deliberately change physical outcomes and require their own
reference-oracle acceptance.

### Export quantization

Quantization belongs at an explicit serialization/render/network compatibility boundary:

```text
authoritative f64 state -> checked finite/range validation -> quantized export
```

Do not feed the quantized export back into the authoritative floating world every tick. That would turn a
wire/storage compatibility format into simulation authority and discard the persistent-state benefit.

Before any float-to-integer conversion:

1. reject non-finite values;
2. validate the scaled value against the destination range without relying on Rust's saturating cast;
3. apply the consumer's explicitly chosen rounding rule; and
4. keep the unquantized state authoritative.

## Promotion criteria for the persistent f64 world

`approximate::World` becomes the general-purpose migration destination only when the concrete consumer
slice proves all of the guarantees it actually needs:

1. **Semantic coverage** — required shapes, material/interaction behavior, lifecycle mutations and
   post-collision reads exist upstream.
2. **Query/character coverage** — consumer-required casts, support/contact evidence and character motion
   are supplied by the engine rather than recreated in game code. Track this under #194.
3. **Contact quality** — dense/resting/corner cases meet independent physical/reference acceptance.
   Track known dense-contact work under #193.
4. **Failure and recovery** — failed steps cannot publish partial game success, and continuation-complete
   checkpoints exist when a consumer persists physics. Track this under #198.
5. **Binding coverage** — native/WASM consumers can use the supported surface without per-body bridge
   churn. Track this under #201.
6. **Measured whole-consumer benefit** — compare equal completed workloads and deterministic work
   evidence. Compatible current-`World` improvements are tracked separately from solver migration
   (including #190).

Until those conditions are demonstrated, `World` remains supported for its existing consumers and
`approximate::World` remains explicitly limited rather than being renamed or substituted silently.

## Owning issues for missing migration prerequisites

Missing behavior stays visible and owned upstream:

| Missing or limited prerequisite | Owning issue |
| --- | --- |
| General ray/overlap/sphere/capsule casts, stable feature-rich query results, and reusable character collide-and-slide | #194 |
| Heightfield/terrain support integrated through that query contract | #195 |
| Dense/resting contact quality beyond the currently passing bounded fixtures | #193 |
| Complete primitive-pair specialization/quality coverage where a consumer needs it | #183 |
| Static arbitrary triangle-mesh collision | #182 |
| General CCD ownership beyond today's translational sweeps, sampled rotation, and dedicated ballistic lane | #3 |
| Failure-safe floating steps and continuation-complete physical checkpoints | #198 |
| Supported native/WASM consumer seams beyond scenario-specific demo adapters | #201 |
| Same-mode/cross-solver/cross-target reference evidence and deterministic ratchets | #202 |
| Portable physical failure replay/debug capture for consumer failures | #203 |

Compatibility-only omissions such as collision layers on the old translational `World` are not invitations
to extend it indefinitely. If a migration requires a capability, add it to the destination world under the
owning issue and prove it with that consumer; do not build a game-local fallback.

## Compiling examples

The repository carries one minimal Cargo example for each public world surface:

- `examples/translational_world.rs`
- `examples/rotating_world.rs`
- `examples/floating_world.rs`

`cargo build --examples --locked` compiles all three. The ordinary `cargo clippy --all-targets` and
test/build tiers also cover them.
