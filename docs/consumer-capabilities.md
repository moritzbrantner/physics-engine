# Consumer capabilities

What the owner's projects use from physics, and where Physics Engine stands against Rapier for
each. Priority: MMORPG, then ARPG ([technical decisions](technical-decisions.md) D5). Status
**parity** means the capability has a scenario in the Rapier oracle harness that passes its
invariants and is not slower than Rapier; everything else is **missing** or **partial**.

Snapshot 2026-10-18: arpg `ae5d988` (rapier3d 0.36.1, 3D f32, default features, 60 Hz, centimetre
game units converted to metres), mmorpg `9e2e006` (still on the integer `World`; its Rapier switch
is in progress and will refine the MMORPG rows), game-server `5d62ddd` (optional `physics` feature
over the integer `World`; no consumer enables it).

## Capabilities

| # | Capability | Consumers | Rapier API used today | Engine status | Issues |
| --- | --- | --- | --- | --- | --- |
| C1 | Static boxes added and removed at runtime (walls, pillars, doors) | ARPG, MMORPG | `ColliderBuilder::cuboid`, `colliders.insert`, `remove_collider` | **parity** (`static-queries`) | — |
| C2 | Infinite floor plane | ARPG | `SharedShape::halfspace(Y)` | missing | — |
| C3 | Capsule kinematic bodies with stable ids | ARPG, MMORPG (boxes today) | `RigidBodyBuilder::kinematic_position_based`, `Capsule::new_y`, `set_translation`, game key → handle map | partial: capsule shape and external motion; ids are caller-chosen but reuse is not prevented | #296 |
| C4 | Kinematic character move with slide | ARPG (monsters planar, player 3D), MMORPG (player velocity moves) | `KinematicCharacterController::move_shape` (offset 0.0025 m, slide, slopes π/4, no autostep) | missing | #294, #295 |
| C5 | Grounded/support state with snap-to-ground and slope limit | ARPG, MMORPG (`is_grounded` overlap probe) | controller `snap_to_ground`, `movement.grounded`; `cast_shape` with `only_fixed` + slope check | partial: `has_support` predicate only | #293 |
| C6 | Planar (2.5D) movement constraint | ARPG | game zeroes y around `move_shape` | missing | #293 |
| C7 | Batched multi-character move, order-independent | ARPG, MMORPG | one `move_shape` per character with a BVH refresh between | missing | #294 |
| C8 | Depenetration reported apart from motion | ARPG | game clamps velocity | missing | #295 |
| C9 | Accurate capsule/capsule contact normals (parallel, deep, coincident) | ARPG | custom `QueryDispatcher` over parry | unverified against oracle | #289 |
| C10 | Accurate capsule-cast normals against long boxes | ARPG | custom `QueryDispatcher` (circle vs rectangle) | unverified against oracle | #290 |
| C11 | Ray cast against static geometry (line of sight, ground height, projectiles) | ARPG, MMORPG | `QueryPipeline::cast_ray` with `only_fixed`, `Cuboid::cast_ray` | **parity** within `static-queries` (`cast_ray_first`); rays alone still ~1.7× Rapier's cost | — |
| C12 | Shape overlap / penetration test (placement, door occupancy) | ARPG, MMORPG | `intersect_shape` + `contact` depth | **parity** (`static-queries`; ~0.3× Rapier's cost) | — |
| C13 | Length-unit-relative tolerances | ARPG | none (absolute metre tolerances; game converts units) | missing | #291 |
| C14 | Query ties broken by stable id; save → load → continue matches | ARPG | none (game rebuilds the world every tick) | partial: stable-id ordering; not proven across rebuilds | #292 |
| C15 | Whole-tick cost at or below Rapier (ARPG: ~18 µs native, ~27 µs WASM, 2.68 MB WASM) | ARPG | — | not measured | #297 |

Scenarios live in `oracle/src/scenarios/`; `cargo run --release --manifest-path oracle/Cargo.toml`
checks them natively (invariants + not more than 5% slower than Rapier) and
`node scripts/oracle-wasm.mjs` reports WASM timings and sizes.

Not used by any consumer today: dynamic rigid bodies, joints, sensors/contact events, collision
groups, CCD, materials, heightfields, triangle meshes. Work on those stays parked until a consumer
uses them.

## Other dependents of the removed worlds

These stay on their current pin until they move to Rapier: battle-royale (`World`), ecs-lab
(`RotatingWorld3d`), medieval (`collider_contact` only; removing the dependency), game-server
(`World` behind an unused feature). collision-lab's current main uses only `approximate::World`.
