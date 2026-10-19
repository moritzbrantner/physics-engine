# Consumer capabilities

What the owner's projects use from physics, and where Physics Engine stands against Rapier for
each. Priority: MMORPG, then ARPG ([technical decisions](technical-decisions.md) D5). Status
**parity** means the capability has a scenario in the Rapier oracle harness that passes its
invariants and is not slower than Rapier; everything else is **missing** or **partial**.

Snapshot 2026-10-18: arpg `ae5d988` (rapier3d 0.36.1, 3D f32, default features, 60 Hz, centimetre
game units converted to metres), MMORPG's Rapier switch (mmorpg#273: rapier3d 0.36.1 f32, kinematic
capsules moved by `KinematicCharacterController`, fixed cuboids for content, doors and NPCs,
halfspace ground and world floor, ARPG's exact upright-shape query dispatcher, ~1,150 Greyhaven
colliders, centimetre units converted to metres), game-server `5d62ddd` (optional `physics` feature
over the integer `World`; no consumer enables it).

## Capabilities

| # | Capability | Consumers | Rapier API used today | Engine status | Issues |
| --- | --- | --- | --- | --- | --- |
| C1 | Static boxes added and removed at runtime (walls, pillars, doors) | ARPG, MMORPG | `ColliderBuilder::cuboid`, `colliders.insert`, `remove_collider` | **parity** (`static-queries`) | — |
| C2 | Infinite floor plane | ARPG, MMORPG | `SharedShape::halfspace(Y)` | **parity** (`character-move`, `arpg-planar`, `crowd`, `mmorpg-tick`): `Shape::HalfSpace` (static, normal = local +Y), closed-form rays and shape queries (box door probes included), characters; no rigid contacts, not checkpointed | — |
| C3 | Capsule kinematic bodies with stable ids | MMORPG, ARPG | `RigidBodyBuilder::kinematic_position_based`, `Capsule::new_y`, `set_translation`, game key → handle map | **parity**: capsule shape, external motion, generational never-reused ids, game keys with `body_id(key)` lookup | #296 |
| C4 | Kinematic character move with slide; no motion under zero desired motion; exact rest against walls and ceilings | MMORPG, ARPG (monsters planar, player 3D) | `KinematicCharacterController::move_shape` (offset 0.0025 m, slide, slopes π/4, no autostep) | **parity** (`character-move`) | #294, #295, #303, #306, #307 |
| C5 | Grounded/support state with snap-to-ground and slope limit; static-only support; exact support on box tops; rest under gravity | MMORPG, ARPG | controller `snap_to_ground`, `movement.grounded`; `cast_shape` with `only_fixed` + slope check | **parity** (`character-move`): static-only support, exact box-top support, snap, rest under gravity | #293, #304, #305, #309 |
| C6 | Planar (2.5D) movement constraint | ARPG | game zeroes y around `move_shape` | **parity** (`arpg-planar`): `CharacterSettings::plane`, no floor collider needed, always grounded | #293 |
| C7 | Batched multi-character move, order-independent | ARPG, MMORPG | one `move_shape` per character with a BVH refresh between | **parity** (`crowd`): one batch per tick in body order, independent of slice order | #294 |
| C8 | Depenetration reported apart from motion | ARPG, MMORPG | game clamps velocity | **parity** (`arpg-planar`): `CharacterMoveResult::depenetration`, never part of `translation` | #295 |
| C9 | Accurate capsule/capsule contact normals (parallel, deep, coincident) | ARPG | custom `QueryDispatcher` over parry | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #289 |
| C10 | Accurate capsule-cast normals against long boxes | ARPG | custom `QueryDispatcher` (circle vs rectangle) | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #290 |
| C11 | Ray cast against static geometry (line of sight, ground height, projectiles) | ARPG, MMORPG | `QueryPipeline::cast_ray` with `only_fixed`, `Cuboid::cast_ray` | **parity** within `static-queries` (`cast_ray_first`); rays alone still ~1.7× Rapier's cost | — |
| C12 | Shape overlap / penetration test (placement, door occupancy) | ARPG, MMORPG | `intersect_shape` + `contact` depth | **parity** (`static-queries`; ~0.3× Rapier's cost) | — |
| C13 | Length-unit-relative tolerances | MMORPG, ARPG | none (absolute metre tolerances; game converts units) | **parity** for characters and queries: `Config::length_unit`, `World::character_settings`; dynamics defaults parked (#318) | #291 |
| C14 | Query ties broken by stable id; save → load → continue matches | MMORPG (rebuilds ~1,150 colliders per tick), ARPG | none (game rebuilds the world every tick) | **parity** for queries/characters: ties by key, independent of slot and insertion history (no per-tick rebuild needed) | #292 |
| C15 | Whole-tick cost at or below Rapier (ARPG: ~18 µs native, ~27 µs WASM, 2.68 MB WASM; MMORPG: WASM 1.76 → 3.29 MB, debug combat ~31 s → ~41 s) | MMORPG, ARPG | — | **parity**: MMORPG (`mmorpg-tick`: its whole `World::step` pattern with its query dispatcher, head-on pairs, doors, line of sight) and ARPG (`arpg-tick`: its `Physics` pattern with players, planar monsters, doors, line of sight, projectiles, placements); engine WASM 0.47 MB vs Rapier 1.46 MB | — |
| C16 | Characters separate and slide around each other (bounded pushing) | MMORPG | none (head-on contact blocks both) | **parity** (`crowd`, `push`): head-on sidestep, half-each separation, bounded pushing under per-character `max_push` ceilings | #308 |
| C17 | Static heightfield terrain: capsule characters standing and walking on hills, ground rays | MMORPG (#159 step 2; caves later) | `SharedShape::heightfield`, KCC on terrain, `cast_ray` | **parity** (`terrain`): `add_heightfield`, parry diagonal, rays, sphere/capsule queries, box/wedge overlaps (door probes), characters; no rigid contacts, not checkpointed; holes/caves later | #195 |

Scenarios live in `oracle/src/scenarios/`; `cargo run --release --manifest-path oracle/Cargo.toml`
checks them natively (invariants + not more than 5% slower than Rapier) and
`node scripts/oracle-wasm.mjs` reports WASM timings and sizes.

Not used by any consumer today: dynamic rigid bodies, joints, sensors/contact events, collision
groups, CCD, materials, triangle meshes. Work on those stays parked until a consumer
uses them.

## Other dependents of the removed worlds

Not eligible: a cross-target replay guarantee (MMORPG stores f32 state in snapshots). Rapier
offers none and the standing rules make no cross-target claim.

These stay on their current pin until they move to Rapier: battle-royale (`World`), ecs-lab
(`RotatingWorld3d`), medieval (`collider_contact` only; removing the dependency), game-server
(`World` behind an unused feature). collision-lab's current main uses only `approximate::World`.
