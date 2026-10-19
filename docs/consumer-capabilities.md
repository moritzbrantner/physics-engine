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
| C1 | Static boxes added, removed and moved at runtime (walls, pillars, doors, moving platforms) | ARPG, MMORPG | `ColliderBuilder::cuboid`, `colliders.insert`, `remove_collider`, `Collider::set_translation` | **parity** (`static-queries`, `platforms`: `set_static_position` moves a box in place; riders carried by the consumer's adapter) | #352 |
| C2 | Infinite floor plane | ARPG, MMORPG | `SharedShape::halfspace(Y)` | **parity** (`character-move`, `arpg-planar`, `crowd`, `mmorpg-tick`): `Shape::HalfSpace` (static, normal = local +Y), closed-form rays and shape queries (box door probes included), characters; no rigid contacts, not checkpointed | — |
| C3 | Capsule kinematic bodies with stable ids | MMORPG, ARPG | `RigidBodyBuilder::kinematic_position_based`, `Capsule::new_y`, `set_translation`, game key → handle map | **parity**: capsule shape, external motion, generational never-reused ids, game keys with `body_id(key)` lookup | #296 |
| C4 | Kinematic character move with slide; no motion under zero desired motion; exact rest against walls and ceilings; slopes steeper than the limit are walls | MMORPG, ARPG (monsters planar, player 3D) | `KinematicCharacterController::move_shape` (offset 0.0025 m, slide, slopes π/4, no autostep) | **parity** (`character-move`, `slopes`: wedge ramps climbed up to `max_slope`, steeper ones stop the character at their foot) | #294, #295, #303, #306, #307, #350 |
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
| C15 | Whole-tick cost at or below Rapier (ARPG: ~18 µs native, ~27 µs WASM, 2.68 MB WASM; MMORPG: WASM 1.76 → 3.29 MB, debug combat ~31 s → ~41 s) | MMORPG, ARPG | — | **parity**: MMORPG (`mmorpg-tick`: its whole `World::step` pattern with its query dispatcher, head-on pairs, doors, line of sight) and ARPG (`arpg-tick`: its `Physics` pattern with players, planar monsters, doors, line of sight, projectiles, placements); engine WASM 0.47 MB vs Rapier 1.46 MB. **Consumer-integrated** (games' real physics paths, test/bench-only `physics-ab` feature): MMORPG 0.24–0.52×, ARPG 0.07–0.30× Rapier per tick ([below](#consumer-integrated-evidence-c15)) | #330 |
| C16 | Characters separate and slide around each other (bounded pushing) | MMORPG | none (head-on contact blocks both) | **parity** (`crowd`, `push`): head-on sidestep, half-each separation, bounded pushing under per-character `max_push` ceilings | #308 |
| C17 | Static heightfield terrain: capsule characters standing and walking on hills, ground rays | MMORPG (#159 step 2; caves later) | `SharedShape::heightfield` (flat cells `CELL_REMOVED`), KCC on terrain, `cast_ray` | **parity** (`terrain`, `terrain-holes`): `add_heightfield`, parry diagonal, removed cells (holes, MMORPG's flat cells over a floor plane), rays, sphere/capsule queries, box/wedge overlaps (door probes), characters; no rigid contacts, not checkpointed; caves later | #195, #342 |

Scenarios live in `oracle/src/scenarios/`; `cargo run --release --manifest-path oracle/Cargo.toml`
checks them natively (invariants + not more than 5% slower than Rapier) and
`node scripts/oracle-wasm.mjs` reports WASM timings and sizes.

Not used by any consumer today: dynamic rigid bodies, joints, sensors/contact events, collision
groups, CCD, materials, triangle meshes. Work on those stays parked until a consumer
uses them.

## Consumer-integrated evidence (C15)

The oracle scenarios above are engine-owned copies of the games' call patterns. #330 adds the
games' **real** physics integrations on both backends: a non-default `physics-ab` feature in each
game selects Rapier (production) or a thin physics-engine adapter behind the unchanged physics
API, and a merge-blocking A/B test in each game's CI asserts shared invariants on both. Timing
is a manual release example; production stays on Rapier.

| Consumer | PR / engine rev | Workloads | Behaviour | Native per-tick physics, engine / Rapier (median) |
| --- | --- | --- | --- | --- |
| MMORPG | mmorpg#323 (`caee2b9`), engine `3a1b841` | two-bots-move, vale-hub-walk, vale-wolf-hunt, Greyhaven idle/fights 8–64 | finite state, no penetration > 0.01 u, grounded at rest, door/LOS/shot/ground-height queries identical on identical states, deterministic replay; scripted players within 0.01 u of Rapier | 0.24–0.52× (e.g. hub-walk 490 vs 1743 µs); zone setup ~2.4× slower |
| ARPG | arpg#205 (`23dacb9`), engine `3a1b841` | 11 seeded workloads × 2 seeds × 120 ticks | invariants, deterministic replay, exact save continuation; identical outcomes and query decisions; positions within 0.5 u until first character contact | 0.07–0.30× (all workloads 2.5 vs 14.9 µs); 58–90% of Rapier's cost is the per-tick rebuild ARPG needs for exact saves |

Documented divergences (not parity claims): the engine's head-on sidestep vs Rapier blocking,
engine depenetration of units authored inside geometry (MMORPG's squat rats, slope spawns,
fixture players inside props; those fights are reported, not bounded), and Rapier's normal
nudge drift. Engine gaps found and fixed since: #336 (exact translation), #337 (local placement
invalidation), #340 (snap from geometry, so MMORPG's rebuilt worlds now continue), #341
(coincident surfaces), #342 (heightfield holes). The games still pin `3a1b841`; moving their
pins picks the fixes up. Not measured: WASM through the games' integrations, `Indexed` storage.

## Other dependents of the removed worlds

Not eligible: a cross-target replay guarantee (MMORPG stores f32 state in snapshots). Rapier
offers none and the standing rules make no cross-target claim.

These stay on their current pin until they move to Rapier: battle-royale (`World`), ecs-lab
(`RotatingWorld3d`), medieval (`collider_contact` only; removing the dependency), game-server
(`World` behind an unused feature). collision-lab's current main uses only `approximate::World`.
