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
| C2 | Infinite floor plane | ARPG, MMORPG | `SharedShape::halfspace(Y)` | **parity** (`character-move`, `arpg-planar`, `crowd`, `mmorpg-tick`): `Shape::HalfSpace` (static, normal = local +Y), closed-form rays and shape queries (box door probes included), characters, rigid contacts with dynamic bodies (`props`); not checkpointed | #366 |
| C3 | Capsule kinematic bodies with stable ids | MMORPG, ARPG | `RigidBodyBuilder::kinematic_position_based`, `Capsule::new_y`, `set_translation`, game key → handle map | **parity**: capsule shape, external motion, generational never-reused ids, game keys with `body_id(key)` lookup | #296 |
| C4 | Kinematic character move with slide; no motion under zero desired motion; exact rest against walls and ceilings; slopes steeper than the limit are walls | MMORPG, ARPG (monsters planar, player 3D) | `KinematicCharacterController::move_shape` (offset 0.0025 m, slide, slopes π/4; ARPG players: `autostep` 0.25 m when walking) | **parity** (`character-move`, `slopes`: wedge ramps climbed up to `max_slope`, steeper ones stop the character at their foot; `stairs`: `CharacterMove::autostep`, chosen per move by ARPG's walking rule, climbs steps up to its height; `stairs-pace`: ARPG's 0.2 m traversal stairs at 60 Hz, ground contact equal and positions within 0.5 u of Rapier every tick on 15 lanes clear of the 45° step threshold, one labelled threshold lane compared on outcomes only) | #294, #295, #303, #306, #307, #350, #353, #406 |
| C5 | Grounded/support state with snap-to-ground and slope limit; static-only support; exact support on box tops; rest under gravity; one static body excluded per move (MMORPG's open lake floor) | MMORPG, ARPG | controller `snap_to_ground`, `movement.grounded`; `cast_shape` with `only_fixed` + slope check; `QueryFilter::exclude_collider(floor)` on `move_shape` and the support cast over a basin | **parity** (`character-move`): static-only support, exact box-top support, snap, rest under gravity; `CharacterMoveResult::ground_contact` (Rapier's `grounded`, a step's rounded edge included; `stairs-pace`: equal every tick on every lane); (`basin`): `CharacterMove::exclude` opens a halfspace floor over a heightfield basin, eight swimmers walk in, dive weightless, swim and walk out up the far slope, positions within 0.02 m of Rapier, grounded equal every tick, support casts equal | #293, #304, #305, #309, #405 |
| C6 | Planar (2.5D) movement constraint | ARPG | game zeroes y around `move_shape` | **parity** (`arpg-planar`): `CharacterSettings::plane`, no floor collider needed, always grounded | #293 |
| C7 | Batched multi-character move, order-independent | ARPG, MMORPG | one `move_shape` per character with a BVH refresh between | **parity** (`crowd`, `crowd-fight`): one batch per tick in body order, independent of slice order | #294 |
| C8 | Depenetration reported apart from motion; embedded starts | ARPG, MMORPG | game clamps velocity; `move_shape` leaves a moving character where it starts embedded | **parity** (`arpg-planar`): `CharacterMoveResult::depenetration`, never part of `translation`; (`embedded-starts`): floor, box face, box corner and heightfield slope overlaps moved four ways, `Embedded::PushOut` (default) within 0.01 m of Rapier started outside, `Embedded::Stay` within 0.02 m of Rapier started embedded (Rapier's nudge works it out faster on halfspace tangents and heightfield diagonals), grounded equal every tick | #295, #404 |
| C9 | Accurate capsule/capsule contact normals (parallel, deep, coincident) | ARPG | custom `QueryDispatcher` over parry | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #289 |
| C10 | Accurate capsule-cast normals against long boxes | ARPG | custom `QueryDispatcher` (circle vs rectangle) | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #290 |
| C11 | Ray cast against static geometry (line of sight, ground height, projectiles) | ARPG, MMORPG | `QueryPipeline::cast_ray` with `only_fixed`, `Cuboid::cast_ray` | **parity** (`ray-sight`: rays alone, MMORPG sight lines, ground probes and projectiles against ~1,150 fixed boxes, ~0.95× Rapier native, ~0.93× WASM; `static-queries`): fixed bodies are queried through a four-wide tree with exact leaf bounds, so a first-hit ray visits ~8 nodes and repeats no per-leaf slab test | #415 |
| C12 | Shape overlap / penetration test (placement, door occupancy) | ARPG, MMORPG | `intersect_shape` + `contact` depth | **parity** (`static-queries`; ~0.3× Rapier's cost) | — |
| C13 | Length-unit-relative tolerances | MMORPG, ARPG | none (absolute metre tolerances; game converts units) | **parity**: `Config::length_unit` scales character, query and dynamics defaults (gravity, slop, sleep, restitution threshold, as Rapier's metre defaults); `World::character_settings` | #291, #318 |
| C14 | Query ties broken by stable id; save → load → continue matches | MMORPG (rebuilds ~1,150 colliders per tick), ARPG | none (game rebuilds the world every tick) | **parity** for queries/characters: ties by key, independent of slot and insertion history (no per-tick rebuild needed) | #292 |
| C15 | Whole-tick cost at or below Rapier (ARPG: ~18 µs native, ~27 µs WASM, 2.68 MB WASM; MMORPG: WASM 1.76 → 3.29 MB, debug combat ~31 s → ~41 s) | MMORPG, ARPG | — | **parity**: MMORPG (`mmorpg-tick`: its whole `World::step` pattern at its 60 Hz tick with its query dispatcher, head-on pairs, doors, line of sight; 0.11× Rapier native, 0.09× WASM) and ARPG (`arpg-tick`: its `Physics` pattern with players, planar monsters, doors, line of sight, projectiles, placements); engine WASM 0.47 MB vs Rapier 1.46 MB. **Consumer-integrated** (games' real physics paths, test/bench-only `physics-ab` feature): MMORPG 0.18–0.40× at 60 Hz, ARPG 0.07–0.30× Rapier per tick ([below](#consumer-integrated-evidence-c15)) | #330 |
| C16 | Characters separate and slide around each other (bounded pushing) | MMORPG | none (head-on contact blocks both) | **parity** (`crowd`, `crowd-fight`, `push`): head-on sidestep, half-each separation, bounded pushing under per-character `max_push` ceilings | #308 |
| C17 | Static heightfield terrain: capsule characters standing and walking on hills, ground rays | MMORPG (#159 step 2; caves later) | `SharedShape::heightfield` (flat cells `CELL_REMOVED`), KCC on terrain, `cast_ray` | **parity** (`terrain`, `terrain-holes`): `add_heightfield`, parry diagonal, removed cells (holes, MMORPG's flat cells over a floor plane), rays, sphere/capsule queries, box/wedge overlaps (door probes), characters, rigid contacts with dynamic boxes (`debris`); not checkpointed; caves later | #195, #342 |
| C18 | Dynamic boxes: upright movable crates (slide, stop at walls and units, rest, wake on the next push) and tumbling debris | MMORPG (mmorpg#58, #288) | `RigidBodyBuilder::dynamic()` (crates `lock_rotations`), cuboid colliders with density 600, friction 0.8, restitution 0.15, linear damping 0.2 (debris also angular 0.8), fixed boxes, capsules, halfspace and heightfield around them, one `step` per tick | **parity** (`props`, `debris`): rotation-locked boxes with friction and linear damping on the floor plane, against fixed boxes and capsules (2 substeps × 1 pass, rests within 5% of each slide of Rapier's position); tumbling debris with restitution, linear and angular damping, debris–debris contacts and box/heightfield contacts, settled by the game's rule into fixed boxes (2 substeps × 4 passes, sleep off): sinks no deeper than Rapier, at least 90% of rest points within 0.5 m (or 25% of the throw) of Rapier's and all within 1.5 m; mixed materials (`materials`: crates 0.8 / 0.15 on geometry at Rapier's 0.5 / 0 defaults, crate on crate, `Multiply` and `Max` slabs; Rapier's `CoefficientCombineRule` per body, default `Average`): every slide within 5% of Rapier's (measured 1.2–2.0%); crates in a 1,200-box town on the floor plane (`debris-town`, kicked in turn, sleep on): every rest within 5 cm of Rapier's, step cost independent of the town (#412); free-rotating debris dropped onto the plane there rests at Rapier's height within the 5 mm slop and stops within 1 s of Rapier's (#424); 48 free-rotating pieces thrown from up to 1.2 m into the town's lanes (`debris-town-tumble`, 2 substeps × 1 pass, sleep on) land at speed, tumble and sleep within 1 s of Rapier's last, never sink into the plane, at least 90% rest within 0.5 m of Rapier's and all within 1.5 m (#426) | #366, #391, #412, #424, #426 |

Scenarios live in `oracle/src/scenarios/`; `cargo run --release -p physics-oracle`
checks them natively (invariants + not more than 5% slower than Rapier) and
`node scripts/oracle-wasm.mjs` gates WASM timings; `bash scripts/size-budget.sh` also
reports WASM sizes against `.performance/size.json` (with `SIZE_BASE_REF=<commit>`, also each
size increase over that commit, measured in the same run). The size budget is advisory
(PRINCIPLE-015): going over it warns but never fails; the oracle gates stay blocking.

Not used by any consumer today: joints, sensors/contact events, collision
groups, CCD, triangle meshes. Work on those stays parked until a consumer
uses them. Engine and sandbox support (`ccd`, `sensor`, `retire_on_impact`, layers) is not
consumer adoption. The engine/game boundary for melee sweeps, contact enter/exit, buoyancy,
filtering, multi-hit queries and projectiles, with the consumer issues that would gate each, is
[D7](technical-decisions.md#d7--physical-interactions-reuse-the-world-consumer-gated-q32q38).

## Consumer-integrated evidence (C15)

The oracle scenarios above are engine-owned copies of the games' call patterns. #330 adds the
games' **real** physics integrations on both backends: a non-default `physics-ab` feature in each
game selects Rapier (production) or a thin physics-engine adapter behind the unchanged physics
API, and a merge-blocking A/B test in each game's CI asserts shared invariants on both. Timing
is a manual release example; production stays on Rapier.

| Consumer | PR / engine rev | Workloads | Behaviour | Native per-tick physics, engine / Rapier (median) |
| --- | --- | --- | --- | --- |
| MMORPG | mmorpg#323 (`caee2b9`), engine `3a1b841` | two-bots-move, vale-hub-walk, vale-wolf-hunt, Greyhaven idle/fights 8–64 | finite state, no penetration > 0.01 u, grounded at rest, door/LOS/shot/ground-height queries identical on identical states, deterministic replay; scripted players within 0.01 u of Rapier | 0.24–0.52× (e.g. hub-walk 490 vs 1743 µs); zone setup ~2.4× slower |
| MMORPG at 60 Hz | mmorpg#458 (`e3d8d09`, #379), engine `52e85ab` | the same plus vale-crate-smash, tick counts doubled for the same wall-clock time | the same invariants; scripted players' 30 Hz timetables kept | 0.18–0.40× (hub-walk 0.20×, crowded-fights-64 0.27×, crate shove/smash 0.34/0.40× with debris and props now counted); zone setup ~2–3× slower |
| ARPG | arpg#205 (`23dacb9`), engine `3a1b841` | 11 seeded workloads × 2 seeds × 120 ticks | invariants, deterministic replay, exact save continuation; identical outcomes and query decisions; positions within 0.5 u until first character contact | 0.07–0.30× (all workloads 2.5 vs 14.9 µs); 58–90% of Rapier's cost is the per-tick rebuild ARPG needs for exact saves |

Documented divergences (not parity claims): the engine's head-on sidestep vs Rapier blocking,
engine depenetration of units authored inside geometry (MMORPG's squat rats, slope spawns,
fixture players inside props; those fights are reported, not bounded, until MMORPG stops
authoring them embedded; `Embedded::Stay` keeps Rapier's behaviour where needed, #404), and Rapier's normal
nudge drift. Engine gaps found and fixed since: #336 (exact translation), #337 (local placement
invalidation), #340 (snap from geometry, so MMORPG's rebuilt worlds now continue), #341
(coincident surfaces), #342 (heightfield holes), #391 (Rapier's per-body material combine
rules, found by mmorpg#442), and, found at 60 Hz by mmorpg#458, #395 (support stopped at a
too-steep face, so a character at a chute's foot was not grounded) and #396 (a terrain cast could
report the cliff face a character was leaving, so it sank into the other face or the floor). The
first table was measured at `3a1b841`; MMORPG pins a revision with the #396 fix (since mmorpg#458)
and ARPG pins `cda9d1d` (2026-10-24); moving their pins picks the later fixes up. Not measured: WASM through the games' integrations, `Indexed` storage.

## Other dependents of the removed worlds

Not eligible: a cross-target replay guarantee (MMORPG stores f32 state in snapshots). Rapier
offers none and the standing rules make no cross-target claim.

These stay on their current pin until they move to Rapier: battle-royale (`World`), ecs-lab
(`RotatingWorld3d`), medieval (`collider_contact` only; removing the dependency), game-server
(`World` behind an unused feature). collision-lab's current main uses only `approximate::World`.
