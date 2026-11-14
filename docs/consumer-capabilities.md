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
| C2 | Infinite floor plane | ARPG, MMORPG | `SharedShape::halfspace(Y)` | **parity** (`character-move`, `arpg-planar`, `crowd`, `mmorpg-tick`): `Shape::HalfSpace` (static, normal = local +Y), closed-form rays and shape queries (box door probes included), characters, rigid contacts with dynamic bodies (`props`); checkpointed inline (#452, engine fixture only) | #366 |
| C3 | Capsule kinematic bodies with stable ids | MMORPG, ARPG | `RigidBodyBuilder::kinematic_position_based`, `Capsule::new_y`, `set_translation`, game key → handle map | **parity**: capsule shape, external motion, generational never-reused ids, game keys with `body_id(key)` lookup | #296 |
| C4 | Kinematic character move with slide; no motion under zero desired motion; exact rest against walls and ceilings; slopes steeper than the limit are walls | MMORPG, ARPG (monsters planar, player 3D), Shooter | `KinematicCharacterController::move_shape` (offset 0.0025 m, slide, slopes π/4; ARPG players: `autostep` 0.25 m when walking) | **parity** (`character-move`, `slopes`: wedge ramps climbed up to `max_slope`, steeper ones stop the character at their foot; `stairs`: `CharacterMove::autostep`, chosen per move by ARPG's walking rule, climbs steps up to its height; `stairs-pace`: ARPG's 0.2 m traversal stairs at 60 Hz, ground contact equal and positions within 0.5 u of Rapier every tick on 15 lanes clear of the 45° step threshold, one labelled threshold lane compared on outcomes only; `shooter-window`: 2 s of a recorded shooter deathmatch (moenarch-games/shooter#37) replayed from each fighter's per-tick move request with step-up on every move, six fighters sprinting, sliding, launched off a jump pad and thrown by a rocket blast: the engine, configured as the shooter's adapter, reproduces the shooter's recorded trajectory and grounded state, Rapier stays within 5 mm (measured 2.1 mm) with equal grounded state) | #294, #295, #303, #306, #307, #350, #353, #406 |
| C5 | Grounded/support state with snap-to-ground and slope limit; static-only support; exact support on box tops; rest under gravity; one static body excluded per move (MMORPG's open lake floor) | MMORPG, ARPG | controller `snap_to_ground`, `movement.grounded`; `cast_shape` with `only_fixed` + slope check; `QueryFilter::exclude_collider(floor)` on `move_shape` and the support cast over a basin | **parity** (`character-move`): static-only support, exact box-top support, snap, rest under gravity; `CharacterMoveResult::ground_contact` (Rapier's `grounded`, a step's rounded edge included; `stairs-pace`: equal every tick on every lane); (`basin`): `CharacterMove::exclude` opens a halfspace floor over a heightfield basin, eight swimmers walk in, dive weightless, swim and walk out up the far slope, positions within 0.02 m of Rapier, grounded equal every tick, support casts equal; a zero-motion move from a verified clear pose reuses its last support outcome exactly while the pose, exclusion and `World::static_revision()` are unchanged (no shape query in a repeated batch), and the revision lets MMORPG cache its own static-only ground probes (`probe_ground_gap`, `grounded()`; consumer adoption pending) | #293, #304, #305, #309, #405, #526 |
| C6 | Planar (2.5D) movement constraint | ARPG | game zeroes y around `move_shape` | **parity** (`arpg-planar`): `CharacterSettings::plane`, no floor collider needed, always grounded | #293 |
| C7 | Batched multi-character move, order-independent | ARPG, MMORPG | one `move_shape` per character with a BVH refresh between | **parity** (`crowd`, `crowd-fight`): one batch per tick in body order, independent of slice order | #294 |
| C8 | Depenetration reported apart from motion; embedded starts | ARPG, MMORPG | game clamps velocity; `move_shape` leaves a moving character where it starts embedded | **parity** (`arpg-planar`): `CharacterMoveResult::depenetration`, never part of `translation`; (`embedded-starts`): floor, box face, box corner and heightfield slope overlaps moved four ways, `Embedded::PushOut` (default) within 0.01 m of Rapier started outside, `Embedded::Stay` within 0.02 m of Rapier started embedded (Rapier's nudge works it out faster on halfspace tangents and heightfield diagonals), grounded equal every tick | #295, #404 |
| C9 | Accurate capsule/capsule contact normals (parallel, deep, coincident) | ARPG | custom `QueryDispatcher` over parry | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #289 |
| C10 | Accurate capsule-cast normals against long boxes | ARPG | custom `QueryDispatcher` (circle vs rectangle) | **parity** (`capsule-normals`): exact against closed forms, no custom dispatcher needed | #290 |
| C11 | Ray cast against static geometry (line of sight, ground height, projectiles) | ARPG, MMORPG | `QueryPipeline::cast_ray` with `only_fixed`, `Cuboid::cast_ray` | **parity** (`ray-sight`: rays alone, MMORPG sight lines, ground probes and projectiles against ~1,150 fixed boxes, ~0.95× Rapier native, ~0.93× WASM; `static-queries`): fixed bodies are queried through a four-wide tree with exact leaf bounds, so a first-hit ray visits ~8 nodes and repeats no per-leaf slab test | #415 |
| C12 | Shape overlap / penetration test (placement, door occupancy) | ARPG, MMORPG | `intersect_shape` + `contact` depth | **parity** (`static-queries`; ~0.3× Rapier's cost) | — |
| C13 | Length-unit-relative tolerances | MMORPG, ARPG | none (absolute metre tolerances; game converts units) | **parity**: `Config::length_unit` scales character, query and dynamics defaults (gravity, slop, sleep, restitution threshold, as Rapier's metre defaults); `World::character_settings` | #291, #318 |
| C14 | Query ties broken by stable id; save → load → continue matches | MMORPG (rebuilds ~1,150 colliders per tick), ARPG | none (game rebuilds the world every tick) | **parity** for queries/characters: ties by key, independent of slot and insertion history (no per-tick rebuild needed). Opt-in checkpoints continue a zone with floor halfspace, heightfield, platform, characters and crates bit-identically on the same build and target, native and WASM (#452: `tests/checkpoint_zone.rs`, `scripts/checkpoint-wasm.sh`; [checkpoint contract](floating-checkpoints.md#terrain-and-character-state-452)); **consumer adoption pending**: no game saves through it yet, MMORPG's recovery acceptance and joints in checkpoints are deferred | #292, #452 |
| C15 | Whole-tick cost at or below Rapier (ARPG: ~18 µs native, ~27 µs WASM, 2.68 MB WASM; MMORPG: WASM 1.76 → 3.29 MB, debug combat ~31 s → ~41 s) | MMORPG, ARPG | — | **parity**: MMORPG (`mmorpg-tick`: its whole `World::step` pattern at its 60 Hz tick with its query dispatcher, head-on pairs, doors, line of sight; 0.11× Rapier native, 0.09× WASM) and ARPG (`arpg-tick`: its `Physics` pattern with players, planar monsters, doors, line of sight, projectiles, placements); engine WASM 0.47 MB vs Rapier 1.46 MB. **Consumer-integrated** (games' real physics paths, test/bench-only `physics-ab` feature): MMORPG 0.18–0.40× at 60 Hz, ARPG 0.07–0.30× Rapier per tick ([below](#consumer-integrated-evidence-c15)) | #330 |
| C16 | Characters separate and slide around each other (bounded pushing) | MMORPG | none (head-on contact blocks both) | **parity** (`crowd`, `crowd-fight`, `push`): head-on sidestep, half-each separation, bounded pushing under per-character `max_push` ceilings | #308 |
| C17 | Static heightfield terrain: capsule characters standing and walking on hills, ground rays | MMORPG (#159 step 2; caves later) | `SharedShape::heightfield` (flat cells `CELL_REMOVED`), KCC on terrain, `cast_ray` | **parity** (`terrain`, `terrain-holes`): `add_heightfield`, parry diagonal, removed cells (holes, MMORPG's flat cells over a floor plane), rays, sphere/capsule queries, box/wedge overlaps (door probes), characters, rigid contacts with dynamic boxes (`debris`) and dynamic spheres (`boulder-slope`, #519); checkpointed by a fingerprint of the description the game supplies again on restore (#452, engine fixture only); caves later | #195, #342 |
| C18 | Dynamic boxes: upright movable crates (slide, stop at walls and units, rest, wake on the next push) and tumbling debris | MMORPG (mmorpg#58, #288) | `RigidBodyBuilder::dynamic()` (crates `lock_rotations`), cuboid colliders with density 600, friction 0.8, restitution 0.15, linear damping 0.2 (debris also angular 0.8), fixed boxes, capsules, halfspace and heightfield around them, one `step` per tick; `additional_solver_iterations` for the debris in a merged world | **parity** (`props`, `debris`): rotation-locked boxes with friction and linear damping on the floor plane, against fixed boxes and capsules (2 substeps × 1 pass, rests within 5% of each slide of Rapier's position); tumbling debris with restitution, linear and angular damping, debris–debris contacts and box/heightfield contacts, settled by the game's rule into fixed boxes (2 substeps × 4 passes, sleep off): sinks no deeper than Rapier, at least 90% of rest points within 0.5 m (or 25% of the throw) of Rapier's and all within 1.5 m; both lanes in one world (`mixed-iterations`, #521): the props at 2 substeps × 1 pass and the debris with `Body::additional_solver_iterations = 3` (Rapier's `additional_solver_iterations(3)`), each against the same bounds as alone; mixed materials (`materials`: crates 0.8 / 0.15 on geometry at Rapier's 0.5 / 0 defaults, crate on crate, `Multiply` and `Max` slabs; Rapier's `CoefficientCombineRule` per body, default `Average`): every slide within 5% of Rapier's (measured 1.2–2.0%); crates in a 1,200-box town on the floor plane (`debris-town`, kicked in turn, sleep on): every rest within 5 cm of Rapier's, step cost independent of the town (#412); free-rotating debris dropped onto the plane there rests at Rapier's height within the 5 mm slop and stops within 1 s of Rapier's (#424); 48 free-rotating pieces thrown from up to 1.2 m into the town's lanes (`debris-town-tumble`, 2 substeps × 1 pass, sleep on) land at speed, tumble and sleep within 1 s of Rapier's last, never sink into the plane, at least 90% rest within 0.5 m of Rapier's and all within 1.5 m (#426); restitution (`bounce`: spheres and rotation-locked boxes thrown at walls at 2–40 m/s and dropped 1–4 m onto the plane, restitution 0.15/0.5/0.8, frictionless): rebound speeds and bounce heights within 5% of Rapier's, never past the surface; a new contact bounces once the step is solved, speculative ones included (#458); rolling boulders (`boulder-slope`, mmorpg#299 trap: a 0.5 m ball at debris density and material rolls down heightfield terrain into the actual Greyhaven rock proxy through MMORPG's retained response lane, 60 Hz, 2 substeps × 4 passes, sleep off): never deeper than Rapier's deepest contact + 5 mm nor under the terrain, rolls downhill, every tick and the rest point within 0.5 m (or 25% of Rapier's travel) of Rapier's, strikes the rock and deflects to Rapier's side of it; CCD spheres sweep terrain and hulls (#519); the MMORPG retained lane (`debris-lane`, 1,200-box town and a far heightfield, about 3,500 static features, 110 pieces added, stepped at 60 Hz, 2 substeps × 4 passes and removed together each tick, settled pieces re-added fixed): `debris`'s invariants hold, ~0.5× Rapier rebuilt per tick; add/step/remove work independent of the statics, and the re-added fixed pieces never rebuild the town's wide query tree (`transient-lane` contract, `World::remove_bodies`, #523) | #366, #391, #412, #424, #426, #458, #519, #521, #523 |
| C19 | Contact enter/exit for physical traps and triggers: crates on pressure plates, a plate removed under a crate, spawned crates, a boulder hitting a player, a kinematic lift moved from under a crate; sensors and masked layers silent | MMORPG (mmorpg#300, Q33; planned) | `ActiveEvents::COLLISION_EVENTS`, `ChannelEventCollector`, `CollisionEvent::{Started, Stopped}` (`REMOVED` on `remove_collider`) | **parity** (`contact-events`): `Report::contact_events`, derived from the persistent contact cache, in pair order; every pair reports Rapier's sequence of transitions, each within 3 ticks of Rapier's (measured 0–1), ~0.7× Rapier native; sensor intersection events stay game-owned (D7); a kinematic plate teleported from under a sleeping crate (`set_kinematic_position`) wakes it, so it falls and its contact stops (#456). Character moves do not wake a sleeper they leave | #456 |
| C20 | Persistent bounced projectiles: two impacts in one 60 Hz tick, a moving obstacle and a glancing hit | MMORPG (mmorpg#290, Q38; planned) | Dynamic bodies with `.ccd_enabled(true)` and restitution; Rapier's default single CCD step stops at its first impact, so the two-hit reference advances the same 60 Hz interval in 64 smaller Rapier steps | **scenario parity, not consumer adoption** (`ccd-ricochet`, `ccd-glancing`, `ccd-moving-wall`, `ccd-synchronized`, `ccd-margin-start`; `ccd-platonic-proxy` for the [#465 experiment](#experiment-circumsphere-proxies-for-dodecahedron-and-icosahedron-visuals-465)): one `Body::ccd` projectile resolves its two impacts in increasing time of impact, consumes the full tick, keeps its tangential speed and stays within the walls on subsequent ticks; eight independent equal-time sphere–wall impacts also rebound without tunnelling; a CCD body starting a tick inside the contact margin of a surface it closes on still takes its swept time of impact and travels the rest of the tick (#465 finding) (ordered/indexed storage and high-resolution Rapier reference). First-tick positions within 5 cm and velocities within 5% + 0.5 m/s of high-resolution Rapier, plus independent analytic two-hit checks, multi-CCD chronological ordering, keyed tie order, and budget fallback tests. CCD scheduling caches unchanged future TOIs in a versioned chronological heap, prunes distant moving sweeps through a transient swept-bound tree (#469), and is bounded by a linear deterministic work contract for noninteracting simultaneous spheres. Single-step Rapier remains the separate blocking performance baseline. Rotation is held fixed during sweeps; exact rotational CCD is not claimed | #450, #466, #469 |
| C21 | Rope joints for dragged objects and suspended cargo: a moving kinematic anchor pulls a crate along the floor, a load hangs from a fixed anchor | MMORPG (owner request, #453; planned) | `RopeJoint` (`ImpulseJointSet::insert`, local anchors, a coupled linear limit with the maximum only; `RigidBodySet::remove` removes a body's joints) | **scenario parity, not consumer adoption** (`rope-joints`): `DistanceJoint` ropes as one frictionless row each in the existing velocity solve, warm-started, rebuilt every substep; kinematic-velocity drag of light, heavy and fast crates (one along a wall), light/heavy box, sphere, floor-scraping and post-striking pendulums, removal of a hanging body with its joint and a new body and joint in its slot, 1/60 s then 1/30 s steps. Attachment drift at most 0.08% of the rope (Rapier 0.03%; limit 0.5%), crates within 3 cm of Rapier's, swings within 1% of the rope and energy within 9% of `g·drop` of Rapier's through the first 60 ticks, hanging tension m·g within 1% (Rapier's equal), steady drag pull within 3% of the analytic friction-plus-damping pull (Rapier's reported limit impulse reads 7–18% lower without its stabilization share); ~0.8× Rapier native, ~0.65× WASM. Not yet: a minimum length (Rapier 0.36.1 ignores it, `FIXME: handle min limit too`), fixed, hinge or motor joints, position-driven anchors (characters moved by `move_characters` or `set_kinematic_position` carry no velocity, so a rope follows them only by its 3 m/s position correction; a rope that holds a character instead is the C27 tether), joint-joint chains and checkpoints of worlds with joints | #453 |
| C22 | Generic collision filtering: layers/masks applied identically by solver contacts, CCD, contact events, rays, shape casts and overlaps; runtime filter changes (trapdoor, hatch, stacked crate, thrown weapon released past its thrower); one body excluded per query | MMORPG (mmorpg#300 triggers, mmorpg#290 thrown weapons; Q35/Q36, planned) | `InteractionGroups` (`InteractionTestMode::And`) on colliders, `Collider::set_collision_groups`, `QueryFilter::groups`, `exclude_sensors`, `exclude_rigid_body` | **scenario parity, not consumer adoption** (`collision-filters`): `Body::layers`, `World::set_layers`, `QueryFilter::{layers, include_sensors, exclude}`; every pair reports Rapier's transitions within 3 ticks (a weapon's post-impact slide within 6), a cached contact a layer change excludes stops in the next step and wakes its sleeper, a fresh-throw mask keeps a CCD weapon off its thrower and a kept one passes the target, rays, sphere casts and box overlaps hit Rapier's bodies at Rapier's fractions through guard and door mask changes; mapping differences in the [API contract](world-api-contract.md#collision-layers). Group and body-pair exclusions stay parked until a consumer adapter needs them | #451 |
| C23 | Authored static triangle-mesh cave surfaces: capsule movement/grounding, rays, overlaps and fixed-orientation sweeps, immutable chunk admission/removal | MMORPG Redbrand Cave (PR #640) | `SharedShape::trimesh`, capsule KCC, rays and shape casts | **engine scenario, consumer adoption pending** (`redbrand-cave`): actual authored floors, ceiling, entrance seam, ramp and stacked shelf; `brook-mesh-edge` adds the authored towpath's grazing support departure at the consumer's centimeter scale; retained mesh-local BVH, input triangle IDs, independent Rapier query/grounding/route checks, deterministic storage and checkpoint continuation. Rigid spheres/translation CCD and non-CCD freely rotating or rotation-locked boxes supported; box CCD and other dynamic rigid shapes explicitly fail. No dynamic meshes or rotational CCD. Consumer adapter/backend change and browser gameplay acceptance remain in MMORPG | #182, #508 |
| C24 | Solid rotating kinematic bodies: blades, rotating platforms and swinging doors that strike and push crates and debris, wake sleepers and block characters | MMORPG (mmorpg#300: swinging blades ship damage-only first; owner request for solid support, #520; planned) | `RigidBodyBuilder::kinematic_position_based`, `set_next_kinematic_position(Pose)` per tick | **scenario parity, not consumer adoption** (`rotating-kinematic`): `World::set_next_kinematic_pose` moves an external body to a pose over the next step with the velocity and angular velocity of the change, ending exactly at the pose and at rest; turning kinematic pairs are swept with their rotation (conservative advancement) and their rows reused across substeps. Box blades at 2 rad/s on nine lanes (four mirrored): a shoved crate, a sleeping crate (wakes) and a sleeping flat debris panel are struck and pushed, each travelling, with peak speed and final distance from the pivot within 1.5× of Rapier's; no body centre ever inside a blade; a 40 m/s CCD ball fired head-on into a blade's face never passes it (it tracks Rapier's within centimetres); a character walking into a blade is pushed out of the overlap and stays within 5 cm of Rapier's controller (which leads by one tick of blade travel), and is passed through once it stands; both storage modes; ~0.85–0.9× Rapier native, ~0.5× WASM. A blade passing a body within one substep (60 rad/s) is an engine test only: Rapier 0.36.1's blade at 6–12 rad/s sweeps through a sleeping 6 cm panel. Characters are not carried by rotation (Rapier's controller adds a touched kinematic body's velocity) | #520 |
| C25 | Buoyancy and current drag: barrels, crates and logs that fall into water, float, drift with the current and rest on the bed | MMORPG (owner request, Greyhaven brook and Stillwater marsh; #525) | none: Rapier has no buoyancy; the oracle applies a parry-computed force through `add_force` | **scenario parity, not consumer adoption** (`float-brook`, `float-marsh`): `World::set_medium` gives a dynamic body a flat fluid surface, density, flow and drag; buoyancy at the kernel's submerged centroid and implicit drag toward the flow each substep. Crates, barrels, logs, buoys and sinking spheres dropped into 3 m of water, equilibrium depth within 2 cm of Archimedes and of Rapier's (measured 2 mm), positions within 0.25 m of Rapier's every tick (measured 0.17 m on the marsh sinker, 0.04 m on the others), floaters reach the brook's flow within 5%, every marsh body sleeps at equilibrium, none sinks into the bed; analytic heave period, terminal velocity and density-dependent cube orientation (`tests/medium.rs`); ~0.28× Rapier native, ~0.21× WASM (brook, against the consumer pattern) and ~0.40× native against bare Rapier | #525 |
| C26 | Persistent sleeping debris: gameplay-significant rubble stays as dynamic bodies, settled piles sleep, later blasts and throws wake them, and recovery from canonical snapshots stays bit-identical; small cosmetic fragments stay in MMORPG's presentation (owner decision 2026-11-09, D8) | MMORPG (owner decisions 2026-11-07 and 2026-11-09, #524, #543) | `RigidBody::sleep()` / `wake_up` on a retained world | **scenario parity, not consumer adoption** (`rubble-wake`): `World::set_sleeping`, `World::is_sleeping`; a world rebuilt from keys, poses, velocities and sleep flags with `warm_start` off continues bit-identically (physical state, both contact-generation modes; `tests/sleeping_debris.rs`, see the API contract); sleepers and their contact pairs add no work per step (`step` contract). The opt-in `ContactGeneration::PerStep` debris path generates contacts once per step as Rapier does (general solver unchanged). A pile of 40 boxes settles on the floor plane, the game sleeps it and a 14-piece second throw wakes it, at MMORPG's lane settings (2 substeps × 4 passes, 30 Hz): no piece sinks deeper than Rapier's deepest, at least 90% of rest points within 0.5 m (or a quarter of the throw) of Rapier's and all within 1.5 m, native and WASM; both storage modes; ~0.9–0.95× Rapier's retained world native, ~0.5–0.6× WASM | #524, #543 |
| C27 | Grapple hook: a kinematic character reeled toward a fixed point on static geometry, swinging on the rope, let go in mid-air or stopped by what lies between | Shooter (moenarch-games/shooter#5, Atlas D2.5; stand-in until this lands) | none: Rapier has no rope on a kinematic character; the shooter's stand-in sets the rope's velocity before each step, and the oracle applies the same tether rule to `move_shape`'s desired motion | **scenario parity, not consumer adoption** (`grapple`): `World::tether_character(id, Tether { anchor, attachment, length, max_pull })`, `set_tether_length` (the reel, shooter-side), `release_tether`, `tether`; inside `move_characters` the part of the desired motion that would carry the attachment past the rope is removed (motion across it kept, so it swings), a stretched rope draws the character back by at most `max_pull` per move, and the slide then resolves collisions as for any move; `CharacterMoveResult::tether` reports the change. The shooter's `GRAPPLE` (0.6 × 1.8 m capsule, 18 m/s², 60 Hz: dragged 9 m across the floor, up a 3 m ledge's face, over the edge, let go, walks into the 8 m pillar on the ledge), a swing under an overhang let go past the bottom of its arc, a reel let go in mid-air that lands short of the wall, and a pull stopped at a pillar in front of the anchor: positions within 1 cm of Rapier's every tick (measured 3.3 mm; one walking tick more where Rapier's controller stalls on the flat ledge top), no box entered, the attachment never more than 5 mm past its rope (measured 1.4 mm); both storage modes; ~0.15× Rapier native, ~0.1× WASM. Per move O(1) per tethered character; tethered worlds are not checkpointed (`UnsupportedTethers`) | #557 |
| C28 | Compound dynamic bodies: robots built from box, sphere and capsule parts with mass at their mounting positions, one rigid body each; an off-centre heavy part tips the build, a high mast rolls it over; each part keeps its own hitbox | Shooter (moenarch-games/shooter#73, Atlas D2.i70.5, D4.i70.1; owner decision 2026-11-14) | one `RigidBodyBuilder::dynamic()` with a `ColliderBuilder` per part and `.mass()`, `cast_ray` collider → part | **scenario parity, not consumer adoption** (`compound-robots`): `PreparedCompound::new` (mass, centre of mass and full parallel-axis inertia tensor of the children, diagonalized; the body frame is the principal frame at the centre of mass), `World::add_compound`, `compound`, `compound_child_pose`; contacts, per-substep sweeps, CCD projectiles and rays, shape casts and overlaps resolve per child, and hits report `child`. Twelve robots on the floor plane at 60 Hz (2 substeps × 4 passes): a 40 × 40 × 80 cm chassis with a 30 kg block over its centre, slightly off it and far off three ways, long decks rolled about their axis with the block low or on a mast, armoured robots pushed off their centre and probed by rays every 10 ticks. Centres of mass equal to Rapier's within 1e-5 m and principal moments within 1e-4 relative; every robot ends upright or fallen as Rapier's does, towards the same side (final tilts measured equal to 1e-3 rad); centres of mass within 0.1 m of Rapier's every tick (measured 3.8 cm), every ray hits Rapier's part at its fraction within 0.01; both storage modes; ~0.9× Rapier native (about 18 vs 20 µs per tick for the twelve), ~0.5× WASM. Rebuilt compounds continue bit-identically (`tests/compound_bodies.rs`). Not yet: removing a part at runtime (re-prepare the rest), media, position correction and checkpoints (`UnsupportedCompounds`) of compounds, and persistent CCD on a compound itself | #567 |

The C20 `ccd-parallel` oracle additionally checks fast, separated spheres whose swept world
bounds overlap but whose common velocity prevents contact. It compares analytic motion with
Rapier at 60 Hz; the dense scaling measurements are kept separately in #477.

Scenarios live in `oracle/src/scenarios/`; `cargo run --release -p physics-oracle`
checks them natively (invariants + not more than 5% slower than Rapier) and
`node scripts/oracle-wasm.mjs` gates WASM timings; `bash scripts/size-budget.sh` also
reports WASM sizes against `.performance/size.json` (with `SIZE_BASE_REF=<commit>`, also each
size increase over that commit, measured in the same run). The size budget is advisory
(PRINCIPLE-015): going over it warns but never fails; the oracle gates stay blocking.

The [Pages comparison lab](https://moritzbrantner.github.io/physics-engine/) indexes all C1–C22
by the oracle scenarios' `capabilities` metadata. It replays both real Rust/WASM implementations
and displays a bounded sample of their observations with synchronized ticks; each scenario's
invariant check still uses its complete traces. Cross-cutting features such as stable IDs (C3),
unit-relative defaults (C13) and stable query ordering/save continuation (C14) have related
workloads in the lab, not a standalone browser proof of every sub-property. Native tests and
the full oracle remain the correctness authority; browser timing is exploratory.

Not used by any consumer today: joints, sensor intersection events, collision
groups, CCD, triangle meshes. Contact transitions (C19), persistent CCD projectiles (C20),
rope joints (C21), collision filtering (C22) and solid rotating kinematic bodies (C24) have engine-side parity scenarios ahead of
mmorpg#300, mmorpg#290 and a narrow MMORPG dragging adapter, but not game-side adapters. Other work stays parked until a consumer uses it. Engine and sandbox support
(`ccd`, `sensor`, `retire_on_impact`, layers) is not consumer adoption. The engine/game boundary for melee sweeps, contact enter/exit, buoyancy,
filtering, multi-hit queries and projectiles, with the consumer issues that would gate each, is
[D7](technical-decisions.md#d7--physical-interactions-reuse-the-world-consumer-gated-q32q38).

### Character continuation from physical poses (#500)

Static penetration decisions no longer depend on a character's previous known-clear hint.
On the reported 400 cm terrain cell, a warm character skipped push-out while a rebuilt
character received an extra correction from the same pose. Static penetration now follows
physical pose/settings. Only a pose actually proved clear can reuse that result before
the next step, with placement and exclusion invalidation; the older clear hint alone
remains an optimization for character/character separation. `Embedded::PushOut` and `Embedded::Stay` retain their
existing distinct policies rather than changing MMORPG to Rapier's embedded-start policy.

The captured-hint regression runs in centimetres. `character-rebuild` in the shared oracle
walks the equivalent cell in metres, reconstructing a fresh world before every tick and
requiring each backend's positions, ground contact and depenetration to match exactly.
Native and WASM comparisons pass in both storage modes; local timings were about 0.55×
and 0.53× Rapier. The existing `terrain` and `embedded-starts` native gates also pass.
This is upstream evidence; the MMORPG recovery/E2E rerun remains deferred until integration.

The same-pose probe now accepts a real heightfield cell:
`cargo run --release --locked -p physics-oracle --bin probe -- --terrain-cell 8,0,-76,4,4,11,11,12.87,11.5 --capsule 0.6,0.3 --at 11.162186,12.470002,-73.88426 --desired 0,-0.002725,0.03 --stay`.
Rapier and Engine produce the same one-move translation with that matching embedded policy.

### Greyhaven authored rock hulls (#180; consumer-gated partial C4/C11/C12)

The first actual Greyhaven convex rock proxy is now exercised by `greyhaven-rock-hulls` in the
Rapier oracle. `World::add_prepared_convex_hull` and `add_prepared_triangle_mesh` admit one prepared
rock or cave chunk into every world that holds it, MMORPG's character world and response lanes
(`greyhaven-rigid-rocks-shared`, `redbrand-rigid-mesh-shared`, #528; the game's adoption is
separate). `World::add_convex_hull` admits fixed hulls and the engine queries their true faces
for rays, sphere/capsule overlaps and sweeps used by kinematic characters. A box no longer has
to replace the authored shape in this engine-side fixture. This is **partial** until the game
updates its rock adapter and the native/WASM cost and behavior gates demonstrate acceptable
parity. Actual game projectiles use complete ray segments, including first-hit reflection;
the fixture covers those segments without a frame-end overlap substitute. Unsupported
dynamic-body/hull pairs return an atomic step error rather than silently separating;
dynamic spheres (including CCD spheres) take rigid hull contacts (#519, `boulder-slope`). Generic
hull×hull response remains unsupported. MMORPG production now runs Physics Engine only
(mmorpg#648, exact hull/mesh geometry in retained isolated Engine response worlds), so
interactions it needs must be supported by the engine; there is no Rapier fallback. Run
`bash scripts/greyhaven-rock-hulls.sh` (optional `--perf` timing gate or `--stats` work
diagnostics) for the local reproducer (`--wasm` selects the WASM gate). The same-pose probe
compares six actual two-rock grounding/seam moves with Rapier. Deterministic work contracts
cover local support casts at 16–256 rocks, one-time preparation and retained geometry bytes.
Native/WASM oracle gates remain required; game integration and browser acceptance are not
established by the engine fixture.

### Rigid box contacts against authored rocks (in progress)

`greyhaven-rigid-rocks` exercises the actual Greyhaven proxy with a launched upright
0.7 m crate and a falling/tumbling 0.58 × 0.58 × 0.06 m crate panel, at 60 Hz with the
game's gravity, density, material and damping. Rapier's reference run passes the fixture:
the crate advances and stops at the rock, both bodies settle, and neither penetrates more
than the independent reference's maximum depth plus 5 mm (with a separate 5 cm cap).
Collision checks run outside the timed simulation. This fixture uses retained worlds;
it does not yet establish the game's rebuilt-world cost or consumer adoption.

Non-CCD boxes now use shared SAT/clipped manifolds and the engine's existing speculative
response rows. The behavior comparison passes on native and WASM in both storage modes:
`cargo run --locked -p physics-oracle -- --only greyhaven-rigid-rocks --no-timing`.
Additional checks cover 6–600 m/s non-CCD boxes without crossing the hull (both body-ID
orders), exact mid-contact checkpoint continuation, and unchanged atomic rejection of
unsupported CCD pairs. The `authored-rock-rigid-contact` work family keeps local SAT and
vertex projections independent of unrelated rocks, with no geometry re-preparation.

The engine now retains orientation/extent-keyed SAT plans and the two most recent exact
pose/margin contact results. Translation refreshes points while reusing projections; changes
to shape, orientation, body identity, position or margin invalidate the relevant retained data.
Unused/removed pairs are pruned. Tests compare retained geometry with fresh preparation
through those transitions, including signed zero, negative contacts and checkpoint recovery.

**Focused local native and WASM gates pass.** Deferring box/edge-axis preparation until
hull-face certificates are insufficient brings the standalone native release comparison to
5.38/5.33 µs per tick (Ordered/Indexed), versus Rapier 6.17 µs (0.87×), from the initial
13.71/13.67 µs. Focused WASM measured 7.21/7.31 µs versus Rapier 19.97 µs (0.36×).
Some local native runs were noisy and mixed; CI must confirm the committed revision.
These are local measurements for this scenario, not whole-zone acceptance. MMORPG production has since moved
to Physics Engine only (mmorpg#648); its acceptance is recorded there, not by this fixture.
The standalone Rapier reference check is
`cargo test --locked -p physics-oracle --lib rigid_rocks`.

## Experiment: circumsphere proxies for dodecahedron and icosahedron visuals (#465)

An owner-approved, bounded experiment, not consumer adoption and not exact convex parity.
**Conclusion: keep experimental.** The proxy is correct and needs no engine change, but its
whole-tick cost at hundreds of simultaneous CCD proxies is bounded by the CCD scheduler, not by
the proxy (see below); adopt it opt-in only after that scales.

**Consumer usage.** Render the regular dodecahedron or icosahedron mesh; collide a plain
`Shape::Sphere(circumradius)` body (`ccd = true` for fast throws). Circumradius for edge `a`:
icosahedron `R = a·sin(2π/5) = a/4·√(10 + 2√5)` (inradius `r = a·√3/12·(3 + √5)`), dodecahedron
`R = a·√3/4·(1 + √5)` (`r = a/2·√(5/2 + 11/10·√5)`); both have `(R − r)/R ≈ 0.2053`. The mesh's
rotation is the body's orientation; mass and inertia are the consumer's choice (the sphere's
by default). There is no Platonic shape, collider enum or second CCD path, and the mesh stays in
the consumer. Never use the insphere or an equal-volume sphere: only the circumsphere is a
conservative outer bound. Tetrahedra, cubes and octahedra stay on accurate paths (`Shape::Box`
for cubes; the others are not proxied).

**Evidence.**

- `tests/platonic_proxies.rs` builds both solids for edges 0.01–100 m, checks their radii against
  the closed forms and compares the engine's swept sphere (`cast_shape` and one CCD
  `World::step` on the identical trajectory) with the solid's exact first contact from the
  complete separating-axis set (dense sampling plus bisection for a spinning visual). The proxy
  never misses a solid contact and never touches later. Early-contact bias along the path,
  in units of R: head-on face-first 0.2053 (`R − r`), edge-first 0.149 (icosahedron) / 0.066
  (dodecahedron), vertex-first 0; 64 random orientations, spinning visuals up to 3,000 rad/s,
  a wall 0.004 R thick crossed at 200 R per step and the `HalfSpace` floor all stay within
  `R − r`. Grazing past a box edge the proxy can touch up to 0.96 R of travel earlier, and
  11–31% of the grazing samples are phantom contacts the solid passes clear of; where both
  touch, the proxy's radial normal differs from the solid's contact normal by 11–15° on average
  (up to 36°). Head-on, both normals are the wall's. Not measured: rolling (a proxy rolls where a
  solid settles on a face) and stacking (a proxy stack rises by `2R` per body where face-to-face solids rise by `2r`).
- The oracle scenario `ccd-platonic-proxy` (C20) runs four proxies (45/45/60/30 m/s, two
  oblique) into a thin wall: engine spheres follow the analytic reflection on every tick and
  match Rapier balls (same masses, trajectories, 60 Hz; 64-step reference) on the impact tick;
  about 1.6 µs/tick vs 4.9 µs bare Rapier natively and 2.6 vs 6.6 µs in WASM. Its check also replays the trajectories on
  Rapier with a rotation-locked `convex_hull` of the real solid — an approximation report, not
  parity: the hull follows the analytic reflection at its own support distance `h` within 1.6 cm,
  and its distance from the proxy is the expected `2 (R − h)` (1.4–6.5 cm here), bounded by
  `2 (R − r)`. `cargo test --release --locked -p physics-oracle --test platonic_proxy -- --nocapture`
  prints the per-lane errors.
- Whole tick, 60 ticks, all CCD, alternating proxies of R ≈ 0.19 m in six walls, sparse (2 m
  lattice, 30 m/s) and dense (0.42 m lattice, 10 m/s); µs/tick, median of 5, shared 4-core
  Xeon @ 2.10 GHz, Rust 1.99.1, release; advisory, not a CI gate
  (`cargo test --release --locked -p physics-oracle --test platonic_proxy -- --ignored --nocapture`):

  | Layout | Proxies | Engine sphere CCD | Engine, no CCD | Rapier ball | Rapier convex hull | Engine pair tests / sweeps / impacts per tick |
  | --- | ---: | ---: | ---: | ---: | ---: | --- |
  | sparse | 32 | 30 | 36 | 85 | 122 | 196 / 15 / 2 |
  | sparse | 256 | 685 | 237 | 372 | 946 | 8,587 / 142 / 12 |
  | sparse | 1,024 | 7,047 | 1,440 | 2,849 | 5,201 | 105,352 / 493 / 38 |
  | dense | 32 | 128 | 52 | 69 | 180 | 893 / 132 / 6 (10 budget fallbacks in 60 ticks) |
  | dense | 256 | 4,370 | 646 | 486 | 1,502 | 51,634 / 1,553 / 51 (83) |
  | dense | 1,024 | 104,976 | 3,509 | 2,944 | 7,564 | 880,615 / 7,524 / 229 (544) |

  Retained contact bytes grow linearly (4.9 / 33.5 / 131.8 KB). Without CCD the engine stays
  within about 1.3× of Rapier balls; with CCD, pair tests grow with the square of the active
  CCD bodies, the scheduler cost [world API contract](world-api-contract.md#persistent-projectile-ccd)
  already records. A proxy is cheaper than Rapier's own convex hull up to a few dozen bodies.

**Pages lab.** `ccd-platonic-proxy` appears under C20 as engine spheres beside Rapier spheres.
The hull comparison is not shown: the adapter replays only the engine and Rapier sides and draws
no meshes. The smallest follow-up is a read-only third "Rapier convex hull" display side in
`oracle/src/wasm.rs` plus a mesh draw in `site/webgl-renderer.js`, with collision truth still
coming from Rust.

## Consumer-integrated evidence (C15)

The oracle scenarios above are engine-owned copies of the games' call patterns. #330 adds the
games' **real** physics integrations on both backends: an A/B diagnostic feature in each game
compares Rapier and a thin physics-engine adapter behind its unchanged physics API, with shared
invariants checked in CI. The measurements below predate the owner's 2026-11-05 decision to
use Physics Engine first in MMORPG where content is supported. Backend rollout belongs to the
games: MMORPG production is Physics Engine only since mmorpg#648 (Rapier only under
`physics-ab`); ARPG's production baseline remains Rapier.

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

The `terrain-seam` oracle covers MMORPG's flat-to-rising Hollow approach: the character
climbs the next triangle instead of accepting a tangential hit as the terrain's only
contact. The native `slope_restore` regression also requires no endpoint penetration
and exact continuation after reconstructing the character from its position.

MMORPG door occupancy, prop respawn and structure regrowth use unfiltered box overlaps.
These now query fixed authored hulls through the same shared SAT kernel as rigid boxes;
fixed triangle meshes already support box overlaps through their polytope kernel. The
`greyhaven-rock-hulls` Rapier scenario includes thin panel occupancy probes. Box casts
remain unsupported for authored hulls/meshes; the game casts capsules. This read-only
query change does not change checkpoint layout or solver continuation semantics.

The `redbrand-rigid-mesh` oracle exercises the actual cave wall/floor with the game's
locked crate and free-rotating thin debris panel. Independent Parry surface-depth checks,
settling and wall-stop invariants run natively and in WASM. Rigid box/mesh response retains
the mesh BVH and has its own logarithmic-node/constant-nearby-triangle work contract.

The intended authored prop/debris adapter pattern uses isolated retained static worlds and
transient rigid body admission/removal, with the game's one upright/four free solver
iterations and no retained character state. The oracle measures that pattern on both
engines; the perf gate is Rapier with that pattern, and persistent bare Rapier, which keeps
the body and its contacts across ticks, is printed as information (#553, as `debris-lane`
#523). Final adoption remains consumer-owned. Physical
continuation across transient cleanup is covered, including the pending removal `Stopped`
events a checkpoint captured after cleanup carries into the next step (format 10, #505).
