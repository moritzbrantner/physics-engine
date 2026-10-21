//! C17 (MMORPG hills): a 128 m heightfield, 48 characters walking over it with game-side
//! gravity (16 more stand idle on slopes) and 32 downward ground rays per tick. Rapier mirrors
//! MMORPG: a world rebuild per tick, `SharedShape::heightfield`, `move_shape` per character,
//! `detect_collisions` per move, a static-only support cast and `cast_ray` for ground height.
//! Eight door-probe boxes per tick test overlap with the terrain (Rapier: `contact`, since
//! parry's `intersection_test` does not support cuboid–heightfield).
//!
//! `terrain-holes` is MMORPG's terrain pattern: the same hills clamped at 0, every flat cell
//! removed (`removed_cells` / `CELL_REMOVED`) and a floor plane at 0 under the holes.

use crate::{Obs, Rng, Scenario, Trace};

const TICKS: u32 = 240;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const FOOT: f32 = HALF + RADIUS;
pub(super) const N: usize = 65;
pub(super) const CELL: f32 = 2.0;
pub(super) const ORIGIN: f32 = -64.0;
const WALKERS: usize = 48;
const IDLE: usize = 16;
const RAYS: usize = 32;
const PROBES: usize = 8;
/// A door leaf's half extents.
const PROBE: [f32; 3] = [0.6, 1.0, 0.1];
/// Collision layer of the floor plane under the holes.
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
const FLOOR_LAYER: u32 = 4;

fn height_at_vertex(i: usize, j: usize, holes: bool) -> f32 {
    let (x, z) = (ORIGIN + CELL * i as f32, ORIGIN + CELL * j as f32);
    let h = 2.0 * (x * 0.11).sin() * (z * 0.09).cos() + 0.6 * (x * 0.37 + z * 0.23).sin();
    if holes { h.max(0.0) } else { h }
}

/// Heights row-major: Z rows, X columns.
pub(super) fn heights(holes: bool) -> Vec<f32> {
    (0..N * N)
        .map(|k| height_at_vertex(k % N, k / N, holes))
        .collect()
}

/// Whether cell (i, j) is removed: with holes, every cell flat at 0 (the floor plane covers
/// it).
pub(super) fn removed(i: usize, j: usize, holes: bool) -> bool {
    holes
        && [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)]
            .into_iter()
            .all(|(a, b)| height_at_vertex(a, b, true) == 0.0)
}

/// Removed cells row-major, `(N - 1)²`.
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
pub(super) fn removed_cells() -> Vec<bool> {
    (0..(N - 1) * (N - 1))
        .map(|k| removed(k % (N - 1), k / (N - 1), true))
        .collect()
}

fn cell_of(x: f32, z: f32) -> (usize, usize) {
    let (fx, fz) = ((x - ORIGIN) / CELL, (z - ORIGIN) / CELL);
    (
        (fx.floor().max(0.0) as usize).min(N - 2),
        (fz.floor().max(0.0) as usize).min(N - 2),
    )
}

/// Surface height with the fixed B–C diagonal (parry's split). Removed cells are flat at 0,
/// where the floor plane lies.
pub(super) fn ground(x: f32, z: f32, holes: bool) -> f32 {
    let height_at_vertex = |i, j| height_at_vertex(i, j, holes);
    let (fx, fz) = ((x - ORIGIN) / CELL, (z - ORIGIN) / CELL);
    let (i, j) = (
        (fx.floor() as usize).min(N - 2),
        (fz.floor() as usize).min(N - 2),
    );
    let (u, v) = (fx - i as f32, fz - j as f32);
    let (a, b, c, d) = (
        height_at_vertex(i, j),
        height_at_vertex(i + 1, j),
        height_at_vertex(i, j + 1),
        height_at_vertex(i + 1, j + 1),
    );
    if u + v <= 1.0 {
        a + u * (b - a) + v * (c - a)
    } else {
        d + (1.0 - u) * (c - d) + (1.0 - v) * (b - d)
    }
}

fn starts(holes: bool) -> Vec<[f32; 3]> {
    let mut rng = Rng(0x7e44_a1e5);
    (0..WALKERS + IDLE)
        .map(|_| {
            let (x, z) = (rng.range(-50.0, 50.0), rng.range(-50.0, 50.0));
            [x, ground(x, z, holes) + FOOT + 0.5, z]
        })
        .collect()
}

#[derive(Clone, Copy)]
struct Intent {
    vy: f32,
}

fn desired(index: usize, tick: u32, grounded: bool, intent: &mut Intent) -> [f32; 3] {
    if grounded {
        intent.vy = 0.0;
    }
    intent.vy -= GRAVITY * DT;
    let (vx, vz) = if index < WALKERS && tick > 30 {
        let mut rng = Rng(0x99 + index as u64 * 31 + u64::from(tick / 60));
        let a = rng.range(0.0, core::f32::consts::TAU);
        (2.0 * a.cos(), 2.0 * a.sin())
    } else {
        (0.0, 0.0)
    };
    [vx * DT, intent.vy * DT, vz * DT]
}

fn ray(tick: u32, k: usize) -> [f32; 2] {
    let mut rng = Rng(0x5a17 + u64::from(tick) * 101 + k as u64);
    [rng.range(-60.0, 60.0), rng.range(-60.0, 60.0)]
}

/// Door probe `k` at `tick`: centre and yaw. Some straddle the surface, some float, some are
/// buried (a surface, not a solid: buried boxes touch nothing).
fn probe(tick: u32, k: usize, holes: bool) -> ([f32; 3], f32) {
    let mut rng = Rng(0xd00e + u64::from(tick) * 37 + k as u64);
    let (x, z) = (rng.range(-55.0, 55.0), rng.range(-55.0, 55.0));
    let lift = rng.range(-3.0, 3.0);
    (
        [x, ground(x, z, holes) + lift, z],
        rng.range(0.0, core::f32::consts::PI),
    )
}

/// Whether probe `k` clearly crosses the surface (`Some(true)`), clearly misses it
/// (`Some(false)`), or is too close to call (also when it reaches over a hole).
fn probe_reference(tick: u32, k: usize, holes: bool) -> Option<bool> {
    let (c, yaw) = probe(tick, k, holes);
    let (s, co) = yaw.sin_cos();
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for i in 0..=8 {
        for j in 0..=8 {
            let (a, b) = (
                PROBE[0] * (i as f32 / 4.0 - 1.0),
                PROBE[2] * (j as f32 / 4.0 - 1.0),
            );
            let (x, z) = (c[0] + a * co + b * s, c[2] - a * s + b * co);
            let (i, j) = cell_of(x, z);
            if removed(i, j, holes) {
                return None;
            }
            let g = ground(x, z, holes);
            lo = lo.min(g);
            hi = hi.max(g);
        }
    }
    let (bottom, top) = (c[1] - PROBE[1], c[1] + PROBE[1]);
    if bottom < lo - 0.05 && top > hi + 0.05 {
        Some(true)
    } else if bottom > hi + 0.05 || top < lo - 0.05 {
        Some(false)
    } else {
        None
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "terrain",
        capabilities: "C17",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new(false)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true, false)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false, false))),
        check: |e, r| check(e, r, false),
    }
}

pub fn scenario_holes() -> Scenario {
    Scenario {
        name: "terrain-holes",
        capabilities: "C17",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true, true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false, true))),
        check: |e, r| check(e, r, true),
    }
}

fn check(engine: &Trace, rapier: &Trace, holes: bool) -> Result<(), String> {
    let n = WALKERS + IDLE;
    let per_tick = 2 * n + RAYS + PROBES;
    if engine.0.len() != rapier.0.len() || engine.0.len() != per_tick * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let mut rest = vec![None; n];
    for tick in 0..TICKS as usize {
        let base = tick * per_tick;
        for (i, slot) in rest.iter_mut().enumerate() {
            let (Obs::Position(p), Obs::Grounded(g)) =
                (engine.0[base + 2 * i], engine.0[base + 2 * i + 1])
            else {
                return Err("unexpected observation".into());
            };
            let foot = p[1] - FOOT;
            if foot < ground(p[0], p[2], holes) - 2e-3 {
                return Err(format!(
                    "character {i} sank into the terrain at tick {tick}"
                ));
            }
            if i >= WALKERS && tick >= 120 {
                if !g {
                    return Err(format!("idle character {i} not grounded at tick {tick}"));
                }
                match *slot {
                    None => *slot = Some(p),
                    Some(q) if q != p => {
                        return Err(format!("idle character {i} slid at tick {tick}"));
                    }
                    _ => {}
                }
            }
        }
        for k in 0..RAYS {
            let [x, z] = ray(tick as u32, k);
            match engine.0[base + 2 * n + k] {
                Obs::Ray(Some((t, normal))) => {
                    let y = 50.0 - 100.0 * t;
                    if (y - ground(x, z, holes)).abs() > 2e-3 || normal[1] <= 0.0 {
                        return Err(format!(
                            "ground ray {k} at tick {tick}: {y} vs {}",
                            ground(x, z, holes)
                        ));
                    }
                }
                ref other => return Err(format!("ground ray {k} at tick {tick}: {other:?}")),
            }
        }
        for k in 0..PROBES {
            let at = base + 2 * n + RAYS + k;
            let (Obs::Penetrates(e), Obs::Penetrates(r)) = (engine.0[at], rapier.0[at]) else {
                return Err("unexpected probe observation".into());
            };
            if let Some(want) = probe_reference(tick as u32, k, holes)
                && (e != want || r != want)
            {
                return Err(format!(
                    "door probe {k} at tick {tick}: engine {e}, rapier {r}, expected {want}"
                ));
            }
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId, CollisionLayers3d,
        approximate::{
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, HeightfieldDesc,
            Quaternion, QueryFilter, QueryHit, QueryPose, RayCast, Shape, Vector, World,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        ids: Vec<BodyId>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
        hits: Vec<QueryHit>,
        holes: bool,
    }
    impl Sim {
        pub fn new(holes: bool) -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            let terrain = world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(ORIGIN, 0.0, ORIGIN),
                        cell_size: (CELL, CELL),
                        columns: N,
                        rows: N,
                        heights: heights(holes),
                        removed_cells: holes.then(removed_cells),
                    },
                    Some(1),
                )
                .unwrap();
            let _ = terrain;
            if holes {
                // The floor under the holes; its own layer keeps it out of the door probes.
                let mut floor = Body::keyed(2, Shape::HalfSpace, Vector::ZERO, 0.0);
                floor.layers = CollisionLayers3d::new(FLOOR_LAYER, u32::MAX);
                world.add_body(floor).unwrap();
            }
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let ids: Vec<_> = starts(holes)
                .into_iter()
                .enumerate()
                .map(|(i, p)| {
                    let mut body =
                        Body::keyed(100 + i as u64, Shape::capsule(HALF, RADIUS), v(p), 1.0);
                    body.external = true;
                    body.layers = CollisionLayers3d::new(2, u32::MAX);
                    let id = world.add_body(body).unwrap();
                    world.add_character(id, settings).unwrap();
                    id
                })
                .collect();
            let n = ids.len();
            Self {
                world,
                ids,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
                moves: Vec::new(),
                results: Vec::new(),
                hits: Vec::new(),
                holes,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (i, &body) in self.ids.iter().enumerate() {
                let d = desired(i, tick, self.grounded[i], &mut self.intents[i]);
                self.moves.push(CharacterMove {
                    body,
                    desired: v(d),
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (i, r) in self.results.iter().enumerate() {
                self.grounded[i] = r.grounded;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(r.grounded));
            }
            let terrain_only = QueryFilter {
                layers: CollisionLayers3d::new(1, 1),
                ..QueryFilter::default()
            };
            let ground_only = QueryFilter {
                layers: CollisionLayers3d::new(1 | FLOOR_LAYER, 1 | FLOOR_LAYER),
                ..QueryFilter::default()
            };
            for k in 0..RAYS {
                let [x, z] = ray(tick, k);
                let hit = self
                    .world
                    .cast_ray_first(
                        RayCast::new(Vector(x, 50.0, z), Vector(0.0, -100.0, 0.0)),
                        ground_only,
                    )
                    .unwrap();
                trace.0.push(Obs::Ray(
                    hit.map(|h| (h.fraction, [h.normal.0, h.normal.1, h.normal.2])),
                ));
            }
            for k in 0..PROBES {
                let (c, yaw) = probe(tick, k, self.holes);
                let mut pose = QueryPose::new(Shape::Box(v(PROBE)), v(c));
                pose.orientation = Quaternion(0.0, (yaw * 0.5).sin(), 0.0, (yaw * 0.5).cos());
                self.world
                    .overlap_shape(pose, terrain_only, &mut self.hits)
                    .unwrap();
                trace.0.push(Obs::Penetrates(!self.hits.is_empty()));
            }
        }
    }
}

#[cfg(feature = "rapier")]
pub(super) mod rapier {
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher, ShapeCastOptions};
    use rapier3d::parry::shape::{HeightField, HeightFieldCellStatus};
    use rapier3d::parry::utils::Array2;
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        terrain: SharedShape,
        controller: KinematicCharacterController,
        bodies: Vec<RigidBodyHandle>,
        positions: Vec<Vector>,
        intents: Vec<Intent>,
        grounded: Vec<bool>,
        shape: Capsule,
        rebuild: bool,
        holes: bool,
    }

    /// The hills as a parry heightfield, centred where [`terrain_position`] puts it.
    pub(in crate::scenarios) fn field(holes: bool) -> SharedShape {
        // parry: rows along Z, columns along X, column-major storage.
        let h = heights(holes);
        let data = (0..N * N).map(|k| h[(k % N) * N + k / N]).collect();
        let size = CELL * (N - 1) as f32;
        let mut field = HeightField::new(Array2::new(N, N, data), Vector::new(size, 1.0, size));
        for j in 0..N - 1 {
            for i in 0..N - 1 {
                if removed(i, j, holes) {
                    field.set_cell_status(j, i, HeightFieldCellStatus::CELL_REMOVED);
                }
            }
        }
        SharedShape::new(field)
    }

    pub(in crate::scenarios) fn terrain_position() -> Vector {
        let centre = ORIGIN + CELL * (N - 1) as f32 * 0.5;
        Vector::new(centre, 0.0, centre)
    }

    fn build(
        terrain: &SharedShape,
        positions: &[Vector],
        shape: Capsule,
        holes: bool,
    ) -> (PhysicsWorld, Vec<RigidBodyHandle>) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        world.integration_parameters.dt = DT;
        if holes {
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
        }
        world.colliders.insert(
            ColliderBuilder::new(terrain.clone())
                .translation(terrain_position())
                .build(),
        );
        let bodies = positions
            .iter()
            .map(|&p| {
                let body = world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(p)
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::new(SharedShape::new(shape)).build(),
                    body,
                    &mut world.bodies,
                );
                body
            })
            .collect();
        world.detect_collisions(&(), &());
        (world, bodies)
    }

    impl Sim {
        pub fn new(rebuild: bool, holes: bool) -> Self {
            let terrain = field(holes);
            let shape = Capsule::new_y(HALF, RADIUS);
            let positions: Vec<Vector> = starts(holes).into_iter().map(v).collect();
            let (world, bodies) = build(&terrain, &positions, shape, holes);
            let n = positions.len();
            Self {
                world,
                terrain,
                controller: KinematicCharacterController {
                    up: Vector::Y,
                    offset: CharacterLength::Absolute(OFFSET),
                    slide: true,
                    autostep: None,
                    max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                    min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                    snap_to_ground: Some(CharacterLength::Absolute(OFFSET + SNAP)),
                    normal_nudge_factor: KinematicCharacterController::default()
                        .normal_nudge_factor,
                },
                bodies,
                positions,
                intents: vec![Intent { vy: 0.0 }; n],
                grounded: vec![false; n],
                shape,
                rebuild,
                holes,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                (self.world, self.bodies) =
                    build(&self.terrain, &self.positions, self.shape, self.holes);
            }
            let dispatcher = DefaultQueryDispatcher;
            for i in 0..self.bodies.len() {
                let d = desired(i, tick, self.grounded[i], &mut self.intents[i]);
                let body = self.bodies[i];
                let from = self.positions[i];
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    &dispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().exclude_rigid_body(body),
                );
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(from),
                    v(d),
                    |_| {},
                );
                let to = from + movement.translation;
                let fixed = w.broad_phase.as_query_pipeline(
                    &dispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::only_fixed(),
                );
                let support = movement.grounded
                    && fixed
                        .cast_shape(
                            &Pose::from_translation(to),
                            -Vector::Y,
                            &self.shape,
                            ShapeCastOptions {
                                target_distance: OFFSET,
                                stop_at_penetration: false,
                                max_time_of_impact: OFFSET + SNAP,
                                compute_impact_geometry_on_penetration: true,
                            },
                        )
                        .is_some();
                self.grounded[i] = support;
                self.positions[i] = to;
                self.world.bodies[body].set_translation(to, false);
                self.world.detect_collisions(&(), &());
                trace.0.push(Obs::Position([to.x, to.y, to.z]));
                trace.0.push(Obs::Grounded(support));
            }
            let w = &self.world;
            let fixed = w.broad_phase.as_query_pipeline(
                &dispatcher,
                &w.bodies,
                &w.colliders,
                QueryFilter::only_fixed(),
            );
            for k in 0..RAYS {
                let [x, z] = ray(tick, k);
                let hit = fixed.cast_ray_and_get_normal(
                    &Ray::new(Vector::new(x, 50.0, z), Vector::new(0.0, -100.0, 0.0)),
                    1.0,
                    true,
                );
                trace.0.push(Obs::Ray(hit.map(|(_, h)| {
                    (h.time_of_impact, [h.normal.x, h.normal.y, h.normal.z])
                })));
            }
            // parry's `intersection_test` does not support cuboid–heightfield (Rapier's
            // `intersect_shape` silently skips the terrain), so the probe asks `contact`.
            let leaf = Cuboid::new(v(PROBE));
            let centre = ORIGIN + CELL * (N - 1) as f32 * 0.5;
            let terrain_pose = Pose::from_translation(Vector::new(centre, 0.0, centre));
            for k in 0..PROBES {
                let (c, yaw) = probe(tick, k, self.holes);
                let pose = Pose::from_parts(v(c), Rotation::from_axis_angle(Vector::Y, yaw));
                let hit = dispatcher
                    .contact(
                        &pose.inv_mul(&terrain_pose),
                        &leaf,
                        self.terrain.as_ref(),
                        0.0,
                    )
                    .ok()
                    .flatten()
                    .is_some_and(|contact| contact.dist <= 0.0);
                trace.0.push(Obs::Penetrates(hit));
            }
        }
    }
}
