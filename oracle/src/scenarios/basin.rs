//! C4/C5 (physics-engine#405): swimmers over a lake basin, as MMORPG's Stillwater Lake (#274)
//! builds it: a halfspace floor at y = 0 and a heightfield whose basin dips below it. Over the
//! basin MMORPG opens the floor: a character's move and its support cast exclude the floor
//! while its footprint is over a cell with a sample below zero, so only the heightfield is
//! ground there.
//!
//! Eight MMORPG-sized capsules, one per lane at its own speed (half the lanes angled across
//! the heightfield's diagonals), walk from the rim down into the basin, dive weightless with a
//! zone-integrated vertical velocity, rise, swim level, sink back to walking and walk out up the
//! far slope, over eleven seconds at 60 Hz with MMORPG's per-tick gravity rule.
//!
//! Each side is its consumer pattern. Rapier: MMORPG's `World::step` (a static ground cast
//! decides the gravity rule, `move_shape`, a support cast, `set_translation` and
//! `detect_collisions`), each query through a filter excluding the open floor. Engine: one
//! `move_characters` batch with `CharacterMove::exclude` naming the floor, the gravity rule
//! driven by the batch's grounded results.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 11 * 60;
const DT: f32 = 1.0 / 60.0;
/// Gravity, metres per tick².
const G: f32 = -9.81 * DT * DT;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
/// The scripted tolerance on positions (and support gaps). Walking up the far slope, the
/// engine gains about 1 cm on Rapier (`slopes` allows the same).
const TOLERANCE: f32 = 0.02;
/// A ground gap below this counts as standing on it (MMORPG's 1e-3 units).
const GAP_TOLERANCE: f32 = 1.0e-5;
const SUPPORT_REACH: f32 = OFFSET + SNAP;
/// Ground this near the end of a support cast's reach may be found on one engine only.
const REACH_SLACK: f32 = 1e-3;
/// MMORPG's unit capsule.
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
const LANES: usize = 8;
const LANE: f32 = 4.0;
/// The basin: depth `DEPTH` at x = 0, rising to the rim at |x| = `RIM` (32° there).
const DEPTH: f32 = 2.5;
const RIM: f32 = 8.0;
/// Heightfield grid: 1 m cells, columns along X from `FIELD_X`, rows along Z from `FIELD_Z`.
const CELL: f32 = 1.0;
const FIELD_X: f32 = -16.0;
const FIELD_Z: f32 = -4.0;
const COLUMNS: usize = 33;
const ROWS: usize = (LANES - 1) * 4 + 12;
/// Where every lane starts, on the rim's flat.
const START_X: f32 = -12.0;
/// Cells around the character that count for the open floor: MMORPG's `reach`, its radius
/// rounded up plus one cell.
const REACH: i32 = 2;
/// Body keys: floor 1, terrain 2, characters from 100.
const CHARACTER_BASE: u64 = 100;

/// Walking speed (m/s) and Z drift per metre along X of lane `k`.
fn lane(k: usize) -> (f32, f32) {
    (2.0 + 0.125 * k as f32, if k % 2 == 1 { 0.15 } else { 0.0 })
}

fn height(x: f32) -> f32 {
    if x.abs() < RIM {
        -DEPTH * (1.0 - (x / RIM).powi(2))
    } else {
        0.0
    }
}

/// Heights row-major: Z rows, X columns.
fn heights() -> Vec<f32> {
    (0..ROWS * COLUMNS)
        .map(|k| height(FIELD_X + CELL * (k % COLUMNS) as f32))
        .collect()
}

/// MMORPG's `open_floor`: whether any sample under the footprint at `(x, z)` lies below zero.
fn open_floor(x: f32, z: f32) -> bool {
    let (cx, cz) = (
        ((x - FIELD_X) / CELL).round() as i32,
        ((z - FIELD_Z) / CELL).round() as i32,
    );
    (cz - REACH..=cz + REACH).any(|r| {
        (cx - REACH..=cx + REACH).any(|c| {
            (0..ROWS as i32).contains(&r)
                && (0..COLUMNS as i32).contains(&c)
                && height(FIELD_X + CELL * c as f32) < 0.0
        })
    })
}

/// The swimmer's scripted vertical velocity (m/s) at `tick`, or `None` while it walks.
fn swim(k: usize, tick: u32) -> Option<f32> {
    let (speed, _) = lane(k);
    // In the water from 6.5 m before the basin's centre to 6 m after it: rise off the slope,
    // swim level, dive to the bottom and along it, rise again, then swim level into the far
    // slope and up it.
    let t = tick as f32 * DT;
    let enter = (-6.5 - START_X) / speed;
    let leave = (6.0 - START_X) / speed;
    if !(enter..leave).contains(&t) {
        return None;
    }
    let phase = (t - enter) / (leave - enter);
    Some(match phase {
        p if p < 0.15 => 0.8,
        p if p < 0.3 => 0.0,
        p if p < 0.6 => -1.8,
        p if p < 0.75 => 0.6,
        _ => 0.0,
    })
}

fn start(k: usize) -> [f32; 3] {
    [START_X, HALF + RADIUS + OFFSET, LANE * k as f32]
}

/// Desired horizontal motion (metres per tick).
fn walk(k: usize) -> [f32; 2] {
    let (speed, drift) = lane(k);
    let norm = (1.0 + drift * drift).sqrt();
    [speed * DT / norm, speed * drift * DT / norm]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "basin",
        capabilities: "C4 C5",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

/// Every tick, every lane: position within the tolerance of Rapier's, the same grounded state
/// and the same support cast result. Every swimmer dives below the floor and walks out.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != 3 * LANES * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let mut lowest = [f32::INFINITY; LANES];
    let mut last = [[0.0; 3]; LANES];
    let per_tick = 3 * LANES;
    for n in 0..LANES * TICKS as usize {
        let (tick, k) = (n / LANES, n % LANES);
        let at = |t: &Trace| {
            let base = tick * per_tick;
            (
                t.0[base + 2 * k],
                t.0[base + 2 * k + 1],
                t.0[base + 2 * LANES + k],
            )
        };
        let (
            (Obs::Position(ep), Obs::Grounded(eg), Obs::Support(es)),
            (Obs::Position(rp), Obs::Grounded(rg), Obs::Support(rs)),
        ) = (at(engine), at(rapier))
        else {
            return Err("unexpected observation".into());
        };
        if (0..3).any(|i| (ep[i] - rp[i]).abs() > TOLERANCE) {
            return Err(format!(
                "lane {k} at tick {tick}: engine {ep:?}, rapier {rp:?}"
            ));
        }
        if eg != rg {
            return Err(format!(
                "lane {k} at tick {tick}: grounded engine {eg}, rapier {rg} at {ep:?}"
            ));
        }
        // Support casts agree: both find ground at gaps within the tolerance, or neither does,
        // except ground one side finds at the very end of the reach (the positions differ by
        // up to the tolerance).
        let agree = match (es, rs) {
            (Some(e), Some(r)) => (e - r).abs() <= TOLERANCE,
            (None, None) => true,
            (Some(gap), None) | (None, Some(gap)) => gap >= SUPPORT_REACH - REACH_SLACK,
        };
        if !agree {
            return Err(format!(
                "lane {k} at tick {tick}: support engine {es:?}, rapier {rs:?} at {ep:?}"
            ));
        }
        lowest[k] = lowest[k].min(ep[1] - HALF - RADIUS);
        last[k] = ep;
    }
    for k in 0..LANES {
        if lowest[k] > -1.0 {
            return Err(format!(
                "lane {k} never dived below the floor: {}",
                lowest[k]
            ));
        }
        if last[k][0] < RIM + 1.0 || (last[k][1] - (HALF + RADIUS + OFFSET)).abs() > 1e-2 {
            return Err(format!("lane {k} did not walk out: {:?}", last[k]));
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
            QueryFilter, QueryHit, QueryPose, Shape, ShapeCast, Vector, World,
        },
    };

    /// Characters on layer 2; the support cast only sees static geometry (layer 1).
    const CHARACTERS: CollisionLayers3d = CollisionLayers3d::new(2, u32::MAX);
    const STATIC_ONLY: CollisionLayers3d = CollisionLayers3d::new(1, 1);

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        floor: BodyId,
        ids: Vec<BodyId>,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
        hits: Vec<QueryHit>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            let floor = world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(FIELD_X, 0.0, FIELD_Z),
                        cell_size: (CELL, CELL),
                        columns: COLUMNS,
                        rows: ROWS,
                        heights: heights(),
                        removed_cells: None,
                        material: Default::default(),
                    },
                    Some(2),
                )
                .unwrap();
            let settings = CharacterSettings {
                offset: OFFSET,
                snap_to_ground: Some(SNAP),
                ..CharacterSettings::default()
            };
            let ids = (0..LANES)
                .map(|k| {
                    let mut body = Body::keyed(
                        CHARACTER_BASE + k as u64,
                        Shape::capsule(HALF, RADIUS),
                        v(start(k)),
                        1.0,
                    );
                    body.external = true;
                    body.layers = CHARACTERS;
                    let id = world.add_body(body).unwrap();
                    world.add_character(id, settings).unwrap();
                    id
                })
                .collect();
            Self {
                world,
                floor,
                ids,
                vy: vec![0.0; LANES],
                grounded: vec![false; LANES],
                moves: Vec::with_capacity(LANES),
                results: Vec::with_capacity(LANES),
                hits: Vec::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for k in 0..LANES {
                if let Some(vy) = swim(k, tick) {
                    self.vy[k] = vy * DT;
                } else if self.grounded[k] && self.vy[k] <= 0.0 {
                    self.vy[k] = 0.0;
                } else {
                    self.vy[k] += G;
                }
                let p = self.world.body(self.ids[k]).unwrap().position;
                let [x, z] = walk(k);
                self.moves.push(CharacterMove {
                    body: self.ids[k],
                    desired: Vector(x, self.vy[k], z),
                    exclude: open_floor(p.0, p.2).then_some(self.floor),
                    autostep: None,
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (k, r) in self.results.iter().enumerate() {
                let vy = self.vy[k];
                if (r.grounded && vy <= 0.0) || (vy > 0.0 && r.translation.1 < vy - 1e-5) {
                    self.vy[k] = 0.0;
                }
                self.grounded[k] = r.grounded;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(r.grounded));
            }
            for k in 0..LANES {
                let p = self.world.body(self.ids[k]).unwrap().position;
                let gap = self.ground_gap(p);
                trace.0.push(Obs::Support(gap));
            }
        }
    }

    impl Sim {
        /// MMORPG's `ground_gap` on this engine: a static-only cast of the capsule inflated by
        /// the offset (Rapier's target distance) down the support reach, the floor left out
        /// over the basin, and only walkable ground counts.
        fn ground_gap(&mut self, p: Vector) -> Option<f32> {
            let filter = QueryFilter {
                layers: STATIC_ONLY,
                exclude: open_floor(p.0, p.2).then_some(self.floor),
                include_sensors: false,
            };
            let pose = QueryPose::new(Shape::capsule(HALF, RADIUS + OFFSET), p);
            let down = Vector(0.0, -SUPPORT_REACH, 0.0);
            self.world
                .cast_shape(ShapeCast::new(pose, down), filter, &mut self.hits)
                .unwrap();
            self.hits
                .iter()
                .find(|h| h.starts_overlapping || h.normal.dot(down) < 0.0)
                .filter(|h| h.normal.1 >= core::f32::consts::FRAC_PI_4.cos())
                .map(|h| h.fraction * SUPPORT_REACH)
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::control::{CharacterLength, KinematicCharacterController};
    use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};
    use rapier3d::parry::shape::HeightField;
    use rapier3d::parry::utils::Array2;
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: PhysicsWorld,
        controller: KinematicCharacterController,
        floor: ColliderHandle,
        handles: Vec<RigidBodyHandle>,
        shape: Capsule,
        positions: Vec<[f32; 3]>,
        velocity: Vec<[f32; 3]>,
        grounded: Vec<bool>,
        support: Vec<Option<f32>>,
    }

    /// MMORPG's `without_open_floor`.
    fn without_open_floor(
        filter: QueryFilter<'_>,
        floor: ColliderHandle,
        p: [f32; 3],
    ) -> QueryFilter<'_> {
        if open_floor(p[0], p[2]) {
            filter.exclude_collider(floor)
        } else {
            filter
        }
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            let floor = world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            // parry: rows along Z, columns along X, column-major storage, centred.
            let h = heights();
            let data = (0..ROWS * COLUMNS)
                .map(|k| h[(k % ROWS) * COLUMNS + k / ROWS])
                .collect();
            let size_x = CELL * (COLUMNS - 1) as f32;
            let size_z = CELL * (ROWS - 1) as f32;
            let field = HeightField::new(
                Array2::new(ROWS, COLUMNS, data),
                Vector::new(size_x, 1.0, size_z),
            );
            world.colliders.insert(
                ColliderBuilder::new(SharedShape::new(field))
                    .translation(Vector::new(
                        FIELD_X + size_x / 2.0,
                        0.0,
                        FIELD_Z + size_z / 2.0,
                    ))
                    .build(),
            );
            let shape = Capsule::new_y(HALF, RADIUS);
            let mut handles = Vec::new();
            let mut positions = Vec::new();
            for k in 0..LANES {
                let p = start(k);
                let handle = world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(v(p))
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::new(SharedShape::new(shape)).build(),
                    handle,
                    &mut world.bodies,
                );
                handles.push(handle);
                positions.push(p);
            }
            world.detect_collisions(&(), &());
            Self {
                world,
                controller: KinematicCharacterController {
                    up: Vector::Y,
                    offset: CharacterLength::Absolute(OFFSET),
                    slide: true,
                    autostep: None,
                    max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
                    min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
                    snap_to_ground: Some(CharacterLength::Absolute(SUPPORT_REACH)),
                    normal_nudge_factor: KinematicCharacterController::default()
                        .normal_nudge_factor,
                },
                floor,
                handles,
                shape,
                positions,
                velocity: vec![[0.0; 3]; LANES],
                grounded: vec![false; LANES],
                support: vec![None; LANES],
            }
        }

        /// MMORPG's `ground_gap`: static ground within the support reach below `position`,
        /// the floor left out over the basin.
        fn ground_gap(&self, position: [f32; 3]) -> Option<f32> {
            let w = &self.world;
            let fixed = w.broad_phase.as_query_pipeline(
                &DefaultQueryDispatcher,
                &w.bodies,
                &w.colliders,
                without_open_floor(QueryFilter::only_fixed(), self.floor, position),
            );
            fixed
                .cast_shape(
                    &Pose::from_translation(v(position)),
                    -Vector::Y,
                    &self.shape,
                    ShapeCastOptions {
                        target_distance: OFFSET,
                        stop_at_penetration: false,
                        max_time_of_impact: SUPPORT_REACH,
                        compute_impact_geometry_on_penetration: true,
                    },
                )
                .filter(|(_, hit)| {
                    hit.normal1.dot(Vector::Y) >= self.controller.max_slope_climb_angle.cos()
                })
                .map(|(_, hit)| hit.time_of_impact)
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for k in 0..LANES {
                let p = self.positions[k];
                let [wx, wz] = walk(k);
                let mut velocity = [wx, self.velocity[k][1], wz];
                let mut settle = 0.0;
                if let Some(vy) = swim(k, tick) {
                    velocity[1] = vy * DT;
                } else {
                    match self.ground_gap(p) {
                        Some(gap) if velocity[1] <= 0.0 => {
                            velocity[1] = 0.0;
                            if gap > GAP_TOLERANCE {
                                settle = -gap;
                            }
                        }
                        _ => velocity[1] += G,
                    }
                }
                let desired = [velocity[0], velocity[1] + settle, velocity[2]];
                let handle = self.handles[k];
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    &DefaultQueryDispatcher,
                    &w.bodies,
                    &w.colliders,
                    without_open_floor(
                        QueryFilter::default().exclude_rigid_body(handle),
                        self.floor,
                        p,
                    ),
                );
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shape,
                    &Pose::from_translation(v(p)),
                    v(desired),
                    |_| {},
                );
                let moved = movement.translation;
                let to = [p[0] + moved.x, p[1] + moved.y, p[2] + moved.z];
                let gap = self.ground_gap(to);
                let supported = gap.is_some();
                self.grounded[k] = movement.grounded;
                self.support[k] = gap;
                if (supported && velocity[1] <= 0.0)
                    || (velocity[1] > 0.0 && moved.y < velocity[1] - 1e-5)
                {
                    velocity[1] = 0.0;
                }
                self.positions[k] = to;
                self.velocity[k] = velocity;
                self.world.bodies[handle].set_translation(v(to), false);
                self.world.detect_collisions(&(), &());
            }
            for k in 0..LANES {
                trace.0.push(Obs::Position(self.positions[k]));
                trace.0.push(Obs::Grounded(self.grounded[k]));
            }
            for k in 0..LANES {
                trace.0.push(Obs::Support(self.support[k]));
            }
        }
    }
}
