//! C4/C5 (physics-engine#404): characters whose first move starts inside static geometry, as
//! MMORPG authors squat creatures and fight fixtures. Four overlaps, each 5 cm deep: a squat
//! capsule in the halfspace floor (a rat), a capsule in a box face, one in a box corner and one
//! in a heightfield slope. Each overlap is moved four ways: not at all, away from the surface,
//! along it and into it, over 120 ticks at 60 Hz with MMORPG's per-tick gravity rule.
//!
//! Both [`Embedded`] modes run against Rapier. `Embedded::Stay` starts at the embedded pose on
//! both engines (Rapier never depenetrates a moving character). `Embedded::PushOut` starts
//! embedded on the engine and at the pushed-out pose on Rapier, as content authored outside
//! the geometry: pushing out on the first move must give Rapier's motion from there.
//!
//! Each side is its consumer pattern. Rapier: MMORPG's `World::step` (a static ground cast
//! decides the gravity rule, `move_shape` for a non-zero motion, a support cast,
//! `set_translation` and `detect_collisions`). Engine: one `move_characters` batch, with the
//! gravity rule driven by the batch's grounded results.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 120;
const DT: f32 = 1.0 / 60.0;
/// Gravity, metres per tick².
const G: f32 = -9.81 * DT * DT;
/// Jump speed, metres per tick.
const JUMP: f32 = 4.0 * DT;
/// Walking speed, metres per tick.
const WALK: f32 = 1.5 * DT;
const OFFSET: f32 = 0.0025;
const SNAP: f32 = 0.05;
/// How deep every character starts inside its surface.
const DEPTH: f32 = 0.05;
/// The scripted tolerance on positions.
const TOLERANCE: f32 = 0.01;
/// `Embedded::Stay` against Rapier's embedded moves. Rapier's slide steps 0.1 mm off a surface
/// it is inside at every contact; the engine does the same where it meets that surface, but
/// Rapier also counts an exactly tangent walk on a halfspace and a heightfield's internal
/// diagonal as contacts, which works it out by up to 1.3 cm more over the 120 ticks.
const STAY_TOLERANCE: f32 = 0.02;
/// A ground gap below this counts as standing on it (MMORPG's 1e-3 units).
const GAP_TOLERANCE: f32 = 1.0e-5;
const SUPPORT_REACH: f32 = OFFSET + SNAP;
/// A standing character's capsule.
const HALF: f32 = 0.5;
const RADIUS: f32 = 0.3;
/// A rat's capsule: half height 0.2 m is shorter than the radius, so the capsule reaches 5 cm
/// below its box, into the floor.
const RAT_RADIUS: f32 = 0.25;
const RAT_HALF_HEIGHT: f32 = 0.2;
/// The box-face site's box face (x) and the corner site's corner (x).
const FACE_X: f32 = 20.25;
const CORNER_X: f32 = 40.0;
/// Heightfield slope: rises along +X with gradient `SLOPE` from `RAMP_X` (height 0).
const SLOPE: f32 = 0.3;
const RAMP_X: f32 = 60.0;
const RAMP_CELL: f32 = 2.0;
const RAMP_COLUMNS: usize = 11;
const RAMP_ROWS: usize = 41;
const RAMP_Z: f32 = -8.0;
const SLOPE_SITE_X: f32 = 68.0;
const LANE: f32 = 8.0;
/// Box keys: floor 1, face box 2, corner boxes from 10, terrain 3, characters from 100.
const CHARACTER_BASE: u64 = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Site {
    Floor,
    Face,
    Corner,
    Slope,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Motion {
    Zero,
    Away,
    Along,
    Into,
}

#[derive(Clone, Copy, Debug)]
struct Case {
    site: Site,
    motion: Motion,
    /// The engine leaves the start in place (`Embedded::Stay`) instead of pushing out.
    stay: bool,
    /// Capsule half segment and radius.
    half: f32,
    radius: f32,
    /// Capsule centre at the embedded start.
    start: [f32; 3],
    /// Outward surface normal at the overlap.
    normal: [f32; 3],
    /// Horizontal velocity, metres per tick.
    walk: [f32; 2],
    /// Jumps at tick 10 (moving away from the floor).
    jump: bool,
}

impl Case {
    /// Rapier's start: embedded for `Stay`, pushed out to the offset for `PushOut`.
    fn rapier_start(&self) -> [f32; 3] {
        if self.stay {
            return self.start;
        }
        let push = DEPTH + OFFSET;
        let [x, y, z] = self.start;
        let [nx, ny, nz] = self.normal;
        [x + nx * push, y + ny * push, z + nz * push]
    }
}

fn lane_z(lane: usize) -> f32 {
    LANE * lane as f32
}

fn corner_box(lane: usize) -> ([f32; 3], [f32; 3]) {
    let z = lane_z(lane);
    ([CORNER_X + 1.0, 1.0, z + 1.0], [1.0, 1.0, 1.0])
}

/// The face box (shared by every lane) and one corner box per lane.
fn boxes() -> Vec<([f32; 3], [f32; 3])> {
    let mut out = vec![([FACE_X + 1.0, 1.0, 32.0], [1.0, 1.0, 40.0])];
    out.extend((0..8).map(corner_box));
    out
}

fn ramp_height(x: f32) -> f32 {
    SLOPE * (x - RAMP_X)
}

/// Heights row-major: Z rows, X columns.
fn ramp_heights() -> Vec<f32> {
    (0..RAMP_ROWS * RAMP_COLUMNS)
        .map(|k| ramp_height(RAMP_X + RAMP_CELL * (k % RAMP_COLUMNS) as f32))
        .collect()
}

fn cases() -> Vec<Case> {
    let mut out = Vec::new();
    let s = core::f32::consts::FRAC_1_SQRT_2;
    let foot = HALF + RADIUS + OFFSET;
    let slope_norm = (1.0 + SLOPE * SLOPE).sqrt();
    for site in [Site::Floor, Site::Face, Site::Corner, Site::Slope] {
        for (m, motion) in [Motion::Zero, Motion::Away, Motion::Along, Motion::Into]
            .into_iter()
            .enumerate()
        {
            for stay in [true, false] {
                let lane = usize::from(!stay) * 4 + m;
                let z = lane_z(lane);
                let (half, radius, start, normal) = match site {
                    Site::Floor => (0.0, RAT_RADIUS, [0.0, RAT_HALF_HEIGHT, z], [0.0, 1.0, 0.0]),
                    Site::Face => (
                        HALF,
                        RADIUS,
                        [FACE_X - RADIUS + DEPTH, foot, z],
                        [-1.0, 0.0, 0.0],
                    ),
                    Site::Corner => {
                        let d = RADIUS - DEPTH;
                        (
                            HALF,
                            RADIUS,
                            [CORNER_X - d * s, foot, z - d * s],
                            [-s, 0.0, -s],
                        )
                    }
                    Site::Slope => {
                        // The bottom sphere's centre `RADIUS - DEPTH` above the plane.
                        let bottom = ramp_height(SLOPE_SITE_X) + (RADIUS - DEPTH) * slope_norm;
                        (
                            HALF,
                            RADIUS,
                            [SLOPE_SITE_X, bottom + HALF, z],
                            [-SLOPE / slope_norm, 1.0 / slope_norm, 0.0],
                        )
                    }
                };
                // Away from, along and into the surface (horizontally); a floor is left by
                // jumping and entered by gravity alone.
                let (nx, nz) = (normal[0], normal[2]);
                let (walk, jump) = match (site, motion) {
                    (_, Motion::Zero) | (Site::Floor, Motion::Into) => ([0.0, 0.0], false),
                    (Site::Floor, Motion::Away) => ([0.0, 0.0], true),
                    (Site::Floor, Motion::Along) => ([0.0, WALK], false),
                    (_, Motion::Away) => {
                        let length = nx.hypot(nz);
                        ([WALK * nx / length, WALK * nz / length], false)
                    }
                    (_, Motion::Into) => {
                        // Head-on into a corner's edge is a balance either engine leaves to
                        // rounding; 20° off it, the slide follows the nearer face.
                        let turn = if site == Site::Corner {
                            20f32.to_radians()
                        } else {
                            0.0
                        };
                        let (sin, cos) = turn.sin_cos();
                        let length = nx.hypot(nz);
                        let (dx, dz) = (-nx / length, -nz / length);
                        (
                            [WALK * (dx * cos - dz * sin), WALK * (dx * sin + dz * cos)],
                            false,
                        )
                    }
                    (_, Motion::Along) => {
                        let length = nx.hypot(nz);
                        ([-WALK * nz / length, WALK * nx / length], false)
                    }
                };
                out.push(Case {
                    site,
                    motion,
                    stay,
                    half,
                    radius,
                    start,
                    normal,
                    walk,
                    jump,
                });
            }
        }
    }
    out
}

fn jumps(case: &Case, tick: u32) -> bool {
    case.jump && tick == 10
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "embedded-starts",
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

/// Every tick, every character: position within the tolerance of Rapier's and the same
/// grounded state.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let cases = cases();
    if engine.0.len() != rapier.0.len() || engine.0.len() != 2 * cases.len() * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    for (k, (e, r)) in engine.0.chunks(2).zip(rapier.0.chunks(2)).enumerate() {
        let (tick, case) = (k / cases.len(), cases[k % cases.len()]);
        let (Obs::Position(ep), Obs::Grounded(eg), Obs::Position(rp), Obs::Grounded(rg)) =
            (e[0], e[1], r[0], r[1])
        else {
            return Err("unexpected observation".into());
        };
        let label = || {
            let mode = if case.stay { "Stay" } else { "PushOut" };
            format!("{:?} {:?} ({mode}) at tick {tick}", case.site, case.motion)
        };
        let tolerance = if case.stay { STAY_TOLERANCE } else { TOLERANCE };
        if (0..3).any(|i| (ep[i] - rp[i]).abs() > tolerance) {
            return Err(format!("{}: engine {ep:?}, rapier {rp:?}", label()));
        }
        if eg != rg {
            return Err(format!(
                "{}: grounded engine {eg}, rapier {rg} at {ep:?}",
                label()
            ));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, CharacterMove, CharacterMoveResult, CharacterSettings, Config, Embedded,
            HeightfieldDesc, Shape, Vector, World,
        },
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        cases: Vec<Case>,
        ids: Vec<BodyId>,
        vy: Vec<f32>,
        grounded: Vec<bool>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector::ZERO,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            for (i, (c, h)) in boxes().into_iter().enumerate() {
                let key = if i == 0 { 2 } else { 9 + i as u64 };
                world
                    .add_body(Body::keyed(key, Shape::Box(v(h)), v(c), 0.0))
                    .unwrap();
            }
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(RAMP_X, 0.0, RAMP_Z),
                        cell_size: (RAMP_CELL, RAMP_CELL),
                        columns: RAMP_COLUMNS,
                        rows: RAMP_ROWS,
                        heights: ramp_heights(),
                        removed_cells: None,
                        material: Default::default(),
                    },
                    Some(3),
                )
                .unwrap();
            let cases = cases();
            let mut ids = Vec::new();
            for (i, case) in cases.iter().enumerate() {
                let mut body = Body::keyed(
                    CHARACTER_BASE + i as u64,
                    Shape::capsule(case.half, case.radius),
                    v(case.start),
                    1.0,
                );
                body.external = true;
                let id = world.add_body(body).unwrap();
                let settings = CharacterSettings {
                    offset: OFFSET,
                    snap_to_ground: Some(SNAP),
                    embedded: if case.stay {
                        Embedded::Stay
                    } else {
                        Embedded::PushOut
                    },
                    ..CharacterSettings::default()
                };
                world.add_character(id, settings).unwrap();
                ids.push(id);
            }
            let n = ids.len();
            Self {
                world,
                cases,
                ids,
                vy: vec![0.0; n],
                grounded: vec![false; n],
                moves: Vec::with_capacity(n),
                results: Vec::with_capacity(n),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for (i, case) in self.cases.iter().enumerate() {
                if self.grounded[i] && jumps(case, tick) {
                    // MMORPG applies gravity on the jump tick too.
                    self.vy[i] = JUMP + G;
                } else if self.grounded[i] && self.vy[i] <= 0.0 {
                    self.vy[i] = 0.0;
                } else {
                    self.vy[i] += G;
                }
                self.moves.push(CharacterMove {
                    body: self.ids[i],
                    desired: Vector(case.walk[0], self.vy[i], case.walk[1]),
                    exclude: None,
                    autostep: None,
                });
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            for (i, r) in self.results.iter().enumerate() {
                let vy = self.vy[i];
                if (r.grounded && vy <= 0.0) || (vy > 0.0 && r.translation.1 < vy - 1e-5) {
                    self.vy[i] = 0.0;
                }
                self.grounded[i] = r.grounded;
                let p = self.world.body(r.body).unwrap().position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Grounded(r.grounded));
            }
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
        cases: Vec<Case>,
        handles: Vec<RigidBodyHandle>,
        shapes: Vec<Capsule>,
        positions: Vec<[f32; 3]>,
        velocity: Vec<[f32; 3]>,
        grounded: Vec<bool>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            for (c, h) in boxes() {
                world.colliders.insert(
                    ColliderBuilder::cuboid(h[0], h[1], h[2])
                        .translation(v(c))
                        .build(),
                );
            }
            // parry: rows along Z, columns along X, column-major storage, centred.
            let h = ramp_heights();
            let data = (0..RAMP_ROWS * RAMP_COLUMNS)
                .map(|k| h[(k % RAMP_ROWS) * RAMP_COLUMNS + k / RAMP_ROWS])
                .collect();
            let size_x = RAMP_CELL * (RAMP_COLUMNS - 1) as f32;
            let size_z = RAMP_CELL * (RAMP_ROWS - 1) as f32;
            let field = HeightField::new(
                Array2::new(RAMP_ROWS, RAMP_COLUMNS, data),
                Vector::new(size_x, 1.0, size_z),
            );
            world.colliders.insert(
                ColliderBuilder::new(SharedShape::new(field))
                    .translation(Vector::new(
                        RAMP_X + size_x / 2.0,
                        0.0,
                        RAMP_Z + size_z / 2.0,
                    ))
                    .build(),
            );
            let cases = cases();
            let mut handles = Vec::new();
            let mut shapes = Vec::new();
            let mut positions = Vec::new();
            for case in &cases {
                let p = case.rapier_start();
                let shape = Capsule::new_y(case.half, case.radius);
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
                shapes.push(shape);
                positions.push(p);
            }
            world.detect_collisions(&(), &());
            let n = cases.len();
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
                cases,
                handles,
                shapes,
                positions,
                velocity: vec![[0.0; 3]; n],
                grounded: vec![false; n],
            }
        }

        /// MMORPG's `ground_gap`: static ground within the support reach below `position`.
        fn ground_gap(&self, i: usize, position: [f32; 3]) -> Option<f32> {
            let w = &self.world;
            let fixed = w.broad_phase.as_query_pipeline(
                &DefaultQueryDispatcher,
                &w.bodies,
                &w.colliders,
                QueryFilter::only_fixed(),
            );
            fixed
                .cast_shape(
                    &Pose::from_translation(v(position)),
                    -Vector::Y,
                    &self.shapes[i],
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
            for i in 0..self.cases.len() {
                let case = self.cases[i];
                let p = self.positions[i];
                self.velocity[i][0] = case.walk[0];
                self.velocity[i][2] = case.walk[1];
                if jumps(&case, tick) && self.ground_gap(i, p).is_some() {
                    self.velocity[i][1] = JUMP;
                }
                let mut velocity = self.velocity[i];
                let mut settle = 0.0;
                let gap = self.ground_gap(i, p);
                self.grounded[i] = gap.is_some();
                match gap {
                    Some(gap) if velocity[1] <= 0.0 => {
                        velocity[1] = 0.0;
                        if gap > GAP_TOLERANCE {
                            settle = -gap;
                        }
                    }
                    _ => velocity[1] += G,
                }
                let desired = [velocity[0], velocity[1] + settle, velocity[2]];
                if desired == [0.0; 3] {
                    self.velocity[i] = velocity;
                    continue;
                }
                let handle = self.handles[i];
                let w = &self.world;
                let queries = w.broad_phase.as_query_pipeline(
                    &DefaultQueryDispatcher,
                    &w.bodies,
                    &w.colliders,
                    QueryFilter::default().exclude_rigid_body(handle),
                );
                let movement = self.controller.move_shape(
                    DT,
                    &queries,
                    &self.shapes[i],
                    &Pose::from_translation(v(p)),
                    v(desired),
                    |_| {},
                );
                let moved = movement.translation;
                let to = [p[0] + moved.x, p[1] + moved.y, p[2] + moved.z];
                let supported = movement.grounded && self.ground_gap(i, to).is_some();
                // Rapier's own grounded is the reference. MMORPG's support cast keeps only its
                // first hit, so a wall the capsule touches or is inside (as near as the floor)
                // can hide the floor from it.
                self.grounded[i] = movement.grounded;
                if (supported && velocity[1] <= 0.0)
                    || (velocity[1] > 0.0 && moved.y < velocity[1] - 1e-5)
                {
                    velocity[1] = 0.0;
                }
                velocity[0] = moved.x;
                velocity[2] = moved.z;
                self.positions[i] = to;
                self.velocity[i] = velocity;
                self.world.bodies[handle].set_translation(v(to), false);
                self.world.detect_collisions(&(), &());
            }
            for i in 0..self.cases.len() {
                trace.0.push(Obs::Position(self.positions[i]));
                trace.0.push(Obs::Grounded(self.grounded[i]));
            }
        }
    }
}
