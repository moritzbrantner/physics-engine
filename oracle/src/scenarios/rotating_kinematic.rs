//! Rotating kinematic bodies (#520, mmorpg#300 solid blades, rotating platforms, swinging
//! doors). Nine lanes on a floor plane, each a box blade (2 × 0.5 × 0.2 m) turning about a
//! vertical pivot at one end, standing on the floor (a kinematic body does not collide with it).
//! Every tick the blade is given its next pose (the engine: `set_next_kinematic_pose`; Rapier: a
//! `kinematic_position_based` body's `set_next_kinematic_position(Pose)`), at 2 rad/s from tick
//! `START`; the last four lanes repeat the others but the projectile, turning the other way.
//! In its path stands:
//!
//! - `Crate`: a 0.5 m crate shoved towards the blade as it starts (awake and sliding), struck
//!   and pushed along.
//! - `Sleeper`: the same crate left asleep: it sleeps when the blade starts and must wake.
//! - `Panel`: a flat debris panel (6 cm thick, 30 cm across), asleep.
//! - `Projectile`: a 10 cm CCD ball (`Body::ccd`; Rapier `ccd_enabled`) fired at 40 m/s (0.67 m
//!   a tick) head-on into the blade's face as it comes round: it stops at the face and is
//!   pushed along, never passing through.
//! - `Character`: a planar character capsule (the consumers' controller and default
//!   `Embedded::PushOut`; Rapier's `move_shape`) walking for `WALK_TICKS` against the blade's
//!   motion, into its face, then standing (the consumers skip zero-motion moves). While it
//!   walks, the blade's face pushes it out of the way on both engines; standing, the blade
//!   passes through it on both (a kinematic blade does not push a kinematic character).
//!
//! A body the blade passed through would stay where it stood: every struck body must travel.
//!
//! A blade spinning fast enough to pass a body within a substep is the engine's own test
//! (`tests/kinematic_pose.rs`): Rapier 0.36.1's blade at 6 to 12 rad/s sweeps through a sleeping
//! 6 cm panel for a second or more, so it is no reference there.
//!
//! 60 Hz, Rapier's defaults; the engine runs the MMORPG adapter's two substeps of one pass. Both
//! sides record each lane's body position and velocity per tick.

use crate::kcc::{HALF, OFFSET, RADIUS};
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 300;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
/// The blade starts turning at this tick, once the sleeper sleeps.
const START: u32 = 150;
const LANES: usize = 9;
const LANE_GAP: f32 = 8.0;
/// Blade half extents and centre height; its centre is `BLADE_HALF[0]` from the pivot.
const BLADE_HALF: [f32; 3] = [1.0, 0.25, 0.1];
const BLADE_Y: f32 = 0.25;
const RATE: f32 = 2.0;
const CRATE_HALF: f32 = 0.25;
const PANEL_HALF: [f32; 3] = [0.15, 0.03, 0.15];
/// The `Crate` lane's shove at `START`, towards the blade (m/s).
const SHOVE: f32 = 1.0;
const DENSITY: f32 = 600.0;
/// The planar character's centre height and walking speed (m/s).
const REST: f32 = HALF + RADIUS + OFFSET;
const WALK: f32 = 1.0;
const WALK_TICKS: u32 = 20;
/// Acceptance. Every struck body travels at least `STRUCK` m on both engines; its peak
/// horizontal speed after the blade starts is within `SPEED_RATIO` of Rapier's (debris-style
/// bound: contact details differ, the strike's energy does not), and so is its final distance
/// from the pivot. Characters stay within `CHARACTER_TOLERANCE` of Rapier's path: Rapier's
/// controller moves a character touching a kinematic body by that body's velocity over the
/// coming tick, the engine out of the current overlap, so Rapier leads by one tick of the
/// blade's travel there (about 4 cm). After each walking move the engine's character is clear of
/// the blade's pose at that time.
const STRUCK: f32 = 0.3;
const SPEED_RATIO: f32 = 1.5;
const CHARACTER_TOLERANCE: f32 = 0.05;
/// The projectile: radius, speed (m/s) and the tick it is fired, so that it meets the blade's
/// face where the blade comes round (at `START` + 24).
const BALL: f32 = 0.1;
const BALL_SPEED: f32 = 40.0;
const FIRE: u32 = START + 20;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Crate,
    Sleeper,
    Panel,
    Projectile,
    Character,
}

fn kind(l: usize) -> Kind {
    [
        Kind::Crate,
        Kind::Sleeper,
        Kind::Panel,
        Kind::Projectile,
        Kind::Character,
        Kind::Crate,
        Kind::Sleeper,
        Kind::Panel,
        Kind::Character,
    ][l]
}

/// +1 for the first five lanes, -1 (turning the other way, mirrored) for the rest.
fn sense(l: usize) -> f32 {
    if l < 5 { 1.0 } else { -1.0 }
}

fn pivot(l: usize) -> [f32; 3] {
    [0.0, BLADE_Y, LANE_GAP * l as f32]
}

fn rate(l: usize) -> f32 {
    sense(l) * RATE
}

/// The blade's yaw after `tick` (its pose for the end of that tick's step).
fn angle(l: usize, tick: u32) -> f32 {
    if tick < START {
        0.0
    } else {
        rate(l) * DT * (tick + 1 - START) as f32
    }
}

/// Blade centre and rotation (`[x, y, z, w]`) at yaw `a` about +Y (+X turns towards -Z).
fn blade_pose(l: usize, a: f32) -> ([f32; 3], [f32; 4]) {
    let p = pivot(l);
    let (s, c) = a.sin_cos();
    let (hs, hc) = (0.5 * a).sin_cos();
    (
        [p[0] + BLADE_HALF[0] * c, p[1], p[2] - BLADE_HALF[0] * s],
        [0.0, hs, 0.0, hc],
    )
}

/// Polar angle and radius of the lane's body about the pivot.
fn placement(l: usize) -> (f32, f32) {
    let (a, r) = match kind(l) {
        Kind::Crate | Kind::Sleeper => (0.8, 1.5),
        Kind::Panel => (0.6, 1.6),
        Kind::Projectile => (0.8, 1.5),
        Kind::Character => (0.9, 1.2),
    };
    (sense(l) * a, r)
}

/// The lane's body: centre, yaw and half extents (a character's are unused).
fn body_start(l: usize) -> ([f32; 3], f32, [f32; 3]) {
    let (a, r) = placement(l);
    let p = pivot(l);
    let (half, y) = match kind(l) {
        Kind::Crate | Kind::Sleeper => ([CRATE_HALF; 3], CRATE_HALF),
        Kind::Panel => (PANEL_HALF, PANEL_HALF[1]),
        Kind::Projectile => ([BALL; 3], BALL),
        Kind::Character => ([0.0; 3], REST),
    };
    // A panel's edge faces the blade's motion at its angle.
    let yaw = if kind(l) == Kind::Panel { a } else { 0.0 };
    let mut at = [p[0] + r * a.cos(), y, p[2] - r * a.sin()];
    if kind(l) == Kind::Projectile {
        // Where its flight from `FIRE` meets the blade's face at `START` + 24.
        let back = BALL_SPEED * DT * (START + 24 - FIRE) as f32;
        let d = towards_blade(l);
        at = [at[0] - d[0] * back, at[1], at[2] - d[2] * back];
    }
    (at, yaw, half)
}

#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn ball_mass() -> f32 {
    DENSITY * 4.0 / 3.0 * core::f32::consts::PI * BALL.powi(3)
}

#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn mass(half: [f32; 3]) -> f32 {
    DENSITY * 8.0 * half[0] * half[1] * half[2]
}

/// Unit direction against the blade's motion at the lane body's polar angle.
fn towards_blade(l: usize) -> [f32; 3] {
    let (a, _) = placement(l);
    // The blade at polar angle `a` moves along s · (-sin a, 0, -cos a).
    [sense(l) * a.sin(), 0.0, sense(l) * a.cos()]
}

/// The character's desired motion at `tick`: walking against the blade's motion from `START`
/// for `WALK_TICKS`, into the blade's face, then standing.
fn walk(l: usize, tick: u32) -> [f32; 3] {
    if !(START..START + WALK_TICKS).contains(&tick) {
        return [0.0; 3];
    }
    towards_blade(l).map(|x| x * WALK * DT)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "rotating-kinematic",
        capabilities: "C18 C24",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        check,
    }
}

/// A lane body's position and velocity after a tick.
type State = ([f32; 3], [f32; 3]);

/// Per tick and lane: (position, velocity).
fn states(trace: &Trace) -> Result<Vec<State>, String> {
    if trace.0.len() != TICKS as usize * LANES * 2 {
        return Err(format!("trace has {} observations", trace.0.len()));
    }
    trace
        .0
        .chunks(2)
        .map(|pair| match (pair[0], pair[1]) {
            (Obs::Position(p), Obs::Velocity(v)) => Ok((p, v)),
            other => Err(format!("unexpected observations {other:?}")),
        })
        .collect()
}

/// Whether `p` lies inside lane `l`'s blade after `tick`.
fn inside_blade(l: usize, tick: u32, p: [f32; 3]) -> bool {
    let (c, _) = blade_pose(l, angle(l, tick));
    let a = angle(l, tick);
    let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
    // Into the blade's frame: undo the yaw.
    let (s, co) = a.sin_cos();
    let local = [co * d[0] - s * d[2], d[1], s * d[0] + co * d[2]];
    (0..3).all(|k| local[k].abs() < BLADE_HALF[k])
}

/// Whether `p` is behind lane `l`'s blade after `tick`: within its length, past its back face.
fn behind_blade(l: usize, tick: u32, p: [f32; 3]) -> bool {
    let a = angle(l, tick);
    let (c, _) = blade_pose(l, a);
    let d = [p[0] - c[0], p[2] - c[2]];
    let (s, co) = a.sin_cos();
    let local = [co * d[0] - s * d[1], s * d[0] + co * d[1]];
    // The blade moves along -sense · local Z: its back face is at +sense · half thickness. A
    // ball through it lies within a tick's flight (0.67 m) behind that face.
    let back = sense(l) * local[1];
    local[0].abs() < BLADE_HALF[0] && back > BLADE_HALF[2] && back < BLADE_HALF[2] + 1.0
}

/// Horizontal distance from `p` to lane `l`'s blade footprint at yaw `a`.
fn blade_clearance(l: usize, a: f32, p: [f32; 3]) -> f32 {
    let (c, _) = blade_pose(l, a);
    let d = [p[0] - c[0], p[2] - c[2]];
    let (s, co) = a.sin_cos();
    let local = [co * d[0] - s * d[1], s * d[0] + co * d[1]];
    let out = [
        (local[0].abs() - BLADE_HALF[0]).max(0.0),
        (local[1].abs() - BLADE_HALF[2]).max(0.0),
    ];
    out[0].hypot(out[1])
}

fn horizontal(v: [f32; 3]) -> f32 {
    v[0].hypot(v[2])
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let engine = states(engine)?;
    let rapier = states(rapier)?;
    let at = |s: &[State], tick: u32, l: usize| s[tick as usize * LANES + l];
    for l in 0..LANES {
        let k = kind(l);
        let (start, ..) = body_start(l);
        if k == Kind::Character {
            for tick in 0..TICKS {
                let (e, r) = (at(&engine, tick, l).0, at(&rapier, tick, l).0);
                let d = (0..3).map(|i| (e[i] - r[i]).powi(2)).sum::<f32>().sqrt();
                if d > CHARACTER_TOLERANCE {
                    return Err(format!(
                        "lane {l} character at tick {tick}: {e:?}, Rapier {r:?} ({d} m apart)"
                    ));
                }
                // Recorded after the step: the move saw the blade at the previous tick's pose.
                let clearance = blade_clearance(l, angle(l, tick.saturating_sub(1)), e);
                if walk(l, tick) != [0.0; 3] && clearance < RADIUS + OFFSET - 1e-3 {
                    return Err(format!(
                        "lane {l} character not pushed out at tick {tick}: {clearance} m clear"
                    ));
                }
            }
            continue;
        }
        // Both engines zero a sleeping body's velocity.
        for (side, s) in [("engine", &engine), ("Rapier", &rapier)] {
            let v = at(s, START - 1, l).1;
            if v != [0.0; 3] {
                return Err(format!(
                    "lane {l} ({k:?}): {side} body not asleep at the start: {v:?}"
                ));
            }
        }
        for tick in 0..TICKS {
            let p = at(&engine, tick, l).0;
            if inside_blade(l, tick, p) {
                return Err(format!(
                    "lane {l} ({k:?}): body centre inside the blade at tick {tick}: {p:?}"
                ));
            }
        }
        let summary = |s: &[State]| {
            let end = at(s, TICKS - 1, l).0;
            let travel = horizontal([end[0] - start[0], 0.0, end[2] - start[2]]);
            let peak = (START..TICKS)
                .map(|tick| horizontal(at(s, tick, l).1))
                .fold(0.0, f32::max);
            let p = pivot(l);
            let reach = horizontal([end[0] - p[0], 0.0, end[2] - p[2]]);
            (travel, peak, reach)
        };
        let (e_travel, e_peak, e_reach) = summary(&engine);
        if k == Kind::Projectile {
            for tick in FIRE..TICKS {
                if behind_blade(l, tick, at(&engine, tick, l).0) {
                    return Err(format!(
                        "lane {l}: the projectile passed the blade at tick {tick}"
                    ));
                }
            }
        }
        let (r_travel, r_peak, r_reach) = summary(&rapier);
        if e_travel < STRUCK || r_travel < STRUCK {
            return Err(format!(
                "lane {l} ({k:?}): not struck: travel {e_travel} m, Rapier {r_travel} m"
            ));
        }
        let within = |e: f32, r: f32| e <= r * SPEED_RATIO && r <= e * SPEED_RATIO;
        if !within(e_peak, r_peak) || !within(e_reach, r_reach) {
            return Err(format!(
                "lane {l} ({k:?}): peak speed {e_peak} m/s (Rapier {r_peak}), final distance \
                 {e_reach} m (Rapier {r_reach})"
            ));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::BodyId;
    use physics_engine::approximate::{
        Body, BodyStorage, CharacterMove, CharacterMoveResult, CharacterSettings, Config,
        Quaternion, Shape, Vector, World,
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    pub struct Sim {
        world: World,
        blades: Vec<BodyId>,
        bodies: Vec<BodyId>,
        moves: Vec<CharacterMove>,
        results: Vec<CharacterMoveResult>,
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // The MMORPG adapter's crate step (as in `props`): two substeps of one pass.
                substeps: 2,
                velocity_iterations: 1,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            let settings = CharacterSettings {
                offset: OFFSET,
                plane: Some(REST),
                ..CharacterSettings::default()
            };
            let (mut blades, mut bodies) = (Vec::new(), Vec::new());
            for l in 0..LANES {
                let (c, _) = blade_pose(l, 0.0);
                let mut blade = Body::keyed(100 + l as u64, Shape::Box(v(BLADE_HALF)), v(c), 1.0);
                blade.external = true;
                blades.push(world.add_body(blade).unwrap());
                let (p, yaw, half) = body_start(l);
                let id = if kind(l) == Kind::Character {
                    let mut body =
                        Body::keyed(200 + l as u64, Shape::capsule(HALF, RADIUS), v(p), 1.0);
                    body.external = true;
                    let id = world.add_body(body).unwrap();
                    world.add_character(id, settings).unwrap();
                    id
                } else if kind(l) == Kind::Projectile {
                    let mut body =
                        Body::keyed(200 + l as u64, Shape::Sphere(BALL), v(p), ball_mass());
                    body.ccd = true;
                    world.add_body(body).unwrap()
                } else {
                    let mut body =
                        Body::keyed(200 + l as u64, Shape::Box(v(half)), v(p), mass(half));
                    let (s, c) = (0.5 * yaw).sin_cos();
                    body.orientation = Quaternion(0.0, s, 0.0, c);
                    world.add_body(body).unwrap()
                };
                bodies.push(id);
            }
            Self {
                world,
                blades,
                bodies,
                moves: Vec::new(),
                results: Vec::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.moves.clear();
            for l in 0..LANES {
                // The consumers skip zero-motion moves.
                if kind(l) == Kind::Character && walk(l, tick) != [0.0; 3] {
                    self.moves.push(CharacterMove {
                        body: self.bodies[l],
                        desired: v(walk(l, tick)),
                        exclude: None,
                        autostep: None,
                    });
                }
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
            if tick == START {
                for l in 0..LANES {
                    if kind(l) == Kind::Crate {
                        let shove = towards_blade(l).map(|x| x * SHOVE);
                        self.world.set_velocity(self.bodies[l], v(shove)).unwrap();
                    }
                }
            }
            if tick == FIRE {
                for l in 0..LANES {
                    if kind(l) == Kind::Projectile {
                        let shot = towards_blade(l).map(|x| x * BALL_SPEED);
                        self.world.set_velocity(self.bodies[l], v(shot)).unwrap();
                    }
                }
            }
            if tick >= START {
                for l in 0..LANES {
                    let (c, q) = blade_pose(l, angle(l, tick));
                    self.world
                        .set_next_kinematic_pose(
                            self.blades[l],
                            v(c),
                            Quaternion(q[0], q[1], q[2], q[3]),
                        )
                        .unwrap();
                }
            }
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.bodies {
                let b = self.world.body(id).unwrap();
                let (p, w) = (b.position, b.velocity);
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Velocity([w.0, w.1, w.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{mirror_controller, v};
    use rapier3d::control::KinematicCharacterController;
    use rapier3d::parry::query::DefaultQueryDispatcher;
    use rapier3d::prelude::*;

    pub struct Sim {
        world: PhysicsWorld,
        controller: KinematicCharacterController,
        blades: Vec<RigidBodyHandle>,
        bodies: Vec<RigidBodyHandle>,
        shape: Capsule,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            let shape = Capsule::new_y(HALF, RADIUS);
            let (mut blades, mut bodies) = (Vec::new(), Vec::new());
            for l in 0..LANES {
                let (c, _) = blade_pose(l, 0.0);
                let blade = world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(v(c))
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(BLADE_HALF[0], BLADE_HALF[1], BLADE_HALF[2]).build(),
                    blade,
                    &mut world.bodies,
                );
                blades.push(blade);
                let (p, yaw, half) = body_start(l);
                let body = if kind(l) == Kind::Character {
                    let body = world.bodies.insert(
                        RigidBodyBuilder::kinematic_position_based()
                            .translation(v(p))
                            .build(),
                    );
                    world.colliders.insert_with_parent(
                        ColliderBuilder::new(SharedShape::new(shape)).build(),
                        body,
                        &mut world.bodies,
                    );
                    body
                } else {
                    let projectile = kind(l) == Kind::Projectile;
                    let body = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .pose(Pose::from_parts(v(p), Rotation::from_rotation_y(yaw)))
                            .ccd_enabled(projectile)
                            .build(),
                    );
                    let collider = if projectile {
                        ColliderBuilder::ball(BALL)
                    } else {
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                    };
                    world.colliders.insert_with_parent(
                        collider.density(DENSITY).build(),
                        body,
                        &mut world.bodies,
                    );
                    body
                };
                bodies.push(body);
            }
            Self {
                world,
                controller: mirror_controller(OFFSET, None, None),
                blades,
                bodies,
                shape,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for l in 0..LANES {
                if kind(l) != Kind::Character || walk(l, tick) == [0.0; 3] {
                    continue;
                }
                let handle = self.bodies[l];
                let from = self.world.bodies[handle].translation();
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
                    &self.shape,
                    &Pose::from_translation(from),
                    v(walk(l, tick)),
                    |_| {},
                );
                let t = movement.translation;
                let to = from + Vector::new(t.x, 0.0, t.z);
                self.world.bodies[handle].set_translation(to, true);
            }
            if tick == START {
                for l in 0..LANES {
                    if kind(l) == Kind::Crate {
                        let shove = towards_blade(l).map(|x| x * SHOVE);
                        self.world.bodies[self.bodies[l]].set_linvel(v(shove), true);
                    }
                }
            }
            if tick == FIRE {
                for l in 0..LANES {
                    if kind(l) == Kind::Projectile {
                        let shot = towards_blade(l).map(|x| x * BALL_SPEED);
                        self.world.bodies[self.bodies[l]].set_linvel(v(shot), true);
                    }
                }
            }
            if tick >= START {
                for l in 0..LANES {
                    let (c, q) = blade_pose(l, angle(l, tick));
                    let rotation = Rotation::from_xyzw(q[0], q[1], q[2], q[3]);
                    self.world.bodies[self.blades[l]]
                        .set_next_kinematic_position(Pose::from_parts(v(c), rotation));
                }
            }
            self.world.step();
            for &handle in &self.bodies {
                let b = &self.world.bodies[handle];
                let (p, w) = (b.translation(), b.linvel());
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Velocity([w.x, w.y, w.z]));
            }
        }
    }
}
