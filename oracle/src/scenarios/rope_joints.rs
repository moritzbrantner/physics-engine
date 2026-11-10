//! C21 rope joints (#453, MMORPG dragging and suspended cargo): Rapier 0.36.1's `RopeJoint`
//! (a coupled linear limit, maximum only) against the engine's `DistanceJoint`.
//!
//! Drag lanes: a kinematic anchor 1 m above the floor plane moves along +x at 1.5–4 m/s from
//! tick 20 to tick 140 and pulls a rotation-locked crate (MMORPG's props: friction 0.8, linear
//! damping 0.2) by a 2 m rope tied to the crate's leading face; densities 150, 2,400 and 600
//! kg/m³; one crate is pulled diagonally against a long wall and slides along it. Hang lanes: a
//! load tied by its top to a fixed anchor swings from 35–60° under gravity; light and heavy
//! boxes (same swing), a sphere, a box whose swing scrapes the floor, one that swings into a
//! post, and one hanging at rest whose body is removed at tick 100 (its joint goes with it) and
//! replaced at tick 110 by a new body and joint released from 30°. The step is 1/60 s until
//! tick 150, then 1/30 s. One world on both sides.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 270;
const SLOW_TICK: u32 = 150;
const GRAVITY: f32 = 9.81;
const DRAG_START: u32 = 20;
const DRAG_STOP: u32 = 140;
const CRATE_HALF: f32 = 0.35;
const CRATE_FRICTION: f32 = 0.8;
const CRATE_DAMPING: f32 = 0.2;
const DRAG_ROPE: f32 = 2.0;
const ANCHOR_HEIGHT: f32 = 1.0;
const ANCHOR_START_X: f32 = 1.2;
/// The drag-along-wall lane: the anchor runs this far beyond the wall face in +z.
const WALL_OFFSET: f32 = 1.0;
const WALL_GAP: f32 = 0.15;
const LOAD_HALF: f32 = 0.25;
const REMOVE_TICK: u32 = 100;
const RESPAWN_TICK: u32 = 110;
/// The step's length, seconds.
fn dt(tick: u32) -> f32 {
    if tick < SLOW_TICK {
        1.0 / 60.0
    } else {
        1.0 / 30.0
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Load {
    Box,
    Sphere,
}

#[derive(Clone, Copy, PartialEq)]
enum Lane {
    Drag {
        speed: f32,
        density: f32,
        wall: bool,
    },
    Hang {
        load: Load,
        density: f32,
        angle: f32,
        height: f32,
        rope: f32,
        post: bool,
        respawn: bool,
    },
}

const LANES: [Lane; 10] = [
    Lane::Drag {
        speed: 1.5,
        density: 150.0,
        wall: false,
    },
    Lane::Drag {
        speed: 1.5,
        density: 2400.0,
        wall: false,
    },
    Lane::Drag {
        speed: 4.0,
        density: 600.0,
        wall: false,
    },
    Lane::Drag {
        speed: 2.0,
        density: 600.0,
        wall: true,
    },
    Lane::Hang {
        load: Load::Box,
        density: 150.0,
        angle: 35.0,
        height: 4.0,
        rope: 2.0,
        post: false,
        respawn: false,
    },
    Lane::Hang {
        load: Load::Box,
        density: 2400.0,
        angle: 35.0,
        height: 4.0,
        rope: 2.0,
        post: false,
        respawn: false,
    },
    Lane::Hang {
        load: Load::Sphere,
        density: 600.0,
        angle: 60.0,
        height: 4.0,
        rope: 2.0,
        post: false,
        respawn: false,
    },
    // Lowest bottom 5 cm below the floor: the swing scrapes it.
    Lane::Hang {
        load: Load::Box,
        density: 600.0,
        angle: 45.0,
        height: 3.0,
        rope: 2.55,
        post: false,
        respawn: false,
    },
    Lane::Hang {
        load: Load::Box,
        density: 600.0,
        angle: 45.0,
        height: 4.0,
        rope: 2.0,
        post: true,
        respawn: false,
    },
    Lane::Hang {
        load: Load::Box,
        density: 2400.0,
        angle: 0.0,
        height: 4.0,
        rope: 2.0,
        post: false,
        respawn: true,
    },
];
/// The respawned load's release angle.
const RESPAWN_ANGLE: f32 = 30.0;
/// The post's centre relative to the hang anchor, and its half extents.
const POST: [f32; 3] = [1.0, -2.2, 0.0];
const POST_HALF: [f32; 3] = [0.1, 0.6, 0.5];

fn lane_z(l: usize) -> f32 {
    5.0 * l as f32
}
fn volume(lane: &Lane) -> f32 {
    match lane {
        Lane::Drag { .. } => 8.0 * CRATE_HALF.powi(3),
        Lane::Hang {
            load: Load::Box, ..
        } => 8.0 * LOAD_HALF.powi(3),
        Lane::Hang {
            load: Load::Sphere, ..
        } => 4.0 / 3.0 * std::f32::consts::PI * LOAD_HALF.powi(3),
    }
}
#[cfg(feature = "engine")]
fn mass(lane: &Lane) -> f32 {
    let density = match *lane {
        Lane::Drag { density, .. } | Lane::Hang { density, .. } => density,
    };
    density * volume(lane)
}
fn rope(lane: &Lane) -> f32 {
    match *lane {
        Lane::Drag { .. } => DRAG_ROPE,
        Lane::Hang { rope, .. } => rope,
    }
}
/// The fixed or starting kinematic anchor point.
fn anchor_start(l: usize) -> [f32; 3] {
    match LANES[l] {
        Lane::Drag { wall, .. } => [
            ANCHOR_START_X,
            ANCHOR_HEIGHT,
            lane_z(l) + if wall { WALL_OFFSET } else { 0.0 },
        ],
        Lane::Hang { height, .. } => [0.0, height, lane_z(l)],
    }
}
/// The kinematic anchor's velocity during `tick`.
fn anchor_velocity(l: usize, tick: u32) -> [f32; 3] {
    match LANES[l] {
        Lane::Drag { speed, .. } if (DRAG_START..DRAG_STOP).contains(&tick) => [speed, 0.0, 0.0],
        _ => [0.0; 3],
    }
}
/// The load's attachment point in its local frame.
fn local_attachment(lane: &Lane) -> [f32; 3] {
    match lane {
        Lane::Drag { .. } => [CRATE_HALF, 0.0, 0.0],
        Lane::Hang { .. } => [0.0, LOAD_HALF, 0.0],
    }
}
/// The load's starting centre: a crate at rest on the floor, a load hanging with a taut rope
/// at `angle` degrees (toward −x).
fn load_start(l: usize, angle: f32) -> [f32; 3] {
    let lane = LANES[l];
    match lane {
        Lane::Drag { .. } => [0.0, CRATE_HALF, lane_z(l)],
        Lane::Hang { rope, .. } => {
            let a = anchor_start(l);
            let (s, c) = angle.to_radians().sin_cos();
            [a[0] - rope * s, a[1] - rope * c - LOAD_HALF, a[2]]
        }
    }
}
fn wall(l: usize) -> Option<([f32; 3], [f32; 3])> {
    match LANES[l] {
        Lane::Drag { wall: true, .. } => {
            let face = lane_z(l) + CRATE_HALF + WALL_GAP;
            Some(([5.0, 1.0, face + 0.25], [8.0, 1.0, 0.25]))
        }
        Lane::Hang { post: true, .. } => {
            let a = anchor_start(l);
            Some(([a[0] + POST[0], a[1] + POST[1], a[2] + POST[2]], POST_HALF))
        }
        _ => None,
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "rope-joints",
        capabilities: "C21",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

/// Observations per lane and tick: the anchor point, the load's attachment point, the load's
/// centre and velocity, the rope's tension.
const PER_LANE: usize = 5;

struct Sample {
    anchor: [f32; 3],
    attachment: [f32; 3],
    centre: [f32; 3],
    velocity: [f32; 3],
    tension: f32,
}
fn sample(t: &Trace, tick: usize, l: usize) -> Option<Sample> {
    let at = (tick * LANES.len() + l) * PER_LANE;
    match t.0.get(at..at + PER_LANE)? {
        &[
            Obs::Position(anchor),
            Obs::Position(attachment),
            Obs::Position(centre),
            Obs::Velocity(velocity),
            Obs::Tension(tension),
        ] => Some(Sample {
            anchor,
            attachment,
            centre,
            velocity,
            tension,
        }),
        _ => None,
    }
}
fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
/// Mechanical energy per unit mass relative to the lowest hanging point: g·y + |v|²/2.
fn energy(s: &Sample) -> f32 {
    GRAVITY * s.centre[1] + 0.5 * (s.velocity.iter().map(|v| v * v).sum::<f32>())
}

/// The steady pull on a crate dragged at constant `speed` along the floor: friction under
/// the rope's lift plus linear damping, `T = m (μ g + c v) / (cos θ + μ sin θ)` with `θ` the
/// rope's rise (`ANCHOR_HEIGHT − CRATE_HALF` over the rope).
fn steady_pull(lane: &Lane) -> f32 {
    let Lane::Drag { speed, density, .. } = *lane else {
        return 0.0;
    };
    let m = density * volume(lane);
    let sin = (ANCHOR_HEIGHT - CRATE_HALF) / DRAG_ROPE;
    let cos = (1.0 - sin * sin).sqrt();
    m * (CRATE_FRICTION * GRAVITY + CRATE_DAMPING * speed) / (cos + CRATE_FRICTION * sin)
}

/// Acceptance, scale-aware (rope length `L`, the anchor's travel, the swing's drop `d`):
/// - attachment drift: the anchors are never more than 0.5% of `L` beyond the rope length
///   (measured: at most 0.08% on the engine, 0.03% on Rapier);
/// - every load stays out of the floor (centre at least its half extent minus 1 cm above it);
///   a dragged crate lifts by the rope's jerk at most 25% + 1 cm more than Rapier's;
/// - drag lanes: the crate's x within 5% of the anchor's travel plus 5 cm of Rapier's every
///   tick, at rest within 5 cm of Rapier's, never into the wall;
/// - hang lanes: the load within 5% of `L` of Rapier's through its first 60 ticks of swing
///   (the box and post lanes are chaotic later), mechanical energy per unit mass within 10% of
///   `g d` (plus 0.05 J/kg) of Rapier's at each of those ticks, never into the post;
/// - tension: a load hanging at rest pulls m·g within 1% and within 5% of Rapier's; a crate
///   dragged at steady speed along the open floor pulls `steady_pull` within 3%. Rapier's
///   reported limit impulse leaves out its position-stabilization share and reads 7–18% lower
///   while its crate keeps the same pace, so it is printed, not compared. Zero after removal.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let expected = TICKS as usize * LANES.len() * PER_LANE;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "trace lengths {} / {}, expected {expected}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let get = |t: &Trace, tick: usize, l: usize| sample(t, tick, l).ok_or("malformed trace");
    for (l, lane) in LANES.iter().enumerate() {
        let length = rope(lane);
        let half = match lane {
            Lane::Drag { .. } => CRATE_HALF,
            Lane::Hang { .. } => LOAD_HALF,
        };
        let (mut worst, mut rapier_worst) = (0.0f32, 0.0f32);
        let (mut lift, mut rapier_lift) = (0.0f32, 0.0f32);
        for tick in 0..TICKS as usize {
            let (e, r) = (get(engine, tick, l)?, get(rapier, tick, l)?);
            let detached = matches!(lane, Lane::Hang { respawn: true, .. })
                && (REMOVE_TICK as usize..RESPAWN_TICK as usize).contains(&tick);
            if detached {
                if e.tension != 0.0 {
                    return Err(format!(
                        "lane {l}: removed joint pulls {} N at tick {tick}",
                        e.tension
                    ));
                }
                continue;
            }
            worst = worst.max(distance(e.anchor, e.attachment) - length);
            rapier_worst = rapier_worst.max(distance(r.anchor, r.attachment) - length);
            lift = lift.max(e.centre[1] - half);
            rapier_lift = rapier_lift.max(r.centre[1] - half);
            if e.centre[1] < half - 0.01 {
                return Err(format!(
                    "lane {l} sank into the floor at tick {tick}: {:?}",
                    e.centre
                ));
            }
        }
        if worst > 0.005 * length {
            return Err(format!(
                "lane {l}: attachment drifted {worst:.4} m past its {length} m rope (Rapier {rapier_worst:.4} m)"
            ));
        }
        match *lane {
            Lane::Drag { speed, wall, .. } => {
                if lift > 1.25 * rapier_lift + 0.01 {
                    return Err(format!(
                        "lane {l}: crate lifted {lift:.3} m, Rapier {rapier_lift:.3} m"
                    ));
                }
                let travel = speed * (DRAG_STOP - DRAG_START) as f32 / 60.0;
                for tick in 0..TICKS as usize {
                    let (e, r) = (get(engine, tick, l)?, get(rapier, tick, l)?);
                    if (e.centre[0] - r.centre[0]).abs() > 0.05 * travel + 0.05 {
                        return Err(format!(
                            "lane {l}: crate at x {:.3} m at tick {tick}, Rapier at {:.3} m",
                            e.centre[0], r.centre[0]
                        ));
                    }
                    if wall {
                        let face = lane_z(l) + CRATE_HALF + WALL_GAP;
                        if e.centre[2] + CRATE_HALF > face + 0.01 {
                            return Err(format!("lane {l} entered the wall at tick {tick}"));
                        }
                    }
                }
                let last = TICKS as usize - 1;
                let (e, r) = (get(engine, last, l)?, get(rapier, last, l)?);
                if distance(e.centre, r.centre) > 0.05 {
                    return Err(format!(
                        "lane {l}: crate rests at {:?}, Rapier at {:?}",
                        e.centre, r.centre
                    ));
                }
                if !wall {
                    // Steady drag: the last 40 ticks of the pull.
                    let mut pull = 0.0;
                    for tick in DRAG_STOP as usize - 40..DRAG_STOP as usize {
                        pull += get(engine, tick, l)?.tension / 40.0;
                    }
                    let want = steady_pull(lane);
                    if (pull - want).abs() > 0.03 * want {
                        return Err(format!(
                            "lane {l}: drags with {pull:.1} N, steady pull {want:.1} N"
                        ));
                    }
                }
            }
            Lane::Hang {
                angle,
                respawn,
                post,
                density,
                ..
            } => {
                let (from, angle) = if respawn {
                    (RESPAWN_TICK as usize, RESPAWN_ANGLE)
                } else {
                    (0, angle)
                };
                let drop = length * (1.0 - angle.to_radians().cos());
                for tick in from..from + 60 {
                    let (e, r) = (get(engine, tick, l)?, get(rapier, tick, l)?);
                    if distance(e.centre, r.centre) > 0.05 * length {
                        return Err(format!(
                            "lane {l}: load at {:?} at tick {tick}, Rapier at {:?}",
                            e.centre, r.centre
                        ));
                    }
                    let (ee, er) = (energy(&e), energy(&r));
                    if (ee - er).abs() > 0.1 * GRAVITY * drop + 0.05 {
                        return Err(format!(
                            "lane {l}: energy {ee:.3} J/kg at tick {tick}, Rapier {er:.3} J/kg"
                        ));
                    }
                    if post {
                        let (c, h) = wall(l).expect("post");
                        // A tilted box reaches at most its half diagonal past its centre.
                        if e.centre[0] > c[0] - h[0] - LOAD_HALF + 0.01 {
                            return Err(format!("lane {l} entered the post at tick {tick}"));
                        }
                    }
                }
                if respawn {
                    let (mut te, mut tr) = (0.0, 0.0);
                    for tick in REMOVE_TICK as usize - 30..REMOVE_TICK as usize {
                        te += get(engine, tick, l)?.tension / 30.0;
                        tr += get(rapier, tick, l)?.tension / 30.0;
                    }
                    let weight = density * volume(lane) * GRAVITY;
                    if (te - tr).abs() > 0.05 * tr || (te - weight).abs() > 0.01 * weight {
                        return Err(format!(
                            "lane {l}: holds {te:.1} N at rest, Rapier {tr:.1} N, weight {weight:.1} N"
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, DistanceJoint, JointId, Shape, Vector, World},
    };

    use crate::kcc::engine::v;

    const SUBSTEPS: u8 = 4;

    struct Rig {
        anchor: BodyId,
        load: BodyId,
        joint: Option<JointId>,
    }
    pub struct Sim {
        world: World,
        rigs: Vec<Rig>,
        /// Every joint id handed out, to check none is reused.
        issued: Vec<JointId>,
    }
    fn load_body(lane: &Lane, at: [f32; 3]) -> Body {
        match *lane {
            Lane::Drag { .. } => {
                let mut b = Body::new(Shape::Box(v([CRATE_HALF; 3])), v(at), mass(lane));
                b.rotation_locked = true;
                b.friction = CRATE_FRICTION;
                b.linear_damping = CRATE_DAMPING;
                b
            }
            Lane::Hang { load, .. } => {
                let shape = match load {
                    Load::Box => Shape::Box(v([LOAD_HALF; 3])),
                    Load::Sphere => Shape::Sphere(LOAD_HALF),
                };
                Body::new(shape, v(at), mass(lane))
            }
        }
    }
    impl Sim {
        pub fn new(storage: physics_engine::approximate::BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // Rapier's four substeps of one pass.
                substeps: SUBSTEPS,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let mut floor = Body::new(Shape::HalfSpace, Vector::ZERO, 0.0);
            floor.friction = CRATE_FRICTION;
            world.add_body(floor).unwrap();
            let mut sim = Self {
                world,
                rigs: Vec::new(),
                issued: Vec::new(),
            };
            for (l, lane) in LANES.iter().enumerate() {
                if let Some((centre, half)) = wall(l) {
                    let mut b = Body::new(Shape::Box(v(half)), v(centre), 0.0);
                    b.friction = CRATE_FRICTION;
                    sim.world.add_body(b).unwrap();
                }
                // A point anchor: a small sensor, kinematic for a drag lane, fixed otherwise.
                let mut anchor = Body::new(
                    Shape::Sphere(0.05),
                    v(anchor_start(l)),
                    if matches!(lane, Lane::Drag { .. }) {
                        1.0
                    } else {
                        0.0
                    },
                );
                anchor.sensor = true;
                anchor.external = matches!(lane, Lane::Drag { .. });
                let anchor = sim.world.add_body(anchor).unwrap();
                let angle = match *lane {
                    Lane::Hang { angle, .. } => angle,
                    Lane::Drag { .. } => 0.0,
                };
                let load = sim
                    .world
                    .add_body(load_body(lane, load_start(l, angle)))
                    .unwrap();
                sim.rigs.push(Rig {
                    anchor,
                    load,
                    joint: None,
                });
                sim.attach(l);
            }
            sim
        }
        fn attach(&mut self, l: usize) {
            let rig = &mut self.rigs[l];
            let id = self
                .world
                .add_joint(DistanceJoint {
                    a: rig.anchor,
                    b: rig.load,
                    anchor_a: Vector::ZERO,
                    anchor_b: v(local_attachment(&LANES[l])),
                    max_length: rope(&LANES[l]),
                })
                .unwrap();
            assert!(!self.issued.contains(&id), "joint id reused");
            self.issued.push(id);
            rig.joint = Some(id);
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (l, lane) in LANES.iter().enumerate() {
                let rig = &self.rigs[l];
                if matches!(lane, Lane::Drag { .. }) {
                    let want = v(anchor_velocity(l, tick));
                    if self.world.body(rig.anchor).unwrap().velocity != want {
                        self.world.set_velocity(rig.anchor, want).unwrap();
                    }
                }
                if matches!(lane, Lane::Hang { respawn: true, .. }) {
                    if tick == REMOVE_TICK {
                        let (load, joint) = (rig.load, rig.joint.unwrap());
                        self.rigs[l].joint = None;
                        self.world.remove_body(load).unwrap();
                        // The body's joint went with it.
                        assert!(self.world.joint(joint).is_none());
                    } else if tick == RESPAWN_TICK {
                        let body = load_body(lane, load_start(l, RESPAWN_ANGLE));
                        self.rigs[l].load = self.world.add_body(body).unwrap();
                        self.attach(l);
                    }
                }
            }
            let h = dt(tick);
            self.world.step(f64::from(h)).unwrap();
            for (l, rig) in self.rigs.iter().enumerate() {
                let a = self.world.body(rig.anchor).unwrap().position;
                let (centre, velocity, attachment) = match self.world.body(rig.load) {
                    Some(b) => (
                        b.position,
                        b.velocity,
                        b.position + b.orientation.rotate(v(local_attachment(&LANES[l]))),
                    ),
                    None => (Vector::ZERO, Vector::ZERO, a),
                };
                let tension = rig
                    .joint
                    .and_then(|j| self.world.joint_impulse(j))
                    .map_or(0.0, |j| j / (h / f32::from(SUBSTEPS)));
                trace.0.extend([
                    Obs::Position([a.0, a.1, a.2]),
                    Obs::Position([attachment.0, attachment.1, attachment.2]),
                    Obs::Position([centre.0, centre.1, centre.2]),
                    Obs::Velocity([velocity.0, velocity.1, velocity.2]),
                    Obs::Tension(tension),
                ]);
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    struct Rig {
        anchor: RigidBodyHandle,
        load: Option<RigidBodyHandle>,
        joint: Option<ImpulseJointHandle>,
    }
    pub struct Sim {
        world: PhysicsWorld,
        rigs: Vec<Rig>,
    }
    fn insert_load(world: &mut PhysicsWorld, lane: &Lane, at: [f32; 3]) -> RigidBodyHandle {
        let (body, collider) = match *lane {
            Lane::Drag { density, .. } => (
                RigidBodyBuilder::dynamic()
                    .translation(v(at))
                    .linear_damping(CRATE_DAMPING)
                    .lock_rotations(),
                ColliderBuilder::cuboid(CRATE_HALF, CRATE_HALF, CRATE_HALF)
                    .friction(CRATE_FRICTION)
                    .density(density),
            ),
            Lane::Hang { load, density, .. } => (
                RigidBodyBuilder::dynamic().translation(v(at)),
                match load {
                    Load::Box => ColliderBuilder::cuboid(LOAD_HALF, LOAD_HALF, LOAD_HALF),
                    Load::Sphere => ColliderBuilder::ball(LOAD_HALF),
                }
                .density(density),
            ),
        };
        world.insert(body, collider).0
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = dt(0);
            world.colliders.insert(
                ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                    .friction(CRATE_FRICTION)
                    .build(),
            );
            let mut sim = Self {
                world,
                rigs: Vec::new(),
            };
            for (l, lane) in LANES.iter().enumerate() {
                if let Some((centre, half)) = wall(l) {
                    sim.world.colliders.insert(
                        ColliderBuilder::cuboid(half[0], half[1], half[2])
                            .translation(v(centre))
                            .friction(CRATE_FRICTION)
                            .build(),
                    );
                }
                let anchor = sim.world.insert_body(
                    match lane {
                        Lane::Drag { .. } => RigidBodyBuilder::kinematic_velocity_based(),
                        Lane::Hang { .. } => RigidBodyBuilder::fixed(),
                    }
                    .translation(v(anchor_start(l))),
                );
                let angle = match *lane {
                    Lane::Hang { angle, .. } => angle,
                    Lane::Drag { .. } => 0.0,
                };
                let load = insert_load(&mut sim.world, lane, load_start(l, angle));
                sim.rigs.push(Rig {
                    anchor,
                    load: Some(load),
                    joint: None,
                });
                sim.attach(l);
            }
            sim
        }
        fn attach(&mut self, l: usize) {
            let rig = &mut self.rigs[l];
            let joint = RopeJointBuilder::new(rope(&LANES[l]))
                .local_anchor1(Vector::ZERO)
                .local_anchor2(v(local_attachment(&LANES[l])));
            rig.joint = Some(
                self.world
                    .insert_impulse_joint(rig.anchor, rig.load.unwrap(), joint),
            );
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            for (l, lane) in LANES.iter().enumerate() {
                if matches!(lane, Lane::Drag { .. }) {
                    let want = v(anchor_velocity(l, tick));
                    let body = &mut self.world.bodies[self.rigs[l].anchor];
                    if body.linvel() != want {
                        body.set_linvel(want, true);
                    }
                }
                if matches!(lane, Lane::Hang { respawn: true, .. }) {
                    if tick == REMOVE_TICK {
                        let load = self.rigs[l].load.take().unwrap();
                        self.world.remove_body(load);
                        self.rigs[l].joint = None;
                    } else if tick == RESPAWN_TICK {
                        let load = insert_load(&mut self.world, lane, load_start(l, RESPAWN_ANGLE));
                        self.rigs[l].load = Some(load);
                        self.attach(l);
                    }
                }
            }
            let h = dt(tick);
            self.world.integration_parameters.dt = h;
            self.world.step();
            let substep = h / self.world.integration_parameters.num_solver_iterations as f32;
            for (l, rig) in self.rigs.iter().enumerate() {
                let a = self.world.bodies[rig.anchor].translation();
                let (centre, velocity, attachment) = match rig.load {
                    Some(h) => {
                        let b = &self.world.bodies[h];
                        let p = b.translation();
                        let attachment = p + b.rotation() * v(local_attachment(&LANES[l]));
                        (p, b.linvel(), attachment)
                    }
                    None => (Vector::ZERO, Vector::ZERO, a),
                };
                let tension = rig
                    .joint
                    .and_then(|j| self.world.impulse_joints.get(j))
                    .and_then(|j| j.data.limits(JointAxis::LinX).map(|l| l.impulse))
                    .map_or(0.0, |j| j / substep);
                trace.0.extend([
                    Obs::Position([a.x, a.y, a.z]),
                    Obs::Position([attachment.x, attachment.y, attachment.z]),
                    Obs::Position([centre.x, centre.y, centre.z]),
                    Obs::Velocity([velocity.x, velocity.y, velocity.z]),
                    Obs::Tension(tension),
                ]);
            }
        }
    }
}
