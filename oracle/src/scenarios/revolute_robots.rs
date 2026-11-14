//! C29 revolute joints with motors (#568, moenarch-games/shooter#73): twelve shooter robots,
//! the match size, each a 20 kg chassis on six motored ball wheels with a two-axis turret (a
//! yaw ring on the chassis, a barrel pitching on the ring): eight revolute joints per robot,
//! 96 in all, on the floor plane at 60 Hz. The engine side uses `RevoluteJoint` with
//! `Motor::Velocity` for the wheels and rotors and `Motor::Position` for the servos; Rapier uses
//! `RevoluteJointBuilder` with `.motor_velocity()`, `.motor_position()`, `.motor_max_force()`
//! and `.limits()`, contacts between joined bodies off on both.
//!
//! - **Normal rolling** (lanes 0–1): wheels about the chassis' Z drive it along +X.
//! - **Side-mounted strafing** (lanes 2–3): the same commands with the wheels mounted 90 degrees
//!   round (about X) drive it along Z.
//! - **Torque-limited stall** (lanes 4–5): wheels limited to 3 and 2 N·m drive into a
//!   frictionless wall, saturate and stop turning.
//! - **One-axis turret** (lanes 6–7): the yaw servo turns the turret to +1.0 and −2.0 rad and
//!   holds it while the wheels brake.
//! - **Paired turret axes** (lanes 8–9): a yaw rotor turns continuously while the chained pitch
//!   servo raises the barrel to 0.5 rad, or is commanded past its 0.7 rad limit.
//! - **Mismatched drive** (lanes 10–11): the left wheels forward and the right ones backward spin
//!   the chassis in place, both ways.
//!
//! Observations per tick and robot: the chassis position, velocity and rotation, then the yaw,
//! pitch and first wheel joints' (angle, rate, motor torque).
//!
//! Invariants against Rapier: rolling and strafing robots reach their wheels' ground speed
//! within 10% of Rapier's along the same axis, drifting sideways under 0.1 m (0.05 m/s), within
//! 0.25 m of Rapier's position; stalled robots stop at the wall within 0.05 m of Rapier's, their
//! wheels nearly still and (engine) at full torque; servos end within 0.02 rad of their target
//! on both engines, rotors within 10% of their rate, and the engine's pitch never passes its
//! limit by more than 0.03 rad (Rapier's stiff servo spring overshoots it for a few ticks);
//! spinning chassis turn the same way as Rapier's within 25% and stay in place; every robot
//! stays upright.
//!
//! The wheels stand 5 cm apart: touching wheels turning the same way rub against each other.
//! Rapier reports motor impulses with the opposite sign; only the engine's torque is checked.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 180;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const LANES: usize = 12;
const SPACING: f32 = 4.0;
const WHEEL: f32 = 0.15;
const WHEEL_MASS: f32 = 2.0;
const WHEEL_FRICTION: f32 = 1.0;
const CHASSIS: [f32; 3] = [0.4, 0.1, 0.25];
const CHASSIS_MASS: f32 = 20.0;
/// The chassis centre's height at rest on its wheels.
const RIDE: f32 = WHEEL + 0.05;
/// Wheel mounts (x, z) on the chassis, 5 cm below its centre.
const WHEELS: [[f32; 2]; 6] = [
    [-0.35, -0.35],
    [-0.35, 0.35],
    [0.0, -0.35],
    [0.0, 0.35],
    [0.35, -0.35],
    [0.35, 0.35],
];
const MOUNT_Y: f32 = -0.05;
const RING: [f32; 3] = [0.12, 0.04, 0.12];
const RING_MASS: f32 = 4.0;
/// The yaw axis' anchor on the chassis (its top) and on the ring (its bottom).
const YAW_ANCHORS: [[f32; 3]; 2] = [[0.0, 0.1, 0.0], [0.0, -0.04, 0.0]];
const BARREL: [f32; 3] = [0.25, 0.03, 0.03];
const BARREL_MASS: f32 = 2.0;
/// The pitch axis' anchor on the ring and on the barrel (5 cm in from its back end).
const PITCH_ANCHORS: [[f32; 3]; 2] = [[0.0, 0.06, 0.0], [-0.2, 0.0, 0.0]];
const PITCH_LIMITS: [f32; 2] = [-0.1, 0.7];
const WHEEL_TORQUE: f32 = 15.0;
const YAW_TORQUE: f32 = 40.0;
const PITCH_TORQUE: f32 = 20.0;
#[cfg(feature = "engine")]
const SERVO_RATE: f32 = 3.0;
/// Rapier's servo spring (acceleration-based) and velocity-motor factor.
#[cfg(feature = "rapier")]
const SERVO_STIFFNESS: f32 = 1.0e5;
#[cfg(feature = "rapier")]
const SERVO_DAMPING: f32 = 630.0;
#[cfg(feature = "rapier")]
const VELOCITY_FACTOR: f32 = 1000.0;
/// The stall lanes' wall: centre offset from the robot's start along X, and half extents.
const WALL_X: f32 = 1.4;
const WALL: [f32; 3] = [0.2, 0.5, 1.0];

#[derive(Clone, Copy, Debug)]
enum Lane {
    Roll(f32),
    Strafe(f32),
    Stall(f32, f32),
    Turret(f32),
    Paired { yaw_rate: f32, pitch: f32 },
    Spin(f32),
}

const KINDS: [Lane; LANES] = [
    Lane::Roll(-8.0),
    Lane::Roll(-12.0),
    Lane::Strafe(-8.0),
    Lane::Strafe(8.0),
    Lane::Stall(-10.0, 3.0),
    Lane::Stall(-10.0, 2.0),
    Lane::Turret(1.0),
    Lane::Turret(-2.0),
    Lane::Paired {
        yaw_rate: 1.5,
        pitch: 0.5,
    },
    Lane::Paired {
        yaw_rate: -1.0,
        pitch: 1.2,
    },
    Lane::Spin(8.0),
    Lane::Spin(-8.0),
];

/// A motor command, shared by both sides.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Command {
    Rate(f32, f32),
    Angle(f32, f32),
}

fn origin(lane: usize) -> [f32; 3] {
    [lane as f32 * SPACING, 0.0, 0.0]
}
fn sideways(lane: usize) -> bool {
    matches!(KINDS[lane], Lane::Strafe(_))
}
fn wheel_axis(lane: usize) -> [f32; 3] {
    if sideways(lane) {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    }
}
/// The wheels' commands, in `WHEELS` order.
fn wheel_commands(lane: usize) -> [Command; 6] {
    let mut out = [Command::Rate(0.0, WHEEL_TORQUE); 6];
    for (k, c) in out.iter_mut().enumerate() {
        let left = WHEELS[k][1] < 0.0;
        *c = match KINDS[lane] {
            Lane::Roll(rate) | Lane::Strafe(rate) => Command::Rate(rate, WHEEL_TORQUE),
            Lane::Stall(rate, torque) => Command::Rate(rate, torque),
            Lane::Spin(rate) => Command::Rate(if left { rate } else { -rate }, WHEEL_TORQUE),
            Lane::Turret(_) | Lane::Paired { .. } => Command::Rate(0.0, WHEEL_TORQUE),
        };
    }
    out
}
fn yaw_command(lane: usize) -> Command {
    match KINDS[lane] {
        Lane::Turret(angle) => Command::Angle(angle, YAW_TORQUE),
        Lane::Paired { yaw_rate, .. } => Command::Rate(yaw_rate, YAW_TORQUE),
        _ => Command::Angle(0.0, YAW_TORQUE),
    }
}
fn pitch_command(lane: usize) -> Command {
    match KINDS[lane] {
        Lane::Paired { pitch, .. } => Command::Angle(pitch, PITCH_TORQUE),
        _ => Command::Angle(0.0, PITCH_TORQUE),
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "revolute-robots",
        capabilities: "C29",
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

const PER_LANE: usize = 6;

struct Tick {
    position: [f32; 3],
    velocity: [f32; 3],
    rotation: [f32; 4],
    yaw: [f32; 3],
    pitch: [f32; 3],
    wheel: [f32; 3],
}

fn parse(trace: &Trace) -> Result<Vec<Vec<Tick>>, String> {
    if trace.0.len() != LANES * PER_LANE * TICKS as usize {
        return Err(format!("trace length {}", trace.0.len()));
    }
    let mut out: Vec<Vec<Tick>> = (0..LANES).map(|_| Vec::new()).collect();
    for (n, chunk) in trace.0.chunks(PER_LANE).enumerate() {
        let lane = n % LANES;
        match chunk {
            [
                Obs::Position(position),
                Obs::Velocity(velocity),
                Obs::Rotation(rotation),
                Obs::Joint(yaw),
                Obs::Joint(pitch),
                Obs::Joint(wheel),
            ] => out[lane].push(Tick {
                position: *position,
                velocity: *velocity,
                rotation: *rotation,
                yaw: *yaw,
                pitch: *pitch,
                wheel: *wheel,
            }),
            _ => return Err(format!("observation {n}: malformed")),
        }
    }
    Ok(out)
}

/// The world direction of a rotation's local +Y.
fn up_axis(q: [f32; 4]) -> [f32; 3] {
    let [x, y, z, w] = q;
    [
        2.0 * (x * y - z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + x * w),
    ]
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let (e, r) = (parse(engine)?, parse(rapier)?);
    let last = TICKS as usize - 1;
    for lane in 0..LANES {
        let fail = |what: String| Err(format!("lane {lane} ({:?}): {what}", KINDS[lane]));
        let (el, rl) = (&e[lane][last], &r[lane][last]);
        for (side, t) in [("engine", el), ("Rapier", rl)] {
            if up_axis(t.rotation)[1] < 0.95 {
                return fail(format!("{side} chassis tipped: {:?}", t.rotation));
            }
        }
        let start = origin(lane);
        // The engine's pitch never passes its limits (Rapier's stiff servo spring overshoots a
        // limit for a few ticks before it holds).
        if let Some((t, x)) = e[lane].iter().enumerate().find(|(_, x)| {
            x.pitch[0] > PITCH_LIMITS[1] + 0.03 || x.pitch[0] < PITCH_LIMITS[0] - 0.03
        }) {
            return fail(format!("pitch {} past its limits at tick {t}", x.pitch[0]));
        }
        match KINDS[lane] {
            Lane::Roll(_) | Lane::Strafe(_) => {
                // The driven axis and the other horizontal one.
                let (along, across) = if sideways(lane) { (2, 0) } else { (0, 2) };
                let (ev, rv) = (el.velocity[along], rl.velocity[along]);
                if ev * rv <= 0.0 || (ev - rv).abs() > 0.1 * rv.abs() {
                    return fail(format!("speed {ev:.3} m/s, Rapier {rv:.3} m/s"));
                }
                if el.velocity[across].abs() > 0.05
                    || (el.position[across] - start[across]).abs() > 0.1
                {
                    return fail(format!(
                        "drifted sideways: {:?} {:?}",
                        el.position, el.velocity
                    ));
                }
                let gap = distance(el.position, rl.position);
                if gap > 0.25 {
                    return fail(format!("{gap:.3} m from Rapier's position"));
                }
            }
            Lane::Stall(_, torque) => {
                for (side, t) in [("engine", el), ("Rapier", rl)] {
                    let speed = distance(t.velocity, [0.0; 3]);
                    if speed > 0.05 || t.wheel[1].abs() > 0.5 {
                        return fail(format!(
                            "{side} not stalled: speed {speed:.3} m/s, wheel {:.3} rad/s",
                            t.wheel[1]
                        ));
                    }
                }
                if (el.position[0] - rl.position[0]).abs() > 0.05 {
                    return fail(format!(
                        "stopped at {:.3}, Rapier at {:.3}",
                        el.position[0], rl.position[0]
                    ));
                }
                if (el.wheel[2].abs() - torque).abs() > 0.02 * torque {
                    return fail(format!(
                        "wheel torque {:.3} N·m, limit {torque}",
                        el.wheel[2]
                    ));
                }
            }
            Lane::Turret(angle) => {
                for (side, t) in [("engine", el), ("Rapier", rl)] {
                    if (t.yaw[0] - angle).abs() > 0.02 || t.pitch[0].abs() > 0.02 {
                        return fail(format!(
                            "{side} yaw {:.4}, pitch {:.4}",
                            t.yaw[0], t.pitch[0]
                        ));
                    }
                }
                if distance(el.position, [start[0], RIDE, start[2]]) > 0.05 {
                    return fail(format!("braked chassis moved to {:?}", el.position));
                }
            }
            Lane::Paired { yaw_rate, pitch } => {
                let target = pitch.min(PITCH_LIMITS[1]);
                for (side, t) in [("engine", el), ("Rapier", rl)] {
                    if (t.yaw[1] - yaw_rate).abs() > 0.1 * yaw_rate.abs()
                        || (t.pitch[0] - target).abs() > 0.02
                    {
                        return fail(format!(
                            "{side} yaw rate {:.3}, pitch {:.4}",
                            t.yaw[1], t.pitch[0]
                        ));
                    }
                }
            }
            Lane::Spin(_) => {
                let (ew, rw) = (e[lane][last].yaw, r[lane][last].yaw);
                // The chassis' own turn rate from its rotation over the last 30 ticks.
                let turn = |ticks: &[Tick]| {
                    let heading = |q: [f32; 4]| {
                        let [x, y, z, w] = q;
                        (2.0 * (x * z + y * w)).atan2(1.0 - 2.0 * (x * x + y * y))
                    };
                    let mut total = 0.0;
                    for pair in ticks[last - 30..=last].windows(2) {
                        let mut d = heading(pair[1].rotation) - heading(pair[0].rotation);
                        if d > std::f32::consts::PI {
                            d -= 2.0 * std::f32::consts::PI;
                        } else if d < -std::f32::consts::PI {
                            d += 2.0 * std::f32::consts::PI;
                        }
                        total += d;
                    }
                    total / (30.0 * DT)
                };
                let (et, rt) = (turn(&e[lane]), turn(&r[lane]));
                if et * rt <= 0.0 || (et - rt).abs() > 0.25 * rt.abs() {
                    return fail(format!(
                        "turns at {et:.3} rad/s, Rapier {rt:.3} rad/s ({ew:?} {rw:?})"
                    ));
                }
                if distance(el.position, [start[0], el.position[1], start[2]]) > 0.15 {
                    return fail(format!("spun away to {:?}", el.position));
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
        approximate::{
            Body, BodyStorage, CombineRule, Config, JointId, Motor, RevoluteJoint, Shape, Vector,
            World,
        },
    };

    use crate::kcc::engine::v;

    struct Robot {
        chassis: BodyId,
        wheels: Vec<JointId>,
        yaw: JointId,
        pitch: JointId,
    }
    pub struct Sim {
        world: World,
        robots: Vec<Robot>,
    }

    fn motor(c: Command) -> Motor {
        match c {
            Command::Rate(rate, max_torque) => Motor::Velocity { rate, max_torque },
            Command::Angle(angle, max_torque) => Motor::Position {
                angle,
                max_rate: SERVO_RATE,
                max_torque,
            },
        }
    }

    fn body(world: &mut World, key: u64, shape: Shape, at: Vector, mass: f32) -> BodyId {
        let mut b = Body::keyed(key, shape, at, mass);
        b.sleep_allowed = false;
        world.add_body(b).unwrap()
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // Rapier's four substeps; four passes each.
                substeps: 4,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            let mut robots = Vec::new();
            for (lane, kind) in KINDS.iter().enumerate() {
                let o = v(origin(lane));
                let key = 100 + 10 * lane as u64;
                if matches!(kind, Lane::Stall(..)) {
                    let mut wall = Body::keyed(
                        key + 9,
                        Shape::Box(v(WALL)),
                        o + Vector(WALL_X, WALL[1], 0.0),
                        0.0,
                    );
                    wall.friction = 0.0;
                    wall.friction_combine = CombineRule::Min;
                    world.add_body(wall).unwrap();
                }
                let chassis = body(
                    &mut world,
                    key,
                    Shape::Box(v(CHASSIS)),
                    o + Vector(0.0, RIDE, 0.0),
                    CHASSIS_MASS,
                );
                let commands = wheel_commands(lane);
                let mut wheels = Vec::new();
                for (k, [x, z]) in WHEELS.into_iter().enumerate() {
                    let mut wheel = Body::keyed(
                        key + 1 + k as u64,
                        Shape::Sphere(WHEEL),
                        o + Vector(x, RIDE + MOUNT_Y, z),
                        WHEEL_MASS,
                    );
                    wheel.friction = WHEEL_FRICTION;
                    wheel.sleep_allowed = false;
                    let wheel = world.add_body(wheel).unwrap();
                    wheels.push(
                        world
                            .add_revolute_joint(
                                RevoluteJoint::new(chassis, wheel, v(wheel_axis(lane)))
                                    .anchors(Vector(x, MOUNT_Y, z), Vector::ZERO)
                                    .motor(motor(commands[k])),
                            )
                            .unwrap(),
                    );
                }
                let ring_at =
                    o + Vector(0.0, RIDE, 0.0) + v(YAW_ANCHORS[0]) + v(YAW_ANCHORS[1]) * -1.0;
                let ring = body(&mut world, key + 7, Shape::Box(v(RING)), ring_at, RING_MASS);
                let barrel_at = ring_at + v(PITCH_ANCHORS[0]) + v(PITCH_ANCHORS[1]) * -1.0;
                let barrel = body(
                    &mut world,
                    key + 8,
                    Shape::Box(v(BARREL)),
                    barrel_at,
                    BARREL_MASS,
                );
                let yaw = world
                    .add_revolute_joint(
                        RevoluteJoint::new(chassis, ring, Vector::Y)
                            .anchors(v(YAW_ANCHORS[0]), v(YAW_ANCHORS[1]))
                            .motor(motor(yaw_command(lane))),
                    )
                    .unwrap();
                let pitch = world
                    .add_revolute_joint(
                        RevoluteJoint::new(ring, barrel, Vector::Z)
                            .anchors(v(PITCH_ANCHORS[0]), v(PITCH_ANCHORS[1]))
                            .limits(PITCH_LIMITS[0], PITCH_LIMITS[1])
                            .motor(motor(pitch_command(lane))),
                    )
                    .unwrap();
                robots.push(Robot {
                    chassis,
                    wheels,
                    yaw,
                    pitch,
                });
            }
            Self { world, robots }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            // The game sets every motor every tick.
            for (lane, robot) in self.robots.iter().enumerate() {
                for (j, c) in robot.wheels.iter().zip(wheel_commands(lane)) {
                    self.world.set_revolute_motor(*j, motor(c)).unwrap();
                }
                self.world
                    .set_revolute_motor(robot.yaw, motor(yaw_command(lane)))
                    .unwrap();
                self.world
                    .set_revolute_motor(robot.pitch, motor(pitch_command(lane)))
                    .unwrap();
            }
            self.world.step(f64::from(DT)).unwrap();
            for robot in &self.robots {
                let b = self.world.body(robot.chassis).unwrap();
                let q = b.orientation;
                trace.0.extend([
                    Obs::Position([b.position.0, b.position.1, b.position.2]),
                    Obs::Velocity([b.velocity.0, b.velocity.1, b.velocity.2]),
                    Obs::Rotation([q.0, q.1, q.2, q.3]),
                ]);
                for j in [robot.yaw, robot.pitch, robot.wheels[0]] {
                    let s = self.world.revolute_state(j).unwrap();
                    trace.0.push(Obs::Joint([s.angle, s.rate, s.motor_torque]));
                }
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    struct Robot {
        chassis: RigidBodyHandle,
        /// (joint, body 1, body 2, local axis) per joint: the wheels, then yaw and pitch.
        joints: Vec<(ImpulseJointHandle, RigidBodyHandle, RigidBodyHandle, Vector)>,
    }
    pub struct Sim {
        world: PhysicsWorld,
        robots: Vec<Robot>,
    }

    fn joint(axis: Vector, anchors: [[f32; 3]; 2], c: Command) -> RevoluteJointBuilder {
        let j = RevoluteJointBuilder::new(axis)
            .local_anchor1(v(anchors[0]))
            .local_anchor2(v(anchors[1]))
            .contacts_enabled(false);
        command(j.build(), c)
    }
    fn command(mut j: RevoluteJoint, c: Command) -> RevoluteJointBuilder {
        match c {
            Command::Rate(rate, max_torque) => {
                j.set_motor_velocity(rate, VELOCITY_FACTOR);
                j.set_motor_max_force(max_torque);
            }
            Command::Angle(angle, max_torque) => {
                j.set_motor_position(angle, SERVO_STIFFNESS, SERVO_DAMPING);
                j.set_motor_max_force(max_torque);
            }
        }
        RevoluteJointBuilder(j)
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            let mut robots = Vec::new();
            for (lane, kind) in KINDS.iter().enumerate() {
                let o = v(origin(lane));
                if matches!(kind, Lane::Stall(..)) {
                    world.colliders.insert(
                        ColliderBuilder::cuboid(WALL[0], WALL[1], WALL[2])
                            .translation(o + Vector::new(WALL_X, WALL[1], 0.0))
                            .friction(0.0)
                            .friction_combine_rule(CoefficientCombineRule::Min)
                            .build(),
                    );
                }
                let mut add = |at: Vector, collider: ColliderBuilder, mass: f32| {
                    world
                        .insert(
                            RigidBodyBuilder::dynamic().translation(at).can_sleep(false),
                            collider.mass(mass),
                        )
                        .0
                };
                let chassis = add(
                    o + Vector::new(0.0, RIDE, 0.0),
                    ColliderBuilder::cuboid(CHASSIS[0], CHASSIS[1], CHASSIS[2]),
                    CHASSIS_MASS,
                );
                let mut bodies = Vec::new();
                for [x, z] in WHEELS {
                    bodies.push(add(
                        o + Vector::new(x, RIDE + MOUNT_Y, z),
                        ColliderBuilder::ball(WHEEL).friction(WHEEL_FRICTION),
                        WHEEL_MASS,
                    ));
                }
                let ring_at =
                    o + Vector::new(0.0, RIDE, 0.0) + v(YAW_ANCHORS[0]) - v(YAW_ANCHORS[1]);
                let ring = add(
                    ring_at,
                    ColliderBuilder::cuboid(RING[0], RING[1], RING[2]),
                    RING_MASS,
                );
                let barrel = add(
                    ring_at + v(PITCH_ANCHORS[0]) - v(PITCH_ANCHORS[1]),
                    ColliderBuilder::cuboid(BARREL[0], BARREL[1], BARREL[2]),
                    BARREL_MASS,
                );
                let mut joints = Vec::new();
                let commands = wheel_commands(lane);
                for (k, &wheel) in bodies.iter().enumerate() {
                    let [x, z] = WHEELS[k];
                    let axis = v(wheel_axis(lane));
                    let j = joint(axis, [[x, MOUNT_Y, z], [0.0; 3]], commands[k]);
                    joints.push((
                        world.insert_impulse_joint(chassis, wheel, j),
                        chassis,
                        wheel,
                        axis,
                    ));
                }
                let yaw = joint(Vector::Y, YAW_ANCHORS, yaw_command(lane));
                joints.push((
                    world.insert_impulse_joint(chassis, ring, yaw),
                    chassis,
                    ring,
                    Vector::Y,
                ));
                let pitch =
                    joint(Vector::Z, PITCH_ANCHORS, pitch_command(lane)).limits(PITCH_LIMITS);
                joints.push((
                    world.insert_impulse_joint(ring, barrel, pitch),
                    ring,
                    barrel,
                    Vector::Z,
                ));
                robots.push(Robot { chassis, joints });
            }
            Self { world, robots }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            // The game sets every motor every tick.
            for (lane, robot) in self.robots.iter().enumerate() {
                let commands = wheel_commands(lane);
                for (k, &(h, ..)) in robot.joints.iter().enumerate() {
                    let c = match k {
                        6 => yaw_command(lane),
                        7 => pitch_command(lane),
                        _ => commands[k],
                    };
                    let j = &mut self.world.impulse_joints.get_mut(h, false).unwrap().data;
                    match c {
                        Command::Rate(rate, max_torque) => {
                            j.set_motor_velocity(JointAxis::AngX, rate, VELOCITY_FACTOR);
                            j.set_motor_max_force(JointAxis::AngX, max_torque);
                        }
                        Command::Angle(angle, max_torque) => {
                            j.set_motor_position(
                                JointAxis::AngX,
                                angle,
                                SERVO_STIFFNESS,
                                SERVO_DAMPING,
                            );
                            j.set_motor_max_force(JointAxis::AngX, max_torque);
                        }
                    }
                }
            }
            self.world.step();
            let substep = DT / self.world.integration_parameters.num_solver_iterations as f32;
            for robot in &self.robots {
                let b = &self.world.bodies[robot.chassis];
                let (p, u, q) = (b.translation(), b.linvel(), b.rotation());
                trace.0.extend([
                    Obs::Position([p.x, p.y, p.z]),
                    Obs::Velocity([u.x, u.y, u.z]),
                    Obs::Rotation([q.x, q.y, q.z, q.w]),
                ]);
                for k in [6, 7, 0] {
                    let (h, b1, b2, axis) = robot.joints[k];
                    let data = self.world.impulse_joints.get(h).unwrap().data;
                    let (r1, r2) = (&self.world.bodies[b1], &self.world.bodies[b2]);
                    let angle = RevoluteJoint { data }.angle(r1.rotation(), r2.rotation());
                    let rate = (r2.angvel() - r1.angvel()).dot(*r1.rotation() * axis);
                    let torque = data
                        .motor(JointAxis::AngX)
                        .map_or(0.0, |m| m.impulse / substep);
                    trace.0.push(Obs::Joint([angle, rate, torque]));
                }
            }
        }
    }
}
