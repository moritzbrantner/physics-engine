//! A fixture whose state hash must be the same on every target (#576, moenarch-games/shooter#120),
//! shared by `tests/cross_target_hash.rs` (native) and `examples/cross_target_hash.rs` (built for
//! wasm32 by `scripts/cross-target-wasm.sh`). It touches each engine call that once went through
//! the platform libm:
//!
//! - a robot on revolute joints (motored wheels and a turret servo): the joint angle's `atan2`;
//! - a kinematic turntable driven by poses: `Quaternion::rotation_to`'s `atan2`;
//! - a character walking up a ramp and across the floor: the `max_slope` cosine;
//! - boxes sliding to rest with friction in a world without joints (the convergence solver):
//!   the friction disk's `hypot`.
//!
//! The setup itself uses no transcendental functions (rotations are literals), so any hash
//! difference between targets comes from the engine.
#![allow(dead_code)]

use physics_engine::approximate::{
    Body, CharacterMove, CharacterSettings, Config, Motor, Quaternion, RevoluteJoint, Shape,
    Vector as V, World,
};

pub const TICKS: u32 = 240;
const DT: f64 = 1.0 / 60.0;

/// The hash after [`TICKS`] steps, the same natively (debug and release) and on wasm32. With
/// std's `atan2`, `cos` and `hypot` (glibc 2.41+) the native hash was 0x3dc0c94c76b431fc.
pub const PINNED: u64 = 0x4a18_b4b1_ae4e_9f7c;

/// FNV-1a over the bits of every value fed to it.
struct Hash(u64);
impl Hash {
    fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    fn f(&mut self, x: f32) {
        self.bytes(&x.to_bits().to_le_bytes());
    }
    fn v(&mut self, v: V) {
        self.f(v.0);
        self.f(v.1);
        self.f(v.2);
    }
    fn q(&mut self, q: Quaternion) {
        self.f(q.0);
        self.f(q.1);
        self.f(q.2);
        self.f(q.3);
    }
    fn world(&mut self, w: &World) {
        for b in w.bodies() {
            self.bytes(&b.id.to_bits().to_le_bytes());
            self.v(b.position);
            self.v(b.velocity);
            self.q(b.orientation);
            self.v(b.angular_velocity);
        }
    }
}

/// Half-angle sine and cosine of 0.4 rad (a 22.9 degree ramp) and of 0.03 rad (the turntable's
/// turn per tick), as literals.
const RAMP: Quaternion = Quaternion(0.0, 0.0, 0.198_669_33, 0.980_066_6);
const TURN: Quaternion = Quaternion(0.0, 0.014_999_438, 0.0, 0.999_887_5);

/// Runs the fixture and returns its hash after [`TICKS`] steps.
pub fn run() -> u64 {
    let mut hash = Hash(0xcbf2_9ce4_8422_2325);

    // The robot world: a chassis on four motored ball wheels with a turret servo, and a
    // kinematic turntable.
    let mut robot = World::new(Config::default()).unwrap();
    robot
        .add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let mut body = Body::keyed(2, Shape::Box(V(0.4, 0.1, 0.25)), V(0.0, 0.3, 0.0), 20.0);
    body.sleep_allowed = false;
    let chassis = robot.add_body(body).unwrap();
    let mut wheels = Vec::new();
    for (k, (x, z)) in [(-0.3, -0.35), (-0.3, 0.35), (0.3, -0.35), (0.3, 0.35)]
        .into_iter()
        .enumerate()
    {
        let mut wheel = Body::keyed(3 + k as u64, Shape::Sphere(0.15), V(x, 0.15, z), 2.0);
        wheel.friction = 1.0;
        wheel.sleep_allowed = false;
        let wheel = robot.add_body(wheel).unwrap();
        wheels.push(
            robot
                .add_revolute_joint(
                    RevoluteJoint::new(chassis, wheel, V::Z).anchors(V(x, -0.05, z), V::ZERO),
                )
                .unwrap(),
        );
    }
    let mut ring = Body::keyed(7, Shape::Box(V(0.15, 0.05, 0.15)), V(0.0, 0.45, 0.0), 3.0);
    ring.sleep_allowed = false;
    let ring = robot.add_body(ring).unwrap();
    let turret = robot
        .add_revolute_joint(
            RevoluteJoint::new(chassis, ring, V::Y)
                .anchors(V(0.0, 0.1, 0.0), V(0.0, -0.05, 0.0))
                .motor(Motor::Position {
                    angle: 1.3,
                    max_rate: 2.0,
                    max_torque: 4.0,
                }),
        )
        .unwrap();
    let mut table = Body::keyed(8, Shape::Box(V(1.0, 0.1, 1.0)), V(4.0, 0.1, 0.0), 50.0);
    table.external = true;
    let table = robot.add_body(table).unwrap();
    robot
        .add_body(Body::keyed(
            9,
            Shape::Box(V(0.2, 0.2, 0.2)),
            V(4.6, 0.41, 0.0),
            1.0,
        ))
        .unwrap();

    // The character world: a floor, a ramp, a character and boxes sliding with friction.
    let mut plain = World::new(Config::default()).unwrap();
    plain
        .add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let mut ramp = Body::keyed(2, Shape::Box(V(3.0, 0.2, 2.0)), V(6.0, 0.6, 0.0), 0.0);
    ramp.orientation = RAMP;
    plain.add_body(ramp).unwrap();
    let mut walker = Body::keyed(3, Shape::capsule(0.5, 0.3), V(0.0, 0.81, 0.0), 1.0);
    walker.external = true;
    let walker = plain.add_body(walker).unwrap();
    plain
        .add_character(walker, CharacterSettings::default())
        .unwrap();
    for k in 0..3 {
        let mut b = Body::keyed(
            10 + k,
            Shape::Box(V(0.3, 0.3, 0.3)),
            V(-3.0, 0.3, -2.0 + 2.0 * k as f32),
            5.0,
        );
        b.velocity = V(-4.0 + k as f32, 0.0, 1.5 - 1.5 * k as f32);
        b.angular_velocity = V(0.0, 2.0 - 2.0 * k as f32, 0.0);
        b.friction = 0.4 + 0.2 * k as f32;
        plain.add_body(b).unwrap();
    }

    let mut turn = Quaternion(0.0, 0.0, 0.0, 1.0);
    let mut moves = Vec::new();
    let mut vy = 0.0f32;
    for tick in 0..TICKS {
        let rate = if tick < 120 { 6.0 } else { -3.0 };
        for (k, j) in wheels.iter().enumerate() {
            let skew = if k < 2 { 1.0 } else { 1.5 };
            robot
                .set_revolute_motor(
                    *j,
                    Motor::Velocity {
                        rate: rate * skew,
                        max_torque: 3.0,
                    },
                )
                .unwrap();
        }
        if tick == 150 {
            robot
                .set_revolute_motor(
                    turret,
                    Motor::Position {
                        angle: -2.0,
                        max_rate: 3.0,
                        max_torque: 4.0,
                    },
                )
                .unwrap();
        }
        turn = turn * TURN;
        robot
            .set_next_kinematic_pose(table, V(4.0, 0.1, 0.0), turn)
            .unwrap();
        robot.step(DT).unwrap();
        if let Some(state) = robot.revolute_state(turret) {
            hash.f(state.angle);
            hash.f(state.rate);
            hash.f(state.motor_torque);
        }

        vy -= 9.81 * DT as f32;
        let desired = V(4.0 * DT as f32, vy * DT as f32, 0.6 * DT as f32);
        plain
            .move_characters(
                &[CharacterMove {
                    body: walker,
                    desired,
                    exclude: None,
                    autostep: None,
                }],
                &mut moves,
            )
            .unwrap();
        if moves[0].grounded {
            vy = 0.0;
        }
        hash.v(moves[0].translation);
        plain.step(DT).unwrap();
    }
    hash.world(&robot);
    hash.world(&plain);
    hash.0
}
