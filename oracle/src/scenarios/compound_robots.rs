//! C28 compound dynamic bodies (#567, moenarch-games/shooter#73): twelve shooter robots, the
//! match size, each one rigid body built from box, sphere and capsule parts with mass at their
//! mounting positions, on the floor plane at 60 Hz. The engine side builds each robot with
//! `PreparedCompound` and `World::add_compound`; Rapier attaches one collider per part with
//! `.mass()` to one dynamic body, as the shooter's oracle backend would.
//!
//! - **Mass placement** (lanes 0–4): a 40 × 40 × 80 cm, 20 kg chassis standing on the floor with
//!   a 30 kg block on its top, over the centre, slightly off it, and far off one side (+X, −Z,
//!   and a corner). The off-centre builds tip over that edge; the others stay upright.
//! - **Rolling stability** (lanes 5–8): a long, flat 1.6 m chassis rolled about its long axis by
//!   an initial spin, its 30 kg block mounted low on the deck or high on a mast. The low build
//!   rocks back onto its tracks; the high one rolls over.
//! - **Hit attribution** (lanes 9–11): armoured robots (chassis, front and side plates, a ball
//!   head, a capsule barrel) pushed off their centre of mass at tick 30; every 10 ticks four rays
//!   aimed at the front plate, the side plate, the head and the barrel report which part they
//!   hit.
//!
//! Observations: first each robot's centre of mass in its authoring frame and its principal
//! moments; then per tick each robot's centre of mass in the world and its authoring frame's
//! rotation, and on probe ticks the rays' (part, fraction).
//!
//! Invariants against Rapier: centres of mass within 1e-5 m and principal moments within 1e-4
//! relative; every robot ends upright or fallen as Rapier's does (and tips towards the same
//! side); every tick each centre of mass within 0.1 m of Rapier's, the final one within 0.05 m
//! for those that stay upright; never deeper than Rapier's lowest centre of mass minus 1 cm;
//! every ray hits the same part as Rapier's at a fraction within 0.01.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 180;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const LANES: usize = 12;
const SPACING: f32 = 4.0;
/// Every `PROBE` ticks the hit lanes cast their rays.
const PROBE: u32 = 10;
const RAYS: usize = 4;
/// The hit lanes' push: tick and impulse (N·s) at a point 0.3 m above the centre.
const PUSH_TICK: u32 = 30;

/// One part: shape, centre in the robot's frame, rotation about Z (radians), mass.
#[derive(Clone, Copy)]
pub(crate) enum Part {
    Box([f32; 3]),
    Ball(f32),
    /// Half segment along the part's Y and radius.
    Capsule(f32, f32),
}
#[derive(Clone, Copy)]
pub(crate) struct Mount {
    part: Part,
    at: [f32; 3],
    /// Rotation about the part's Z axis.
    roll: f32,
    mass: f32,
}
const fn mount(part: Part, at: [f32; 3], mass: f32) -> Mount {
    Mount {
        part,
        at,
        roll: 0.0,
        mass,
    }
}

#[derive(Clone)]
pub(crate) struct Robot {
    parts: Vec<Mount>,
    /// The authoring origin's start position.
    origin: [f32; 3],
    /// Initial angular velocity (rad/s).
    spin: [f32; 3],
    /// Impulse applied at `PUSH_TICK`, at 0.3 m above the origin.
    push: [f32; 3],
}

const CHASSIS: [f32; 3] = [0.2, 0.4, 0.2];
const BLOCK: Part = Part::Box([0.1, 0.1, 0.1]);
const DECK: [f32; 3] = [0.8, 0.1, 0.3];

fn robots() -> Vec<Robot> {
    let mut out = Vec::new();
    let x = |lane: usize| lane as f32 * SPACING;
    // Mass placement: the block over the centre, slightly off it, and far off three ways.
    for at in [
        [0.0, 0.5, 0.0],
        [0.15, 0.5, 0.0],
        [0.6, 0.5, 0.0],
        [0.0, 0.5, -0.6],
        [0.5, 0.5, 0.25],
    ] {
        let lane = out.len();
        out.push(Robot {
            parts: vec![
                mount(Part::Box(CHASSIS), [0.0; 3], 20.0),
                mount(BLOCK, at, 30.0),
            ],
            origin: [x(lane), CHASSIS[1] + 0.001, 0.0],
            spin: [0.0; 3],
            push: [0.0; 3],
        });
    }
    // Rolling stability: low and high mounted mass, rolled about the long (X) axis.
    for (high, spin) in [(false, 4.0), (true, 4.0), (false, 6.0), (true, 2.5)] {
        let lane = out.len();
        let mut parts = vec![mount(Part::Box(DECK), [0.0; 3], 20.0)];
        if high {
            parts.push(mount(Part::Box([0.03, 0.4, 0.03]), [0.0, 0.5, 0.0], 1.0));
            parts.push(mount(BLOCK, [0.0, 1.0, 0.0], 30.0));
        } else {
            parts.push(mount(BLOCK, [0.0, 0.2, 0.0], 30.0));
        }
        out.push(Robot {
            parts,
            origin: [x(lane), DECK[1] + 0.001, 0.0],
            spin: [spin, 0.0, 0.0],
            push: [0.0; 3],
        });
    }
    // Hit attribution: armoured robots, pushed off their centre of mass.
    for push in [[0.0; 3], [0.0, 0.0, -25.0], [20.0, 0.0, 10.0]] {
        let lane = out.len();
        out.push(Robot {
            parts: vec![
                mount(Part::Box(CHASSIS), [0.0; 3], 20.0),
                // Front (+Z) and side (+X) armour plates.
                mount(Part::Box([0.16, 0.3, 0.02]), [0.0, 0.0, 0.23], 6.0),
                mount(Part::Box([0.02, 0.3, 0.16]), [0.23, 0.0, 0.0], 6.0),
                mount(Part::Ball(0.15), [0.0, 0.58, 0.0], 3.0),
                // A barrel lying along X on the left (−X) side, rolled onto its side.
                Mount {
                    part: Part::Capsule(0.25, 0.05),
                    at: [-0.3, 0.2, 0.0],
                    roll: std::f32::consts::FRAC_PI_2,
                    mass: 4.0,
                },
                mount(Part::Box([0.08, 0.08, 0.08]), [0.0, -0.2, 0.0], 8.0),
            ],
            origin: [x(lane), CHASSIS[1] + 0.001, 0.0],
            spin: [0.0; 3],
            push,
        });
    }
    debug_assert_eq!(out.len(), LANES);
    out
}

/// The rays of a hit lane at `x`: origin and displacement, aimed at the front plate, the side
/// plate, the head and the barrel.
fn rays(x: f32) -> [([f32; 3], [f32; 3]); RAYS] {
    [
        ([x, 0.45, 1.5], [0.0, 0.0, -3.0]),
        ([x + 1.5, 0.3, 0.05], [-3.0, 0.0, 0.0]),
        ([x + 0.02, 2.0, 0.03], [0.0, -2.0, 0.0]),
        ([x - 0.35, 1.5, 0.01], [0.0, -1.5, 0.0]),
    ]
}
fn hit_lane(lane: usize) -> bool {
    lane >= 9
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "compound-robots",
        capabilities: "C28",
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

/// Positions and lengths of the trace: setup, then per tick.
fn layout() -> (usize, usize) {
    (2 * LANES, 2 * LANES)
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let (setup, per_tick) = layout();
    let probes: usize = (0..TICKS).filter(|t| t.is_multiple_of(PROBE)).count() * 3 * RAYS;
    let len = setup + per_tick * TICKS as usize + probes;
    if engine.0.len() != len || rapier.0.len() != len {
        return Err(format!(
            "trace lengths differ: {} / {} / {len}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    // Mass properties.
    for lane in 0..LANES {
        match (
            &engine.0[2 * lane..2 * lane + 2],
            &rapier.0[2 * lane..2 * lane + 2],
        ) {
            ([Obs::Position(c), Obs::Inertia(i)], [Obs::Position(rc), Obs::Inertia(ri)]) => {
                for k in 0..3 {
                    if (c[k] - rc[k]).abs() > 1e-5 {
                        return Err(format!("lane {lane}: centre of mass {c:?}, Rapier {rc:?}"));
                    }
                    if (i[k] - ri[k]).abs() > 1e-4 * ri[k] {
                        return Err(format!("lane {lane}: inertia {i:?}, Rapier {ri:?}"));
                    }
                }
            }
            _ => return Err(format!("lane {lane}: mass properties missing")),
        }
    }
    // Per tick: centre of mass and rotation; probes interleaved after their tick.
    let mut at = setup;
    let mut com = vec![Vec::new(); 2];
    let mut up = vec![Vec::new(); 2];
    for tick in 0..TICKS {
        for (side, trace) in [engine, rapier].into_iter().enumerate() {
            let mut c = Vec::with_capacity(LANES);
            let mut u = Vec::with_capacity(LANES);
            for lane in 0..LANES {
                match (&trace.0[at + 2 * lane], &trace.0[at + 2 * lane + 1]) {
                    (Obs::Position(p), Obs::Rotation(q)) => {
                        c.push(*p);
                        u.push(up_axis(*q));
                    }
                    _ => return Err(format!("tick {tick} lane {lane}: malformed")),
                }
            }
            com[side].push(c);
            up[side].push(u);
        }
        at += per_tick;
        if tick.is_multiple_of(PROBE) {
            for k in 0..3 * RAYS {
                let (e, r) = (&engine.0[at + k], &rapier.0[at + k]);
                match (e, r) {
                    (Obs::Hit(Some((ep, ef))), Obs::Hit(Some((rp, rf))))
                        if ep == rp && (ef - rf).abs() <= 0.01 => {}
                    (Obs::Hit(None), Obs::Hit(None)) => {}
                    _ => {
                        return Err(format!(
                            "tick {tick} ray {k}: engine {e:?}, Rapier {r:?} (part, fraction)"
                        ));
                    }
                }
            }
            at += 3 * RAYS;
        }
    }
    let last = TICKS as usize - 1;
    for lane in 0..LANES {
        let tilt = |side: usize| up[side][last][lane][1].clamp(-1.0, 1.0).acos();
        let fallen = |t: f32| t > 0.8;
        let (te, tr) = (tilt(0), tilt(1));
        if fallen(te) != fallen(tr) || (!fallen(tr) && te > 0.15) {
            return Err(format!(
                "lane {lane} ends tilted {te:.3} rad, Rapier {tr:.3} rad"
            ));
        }
        let start = com[1][0][lane];
        if fallen(tr) {
            // Towards the same side.
            let (e, r) = (com[0][last][lane], com[1][last][lane]);
            let (de, dr) = (
                [e[0] - start[0], e[2] - start[2]],
                [r[0] - start[0], r[2] - start[2]],
            );
            if de[0] * dr[0] + de[1] * dr[1] <= 0.0 {
                return Err(format!("lane {lane} fell towards {de:?}, Rapier {dr:?}"));
            }
        }
        let lowest = (0..TICKS as usize)
            .map(|t| com[1][t][lane][1])
            .fold(f32::INFINITY, f32::min);
        let mut worst = 0.0f32;
        for (t, (engine, rapier)) in com[0].iter().zip(&com[1]).enumerate() {
            let (e, r) = (engine[lane], rapier[lane]);
            let d = ((e[0] - r[0]).powi(2) + (e[1] - r[1]).powi(2) + (e[2] - r[2]).powi(2)).sqrt();
            worst = worst.max(d);
            if e[1] < lowest - 0.01 {
                return Err(format!(
                    "lane {lane} tick {t}: centre of mass at {} m, Rapier never below {lowest}",
                    e[1]
                ));
            }
        }
        let (e, r) = (com[0][last][lane], com[1][last][lane]);
        let end = ((e[0] - r[0]).powi(2) + (e[1] - r[1]).powi(2) + (e[2] - r[2]).powi(2)).sqrt();
        if worst > 0.1 || (!fallen(tr) && end > 0.05) {
            return Err(format!(
                "lane {lane}: centre of mass up to {worst:.4} m from Rapier's, {end:.4} m at the end"
            ));
        }
    }
    Ok(())
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

/// Ascending principal moments.
fn sorted(mut m: [f32; 3]) -> [f32; 3] {
    m.sort_by(f32::total_cmp);
    m
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, BodyStorage, CompoundChild, Config, PreparedCompound, Quaternion, QueryFilter,
            RayCast, Shape, Vector, World,
        },
    };

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        robots: Vec<(BodyId, PreparedCompound)>,
        setup: Option<Vec<Obs>>,
    }

    fn roll(angle: f32) -> Quaternion {
        Quaternion(0.0, 0.0, (angle * 0.5).sin(), (angle * 0.5).cos())
    }

    pub(crate) fn prepared(robot: &Robot) -> PreparedCompound {
        let children: Vec<_> = robot
            .parts
            .iter()
            .map(|m| CompoundChild {
                shape: match m.part {
                    Part::Box(h) => Shape::Box(v(h)),
                    Part::Ball(r) => Shape::Sphere(r),
                    Part::Capsule(h, r) => Shape::capsule(h, r),
                },
                position: v(m.at),
                orientation: roll(m.roll),
                mass: m.mass,
            })
            .collect();
        PreparedCompound::new(&children).unwrap()
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0))
                .unwrap();
            let mut setup = Vec::new();
            let robots = robots()
                .iter()
                .enumerate()
                .map(|(lane, robot)| {
                    let c = prepared(robot);
                    let (position, orientation) =
                        c.body_pose(v(robot.origin), Quaternion::IDENTITY);
                    let mut body =
                        Body::keyed(100 + lane as u64, Shape::Sphere(1.0), position, 1.0);
                    body.orientation = orientation;
                    body.angular_velocity = v(robot.spin);
                    body.sleep_allowed = false;
                    let id = world.add_compound(&c, body).unwrap();
                    let com = c.center_of_mass();
                    let i = c.principal_inertia();
                    setup.push(Obs::Position([com.0, com.1, com.2]));
                    setup.push(Obs::Inertia(sorted([i.0, i.1, i.2])));
                    (id, c)
                })
                .collect();
            Self {
                world,
                robots,
                setup: Some(setup),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(setup) = self.setup.take() {
                trace.0.extend(setup);
            }
            if tick == PUSH_TICK {
                for (lane, robot) in robots().iter().enumerate() {
                    if robot.push != [0.0; 3] {
                        let (id, c) = &self.robots[lane];
                        let b = self.world.body(*id).unwrap();
                        let (origin, _) = c.origin_pose(b.position, b.orientation);
                        let point = origin + Vector(0.0, 0.3, 0.0);
                        self.world.apply_impulse(*id, v(robot.push), point).unwrap();
                    }
                }
            }
            self.world.step(f64::from(DT)).unwrap();
            for (id, c) in &self.robots {
                let b = self.world.body(*id).unwrap();
                let (_, q) = c.origin_pose(b.position, b.orientation);
                trace
                    .0
                    .push(Obs::Position([b.position.0, b.position.1, b.position.2]));
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
            }
            if tick.is_multiple_of(PROBE) {
                for lane in (0..LANES).filter(|&l| hit_lane(l)) {
                    for (o, d) in rays(lane as f32 * SPACING) {
                        let hit = self
                            .world
                            .cast_ray_first(RayCast::new(v(o), v(d)), QueryFilter::default())
                            .unwrap()
                            .and_then(|h| h.child.map(|c| (c, h.fraction)));
                        trace.0.push(Obs::Hit(hit));
                    }
                }
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;
    use std::collections::HashMap;

    use crate::kcc::rapier::v;

    pub struct Sim {
        world: PhysicsWorld,
        robots: Vec<RigidBodyHandle>,
        /// Each part collider's index in its robot.
        parts: HashMap<ColliderHandle, u32>,
        setup: Option<Vec<Obs>>,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            let mut parts = HashMap::new();
            let mut setup = Vec::new();
            let robots = robots()
                .iter()
                .map(|robot| {
                    let handle = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(v(robot.origin))
                            .angvel(v(robot.spin))
                            .can_sleep(false)
                            .build(),
                    );
                    for (k, m) in robot.parts.iter().enumerate() {
                        let builder = match m.part {
                            Part::Box(h) => ColliderBuilder::cuboid(h[0], h[1], h[2]),
                            Part::Ball(r) => ColliderBuilder::ball(r),
                            Part::Capsule(h, r) => ColliderBuilder::capsule_y(h, r),
                        };
                        let pose =
                            Pose::from_parts(v(m.at), Rotation::from_axis_angle(Vector::Z, m.roll));
                        let collider = world.colliders.insert_with_parent(
                            builder.position(pose).mass(m.mass).build(),
                            handle,
                            &mut world.bodies,
                        );
                        parts.insert(collider, k as u32);
                    }
                    let local = world.bodies[handle].mass_properties().local_mprops;
                    let com = local.local_com;
                    let i = local.principal_inertia();
                    setup.push(Obs::Position([com.x, com.y, com.z]));
                    setup.push(Obs::Inertia(sorted([i.x, i.y, i.z])));
                    handle
                })
                .collect();
            Self {
                world,
                robots,
                parts,
                setup: Some(setup),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if let Some(setup) = self.setup.take() {
                trace.0.extend(setup);
            }
            if tick == PUSH_TICK {
                for (lane, robot) in robots().iter().enumerate() {
                    if robot.push != [0.0; 3] {
                        let rb = &mut self.world.bodies[self.robots[lane]];
                        let point = rb.translation() + Vector::new(0.0, 0.3, 0.0);
                        rb.apply_impulse_at_point(v(robot.push), point, true);
                    }
                }
            }
            self.world.step();
            for &h in &self.robots {
                let rb = &self.world.bodies[h];
                let c = rb.center_of_mass();
                let q = rb.rotation();
                trace.0.push(Obs::Position([c.x, c.y, c.z]));
                trace.0.push(Obs::Rotation([q.x, q.y, q.z, q.w]));
            }
            if tick.is_multiple_of(PROBE) {
                for lane in (0..LANES).filter(|&l| hit_lane(l)) {
                    for (o, d) in rays(lane as f32 * SPACING) {
                        let hit = self
                            .world
                            .cast_ray(&Ray::new(v(o), v(d)), 1.0, true, QueryFilter::default())
                            .and_then(|(c, t)| self.parts.get(&c).map(|&k| (k, t)));
                        trace.0.push(Obs::Hit(hit));
                    }
                }
            }
        }
    }
}
