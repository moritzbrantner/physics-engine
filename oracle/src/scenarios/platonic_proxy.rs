//! #465 experiment: regular icosahedron and dodecahedron *visuals* collided as their
//! circumsphere, a plain `Shape::Sphere(circumradius)` CCD body on the engine and a
//! `ColliderBuilder::ball(circumradius)` on Rapier, striking a thin wall at 30–60 m/s (head-on
//! and oblique) inside one 60 Hz tick and flying back. Parity is sphere against sphere, with
//! the same masses, trajectories and timestep; as in `ccd-ricochet`, Rapier's single CCD step
//! stops at its impact, so the parity side subdivides each tick into 64 Rapier steps and a bare
//! single-step run is the performance baseline.
//!
//! Separately, and only as an approximation report (never a parity failure), the check replays
//! the same trajectories on Rapier with a rotation-locked `convex_hull` of the real solid. Its
//! positions must match the analytic reflection at the solid's own support distance `h` along
//! the wall normal, and its distance from the proxy is the circumsphere's early-contact bias,
//! `2 (R − h)` after the bounce, bounded by `2 (R − r)`.

use crate::{Obs, Scenario, Trace};

const DT: f32 = 1.0 / 60.0;
const TICKS: u32 = 60;
const DENSITY: f32 = 600.0;
/// The wall's face at x = FACE; the wall is a `HALF` box behind it.
const FACE: f32 = 0.40;
const HALF: [f32; 3] = [0.05, 4.0, 16.0];
const TOLERANCE: f32 = 0.035;
#[cfg(feature = "rapier")]
const RAPIER_STEPS: usize = 64;
const PHI: f32 = std::f32::consts::GOLDEN_RATIO;

#[derive(Clone, Copy)]
enum Solid {
    Icosahedron,
    Dodecahedron,
}

#[derive(Clone, Copy)]
struct Lane {
    solid: Solid,
    edge: f32,
    /// Axis-angle rotation of the visual from its canonical golden-rectangle frame.
    axis: [f32; 3],
    angle: f32,
    speed: f32,
    tangent: f32,
}

const LANES: [Lane; 4] = [
    Lane {
        solid: Solid::Icosahedron,
        edge: 0.2,
        axis: [0.0, 0.0, 1.0],
        angle: 0.0,
        speed: 45.0,
        tangent: 0.0,
    },
    Lane {
        solid: Solid::Dodecahedron,
        edge: 0.14,
        axis: [0.0, 0.0, 1.0],
        angle: 0.0,
        speed: 45.0,
        tangent: 0.0,
    },
    Lane {
        solid: Solid::Icosahedron,
        edge: 0.2,
        axis: [0.0, 0.0, 1.0],
        angle: 0.5,
        speed: 60.0,
        tangent: 6.0,
    },
    Lane {
        solid: Solid::Dodecahedron,
        edge: 0.14,
        axis: [1.0, 1.0, 0.0],
        angle: 0.9,
        speed: 30.0,
        tangent: -4.0,
    },
];

fn lane_y(k: usize) -> f32 {
    k as f32
}

fn cyclic_signed(v: [f32; 3], out: &mut Vec<[f32; 3]>) {
    for p in [v, [v[2], v[0], v[1]], [v[1], v[2], v[0]]] {
        for sx in [1.0, -1.0] {
            for sy in [1.0, -1.0] {
                for sz in [1.0, -1.0] {
                    let q = [p[0] * sx, p[1] * sy, p[2] * sz];
                    if !out.contains(&q) {
                        out.push(q);
                    }
                }
            }
        }
    }
}

/// The lane's solid vertices, rotated, in metres about the body centre.
fn vertices(lane: Lane) -> Vec<[f32; 3]> {
    let mut raw = Vec::new();
    let raw_edge = match lane.solid {
        Solid::Icosahedron => {
            cyclic_signed([0.0, 1.0, PHI], &mut raw);
            2.0
        }
        Solid::Dodecahedron => {
            for sx in [1.0, -1.0] {
                for sy in [1.0, -1.0] {
                    for sz in [1.0, -1.0] {
                        raw.push([sx, sy, sz]);
                    }
                }
            }
            cyclic_signed([0.0, 1.0 / PHI, PHI], &mut raw);
            2.0 / PHI
        }
    };
    let n = (lane.axis[0].powi(2) + lane.axis[1].powi(2) + lane.axis[2].powi(2)).sqrt();
    let k = [lane.axis[0] / n, lane.axis[1] / n, lane.axis[2] / n];
    let (s, c) = lane.angle.sin_cos();
    raw.iter()
        .map(|&v| {
            let v = [
                v[0] * lane.edge / raw_edge,
                v[1] * lane.edge / raw_edge,
                v[2] * lane.edge / raw_edge,
            ];
            // Rodrigues: v cos + (k × v) sin + k (k·v)(1 − cos).
            let kxv = [
                k[1] * v[2] - k[2] * v[1],
                k[2] * v[0] - k[0] * v[2],
                k[0] * v[1] - k[1] * v[0],
            ];
            let kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
            [0, 1, 2].map(|i| v[i] * c + kxv[i] * s + k[i] * kv * (1.0 - c))
        })
        .collect()
}

/// Circumradius (closed form) and inradius of the lane's solid.
fn radii(lane: Lane) -> (f32, f32) {
    let a = lane.edge;
    match lane.solid {
        Solid::Icosahedron => (
            a / 4.0 * (10.0 + 2.0 * 5f32.sqrt()).sqrt(),
            a * 3f32.sqrt() / 12.0 * (3.0 + 5f32.sqrt()),
        ),
        Solid::Dodecahedron => (
            a * 3f32.sqrt() / 4.0 * (1.0 + 5f32.sqrt()),
            a / 2.0 * (2.5 + 1.1 * 5f32.sqrt()).sqrt(),
        ),
    }
}

fn mass(lane: Lane) -> f32 {
    let r = radii(lane).0;
    4.0 / 3.0 * std::f32::consts::PI * r * r * r * DENSITY
}

/// Analytic elastic reflection off the wall for a body whose support distance towards +x is
/// `support`: its centre x and x velocity after time `t`.
fn reflect(support: f32, speed: f32, t: f32) -> (f32, f32) {
    let contact = FACE - support;
    let s = speed * t;
    if s <= contact {
        (s, speed)
    } else {
        (2.0 * contact - s, -speed)
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "ccd-platonic-proxy",
        capabilities: "C20",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(RAPIER_STEPS, false)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(1, false))),
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

/// Rapier's 64-step reference resolves the impact to within one of its substeps' travel, on
/// top of the shared position tolerance.
fn rapier_tolerance(lane: Lane) -> f32 {
    #[cfg(feature = "rapier")]
    let steps = RAPIER_STEPS as f32;
    #[cfg(not(feature = "rapier"))]
    let steps = 64.0;
    TOLERANCE + lane.speed * DT / steps
}

fn state(trace: &Trace, tick: usize, lane: usize) -> Result<([f32; 3], [f32; 3]), String> {
    let i = (tick * LANES.len() + lane) * 2;
    match (trace.0.get(i), trace.0.get(i + 1)) {
        (Some(Obs::Position(p)), Some(Obs::Velocity(v))) => Ok((*p, *v)),
        other => Err(format!("bad proxy observations at {i}: {other:?}")),
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let expected = TICKS as usize * LANES.len() * 2;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err(format!(
            "unexpected trace lengths: {}, {}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    for (k, &lane) in LANES.iter().enumerate() {
        let (radius, _) = radii(lane);
        // The proxy is an ordinary CCD sphere: it follows the analytic reflection at every
        // tick, never passes the wall and keeps its speed and tangential velocity.
        for tick in 0..TICKS as usize {
            let (x, vx) = reflect(radius, lane.speed, (tick + 1) as f32 * DT);
            let (p, v) = state(engine, tick, k)?;
            if (p[0] - x).abs() > TOLERANCE
                || (v[0] - vx).abs() > 0.02 * lane.speed
                || (v[2] - lane.tangent).abs() > 0.1
                || (p[1] - lane_y(k)).abs() > 1e-3
                || p[0] > FACE - radius + 0.01
            {
                return Err(format!(
                    "lane {k} tick {tick}: engine proxy {p:?}/{v:?}, analytic x={x} vx={vx}"
                ));
            }
        }
        // Sphere-to-sphere parity with Rapier where its 64-step reference is equivalent: the
        // impact tick, as in `ccd-ricochet`.
        let (p, v) = state(engine, 0, k)?;
        let (rp, rv) = state(rapier, 0, k)?;
        for axis in [0, 2] {
            if (p[axis] - rp[axis]).abs() > rapier_tolerance(lane)
                || (v[axis] - rv[axis]).abs() > 0.05 * rv[axis].abs() + 0.5
            {
                return Err(format!(
                    "lane {k} axis {axis}: engine {p:?}/{v:?} vs Rapier sphere {rp:?}/{rv:?}"
                ));
            }
        }
    }
    #[cfg(feature = "rapier")]
    hull_report().map(|_| ())?;
    Ok(())
}

/// One lane's approximation error against Rapier's convex hull of the real solid.
#[derive(Clone, Copy, Debug)]
pub struct HullError {
    pub circumradius: f32,
    pub inradius: f32,
    /// The solid's support distance along the wall normal, in `[r, R]`.
    pub support: f32,
    /// Largest |hull x − analytic hull x| over every tick (Rapier hull vs its closed form).
    pub hull_residual: f32,
    /// Largest |hull x − proxy x| over every tick (the proxy follows its closed form within
    /// `TOLERANCE`; the check holds the engine to it).
    pub error: f32,
    /// The analytic value of `error`: `2 (R − h)` once both have bounced.
    pub expected: f32,
    /// Its bound, `2 (R − r)`.
    pub bound: f32,
    /// Early-contact bias of the impact, in seconds: `(R − h) / speed`.
    pub early: f32,
}

/// Replays every lane on Rapier with a rotation-locked `convex_hull` of the solid and returns
/// the approximation error against the proxy. Fails only when the Rapier hull leaves the
/// analytic reflection at its own support distance or the error exceeds the analytic bound.
#[cfg(feature = "rapier")]
pub fn hull_report() -> Result<Vec<HullError>, String> {
    use crate::Sim as _;
    let mut hull = rapier::Sim::new(RAPIER_STEPS, true);
    let mut trace = Trace::default();
    for tick in 0..TICKS {
        hull.tick(tick, &mut trace);
    }
    let mut out = Vec::new();
    for (k, &lane) in LANES.iter().enumerate() {
        let (big_r, small_r) = radii(lane);
        let support = vertices(lane).iter().map(|v| v[0]).fold(f32::MIN, f32::max);
        if support < small_r - 1e-4 || support > big_r + 1e-4 {
            return Err(format!("lane {k}: support {support} outside [r, R]"));
        }
        let (mut residual, mut error, mut expected) = (0.0f32, 0.0f32, 0.0f32);
        for tick in 0..TICKS as usize {
            let t = (tick + 1) as f32 * DT;
            let (p, _) = state(&trace, tick, k)?;
            let solid = reflect(support, lane.speed, t).0;
            let proxy = reflect(big_r, lane.speed, t).0;
            residual = residual.max((p[0] - solid).abs());
            error = error.max((p[0] - proxy).abs());
            expected = expected.max((solid - proxy).abs());
        }
        let bound = 2.0 * (big_r - small_r);
        if residual > rapier_tolerance(lane) {
            return Err(format!(
                "lane {k}: Rapier convex hull leaves its analytic reflection by {residual}"
            ));
        }
        if error > bound + rapier_tolerance(lane) {
            return Err(format!(
                "lane {k}: hull/proxy error {error} beyond the analytic bound {bound}"
            ));
        }
        out.push(HullError {
            circumradius: big_r,
            inradius: small_r,
            support,
            hull_residual: residual,
            error,
            expected,
            bound,
            early: (big_r - support) / lane.speed,
        });
    }
    Ok(out)
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, BodyStorage, Config, Shape, Vector, World},
    };

    pub struct Sim {
        world: World,
        proxies: Vec<BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector::ZERO,
                substeps: 1,
                velocity_iterations: 8,
                ..Config::default()
            })
            .unwrap();
            let mut wall = Body::new(
                Shape::Box(Vector(HALF[0], HALF[1], HALF[2])),
                Vector(FACE + HALF[0], 1.5, 0.0),
                0.0,
            );
            wall.restitution = 1.0;
            wall.friction = 0.0;
            world.add_body(wall).unwrap();
            let proxies = LANES
                .iter()
                .enumerate()
                .map(|(k, &lane)| {
                    // The consumer renders the solid's mesh and collides its circumsphere.
                    let mut proxy = Body::new(
                        Shape::Sphere(radii(lane).0),
                        Vector(0.0, lane_y(k), 0.0),
                        mass(lane),
                    );
                    proxy.velocity = Vector(lane.speed, 0.0, lane.tangent);
                    proxy.restitution = 1.0;
                    proxy.friction = 0.0;
                    proxy.ccd = true;
                    proxy.sleep_allowed = false;
                    world.add_body(proxy).unwrap()
                })
                .collect();
            Self { world, proxies }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.proxies {
                let b = self.world.body(id).unwrap();
                trace
                    .0
                    .push(Obs::Position([b.position.0, b.position.1, b.position.2]));
                trace
                    .0
                    .push(Obs::Velocity([b.velocity.0, b.velocity.1, b.velocity.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    pub struct Sim {
        world: PhysicsWorld,
        bodies: Vec<RigidBodyHandle>,
        steps: usize,
    }
    impl Sim {
        /// `hull`: the real solid as a rotation-locked convex hull (the approximation
        /// reference) instead of the circumsphere ball.
        pub fn new(steps: usize, hull: bool) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT / steps as f32;
            world.colliders.insert(
                ColliderBuilder::cuboid(HALF[0], HALF[1], HALF[2])
                    .translation(Vector::new(FACE + HALF[0], 1.5, 0.0))
                    .restitution(1.0)
                    .friction(0.0)
                    .build(),
            );
            let bodies = LANES
                .iter()
                .enumerate()
                .map(|(k, &lane)| {
                    let mut builder = RigidBodyBuilder::dynamic()
                        .translation(Vector::new(0.0, lane_y(k), 0.0))
                        .linvel(Vector::new(lane.speed, 0.0, lane.tangent))
                        .ccd_enabled(true)
                        .can_sleep(false);
                    if hull {
                        builder = builder.lock_rotations();
                    }
                    let id = world.bodies.insert(builder.build());
                    let collider = if hull {
                        let points: Vec<Vector> = vertices(lane)
                            .into_iter()
                            .map(|v| Vector::new(v[0], v[1], v[2]))
                            .collect();
                        ColliderBuilder::convex_hull(&points)
                            .expect("a regular solid has a convex hull")
                            .mass(mass(lane))
                    } else {
                        ColliderBuilder::ball(radii(lane).0).density(DENSITY)
                    };
                    world.colliders.insert_with_parent(
                        collider.restitution(1.0).friction(0.0).build(),
                        id,
                        &mut world.bodies,
                    );
                    id
                })
                .collect();
            Self {
                world,
                bodies,
                steps,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            for _ in 0..self.steps {
                self.world.step();
            }
            for &id in &self.bodies {
                let b = &self.world.bodies[id];
                let p = b.translation();
                let v = b.linvel();
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace.0.push(Obs::Velocity([v.x, v.y, v.z]));
            }
        }
    }
}
