//! C25 fluids (#525): MMORPG's barrels, crates and logs dropped into the Greyhaven brook
//! (a current) or Stillwater marsh (still water). Physics Engine runs its `Medium`; Rapier has no
//! buoyancy, so its side applies the same force through `add_force`: the volume and centroid
//! below the surface are clipped from parry's own tessellation of each collider (`to_trimesh`,
//! rescaled to the exact volume) with independent drag, and the same implicit drag as a force.
//! Rapier rebuilds the world per tick as MMORPG's response lanes do; the raw variant keeps it.
//!
//! Invariants, both traces: positions within [`BOUND`] of Rapier's every tick; a floater's
//! final depth within 2 cm of its Archimedes depth and settled within 90 ticks of Rapier's;
//! nothing sinks into the bed; in the brook floaters reach the flow, in the marsh every body
//! sleeps (the engine reports a zero velocity).

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 900;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const WATER: f32 = 1000.0;
const BED: f32 = -3.0;
/// Worst position difference from Rapier: Rapier samples its force once per tick and rebuilds
/// its world, so a sinking sphere lags (measured 0.17 m, the marsh sinker; every other lane within
/// 0.04 m).
const BOUND: f32 = 0.25;
/// Worst final depth difference from the closed form and from Rapier (measured 2 mm).
const DEPTH_TOLERANCE: f32 = 0.02;
/// Largest settle-tick difference from Rapier (measured 38 ticks, the brook crates).
const SETTLE_TICKS: u32 = 60;
/// The same for the light buoys, whose lightly damped bob Rapier's once-per-tick force keeps
/// up longer (measured 201 ticks).
const BUOY_SETTLE_TICKS: u32 = 240;

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Crate,
    Barrel,
    Log,
    Sinker,
    Buoy,
}

#[derive(Clone, Copy)]
struct Water {
    flow: [f32; 3],
    linear_drag: f32,
    angular_drag: f32,
}

const BROOK: Water = Water {
    flow: [1.2, 0.0, 0.0],
    linear_drag: 2.5,
    angular_drag: 2.5,
};
const MARSH: Water = Water {
    flow: [0.0, 0.0, 0.0],
    linear_drag: 5.0,
    angular_drag: 5.0,
};

const KINDS: [Kind; 5] = [
    Kind::Crate,
    Kind::Barrel,
    Kind::Log,
    Kind::Sinker,
    Kind::Buoy,
];
const LANES: usize = KINDS.len() * 3;

/// Half-extents or `[half_segment, radius, 0]`.
const CRATE_HALF: f32 = 0.35;
const BARREL: [f32; 2] = [0.3, 0.3];
const LOG: [f32; 2] = [0.8, 0.2];
const SINKER: f32 = 0.25;
const BUOY_HALF: f32 = 0.3;

const fn kind(l: usize) -> Kind {
    KINDS[l % KINDS.len()]
}
const fn density(kind: Kind) -> f32 {
    match kind {
        Kind::Crate => 600.0,
        Kind::Barrel => 500.0,
        Kind::Log => 650.0,
        Kind::Sinker => 2500.0,
        Kind::Buoy => 300.0,
    }
}
/// Rotation locked: mid-density crates, barrels and buoys would tip over, and the invariants
/// compare their depth with Archimedes' upright depth.
const fn locked(kind: Kind) -> bool {
    matches!(kind, Kind::Crate | Kind::Barrel | Kind::Buoy)
}
/// Starting tilt about +Z for the free logs, so their torque has work to do.
fn tilt(kind: Kind) -> f32 {
    match kind {
        Kind::Log => 0.2,
        _ => 0.0,
    }
}
fn start(l: usize) -> [f32; 3] {
    // The light buoys are laid on the water: buoyancy at three times gravity sampled once per
    // Rapier tick would fling them far apart from a real plunge.
    let height = if kind(l) == Kind::Buoy {
        0.35
    } else {
        1.0 + 0.5 * (l % 3) as f32
    };
    [0.0, height, 4.0 * l as f32]
}
fn volume(kind: Kind) -> f32 {
    use std::f32::consts::PI;
    match kind {
        Kind::Crate => 8.0 * CRATE_HALF.powi(3),
        Kind::Barrel => {
            let [h, r] = BARREL;
            PI * r * r * 2.0 * h + 4.0 / 3.0 * PI * r * r * r
        }
        Kind::Log => {
            let [h, r] = LOG;
            PI * r * r * 2.0 * h + 4.0 / 3.0 * PI * r * r * r
        }
        Kind::Sinker => 4.0 / 3.0 * PI * SINKER.powi(3),
        Kind::Buoy => 8.0 * BUOY_HALF.powi(3),
    }
}
fn mass(kind: Kind) -> f32 {
    density(kind) * volume(kind)
}
/// The Archimedes rest height of a floater's centre, or `None` where no closed form applies.
fn archimedes(kind: Kind) -> Option<f32> {
    let box_rest = |half: f32, d: f32| half - 2.0 * half * d / WATER;
    match kind {
        Kind::Crate => Some(box_rest(CRATE_HALF, density(kind))),
        Kind::Buoy => Some(box_rest(BUOY_HALF, density(kind))),
        // Half density, symmetric: the centre rides on the surface.
        Kind::Barrel => Some(0.0),
        Kind::Log | Kind::Sinker => None,
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "float-brook",
        capabilities: "C25",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage, BROOK)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(BROOK, true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(BROOK, false))),
        check: |engine, rapier| check(engine, rapier, BROOK),
    }
}

pub fn scenario_marsh() -> Scenario {
    Scenario {
        name: "float-marsh",
        capabilities: "C25",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage, MARSH)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(MARSH, true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(MARSH, false))),
        check: |engine, rapier| check(engine, rapier, MARSH),
    }
}

/// Per tick per lane: `Position`, `Velocity`.
fn sample(trace: &Trace, tick: usize, lane: usize) -> ([f32; 3], [f32; 3]) {
    let base = (tick * LANES + lane) * 2;
    match (trace.0[base], trace.0[base + 1]) {
        (Obs::Position(p), Obs::Velocity(v)) => (p, v),
        other => panic!("unexpected observations {other:?}"),
    }
}

/// The first tick from which the lane stays within 2 cm of its final height.
fn settle_tick(trace: &Trace, lane: usize) -> u32 {
    let last = TICKS as usize - 1;
    let end = sample(trace, last, lane).0[1];
    (0..=last)
        .rev()
        .find(|&tick| (sample(trace, tick, lane).0[1] - end).abs() > 0.02)
        .map_or(0, |tick| tick as u32 + 1)
}

fn check(engine: &Trace, rapier: &Trace, water: Water) -> Result<(), String> {
    if engine.0.len() != rapier.0.len() || engine.0.len() != TICKS as usize * LANES * 2 {
        return Err("trace lengths differ".into());
    }
    let last = TICKS as usize - 1;
    let flowing = water.flow != [0.0, 0.0, 0.0];
    for lane in 0..LANES {
        let k = kind(lane);
        for tick in 0..TICKS as usize {
            let (e, _) = sample(engine, tick, lane);
            let (r, _) = sample(rapier, tick, lane);
            let gap = (0..3).map(|a| (e[a] - r[a]).powi(2)).sum::<f32>().sqrt();
            if gap > BOUND {
                return Err(format!(
                    "lane {lane} tick {tick}: {gap:.3} m from Rapier ({e:?} vs {r:?})"
                ));
            }
            if k == Kind::Sinker && e[1] < BED + SINKER - 0.02 {
                return Err(format!(
                    "lane {lane} tick {tick}: under the bed at {}",
                    e[1]
                ));
            }
        }
        let (end, velocity) = sample(engine, last, lane);
        if let Some(depth) = archimedes(k)
            && (end[1] - depth).abs() > DEPTH_TOLERANCE
        {
            return Err(format!("lane {lane}: rest height {} vs {depth}", end[1]));
        }
        let rapier_end = sample(rapier, last, lane).0[1];
        if k != Kind::Log && (end[1] - rapier_end).abs() > DEPTH_TOLERANCE {
            return Err(format!(
                "lane {lane}: engine rests at {}, Rapier at {rapier_end}",
                end[1]
            ));
        }
        let (se, sr) = (settle_tick(engine, lane), settle_tick(rapier, lane));
        let settle_bound = if k == Kind::Buoy {
            BUOY_SETTLE_TICKS
        } else {
            SETTLE_TICKS
        };
        if se.abs_diff(sr) > settle_bound {
            return Err(format!("lane {lane}: settles at tick {se}, Rapier at {sr}"));
        }
        if flowing {
            if k != Kind::Sinker && (velocity[0] - water.flow[0]).abs() > 0.05 * water.flow[0] {
                return Err(format!("lane {lane}: drifts at {velocity:?}"));
            }
        } else if velocity != [0.0, 0.0, 0.0] {
            return Err(format!(
                "lane {lane}: still moving {velocity:?} in the marsh"
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
        approximate::{Body, BodyStorage, Config, Medium, Quaternion, Shape, Vector, World},
    };

    pub struct Sim {
        world: World,
        ids: Vec<BodyId>,
    }

    fn orientation(k: Kind) -> Quaternion {
        // The log's capsule lies along Z.
        let (mut x, mut w) = (0.0, 1.0);
        if k == Kind::Log {
            let half = std::f32::consts::FRAC_PI_4;
            (x, w) = (half.sin(), half.cos());
        }
        let half = tilt(k) * 0.5;
        let (z, wz) = (half.sin(), half.cos());
        // q_z * q_x, the same composition as Rapier's side.
        Quaternion(wz * x, z * x, z * w, wz * w).normalized()
    }

    impl Sim {
        pub fn new(storage: BodyStorage, water: Water) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::new(Shape::HalfSpace, Vector(0.0, BED, 0.0), 0.0))
                .unwrap();
            let medium = Medium {
                surface_point: Vector::ZERO,
                surface_normal: Vector(0.0, 1.0, 0.0),
                density: WATER,
                flow: Vector(water.flow[0], water.flow[1], water.flow[2]),
                linear_drag: water.linear_drag,
                angular_drag: water.angular_drag,
            };
            let ids = (0..LANES)
                .map(|l| {
                    let k = kind(l);
                    let shape = match k {
                        Kind::Crate => Shape::Box(Vector(CRATE_HALF, CRATE_HALF, CRATE_HALF)),
                        Kind::Barrel => Shape::capsule(BARREL[0], BARREL[1]),
                        Kind::Log => Shape::capsule(LOG[0], LOG[1]),
                        Kind::Sinker => Shape::Sphere(SINKER),
                        Kind::Buoy => Shape::Box(Vector(BUOY_HALF, BUOY_HALF, BUOY_HALF)),
                    };
                    let [x, y, z] = start(l);
                    let mut body = Body::new(shape, Vector(x, y, z), mass(k));
                    body.orientation = orientation(k);
                    body.rotation_locked = locked(k);
                    let id = world.add_body(body).unwrap();
                    world.set_medium(id, Some(medium)).unwrap();
                    id
                })
                .collect();
            Self { world, ids }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            self.world.step(f64::from(DT)).unwrap();
            for &id in &self.ids {
                let body = self.world.body(id).unwrap();
                let (p, v) = (body.position, body.velocity);
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Velocity([v.0, v.1, v.2]));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::shape::{Ball, Capsule, Cuboid};
    use rapier3d::prelude::*;

    /// A collider's tessellation in its local frame, rescaled to the exact volume.
    struct Hull {
        vertices: Vec<Vector>,
        triangles: Vec<[u32; 3]>,
    }

    fn signed_volume(a: Vector, b: Vector, c: Vector, apex: Vector) -> f32 {
        (a - apex).dot((b - apex).cross(c - apex)) / 6.0
    }

    /// Scales `vertices` so their volume is exact (a tessellated round shape is a few percent
    /// short); the rest of the geometry stays parry's.
    fn hull(kind: Kind) -> Hull {
        let (vertices, triangles) = match kind {
            Kind::Crate => Cuboid::new(Vector::splat(CRATE_HALF)).to_trimesh(),
            Kind::Buoy => Cuboid::new(Vector::splat(BUOY_HALF)).to_trimesh(),
            Kind::Barrel => Capsule::new_y(BARREL[0], BARREL[1]).to_trimesh(16, 8),
            Kind::Log => Capsule::new_y(LOG[0], LOG[1]).to_trimesh(16, 8),
            Kind::Sinker => Ball::new(SINKER).to_trimesh(16, 16),
        };
        let mesh: f32 = triangles
            .iter()
            .map(|&[a, b, c]| {
                signed_volume(
                    vertices[a as usize],
                    vertices[b as usize],
                    vertices[c as usize],
                    Vector::ZERO,
                )
            })
            .sum();
        let scale = (volume(kind) / mesh).cbrt();
        Hull {
            vertices: vertices.iter().map(|v| *v * scale).collect(),
            triangles,
        }
    }

    /// Volume and centroid below `y = 0` of `hull` at `pose`: the surface triangles clipped to
    /// the plane, each with a tetrahedron from an apex on the plane so the cap contributes
    /// nothing.
    fn submerged(hull: &Hull, pose: &Pose, points: &mut Vec<Vector>) -> (f32, Vector) {
        let centre = pose.translation;
        let apex = Vector::new(centre.x, 0.0, centre.z);
        points.clear();
        points.extend(hull.vertices.iter().map(|v| *pose * *v));
        let (mut total, mut moment) = (0.0, Vector::ZERO);
        let mut add = |a: Vector, b: Vector, c: Vector| {
            let v = signed_volume(a, b, c, apex);
            total += v;
            moment += (a + b + c + apex) * (v * 0.25);
        };
        for &[a, b, c] in &hull.triangles {
            let t = [points[a as usize], points[b as usize], points[c as usize]];
            let mut poly: [Vector; 6] = [Vector::ZERO; 6];
            let mut n = 0;
            for i in 0..3 {
                let (p, q) = (t[i], t[(i + 1) % 3]);
                if p.y <= 0.0 {
                    poly[n] = p;
                    n += 1;
                }
                if (p.y <= 0.0) != (q.y <= 0.0) {
                    poly[n] = p + (q - p) * (p.y / (p.y - q.y));
                    n += 1;
                }
            }
            for i in 1..n.saturating_sub(1) {
                add(poly[0], poly[i], poly[i + 1]);
            }
        }
        if total > 0.0 {
            (total, moment / total)
        } else {
            (0.0, centre)
        }
    }

    struct State {
        kind: Kind,
        hull: Hull,
        pose: Pose,
        velocity: Vector,
        spin: Vector,
        handle: Option<RigidBodyHandle>,
    }

    pub struct Sim {
        world: PhysicsWorld,
        states: Vec<State>,
        water: Water,
        rebuild: bool,
        /// Reused for the posed vertices, so the parry clip allocates nothing per body.
        points: Vec<Vector>,
    }

    fn statics(world: &mut PhysicsWorld) {
        world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
        world.integration_parameters.dt = DT;
        world.integration_parameters.num_solver_iterations = 2;
        world.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                .translation(Vector::new(0.0, BED, 0.0))
                .build(),
        );
    }

    fn collider(kind: Kind) -> ColliderBuilder {
        match kind {
            Kind::Crate => ColliderBuilder::cuboid(CRATE_HALF, CRATE_HALF, CRATE_HALF),
            Kind::Buoy => ColliderBuilder::cuboid(BUOY_HALF, BUOY_HALF, BUOY_HALF),
            Kind::Barrel => ColliderBuilder::capsule_y(BARREL[0], BARREL[1]),
            Kind::Log => ColliderBuilder::capsule_y(LOG[0], LOG[1]),
            Kind::Sinker => ColliderBuilder::ball(SINKER),
        }
        .density(density(kind))
    }

    fn insert(world: &mut PhysicsWorld, s: &State) -> RigidBodyHandle {
        let mut builder = RigidBodyBuilder::dynamic()
            .pose(s.pose)
            .linvel(s.velocity)
            .angvel(s.spin)
            .can_sleep(false);
        if locked(s.kind) {
            builder = builder.lock_rotations();
        }
        let handle = world.bodies.insert(builder.build());
        world
            .colliders
            .insert_with_parent(collider(s.kind).build(), handle, &mut world.bodies);
        handle
    }

    impl Sim {
        pub fn new(water: Water, rebuild: bool) -> Self {
            let mut world = PhysicsWorld::new();
            statics(&mut world);
            let mut states: Vec<State> = (0..LANES)
                .map(|l| {
                    let k = kind(l);
                    let z = Rotation::from_rotation_z(tilt(k));
                    let rotation = if k == Kind::Log {
                        z * Rotation::from_rotation_x(std::f32::consts::FRAC_PI_2)
                    } else {
                        z
                    };
                    let [x, y, w] = start(l);
                    State {
                        kind: k,
                        hull: hull(k),
                        pose: Pose::from_parts(Vector::new(x, y, w), rotation),
                        velocity: Vector::ZERO,
                        spin: Vector::ZERO,
                        handle: None,
                    }
                })
                .collect();
            if !rebuild {
                for s in &mut states {
                    s.handle = Some(insert(&mut world, s));
                }
            }
            Self {
                world,
                states,
                water,
                rebuild,
                points: Vec::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            if self.rebuild {
                let mut world = PhysicsWorld::new();
                statics(&mut world);
                for s in &mut self.states {
                    s.handle = Some(insert(&mut world, s));
                }
                self.world = world;
            }
            let flow = Vector::new(self.water.flow[0], self.water.flow[1], self.water.flow[2]);
            for s in &self.states {
                let (below, centroid) = submerged(&s.hull, &s.pose, &mut self.points);
                let fraction = below / volume(s.kind);
                let body = &mut self.world.bodies[s.handle.unwrap()];
                body.reset_forces(false);
                body.reset_torques(false);
                let mass = mass(s.kind);
                // The engine's implicit drag `u + (v - u) / (1 + dt k phi)` over this tick.
                let rate = self.water.linear_drag * fraction;
                let drag = (flow - s.velocity) * (mass * rate / (1.0 + DT * rate));
                let lift = Vector::new(0.0, WATER * below * GRAVITY, 0.0);
                body.add_force(lift + drag, false);
                if !locked(s.kind) {
                    let arm = centroid - s.pose.translation;
                    body.add_torque(arm.cross(lift), false);
                }
                body.set_angular_damping(self.water.angular_drag * fraction);
            }
            self.world.step();
            for s in &mut self.states {
                let body = &self.world.bodies[s.handle.unwrap()];
                s.pose = *body.position();
                s.velocity = body.linvel();
                s.spin = body.angvel();
                let p = s.pose.translation;
                trace.0.push(Obs::Position([p.x, p.y, p.z]));
                trace
                    .0
                    .push(Obs::Velocity([s.velocity.x, s.velocity.y, s.velocity.z]));
            }
        }
    }
}
