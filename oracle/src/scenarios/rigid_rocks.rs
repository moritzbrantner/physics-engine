//! The rigid box contacts needed to retire MMORPG's authored-rock Rapier fallback.
//! Uses the shipped proxy and the game's crate/panel material, gravity and 60 Hz tick.
use crate::{Obs, Scenario, Trace};
const TICKS: u32 = 360;
const DT: f32 = 1.0 / 60.0;
const LANES: usize = 2;
const HALF: [[f32; 3]; LANES] = [[0.35; 3], [0.29, 0.29, 0.03]];
const START: [[f32; 3]; LANES] = [[-3.0, 0.35, 0.0], [-0.3, 3.5, 8.0]];
const VELOCITY: [[f32; 3]; LANES] = [[5.91, 0.0, 0.0], [0.4, -1.0, 0.2]];
fn rock(lane: usize) -> [f32; 3] {
    [0.0, 1.1, lane as f32 * 8.0]
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "greyhaven-rigid-rocks",
        capabilities: "C18",
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
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    crate::finite(engine)?;
    crate::finite(rapier)?;
    let expected = TICKS as usize * LANES * 3;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err("rigid-rock trace has the wrong observation count".into());
    }
    #[cfg(feature = "rapier")]
    let reference_depths: [f32; LANES] = std::array::from_fn(|lane| {
        rapier
            .0
            .as_chunks::<3>()
            .0
            .iter()
            .skip(lane)
            .step_by(LANES)
            .map(|state| match (state[0], state[2]) {
                (Obs::Position(p), Obs::Rotation(q)) => rapier::separation(lane, p, q),
                _ => f32::NEG_INFINITY,
            })
            .fold(0.0_f32, f32::min)
    });
    for (name, trace) in [("engine", engine), ("Rapier", rapier)] {
        #[cfg(feature = "rapier")]
        for (i, observation) in trace.0.as_chunks::<3>().0.iter().enumerate() {
            let (Obs::Position(p), Obs::Rotation(q)) = (observation[0], observation[2]) else {
                return Err("malformed rigid-rock observation".into());
            };
            let lane = i % LANES;
            let reference_depth = reference_depths[lane];
            let separation = rapier::separation(lane, p, q);
            if separation < reference_depth - 0.005 || separation < -0.05 {
                return Err(format!(
                    "{name} penetrates authored rock or floor by {} m at observation {i}, reference {}",
                    -separation, -reference_depth
                ));
            }
        }
        let last = (TICKS as usize - 1) * LANES * 3;
        let Obs::Position(crate_at) = trace.0[last] else {
            unreachable!()
        };
        if crate_at[0] >= -1.0 || crate_at[0] <= START[0][0] + 0.2 {
            return Err(format!(
                "{name} crate must advance and stop at the rock: {crate_at:?}"
            ));
        }
        for lane in 0..LANES {
            let Obs::Velocity(velocity) = trace.0[last + lane * 3 + 1] else {
                unreachable!()
            };
            if velocity.iter().map(|v| v * v).sum::<f32>().sqrt() > 0.18 {
                return Err(format!("{name} lane {lane} did not settle: {velocity:?}"));
            }
        }
    }
    let last = (TICKS as usize - 1) * LANES * 3;
    for lane in 0..LANES {
        let Obs::Position(a) = engine.0[last + lane * 3] else {
            unreachable!()
        };
        let Obs::Position(b) = rapier.0[last + lane * 3] else {
            unreachable!()
        };
        let distance = a
            .into_iter()
            .zip(b)
            .map(|(x, y)| (x - y).powi(2))
            .sum::<f32>()
            .sqrt();
        // Existing props/debris oracle tolerances, applied to these real hull contacts.
        let bound = if lane == 0 {
            0.05 * (b[0] - START[0][0]).abs() + 0.01
        } else {
            1.5
        };
        if distance > bound {
            return Err(format!(
                "lane {lane} rest differs by {distance} m (bound {bound})"
            ));
        }
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::approximate::{
        Body, BodyStorage, Config, ConvexHullDesc, Material, Shape, Vector as V, World,
    };
    pub struct Sim {
        world: World,
        boxes: Vec<physics_engine::BodyId>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: V(0.0, -9.0, 0.0),
                substeps: 2,
                ..Config::default()
            })
            .unwrap();
            world
                .add_body(Body::new(Shape::HalfSpace, V::ZERO, 0.0))
                .unwrap();
            for lane in 0..LANES {
                world
                    .add_convex_hull(
                        ConvexHullDesc {
                            position: v(rock(lane)),
                            vertices: crate::greyhaven_fixture::vertices()
                                .into_iter()
                                .map(v)
                                .collect(),
                            material: Material::default(),
                        },
                        Some(lane as u64 + 1),
                    )
                    .unwrap();
            }
            let boxes = (0..LANES)
                .map(|lane| {
                    let h = HALF[lane];
                    let mut body = Body::new(
                        Shape::Box(v(h)),
                        v(START[lane]),
                        600.0 * 8.0 * h[0] * h[1] * h[2],
                    );
                    body.velocity = v(VELOCITY[lane]);
                    body.friction = 0.8;
                    body.restitution = 0.15;
                    body.linear_damping = 0.2;
                    body.angular_damping = 0.8;
                    body.rotation_locked = lane == 0;
                    body.sleep_allowed = false;
                    if lane == 1 {
                        body.angular_velocity = V(0.7, 0.3, -0.5);
                    }
                    world.add_body(body).unwrap()
                })
                .collect();
            Self { world, boxes }
        }
    }
    fn v(p: [f32; 3]) -> V {
        V(p[0], p[1], p[2])
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.world
                .step(f64::from(DT))
                .unwrap_or_else(|e| panic!("rigid rock tick {tick}: {e:?}"));
            for id in &self.boxes {
                let body = self.world.body(*id).unwrap();
                let p = body.position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Velocity([
                    body.velocity.0,
                    body.velocity.1,
                    body.velocity.2,
                ]));
                let q = body.orientation;
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
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
        boxes: Vec<RigidBodyHandle>,
    }
    fn hull() -> SharedShape {
        SharedShape::convex_hull(
            &crate::greyhaven_fixture::vertices()
                .into_iter()
                .map(v)
                .collect::<Vec<_>>(),
        )
        .unwrap()
    }
    fn v(p: [f32; 3]) -> Vector {
        Vector::new(p[0], p[1], p[2])
    }
    pub fn separation(lane: usize, p: [f32; 3], q: [f32; 4]) -> f32 {
        let shape = Cuboid::new(v(HALF[lane]));
        let pose = Pose::from_parts(v(p), Rotation::from_xyzw(q[0], q[1], q[2], q[3]));
        let rock = rapier3d::parry::query::contact(
            &pose,
            &shape,
            &Pose::from_translation(v(rock(lane))),
            hull().as_ref(),
            0.0,
        )
        .unwrap();
        let floor = rapier3d::parry::query::contact(
            &Pose::IDENTITY,
            &HalfSpace::new(Vector::Y),
            &pose,
            &shape,
            0.0,
        )
        .unwrap();
        [rock, floor]
            .into_iter()
            .flatten()
            .map(|c| c.dist)
            .fold(0.0_f32, f32::min)
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -9.0, 0.0);
            world.integration_parameters.dt = DT;
            world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            for lane in 0..LANES {
                world.colliders.insert(
                    ColliderBuilder::new(hull())
                        .translation(v(rock(lane)))
                        .build(),
                );
            }
            let boxes = (0..LANES)
                .map(|lane| {
                    let mut builder = RigidBodyBuilder::dynamic()
                        .translation(v(START[lane]))
                        .linvel(v(VELOCITY[lane]))
                        .linear_damping(0.2)
                        .angular_damping(0.8)
                        .can_sleep(false);
                    if lane == 0 {
                        builder = builder.lock_rotations();
                    } else {
                        builder = builder.angvel(Vector::new(0.7, 0.3, -0.5));
                    }
                    let id = world.bodies.insert(builder.build());
                    let h = HALF[lane];
                    world.colliders.insert_with_parent(
                        ColliderBuilder::cuboid(h[0], h[1], h[2])
                            .density(600.0)
                            .friction(0.8)
                            .restitution(0.15)
                            .build(),
                        id,
                        &mut world.bodies,
                    );
                    id
                })
                .collect();
            Self { world, boxes }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _: u32, trace: &mut Trace) {
            self.world.step();
            for id in &self.boxes {
                let body = &self.world.bodies[*id];
                let p = body.translation();
                let q = body.rotation().normalize().to_array();
                trace.0.push(Obs::Position(p.to_array()));
                trace.0.push(Obs::Velocity(body.linvel().to_array()));
                trace.0.push(Obs::Rotation(q));
            }
        }
    }
}

#[cfg(all(test, feature = "rapier"))]
mod tests {
    use super::*;
    #[test]
    fn rapier_reference_exercises_the_real_rigid_rock_contacts() {
        let s = scenario();
        let mut trace = Trace::default();
        s.run(&mut rapier::Sim::new(), &mut trace);
        check(&trace, &trace).unwrap();
    }
}
