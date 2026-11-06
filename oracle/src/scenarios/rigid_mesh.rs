//! The rigid box contacts with the actual authored Redbrand Cave surfaces (#182).
//! Uses the shipped cave and the game's crate/panel material, gravity and 60 Hz tick.
//! Measures intended authored adapter body churn in retained isolated static worlds on both
//! engines; bare Rapier additionally retains response across ticks. Game adoption is separate.
use crate::{Obs, Scenario, Trace};
const TICKS: u32 = 360;
const DT: f32 = 1.0 / 60.0;
const LANES: usize = 2;
const HALF: [[f32; 3]; LANES] = [[0.35; 3], [0.29, 0.29, 0.03]];
const START: [[f32; 3]; LANES] = [[50.0, 11.35, -138.0], [62.0, 14.0, -138.0]];
const VELOCITY: [[f32; 3]; LANES] = [[-5.91, 0.0, 0.0], [0.4, -1.0, 0.2]];

pub fn scenario() -> Scenario {
    Scenario {
        name: "redbrand-rigid-mesh",
        capabilities: "C23",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check,
    }
}
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    crate::finite(engine)?;
    crate::finite(rapier)?;
    let expected = TICKS as usize * LANES * 3;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err("rigid-mesh trace has the wrong observation count".into());
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
                return Err("malformed rigid-mesh observation".into());
            };
            let lane = i % LANES;
            let reference_depth = reference_depths[lane];
            let separation = rapier::separation(lane, p, q);
            if separation < reference_depth - 0.005 || separation < -0.05 {
                return Err(format!(
                    "{name} penetrates authored cave surface by {} m at observation {i}, reference {}",
                    -separation, -reference_depth
                ));
            }
        }
        let last = (TICKS as usize - 1) * LANES * 3;
        let Obs::Position(crate_at) = trace.0[last] else {
            unreachable!()
        };
        if crate_at[0] < 48.34 || crate_at[0] > START[0][0] - 0.2 {
            return Err(format!(
                "{name} crate must advance and stop at the cave wall: {crate_at:?}"
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
        // Existing props/debris oracle tolerances, applied to these real cave contacts.
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
    use physics_engine::approximate::{Body, BodyStorage, Config, Shape, Vector as V, World};
    pub struct Sim {
        templates: Vec<World>,
        boxes: Vec<Option<Body>>,
        static_bodies: usize,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let templates: Vec<World> = (0..LANES)
                .map(|lane| {
                    let mut world = World::new(Config {
                        body_storage: storage,
                        gravity: V(0.0, -9.0, 0.0),
                        substeps: 2,
                        velocity_iterations: if lane == 0 { 1 } else { 4 },
                        ..Config::default()
                    })
                    .unwrap();
                    for chunk in super::super::redbrand_cave::chunks() {
                        world
                            .add_triangle_mesh(
                                physics_engine::approximate::TriangleMeshDesc {
                                    position: v(chunk.position),
                                    vertices: chunk.vertices.into_iter().map(v).collect(),
                                    triangles: chunk.triangles,
                                    material: Default::default(),
                                },
                                Some(chunk.key),
                            )
                            .unwrap();
                    }
                    world
                })
                .collect();
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
                    Some(body)
                })
                .collect();
            let static_bodies = templates[0].body_count();
            Self {
                templates,
                boxes,
                static_bodies,
            }
        }
    }
    fn v(p: [f32; 3]) -> V {
        V(p[0], p[1], p[2])
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            // Intended authored adapter: retained static world, transient response bodies.
            // No mutable character state or dynamic bodies survive between game calls.
            for (world, state) in self.templates.iter_mut().zip(&mut self.boxes) {
                let id = world.add_body(state.take().unwrap()).unwrap();
                world
                    .step(f64::from(DT))
                    .unwrap_or_else(|e| panic!("rigid mesh tick {tick}: {e:?}"));
                let body = world.remove_body(id).unwrap();
                let p = body.position;
                trace.0.push(Obs::Position([p.0, p.1, p.2]));
                trace.0.push(Obs::Velocity([
                    body.velocity.0,
                    body.velocity.1,
                    body.velocity.2,
                ]));
                let q = body.orientation;
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
                assert_eq!(world.body_count(), self.static_bodies);
                *state = Some(body);
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;
    pub struct Sim {
        worlds: Vec<PhysicsWorld>,
        boxes: Vec<RigidBodyHandle>,
        rebuild: bool,
    }
    fn v(p: [f32; 3]) -> Vector {
        Vector::new(p[0], p[1], p[2])
    }
    pub fn separation(lane: usize, p: [f32; 3], q: [f32; 4]) -> f32 {
        let shape = Cuboid::new(v(HALF[lane]));
        let pose = Pose::from_parts(v(p), Rotation::from_xyzw(q[0], q[1], q[2], q[3]));
        super::super::redbrand_cave::chunks()
            .into_iter()
            .filter_map(|chunk| {
                let mesh = SharedShape::trimesh(
                    chunk.vertices.into_iter().map(v).collect(),
                    chunk.triangles,
                )
                .unwrap();
                rapier3d::parry::query::contact(
                    &pose,
                    &shape,
                    &Pose::from_translation(v(chunk.position)),
                    mesh.as_ref(),
                    0.0,
                )
                .unwrap()
                .map(|c| c.dist)
            })
            .fold(0.0_f32, f32::min)
    }
    fn geometry() -> Vec<(Vector, SharedShape)> {
        super::super::redbrand_cave::chunks()
            .into_iter()
            .map(|chunk| {
                (
                    v(chunk.position),
                    SharedShape::trimesh(
                        chunk.vertices.into_iter().map(v).collect(),
                        chunk.triangles,
                    )
                    .unwrap(),
                )
            })
            .collect()
    }
    fn world(
        geometry: &[(Vector, SharedShape)],
        lane: usize,
        pose: Pose,
        velocity: Vector,
        spin: Vector,
    ) -> (PhysicsWorld, RigidBodyHandle) {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::new(0.0, -9.0, 0.0);
        world.integration_parameters.dt = DT;
        for (position, shape) in geometry {
            world.colliders.insert(
                ColliderBuilder::new(shape.clone())
                    .translation(*position)
                    .build(),
            );
        }
        let id = add_body(&mut world, lane, pose, velocity, spin);
        (world, id)
    }
    fn add_body(
        world: &mut PhysicsWorld,
        lane: usize,
        pose: Pose,
        velocity: Vector,
        spin: Vector,
    ) -> RigidBodyHandle {
        let mut builder = RigidBodyBuilder::dynamic()
            .pose(pose)
            .linvel(velocity)
            .angvel(spin)
            .linear_damping(0.2)
            .angular_damping(0.8)
            .can_sleep(false);
        if lane == 0 {
            builder = builder.lock_rotations();
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
    }
    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let geometry = geometry();
            let (worlds, boxes) = (0..LANES)
                .map(|lane| {
                    world(
                        &geometry,
                        lane,
                        Pose::from_translation(v(START[lane])),
                        v(VELOCITY[lane]),
                        if lane == 1 {
                            Vector::new(0.7, 0.3, -0.5)
                        } else {
                            Vector::ZERO
                        },
                    )
                })
                .unzip();
            Self {
                worlds,
                boxes,
                rebuild,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, _: u32, trace: &mut Trace) {
            for lane in 0..LANES {
                if self.rebuild {
                    let body = &self.worlds[lane].bodies[self.boxes[lane]];
                    let (pose, velocity, spin) = (*body.position(), body.linvel(), body.angvel());
                    self.worlds[lane].remove_body(self.boxes[lane]).unwrap();
                    self.boxes[lane] = add_body(&mut self.worlds[lane], lane, pose, velocity, spin);
                }
                self.worlds[lane].step();
                let body = &self.worlds[lane].bodies[self.boxes[lane]];
                trace.0.push(Obs::Position(body.translation().to_array()));
                trace.0.push(Obs::Velocity(body.linvel().to_array()));
                trace
                    .0
                    .push(Obs::Rotation(body.rotation().normalize().to_array()));
            }
        }
    }
}

#[cfg(all(test, feature = "rapier"))]
mod tests {
    use super::*;
    #[test]
    fn rapier_reference_exercises_the_real_rigid_mesh_contacts() {
        let s = scenario();
        let mut trace = Trace::default();
        s.run(&mut rapier::Sim::new(true), &mut trace);
        check(&trace, &trace).unwrap();
    }
}
