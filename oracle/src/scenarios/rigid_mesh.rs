//! Crates and tumbling debris on the actual Redbrand cave floor and upper shelf (#182).
use crate::{Obs, Scenario, Trace};
const TICKS: u32 = 360;
const DT: f32 = 1.0 / 60.0;
const LANES: usize = 3;
const HALF: [[f32; 3]; LANES] = [[0.35; 3], [0.35; 3], [0.29, 0.29, 0.03]];
const START: [[f32; 3]; LANES] = [
    [62.0, 12.0, -138.0],
    [62.0, 14.0, -143.0],
    [64.0, 13.0, -138.0],
];
const VELOCITY: [[f32; 3]; LANES] = [[0.6, 0.0, 0.0], [0.2, 0.0, 0.0], [0.4, -1.0, 0.2]];
pub fn scenario() -> Scenario {
    Scenario {
        name: "redbrand-rigid-mesh",
        capabilities: "C23",
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
fn check(engine: &Trace, reference: &Trace) -> Result<(), String> {
    let expected = TICKS as usize * LANES * 3;
    #[cfg(feature = "rapier")]
    let surfaces = rapier::surfaces();
    for (name, trace) in [("engine", engine), ("Rapier", reference)] {
        crate::finite(trace)?;
        if trace.0.len() != expected {
            return Err("rigid mesh trace count".into());
        }
        #[cfg(feature = "rapier")]
        for (i, state) in trace.0.as_chunks::<3>().0.iter().enumerate() {
            let (Obs::Position(p), Obs::Rotation(q)) = (state[0], state[2]) else {
                return Err("rigid mesh observation".into());
            };
            let separation = rapier::separation(&surfaces, i % LANES, p, q);
            if separation < -0.02 {
                return Err(format!(
                    "{name} mesh penetration {separation} at observation {i}"
                ));
            }
        }
        let end = (TICKS as usize - 1) * LANES * 3;
        for (lane, half) in HALF.iter().enumerate() {
            let (Obs::Position(p), Obs::Velocity(v)) =
                (trace.0[end + lane * 3], trace.0[end + lane * 3 + 1])
            else {
                return Err("rigid mesh end observation".into());
            };
            let floor = if lane == 1 { 13.2 } else { 11.0 };
            if lane < 2 && (p[1] - floor - half[1]).abs() > 0.02 {
                return Err(format!(
                    "{name} crate {lane} failed to rest on shelf: {p:?}"
                ));
            }
            if lane == 2 && !(floor..floor + 0.6).contains(&p[1]) {
                return Err(format!("{name} panel escaped cave floor: {p:?}"));
            }
            if v.iter().map(|v| v * v).sum::<f32>().sqrt() > 0.18 {
                return Err(format!("{name} lane {lane} did not settle: {v:?}"));
            }
        }
    }
    Ok(())
}
#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::approximate::{
        Body, BodyStorage, Config, Material, Shape, TriangleMeshDesc, Vector as V, World,
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
            for chunk in super::super::redbrand_cave::chunks() {
                world
                    .add_triangle_mesh(
                        TriangleMeshDesc {
                            position: v(chunk.position),
                            vertices: chunk.vertices.into_iter().map(v).collect(),
                            triangles: chunk.triangles,
                            material: Material::default(),
                        },
                        Some(chunk.key),
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
                    body.rotation_locked = lane != 2;
                    body.sleep_allowed = false;
                    if lane == 2 {
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
                .unwrap_or_else(|e| panic!("rigid mesh tick {tick}: {e:?}"));
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
    fn v(p: [f32; 3]) -> Vector {
        Vector::new(p[0], p[1], p[2])
    }
    pub fn surfaces() -> Vec<(Pose, SharedShape)> {
        super::super::redbrand_cave::chunks()
            .into_iter()
            .map(|chunk| {
                (
                    Pose::from_translation(v(chunk.position)),
                    SharedShape::trimesh(
                        chunk.vertices.into_iter().map(v).collect(),
                        chunk.triangles,
                    )
                    .unwrap(),
                )
            })
            .collect()
    }
    pub fn separation(
        surfaces: &[(Pose, SharedShape)],
        lane: usize,
        p: [f32; 3],
        q: [f32; 4],
    ) -> f32 {
        let shape = Cuboid::new(v(HALF[lane]));
        let pose = Pose::from_parts(v(p), Rotation::from_xyzw(q[0], q[1], q[2], q[3]));
        surfaces
            .iter()
            .filter_map(|(at, mesh)| {
                rapier3d::parry::query::contact(&pose, &shape, at, mesh.as_ref(), 0.0).unwrap()
            })
            .map(|c| c.dist)
            .fold(0.0_f32, f32::min)
    }
    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -9.0, 0.0);
            world.integration_parameters.dt = DT;
            for (pose, shape) in surfaces() {
                world
                    .colliders
                    .insert(ColliderBuilder::new(shape).position(pose).build());
            }
            let boxes = (0..LANES)
                .map(|lane| {
                    let mut builder = RigidBodyBuilder::dynamic()
                        .translation(v(START[lane]))
                        .linvel(v(VELOCITY[lane]))
                        .linear_damping(0.2)
                        .angular_damping(0.8)
                        .can_sleep(false);
                    if lane != 2 {
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
    fn rapier_reference_settles_crates_and_panel_on_actual_cave_meshes() {
        let mut trace = Trace::default();
        scenario().run(&mut rapier::Sim::new(), &mut trace);
        check(&trace, &trace).unwrap();
    }
}
