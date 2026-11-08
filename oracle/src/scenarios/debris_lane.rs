//! C18 retained debris lane (#523, mmorpg#648), the MMORPG adapter pattern: one world holds the
//! town and a heightfield, about 3,500 static features, and every 60 Hz tick the 110 thrown
//! pieces of 16 broken props (`debris`) are added as keyless transient bodies, stepped once at
//! 2 substeps × 4 passes and removed together. A piece that settles by the game's rule is
//! re-added as a fixed box on later ticks. The pieces' invariants are `debris`'s.
//!
//! The Rapier side rebuilds the world per tick, statics included, as MMORPG's adapter did. The
//! perf gate is that pattern only (owner decision, #523): bare persistent Rapier keeps bodies and
//! warm starts, a different workload from a lane that re-adds every piece.

use super::debris::{self, Start, starts};
use crate::{Scenario, Trace};

const TICKS: u32 = 2 * debris::TICKS;
const DT: f32 = 1.0 / 60.0;
const COLS: usize = 40;
const ROWS: usize = 30;
const HOUSE: f32 = 1.0;
const SPACING: f32 = 4.0;
/// The town and the extra heightfield stand clear of the breaks (x, z within 16 m).
const TOWN_OFFSET: f32 = 80.0;
/// 48 × 48 cells, flat and far from the breaks: static features the step must not pay for.
const FIELD_VERTICES: usize = 49;
const FIELD_ORIGIN: f32 = -300.0;

fn house(i: usize, j: usize) -> [f32; 3] {
    [
        TOWN_OFFSET + i as f32 * SPACING,
        HOUSE,
        TOWN_OFFSET + j as f32 * SPACING,
    ]
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    debris::check_ticks(engine, rapier, TICKS)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "debris-lane",
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

#[cfg(feature = "engine")]
mod engine {
    use super::debris::engine::{add_ground, material};
    use super::*;
    use crate::Obs;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, BodyStorage, Config, HeightfieldDesc, Material, Quaternion, Shape, Vector, World,
        },
    };

    use crate::kcc::engine::v;

    struct Piece {
        start: Start,
        position: Vector,
        orientation: Quaternion,
        velocity: Vector,
        spin: Vector,
        resting: bool,
    }

    pub struct Sim {
        world: World,
        pieces: Vec<Piece>,
        ids: Vec<BodyId>,
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -9.0, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            add_ground(&mut world);
            for j in 0..ROWS {
                for i in 0..COLS {
                    let key = (1_000 + j * COLS + i) as u64;
                    let mut b = Body::keyed(key, Shape::Box(v([HOUSE; 3])), v(house(i, j)), 0.0);
                    material(&mut b);
                    world.add_body(b).unwrap();
                }
            }
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(FIELD_ORIGIN, 0.0, FIELD_ORIGIN),
                        cell_size: (2.0, 2.0),
                        columns: FIELD_VERTICES,
                        rows: FIELD_VERTICES,
                        heights: vec![0.5; FIELD_VERTICES * FIELD_VERTICES],
                        removed_cells: None,
                        material: Material::default(),
                    },
                    Some(3),
                )
                .unwrap();
            let pieces = starts()
                .into_iter()
                .map(|start| Piece {
                    position: v(start.position),
                    orientation: {
                        let [x, y, z, w] = start.rotation;
                        Quaternion(x, y, z, w)
                    },
                    velocity: v(start.velocity),
                    spin: v(start.spin),
                    start,
                    resting: false,
                })
                .collect();
            Self {
                world,
                pieces,
                ids: Vec::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.pieces.iter().any(|p| !p.resting) {
                self.ids.clear();
                for p in &self.pieces {
                    let shape = Shape::Box(v(p.start.half));
                    let mut body = if p.resting {
                        Body::new(shape, p.position, 0.0)
                    } else {
                        let mut b = Body::new(shape, p.position, p.start.mass());
                        b.velocity = p.velocity;
                        b.angular_velocity = p.spin;
                        b.linear_damping = 0.2;
                        b.angular_damping = 0.8;
                        b.sleep_allowed = false;
                        b
                    };
                    body.orientation = p.orientation;
                    material(&mut body);
                    self.ids.push(self.world.add_body(body).unwrap());
                }
                self.world.step(f64::from(DT)).unwrap();
                let removed = self.world.remove_bodies(&self.ids).unwrap();
                for (piece, body) in self.pieces.iter_mut().zip(removed) {
                    if piece.resting {
                        continue;
                    }
                    piece.position = body.position;
                    piece.orientation = body.orientation;
                    piece.velocity = body.velocity;
                    piece.spin = body.angular_velocity;
                    let (u, w) = (piece.velocity, piece.spin);
                    if debris::settles(tick + 1, [u.0, u.1, u.2], [w.0, w.1, w.2]) {
                        piece.resting = true;
                        piece.velocity = Vector::ZERO;
                        piece.spin = Vector::ZERO;
                    }
                }
            }
            for p in &self.pieces {
                let q = p.orientation;
                let at = p.position;
                trace.0.push(Obs::Position([at.0, at.1, at.2]));
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
                trace.0.push(Obs::Grounded(p.resting));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::debris::rapier::{Piece, insert, pieces, read_back, record, statics, terrain};
    use super::*;
    use rapier3d::parry::shape::HeightField;
    use rapier3d::parry::utils::Array2;
    use rapier3d::prelude::*;

    use crate::kcc::rapier::v;

    pub struct Sim {
        terrain: SharedShape,
        field: SharedShape,
        pieces: Vec<Piece>,
    }

    fn lane_statics(world: &mut PhysicsWorld, terrain: &SharedShape, field: &SharedShape) {
        statics(world, terrain);
        world.integration_parameters.dt = DT;
        for j in 0..ROWS {
            for i in 0..COLS {
                world.colliders.insert(
                    ColliderBuilder::cuboid(HOUSE, HOUSE, HOUSE)
                        .translation(v(house(i, j)))
                        .friction(0.8)
                        .restitution(0.15)
                        .build(),
                );
            }
        }
        world.colliders.insert(
            ColliderBuilder::new(field.clone())
                .translation(Vector::new(FIELD_ORIGIN + 48.0, 0.5, FIELD_ORIGIN + 48.0))
                .build(),
        );
    }

    fn field() -> SharedShape {
        let size = 2.0 * (FIELD_VERTICES - 1) as f32;
        SharedShape::new(HeightField::new(
            Array2::new(
                FIELD_VERTICES,
                FIELD_VERTICES,
                vec![0.0; FIELD_VERTICES * FIELD_VERTICES],
            ),
            Vector::new(size, 1.0, size),
        ))
    }

    impl Sim {
        pub fn new() -> Self {
            Self {
                terrain: terrain(),
                field: field(),
                pieces: pieces(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut world = PhysicsWorld::new();
            if self.pieces.iter().any(|p| !p.resting) {
                lane_statics(&mut world, &self.terrain, &self.field);
                for p in &mut self.pieces {
                    p.handle = insert(&mut world, p, 0);
                }
                world.step();
            }
            read_back(&mut world, &mut self.pieces, tick, false);
            record(&self.pieces, trace);
        }
    }
}
