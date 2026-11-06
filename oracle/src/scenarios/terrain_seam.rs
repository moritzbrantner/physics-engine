//! MMORPG's flat-to-rising terrain seam: a tangential hit must not hide the next triangle.
use crate::{Obs, Scenario, Trace};

const HEIGHTS: [f32; 25] = [
    1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1100., 1287., 1150., 1100.,
    1100., 1100., 1027., 1242., 1198., 1100., 1100., 324., 560., 560., 560., 560.,
];
const START: [f32; 3] = [1204.7852, 1190.25, -7605.4375];
fn desired(tick: u32) -> [f32; 3] {
    if tick == 0 {
        [-1.5664063, 0., 3.6953125]
    } else {
        [0.; 3]
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "terrain-seam",
        capabilities: "C4 C17",
        ticks: 2,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check: |e, r| {
            for (tick, (e, r)) in e.0.iter().zip(&r.0).enumerate() {
                let (Obs::Position(e), Obs::Position(r)) = (e, r) else {
                    return Err("missing seam position".into());
                };
                if e.iter().zip(r).any(|(e, r)| (e - r).abs() > 0.01) || e[1] <= START[1] / 100. {
                    return Err(format!(
                        "tick {tick}: engine {e:?}, Rapier {r:?}; both must climb"
                    ));
                }
            }
            Ok(())
        },
    }
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, BodyStorage, CharacterMove, CharacterSettings, Config, HeightfieldDesc, Shape,
            Vector as V, World,
        },
    };
    pub struct Sim {
        world: World,
        id: BodyId,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                gravity: V::ZERO,
                length_unit: 100.,
                body_storage: storage,
                ..Config::default()
            })
            .unwrap();
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: V(800., 0., -8000.),
                        cell_size: (400., 400.),
                        columns: 5,
                        rows: 5,
                        heights: HEIGHTS.to_vec(),
                        removed_cells: None,
                        material: Default::default(),
                    },
                    Some(1),
                )
                .unwrap();
            let mut body = Body::keyed(
                401,
                Shape::capsule(60., 30.),
                V(START[0], START[1], START[2]),
                1.,
            );
            body.external = true;
            let id = world.add_body(body).unwrap();
            world
                .add_character(
                    id,
                    CharacterSettings {
                        offset: 0.25,
                        ..world.character_settings()
                    },
                )
                .unwrap();
            Self { world, id }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let d = desired(tick);
            self.world
                .move_characters(
                    &[CharacterMove {
                        body: self.id,
                        desired: V(d[0], d[1], d[2]),
                        exclude: None,
                        autostep: None,
                    }],
                    &mut Vec::new(),
                )
                .unwrap();
            let p = self.world.body(self.id).unwrap().position;
            trace
                .0
                .push(Obs::Position([p.0 / 100., p.1 / 100., p.2 / 100.]));
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, mirror_controller, v};
    use rapier3d::prelude::*;
    pub struct Sim {
        characters: Characters,
    }
    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let data = (0..25).map(|k| HEIGHTS[(k % 5) * 5 + k / 5]).collect();
            let field = HeightField::new(Array2::new(5, 5, data), Vector::new(1600., 1., 1600.));
            let terrain = ColliderBuilder::new(SharedShape::new(field))
                .translation(Vector::new(1600., 0., -7200.));
            Self {
                characters: Characters::new(
                    vec![terrain],
                    [START],
                    Capsule::new_y(60., 30.),
                    mirror_controller(0.25, Some(5.), None),
                    rebuild,
                ),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.start_tick();
            let movement = self.characters.move_shape(0, v(desired(tick)));
            let p = self.characters.end(0, &movement);
            self.characters.commit(0, p);
            trace
                .0
                .push(Obs::Position([p.x / 100., p.y / 100., p.z / 100.]));
        }
    }
}
