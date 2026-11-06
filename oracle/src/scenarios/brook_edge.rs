//! Actual MMORPG #644 towpath edge at the consumer's centimeter engine scale (#508).
use crate::{Obs, Scenario, Trace};
#[path = "../../../tests/support/brook_mesh.rs"]
mod brook;
const TICKS: u32 = 16;
fn desired(tick: u32) -> [f32; 3] {
    // The captured departure step, next-step settling, then falling off the bank.
    [
        -10.5,
        match tick {
            0 => 0.0,
            1 => -1.5094768,
            _ => -0.25 * (tick - 1) as f32,
        },
        0.0,
    ]
}
pub fn scenario() -> Scenario {
    Scenario {
        name: "brook-mesh-edge",
        capabilities: "C4 C23",
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
fn check(e: &Trace, r: &Trace) -> Result<(), String> {
    crate::finite(e)?;
    crate::finite(r)?;
    if e.0.len() != TICKS as usize * 2 || r.0.len() != TICKS as usize * 2 {
        return Err("malformed brook trace".into());
    }
    for (tick, (e, r)) in
        e.0.as_chunks::<2>()
            .0
            .iter()
            .zip(r.0.as_chunks::<2>().0)
            .enumerate()
    {
        let (Obs::Position(a), Obs::Position(b)) = (e[0], r[0]) else {
            return Err("missing brook position".into());
        };
        if a.into_iter().zip(b).any(|(a, b)| (a - b).abs() > 0.0134) {
            return Err(format!("brook tick {tick}: Engine {a:?}, Rapier {b:?}"));
        }
        if tick == 0 && a[1] < brook::START[1] * 0.01 - 0.001 {
            return Err(format!("grazing the towpath snapped down early: {a:?}"));
        }
        if tick >= 2 && (e[1] != Obs::Grounded(false) || r[1] != Obs::Grounded(false)) {
            return Err(format!("departed towpath retained support at tick {tick}"));
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
            Body, BodyStorage, CharacterMove, CharacterMoveResult, Config, Shape, TriangleMeshDesc,
            Vector as V, World,
        },
    };
    pub struct Sim {
        world: World,
        id: BodyId,
        results: Vec<CharacterMoveResult>,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                gravity: V::ZERO,
                length_unit: 100.0,
                body_storage: storage,
                ..Config::default()
            })
            .unwrap();
            world
                .add_triangle_mesh(
                    TriangleMeshDesc {
                        position: V::ZERO,
                        vertices: brook::VERTICES.map(|p| V(p[0], p[1], p[2])).to_vec(),
                        triangles: brook::TRIANGLES.to_vec(),
                        material: Default::default(),
                    },
                    Some(65125),
                )
                .unwrap();
            let p = brook::START;
            let mut body = Body::keyed(
                1000001,
                Shape::capsule(60.0, 30.0),
                V(p[0], p[1], p[2]),
                1.0,
            );
            body.external = true;
            let id = world.add_body(body).unwrap();
            let mut settings = world.character_settings();
            settings.offset = 0.25;
            world.add_character(id, settings).unwrap();
            Self {
                world,
                id,
                results: Vec::new(),
            }
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
                    &mut self.results,
                )
                .unwrap();
            let p = self.world.body(self.id).unwrap().position;
            trace
                .0
                .push(Obs::Position([p.0 / 100.0, p.1 / 100.0, p.2 / 100.0]));
            trace.0.push(Obs::Grounded(self.results[0].ground_contact));
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
            let vertices = brook::VERTICES.map(|p| v(p.map(|x| x * 0.01))).to_vec();
            let mesh = ColliderBuilder::new(
                SharedShape::trimesh(vertices, brook::TRIANGLES.to_vec()).unwrap(),
            );
            Self {
                characters: Characters::new(
                    vec![mesh],
                    [brook::START.map(|x| x * 0.01)],
                    Capsule::new_y(0.6, 0.3),
                    mirror_controller(0.0025, Some(0.05), None),
                    rebuild,
                ),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            self.characters.start_tick();
            self.characters
                .move_all(|_| v(desired(tick).map(|x| x * 0.01)));
            let p = self.characters.position(0);
            trace.0.push(Obs::Position(p));
            trace
                .0
                .push(Obs::Grounded(self.characters.movements[0].grounded));
        }
    }
}
