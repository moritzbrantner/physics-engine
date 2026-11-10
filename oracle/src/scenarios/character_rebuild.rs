//! #500: identical character continuation after reconstructing a world from physical poses.
use crate::probe::{Controller, Geometry, TerrainCell};
use crate::{Obs, Scenario, Trace};
const TICKS: u32 = 240;
const START: [f32; 3] = [11.162186, 12.6, -73.88426];
fn geometry() -> Geometry {
    Geometry {
        terrain_cells: vec![TerrainCell {
            origin: [8.0, 0.0, -76.0],
            cell_size: [4.0, 4.0],
            heights: [11.0, 11.0, 12.87, 11.5],
        }],
        ..Geometry::default()
    }
}
fn controller() -> Controller {
    Controller {
        half_segment: 0.6,
        radius: 0.3,
        ..Controller::default()
    }
}
fn desired(tick: u32) -> [f32; 3] {
    let angle = (tick % 80) as f32 * std::f32::consts::TAU / 80.0;
    [angle.sin() * 0.01, -0.002725, angle.cos() * 0.01]
}
pub fn scenario() -> Scenario {
    Scenario {
        name: "character-rebuild",
        capabilities: "C4 C5 C8 C14",
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
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    for (name, trace) in [("Engine", engine), ("Rapier", rapier)] {
        crate::finite(trace)?;
        if trace.0.len() != TICKS as usize * 6 {
            return Err(format!("{name}: malformed rebuild trace"));
        }
        let mut grounded = false;
        for (tick, obs) in trace.0.as_chunks::<6>().0.iter().enumerate() {
            if obs[0] != obs[1] || obs[2] != obs[3] || obs[4] != obs[5] {
                return Err(format!(
                    "{name}: rebuilding from the same pose changes tick {tick}: {obs:?}"
                ));
            }
            grounded |= obs[2] == Obs::Grounded(true);
        }
        if !grounded {
            return Err(format!("{name}: terrain was never reached"));
        }
    }
    for (tick, (a, b)) in engine
        .0
        .as_chunks::<6>()
        .0
        .iter()
        .zip(rapier.0.as_chunks::<6>().0)
        .enumerate()
    {
        let (Obs::Position(a), Obs::Position(b)) = (a[0], b[0]) else {
            return Err("missing rebuild position".into());
        };
        if a.into_iter().zip(b).any(|(x, y)| (x - y).abs() > 0.02) {
            return Err(format!("rebuild tick {tick}: Engine {a:?}, Rapier {b:?}"));
        }
    }
    Ok(())
}
#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use crate::kcc::engine::{Characters, v};
    use physics_engine::approximate::{BodyStorage, CharacterMove};
    pub struct Sim {
        original: Characters,
        storage: BodyStorage,
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            Self {
                original: crate::probe::engine_characters(
                    &geometry(),
                    controller(),
                    START,
                    storage,
                ),
                storage,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let position = self.original.position(self.original.ids[0]);
            let mut rebuilt =
                crate::probe::engine_characters(&geometry(), controller(), position, self.storage);
            let step = |_, body| CharacterMove {
                body,
                desired: v(desired(tick)),
                exclude: None,
                autostep: None,
            };
            self.original.move_each(step);
            rebuilt.move_each(step);
            trace
                .0
                .push(Obs::Position(self.original.position(self.original.ids[0])));
            trace
                .0
                .push(Obs::Position(rebuilt.position(rebuilt.ids[0])));
            trace
                .0
                .push(Obs::Grounded(self.original.results[0].ground_contact));
            trace
                .0
                .push(Obs::Grounded(rebuilt.results[0].ground_contact));
            let a = self.original.results[0].depenetration;
            let b = rebuilt.results[0].depenetration;
            trace.0.push(Obs::Depenetration([a.0, a.1, a.2]));
            trace.0.push(Obs::Depenetration([b.0, b.1, b.2]));
        }
    }
}
#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use crate::kcc::rapier::{Characters, v};
    pub struct Sim {
        original: Characters,
    }
    impl Sim {
        pub fn new() -> Self {
            Self {
                original: crate::probe::rapier_characters(&geometry(), controller(), START),
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            let mut rebuilt = crate::probe::rapier_characters(
                &geometry(),
                controller(),
                self.original.position(0),
            );
            self.original.move_all(|_| v(desired(tick)));
            rebuilt.move_all(|_| v(desired(tick)));
            trace.0.push(Obs::Position(self.original.position(0)));
            trace.0.push(Obs::Position(rebuilt.position(0)));
            trace
                .0
                .push(Obs::Grounded(self.original.movements[0].grounded));
            trace.0.push(Obs::Grounded(rebuilt.movements[0].grounded));
            trace.0.push(Obs::Depenetration([0.0; 3]));
            trace.0.push(Obs::Depenetration([0.0; 3]));
        }
    }
}
