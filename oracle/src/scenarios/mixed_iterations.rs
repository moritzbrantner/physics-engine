//! C18 merged response lanes (#521): MMORPG keeps two retained response worlds only because
//! solver passes were per world, one pass for upright props and four for tumbling debris
//! (mmorpg#648, `step_authored`'s `ResponseLane`). Here one world holds both: the `props` lanes
//! (shifted to start at z = 40 m, beyond the debris hill) on the debris scenario's floor plane
//! and hill, and the `debris` throws, whose pieces request three additional solver iterations.
//! The engine world runs 2 substeps × 1 pass, so the debris islands get 1 + 3 passes and the
//! crates keep their one. Rapier rebuilds one world per tick as MMORPG does, with the pieces'
//! `additional_solver_iterations(3)`; the raw variant keeps one world.
//!
//! Invariants: the existing `props` bounds over all 240 ticks and the `debris` bounds over its
//! 150 ticks, each against Rapier's merged world.

use crate::{Scenario, Trace};

use super::{debris, props};

const TICKS: u32 = props::TICKS;
/// Where the props lanes start along Z, clear of the hill and the throws.
const PROPS_Z: f32 = 40.0;
/// Extra passes for the debris pieces: MMORPG's four debris passes over the props' one.
const EXTRA: u8 = 3;

pub fn scenario() -> Scenario {
    Scenario {
        name: "mixed-iterations",
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

/// Splits a tick-major trace (each tick the crates' positions, then the pieces' records) into
/// the `props` and `debris` traces, the latter cut to its own length.
fn split(trace: &Trace) -> Option<(Trace, Trace)> {
    let crates = props::LANES;
    let per_tick = trace.0.len() / TICKS as usize;
    if per_tick * TICKS as usize != trace.0.len() || per_tick < crates {
        return None;
    }
    let (mut lanes, mut pieces) = (Trace::default(), Trace::default());
    for (tick, record) in trace.0.chunks(per_tick).enumerate() {
        lanes.0.extend_from_slice(&record[..crates]);
        if tick < debris::TICKS as usize {
            pieces.0.extend_from_slice(&record[crates..]);
        }
    }
    Some((lanes, pieces))
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let (Some((engine_props, engine_debris)), Some((rapier_props, rapier_debris))) =
        (split(engine), split(rapier))
    else {
        return Err("trace lengths differ".into());
    };
    props::check_at(&engine_props, &rapier_props, PROPS_Z).map_err(|e| format!("props: {e}"))?;
    debris::check(&engine_debris, &rapier_debris).map_err(|e| format!("debris: {e}"))?;
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{BodyStorage, Config, Vector, World},
    };

    pub struct Sim {
        world: World,
        crates: Vec<BodyId>,
        pieces: Vec<debris::engine::Piece>,
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -props::GRAVITY, 0.0),
                // One retained world for both lanes: the props' two substeps of one pass; the
                // pieces add their own three.
                substeps: 2,
                velocity_iterations: 1,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            debris::engine::add_ground(&mut world);
            props::engine::add_obstacles(&mut world, PROPS_Z);
            let crates = props::engine::add_crates(&mut world, PROPS_Z);
            let pieces = debris::engine::add_pieces(&mut world, EXTRA);
            Self {
                world,
                crates,
                pieces,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            props::engine::push_crates(&mut self.world, &self.crates, tick);
            self.world.step(f64::from(props::DT)).unwrap();
            props::engine::record(&self.world, &self.crates, trace);
            debris::engine::settle_and_record(&mut self.world, &mut self.pieces, tick, trace);
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;

    pub struct Sim {
        world: PhysicsWorld,
        terrain: SharedShape,
        crates: Vec<props::rapier::Crate>,
        pieces: Vec<debris::rapier::Piece>,
        rebuild: bool,
    }

    fn statics(world: &mut PhysicsWorld, terrain: &SharedShape) {
        debris::rapier::statics(world, terrain);
        props::rapier::obstacles(world, PROPS_Z);
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let terrain = debris::rapier::terrain();
            let mut world = PhysicsWorld::new();
            statics(&mut world, &terrain);
            let mut crates = props::rapier::crates(PROPS_Z, rebuild);
            let mut pieces = debris::rapier::pieces();
            if !rebuild {
                props::rapier::insert_all(&mut world, &mut crates);
                for p in &mut pieces {
                    p.handle = debris::rapier::insert(&mut world, p, usize::from(EXTRA));
                }
            }
            Self {
                world,
                terrain,
                crates,
                pieces,
                rebuild,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.rebuild {
                // MMORPG's pattern, both lanes merged: a fresh world per tick, resting crates
                // and settled pieces fixed.
                props::rapier::push_crates(&mut self.crates, tick);
                let mut world = PhysicsWorld::new();
                statics(&mut world, &self.terrain);
                props::rapier::insert_all(&mut world, &mut self.crates);
                for p in &mut self.pieces {
                    p.handle = debris::rapier::insert(&mut world, p, usize::from(EXTRA));
                }
                world.step();
                props::rapier::read_back(&world, &mut self.crates);
                self.world = world;
            } else {
                props::rapier::push_retained(&mut self.world, &self.crates, tick);
                self.world.step();
                props::rapier::read_positions(&self.world, &mut self.crates);
            }
            props::rapier::record(&self.crates, trace);
            debris::rapier::read_back(&mut self.world, &mut self.pieces, tick, !self.rebuild);
            debris::rapier::record(&self.pieces, trace);
        }
    }
}
