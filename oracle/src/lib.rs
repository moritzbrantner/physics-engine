//! Rapier oracle (technical decisions D4). Every parity scenario runs the same workload on Rapier
//! and on physics-engine, records comparable observations, and checks behaviour invariants. The
//! native binary also times both sides and fails when physics-engine is slower than Rapier.

pub mod scenarios;
#[cfg(target_arch = "wasm32")]
mod wasm;

/// One comparable observation. Positions and lengths are metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Obs {
    /// First hit of a segment: fraction of the segment in `0..=1` and outward surface normal.
    Ray(Option<(f32, [f32; 3])>),
    /// Whether a query shape penetrates static geometry deeper than the scenario tolerance.
    Penetrates(bool),
    /// A body position after a tick.
    Position([f32; 3]),
    /// Whether a character stands on static support after a tick.
    Grounded(bool),
    /// Translation applied to leave penetration (zero where an engine does not report it).
    Depenetration([f32; 3]),
}

/// Observations in the order a scenario recorded them.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Trace(pub Vec<Obs>);

/// One side's live simulation for a scenario.
pub trait Sim {
    fn tick(&mut self, tick: u32, trace: &mut Trace);
}

/// Which engine runs a side.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Side {
    Engine,
    Rapier,
    RapierRaw,
}

/// A parity scenario: the same workload on both engines.
pub struct Scenario {
    pub name: &'static str,
    /// Capability ids from docs/consumer-capabilities.md.
    pub capabilities: &'static str,
    pub ticks: u32,
    #[cfg(feature = "engine")]
    pub engine: fn() -> Box<dyn Sim>,
    /// Rapier with the consumer's actual call pattern: the perf gate's baseline.
    #[cfg(feature = "rapier")]
    pub rapier: fn() -> Box<dyn Sim>,
    /// Rapier without the consumer's workarounds (printed only): its raw cost.
    #[cfg(feature = "rapier")]
    pub rapier_raw: Option<fn() -> Box<dyn Sim>>,
    /// Behaviour invariants over both traces (engine, rapier).
    pub check: fn(&Trace, &Trace) -> Result<(), String>,
}

impl Scenario {
    /// Builds a side, untimed.
    pub fn build(&self, side: Side) -> Box<dyn Sim> {
        match side {
            #[cfg(feature = "engine")]
            Side::Engine => (self.engine)(),
            #[cfg(feature = "rapier")]
            Side::Rapier => (self.rapier)(),
            #[cfg(feature = "rapier")]
            Side::RapierRaw => self.rapier_raw.expect("scenario has a raw Rapier variant")(),
            #[allow(unreachable_patterns)]
            _ => panic!("{side:?} is not built into this oracle"),
        }
    }

    /// Runs every tick of a built side.
    pub fn run(&self, sim: &mut dyn Sim, trace: &mut Trace) {
        trace.0.clear();
        for tick in 0..self.ticks {
            sim.tick(tick, trace);
        }
    }

    /// Runs both sides once and checks the invariants.
    #[cfg(all(feature = "engine", feature = "rapier"))]
    pub fn verify(&self) -> Result<(), String> {
        let (mut engine, mut rapier) = (Trace::default(), Trace::default());
        self.run(&mut *self.build(Side::Engine), &mut engine);
        self.run(&mut *self.build(Side::Rapier), &mut rapier);
        finite(&engine)?;
        (self.check)(&engine, &rapier)
    }
}

/// Every observation is finite.
pub fn finite(trace: &Trace) -> Result<(), String> {
    for (i, obs) in trace.0.iter().enumerate() {
        let ok = match obs {
            Obs::Ray(Some((t, n))) => t.is_finite() && n.iter().all(|v| v.is_finite()),
            Obs::Position(p) | Obs::Depenetration(p) => p.iter().all(|v| v.is_finite()),
            _ => true,
        };
        if !ok {
            return Err(format!("observation {i} is not finite: {obs:?}"));
        }
    }
    Ok(())
}

/// Deterministic xorshift generator shared by both sides of a scenario.
#[derive(Clone, Copy, Debug)]
pub struct Rng(pub u64);
impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    /// Uniform in `lo..hi`.
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * ((self.next_u64() >> 40) as f32 / (1u64 << 24) as f32)
    }
}

/// All scenarios in the harness. Every parity PR adds its scenario here.
pub fn all() -> Vec<Scenario> {
    vec![
        scenarios::static_queries::scenario(),
        scenarios::character_move::scenario(),
        scenarios::arpg_planar::scenario(),
    ]
}
