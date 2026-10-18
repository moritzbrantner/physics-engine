//! Native oracle: checks every scenario's invariants and fails when physics-engine is more than
//! 5% slower than Rapier (median of interleaved samples), both with the consumer's call pattern
//! and, where a scenario has one, bare Rapier without the consumer's workarounds.

use physics_oracle::{Scenario, Side, Trace, all};
use std::{process::ExitCode, time::Instant};

const SAMPLES: usize = 11;
const MARGIN: f64 = 1.05;

fn time(scenario: &Scenario, side: Side) -> f64 {
    let mut sim = scenario.build(side);
    let mut trace = Trace::default();
    trace.0.reserve(1 << 16);
    let start = Instant::now();
    scenario.run(&mut *sim, &mut trace);
    let elapsed = start.elapsed().as_secs_f64();
    std::hint::black_box(&trace);
    elapsed * 1e6 / f64::from(scenario.ticks)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() -> ExitCode {
    let mut failed = false;
    println!(
        "{:<20} {:<12} {:>14} {:>14} {:>7} {:>11}  result",
        "scenario", "capability", "engine µs/tick", "rapier µs/tick", "ratio", "rapier raw"
    );
    for scenario in all() {
        if let Err(error) = scenario.verify() {
            println!(
                "{:<20} {:<12} invariant failed: {error}",
                scenario.name, scenario.capabilities
            );
            failed = true;
            continue;
        }
        let (mut engine, mut rapier, mut raw) = (Vec::new(), Vec::new(), Vec::new());
        for _ in 0..SAMPLES {
            engine.push(time(&scenario, Side::Engine));
            rapier.push(time(&scenario, Side::Rapier));
            if scenario.rapier_raw.is_some() {
                raw.push(time(&scenario, Side::RapierRaw));
            }
        }
        let (e, r) = (median(engine), median(rapier));
        // Rapier without the consumer's workarounds: gated too, so the engine beats bare Rapier.
        let raw = (!raw.is_empty()).then(|| median(raw));
        let slow = e > r * MARGIN || raw.is_some_and(|raw| e > raw * MARGIN);
        failed |= slow;
        println!(
            "{:<20} {:<12} {:>14.2} {:>14.2} {:>7.2} {:>11}  {}",
            scenario.name,
            scenario.capabilities,
            e,
            r,
            e / r,
            raw.map_or("-".to_string(), |raw| format!("{raw:.2}")),
            if slow { "SLOWER THAN RAPIER" } else { "ok" }
        );
    }
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
