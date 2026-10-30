//! Native oracle: checks every scenario's invariants and fails when physics-engine is more than
//! 5% slower than Rapier (median of interleaved samples), both with the consumer's call pattern
//! and, where a scenario has one, bare Rapier without the consumer's workarounds. Both engine
//! storage modes (`BodyStorage::Ordered`, the default, and `Indexed`) are timed and gated.
//!
//! CI runs it without arguments. For debugging:
//!
//! ```text
//! oracle --list                      scenario names
//! oracle --only stairs [--only ...]  run only these scenarios
//! oracle --no-timing                 invariants only
//! oracle --keep-going                time scenarios whose invariants fail
//! oracle --dump DIR                  write every side's trace to DIR/<scenario>.<side>.txt
//! ```

use physics_oracle::{Scenario, Side, Trace, all};
use std::{io::Write, path::PathBuf, process::ExitCode, time::Instant};

#[derive(Default)]
struct Options {
    only: Vec<String>,
    timing: bool,
    keep_going: bool,
    dump: Option<PathBuf>,
    list: bool,
}

fn options() -> Result<Options, String> {
    let mut o = Options {
        timing: true,
        ..Options::default()
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--only" => o
                .only
                .push(args.next().ok_or("--only needs a scenario name")?),
            "--no-timing" => o.timing = false,
            "--keep-going" => o.keep_going = true,
            "--dump" => o.dump = Some(args.next().ok_or("--dump needs a directory")?.into()),
            "--list" => o.list = true,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(o)
}

/// Writes both sides' traces, one observation per line (`tick-ordered index<TAB>observation`).
fn dump(scenario: &Scenario, dir: &std::path::Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let sides = [
        (Side::Engine, "engine"),
        (Side::EngineIndexed, "indexed"),
        (Side::Rapier, "rapier"),
        (Side::RapierRaw, "rapier_raw"),
    ];
    for (side, name) in sides {
        if side == Side::RapierRaw && scenario.rapier_raw.is_none() {
            continue;
        }
        let mut trace = Trace::default();
        scenario.run(&mut *scenario.build(side), &mut trace);
        let path = dir.join(format!("{}.{name}.txt", scenario.name));
        let mut out = std::io::BufWriter::new(std::fs::File::create(&path)?);
        let per_tick = trace.0.len() / scenario.ticks.max(1) as usize;
        for (i, obs) in trace.0.iter().enumerate() {
            writeln!(
                out,
                "{}:{}\t{obs:?}",
                i / per_tick.max(1),
                i % per_tick.max(1)
            )?;
        }
        println!("wrote {}", path.display());
    }
    Ok(())
}

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
    let options = match options() {
        Ok(o) => o,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let scenarios = all();
    if options.list {
        for s in &scenarios {
            println!("{:<20} {}", s.name, s.capabilities);
        }
        return ExitCode::SUCCESS;
    }
    if let Some(unknown) = options
        .only
        .iter()
        .find(|name| !scenarios.iter().any(|s| s.name == name.as_str()))
    {
        eprintln!("unknown scenario {unknown} (see --list)");
        return ExitCode::FAILURE;
    }
    let mut failed = false;
    println!(
        "{:<20} {:<12} {:>14} {:>14} {:>14} {:>7} {:>11}  result",
        "scenario",
        "capability",
        "engine µs/tick",
        "indexed µs/tick",
        "rapier µs/tick",
        "ratio",
        "rapier raw"
    );
    for scenario in scenarios
        .iter()
        .filter(|s| options.only.is_empty() || options.only.iter().any(|o| o == s.name))
    {
        if let Some(dir) = &options.dump
            && let Err(error) = dump(scenario, dir)
        {
            eprintln!("cannot dump {}: {error}", scenario.name);
            return ExitCode::FAILURE;
        }
        if let Err(error) = scenario.verify() {
            println!(
                "{:<20} {:<12} invariant failed: {error}",
                scenario.name, scenario.capabilities
            );
            failed = true;
            if !options.keep_going {
                continue;
            }
        }
        if !options.timing {
            println!(
                "{:<20} {:<12} invariants checked",
                scenario.name, scenario.capabilities
            );
            continue;
        }
        let (mut engine, mut indexed, mut rapier, mut raw) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for _ in 0..SAMPLES {
            engine.push(time(scenario, Side::Engine));
            indexed.push(time(scenario, Side::EngineIndexed));
            rapier.push(time(scenario, Side::Rapier));
            if scenario.rapier_raw.is_some() {
                raw.push(time(scenario, Side::RapierRaw));
            }
        }
        let (e, x, r) = (median(engine), median(indexed), median(rapier));
        // Rapier without the consumer's workarounds: gated too, so the engine beats bare Rapier.
        let raw = (!raw.is_empty()).then(|| median(raw));
        let slow = [e, x]
            .into_iter()
            .any(|e| e > r * MARGIN || raw.is_some_and(|raw| e > raw * MARGIN));
        failed |= slow;
        println!(
            "{:<20} {:<12} {:>14.2} {:>14.2} {:>14.2} {:>7.2} {:>11}  {}",
            scenario.name,
            scenario.capabilities,
            e,
            x,
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
