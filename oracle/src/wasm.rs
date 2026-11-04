//! WASM oracle exports for CI timing and the read-only Pages comparison lab.

use std::cell::RefCell;

use crate::{Obs, Side, Trace, all};

// Browser-only presentation data. The full, unsampled traces are still checked by `verify`.
// Each row is [tick, observation ordinal, kind, six numeric values]. The ordinal is kept
// when display rows are sampled so the two sides can be matched without inventing physics.
const DISPLAY_STRIDE: usize = 9;
const MAX_DISPLAY_OBSERVATIONS_PER_TICK: usize = 64;

#[derive(Default)]
struct DisplayTrace {
    rows: Vec<f64>,
    total_observations: u32,
}

std::thread_local! {
    static DISPLAY: RefCell<DisplayTrace> = RefCell::new(DisplayTrace::default());
}

#[link(wasm_import_module = "env")]
unsafe extern "C" {
    /// Host clock in milliseconds (`performance.now()`).
    fn oracle_now() -> f64;
}

fn side(value: u32) -> Side {
    match value {
        0 => Side::Engine,
        1 => Side::Rapier,
        2 => Side::RapierRaw,
        _ => Side::EngineIndexed,
    }
}

/// 1 when scenario `index` has a raw Rapier variant (side 2).
#[unsafe(no_mangle)]
pub extern "C" fn oracle_has_raw(index: u32) -> i32 {
    #[cfg(feature = "rapier")]
    let raw = all()[index as usize].rapier_raw.is_some();
    #[cfg(not(feature = "rapier"))]
    let raw = {
        let _ = index;
        false
    };
    i32::from(raw)
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_scenarios() -> u32 {
    all().len() as u32
}

/// Microseconds per tick of one run of scenario `index` on `side` (0 engine, 1 Rapier, 2 bare
/// Rapier, 3 engine with `BodyStorage::Indexed`).
#[unsafe(no_mangle)]
pub extern "C" fn oracle_time(index: u32, which: u32) -> f64 {
    let scenarios = all();
    let scenario = &scenarios[index as usize];
    let mut sim = scenario.build(side(which));
    let mut trace = Trace::default();
    // SAFETY: a plain host import without arguments or memory access.
    let start = unsafe { oracle_now() };
    scenario.run(&mut *sim, &mut trace);
    let elapsed = unsafe { oracle_now() } - start;
    core::hint::black_box(&trace);
    elapsed * 1e3 / f64::from(scenario.ticks)
}

/// 0 when scenario `index` passes its invariants on this target, else 1.
#[cfg(all(feature = "engine", feature = "rapier"))]
#[unsafe(no_mangle)]
pub extern "C" fn oracle_check(index: u32) -> i32 {
    i32::from(all()[index as usize].verify().is_err())
}

/// Scenario name bytes: address and length in linear memory.
#[unsafe(no_mangle)]
pub extern "C" fn oracle_name_ptr(index: u32) -> usize {
    all()[index as usize].name.as_ptr() as usize
}
#[unsafe(no_mangle)]
pub extern "C" fn oracle_name_len(index: u32) -> usize {
    all()[index as usize].name.len()
}

/// Capability ids attached to the registered scenario (space-separated, UTF-8).
#[unsafe(no_mangle)]
pub extern "C" fn oracle_capabilities_ptr(index: u32) -> usize {
    all()[index as usize].capabilities.as_ptr() as usize
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_capabilities_len(index: u32) -> usize {
    all()[index as usize].capabilities.len()
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_ticks(index: u32) -> u32 {
    all()[index as usize].ticks
}

/// Encode one authoritative oracle observation for browser display. `kind` has this stable
/// display-only mapping: 1 ray, 2 penetration, 3 position, 4 rotation, 5 grounded,
/// 6 depenetration, 7 contact, 8 blocked, 9 support, 10 contact transition,
/// 11 linear velocity, 12 joint tension, 13 labelled first hit, 14 overlap label set.
fn display_values(obs: &Obs) -> (u8, [f64; 6]) {
    let mut data = [0.0; 6];
    let kind = match *obs {
        Obs::Ray(hit) => {
            if let Some((fraction, normal)) = hit {
                data[..5].copy_from_slice(&[
                    1.0,
                    f64::from(fraction),
                    f64::from(normal[0]),
                    f64::from(normal[1]),
                    f64::from(normal[2]),
                ]);
            }
            1
        }
        Obs::Penetrates(value) => {
            data[0] = f64::from(u8::from(value));
            2
        }
        Obs::Position(p) => {
            data[..3].copy_from_slice(&p.map(f64::from));
            3
        }
        Obs::Velocity(v) => {
            data[..3].copy_from_slice(&v.map(f64::from));
            11
        }
        Obs::Tension(t) => {
            data[0] = f64::from(t);
            12
        }
        Obs::Rotation(q) => {
            data[..4].copy_from_slice(&q.map(f64::from));
            4
        }
        Obs::Grounded(value) => {
            data[0] = f64::from(u8::from(value));
            5
        }
        Obs::Depenetration(v) => {
            data[..3].copy_from_slice(&v.map(f64::from));
            6
        }
        Obs::Contact(hit) => {
            if let Some((depth, normal)) = hit {
                data[..5].copy_from_slice(&[
                    1.0,
                    f64::from(depth),
                    f64::from(normal[0]),
                    f64::from(normal[1]),
                    f64::from(normal[2]),
                ]);
            }
            7
        }
        Obs::Blocked(value) => {
            data[0] = f64::from(u8::from(value));
            8
        }
        Obs::Support(gap) => {
            if let Some(gap) = gap {
                data[0] = 1.0;
                data[1] = f64::from(gap);
            }
            9
        }
        Obs::Hit(hit) => {
            if let Some((label, fraction)) = hit {
                data[..3].copy_from_slice(&[1.0, f64::from(label), f64::from(fraction)]);
            }
            13
        }
        Obs::Overlaps(labels) => {
            // Labels below 64: exact in two 32-bit halves.
            data[0] = f64::from(labels as u32);
            data[1] = f64::from((labels >> 32) as u32);
            14
        }
        Obs::ContactEvent { started, pair, .. } => {
            data[..3].copy_from_slice(&[
                f64::from(u8::from(started)),
                f64::from(pair[0]),
                f64::from(pair[1]),
            ]);
            10
        }
    };
    (kind, data)
}

/// Replay the *actual* scenario side and collect a bounded, deterministic display projection.
/// Side 0 is physics-engine, 1 is Rapier, 3 is Indexed. Returns -1 for invalid inputs, 0 on
/// success. This does not replace the full scenario invariant checks or perf oracle.
#[unsafe(no_mangle)]
pub extern "C" fn oracle_trace(index: u32, which: u32) -> i32 {
    let scenarios = all();
    let Some(scenario) = scenarios.get(index as usize) else {
        return -1;
    };
    if !matches!(which, 0 | 1 | 3) {
        return -1;
    }
    let mut sim = scenario.build(side(which));
    DISPLAY.with(|display| {
        let mut display = display.borrow_mut();
        display.rows.clear();
        display.total_observations = 0;
        let mut step = Trace::default();
        for tick in 0..scenario.ticks {
            step.0.clear();
            sim.tick(tick, &mut step);
            let count = step.0.len();
            display.total_observations = display.total_observations.saturating_add(count as u32);
            let samples = count.min(MAX_DISPLAY_OBSERVATIONS_PER_TICK);
            for sample in 0..samples {
                let ordinal = if samples <= 1 || samples == count {
                    sample
                } else {
                    sample * (count - 1) / (samples - 1)
                };
                let (kind, values) = display_values(&step.0[ordinal]);
                display
                    .rows
                    .extend([f64::from(tick), ordinal as f64, f64::from(kind)]);
                display.rows.extend(values);
            }
        }
    });
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_trace_ptr() -> usize {
    DISPLAY.with(|display| display.borrow().rows.as_ptr() as usize)
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_trace_len() -> usize {
    DISPLAY.with(|display| display.borrow().rows.len())
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_trace_total() -> u32 {
    DISPLAY.with(|display| display.borrow().total_observations)
}

#[unsafe(no_mangle)]
pub extern "C" fn oracle_trace_stride() -> usize {
    DISPLAY_STRIDE
}
