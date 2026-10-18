//! WASM exports for `scripts/oracle-wasm.mjs`: run a scenario side with the host clock.

use crate::{Side, Trace, all};

#[link(wasm_import_module = "env")]
unsafe extern "C" {
    /// Host clock in milliseconds (`performance.now()`).
    fn oracle_now() -> f64;
}

fn side(value: u32) -> Side {
    match value {
        0 => Side::Engine,
        1 => Side::Rapier,
        _ => Side::RapierRaw,
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

/// Microseconds per tick of one run of scenario `index` on `side` (0 engine, 1 Rapier).
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
