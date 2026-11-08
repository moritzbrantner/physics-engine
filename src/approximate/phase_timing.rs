//! Per-phase solver timing for the #545 comparison, on a host-supplied clock. Without the
//! `phase-timing` feature every call is a no-op and no clock is read.
#[cfg(feature = "phase-timing")]
use std::cell::Cell;
#[cfg(feature = "phase-timing")]
use std::sync::OnceLock;

#[derive(Clone, Copy)]
pub enum Phase {
    /// Building, advancing and retesting the contact and joint rows.
    Rows = 0,
    /// The whole solve phase, warm start and `Prepare` included.
    Solve = 1,
    /// A variant's copy-in, grouping or coloring, and copy-out.
    Prepare = 2,
}

#[cfg(feature = "phase-timing")]
static CLOCK: OnceLock<fn() -> f64> = OnceLock::new();
#[cfg(feature = "phase-timing")]
std::thread_local! {
    static TOTALS: Cell<[f64; 3]> = const { Cell::new([0.0; 3]) };
}

/// Installs the clock, in microseconds. The first call wins.
#[cfg(feature = "phase-timing")]
pub fn set_clock(clock: fn() -> f64) {
    let _ = CLOCK.set(clock);
}

/// Microseconds per phase since the last call, indexed as [`Phase`].
#[cfg(feature = "phase-timing")]
pub fn take() -> [f64; 3] {
    TOTALS.replace([0.0; 3])
}

#[cfg(feature = "phase-timing")]
#[inline]
pub(super) fn timed<T>(phase: Phase, f: impl FnOnce() -> T) -> T {
    let Some(clock) = CLOCK.get() else {
        return f();
    };
    let start = clock();
    let result = f();
    let elapsed = clock() - start;
    TOTALS.with(|totals| {
        let mut t = totals.get();
        t[phase as usize] += elapsed;
        totals.set(t);
    });
    result
}

#[cfg(not(feature = "phase-timing"))]
#[inline(always)]
pub(super) fn timed<T>(_phase: Phase, f: impl FnOnce() -> T) -> T {
    f()
}
