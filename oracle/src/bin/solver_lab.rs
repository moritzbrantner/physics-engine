//! Prints one JSON line per sample of the #545 workloads; `scripts/solver-lab.mjs` drives it.

use physics_oracle::solver_lab::{names, run};
use std::sync::OnceLock;
use std::time::Instant;

fn now() -> f64 {
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1e6
}

fn main() {
    let samples: usize = std::env::args()
        .nth(1)
        .map_or(1, |arg| arg.parse().expect("samples is a count"));
    let wanted: Vec<String> = std::env::args().skip(2).collect();
    for (index, name) in names().iter().enumerate() {
        if !wanted.is_empty() && !wanted.contains(name) {
            continue;
        }
        for _ in 0..samples {
            let s = run(index, now);
            println!(
                "{{\"workload\":\"{name}\",\"us\":{},\"rows\":{},\"solve\":{},\"prepare\":{}}}",
                s.us_per_tick, s.phases[0], s.phases[1], s.phases[2]
            );
        }
    }
}
