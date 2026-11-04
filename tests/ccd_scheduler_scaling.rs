//! A bounded 60 Hz workload isolating CCD scheduler work from true collision density.
//! Independent synchronized impacts must not tunnel, regardless of storage or unrelated bodies.

#[path = "support/ccd_scaling.rs"]
mod fixture;

use physics_engine::approximate::BodyStorage;

#[test]
fn independent_simultaneous_impacts_have_reproducible_work() {
    for irrelevant in [0, 64, 128, 256] {
        for count in [1, 2, 4, 8, 16, 24, 32] {
            let ordered = fixture::run(count, irrelevant, BodyStorage::Ordered);
            let indexed = fixture::run(count, irrelevant, BodyStorage::Indexed);
            assert_eq!(ordered, indexed, "storage divergence at count={count}");
            assert_eq!(ordered.fallbacks, 0);
            assert_eq!(ordered.impacts, count as u64);
            for (p, v) in &ordered.states {
                assert!((f32::from_bits(p[0]) + 0.15).abs() < 0.01);
                assert!((f32::from_bits(v[0]) + 45.0).abs() < 0.01);
            }
            eprintln!(
                "ccd-independent count={count} irrelevant={irrelevant} pairs={} narrows={} sweeps={} tree_nodes={} impacts={}",
                ordered.pairs, ordered.narrows, ordered.sweeps, ordered.tree_nodes, ordered.impacts
            );
        }
    }
}

#[test]
fn synchronized_double_ricochets_keep_every_collision_in_chronological_order() {
    for irrelevant in [0, 128, 256] {
        for count in [1, 2, 4, 8, 16, 32] {
            let ordered = fixture::run_ricochets(count, irrelevant, BodyStorage::Ordered);
            let indexed = fixture::run_ricochets(count, irrelevant, BodyStorage::Indexed);
            assert_eq!(ordered, indexed, "storage divergence at count={count}");
            assert_eq!(ordered.impacts, 2 * count as u64, "{ordered:?}");
            assert_eq!(ordered.fallbacks, 0);
            for (p, v) in &ordered.states {
                assert!(f32::from_bits(p[0]).abs() < 0.01, "{ordered:?}");
                assert!((f32::from_bits(v[0]) - 24.0).abs() < 0.01, "{ordered:?}");
            }
            // Distant projectiles never become each other's candidates (#469).
            assert!(ordered.pairs <= 6 * count as u64, "{ordered:?}");
            eprintln!(
                "ccd-double-ricochet count={count} irrelevant={irrelevant} pairs={} sweeps={} impacts={}",
                ordered.pairs, ordered.sweeps, ordered.impacts
            );
        }
    }
}

/// Informational, first-tick CPU timings with world construction excluded. Shared-runner
/// timings are not a CI gate; the test above and the complexity contract own the work bound.
#[test]
#[ignore = "advisory first-tick timing; run explicitly with --ignored --nocapture"]
fn print_first_tick_ccd_vs_nonccd_timings() {
    use std::time::Instant;

    // Deterministic work split for the timed points (tree nodes need `work-counters`).
    for count in [32, 64, 128] {
        for (label, s) in [
            ("one-hit", fixture::run(count, 128, BodyStorage::Ordered)),
            (
                "two-hit",
                fixture::run_ricochets(count, 128, BodyStorage::Ordered),
            ),
        ] {
            eprintln!(
                "ccd-work {label} count={count} irrelevant=128 pairs={} narrows={} sweeps={} tree_nodes={} impacts={}",
                s.pairs, s.narrows, s.sweeps, s.tree_nodes, s.impacts
            );
        }
    }
    for irrelevant in [0, 128] {
        for count in [1, 8, 16, 32, 64, 128] {
            for ccd in [false, true] {
                let mut times = Vec::new();
                for _ in 0..101 {
                    let mut w =
                        fixture::world_with_ccd(count, irrelevant, BodyStorage::Ordered, ccd);
                    let start = Instant::now();
                    let report = w.step(fixture::H).unwrap();
                    times.push(start.elapsed().as_nanos());
                    if ccd {
                        assert_eq!(report.swept_contacts, count as u64);
                        assert_eq!(report.geometry.ccd_budget_fallbacks, 0);
                        for i in 0..count {
                            let id = w.body_id(10_000 + i as u64).unwrap();
                            let ball = w.body(id).unwrap();
                            assert!((ball.position.0 + 0.15).abs() < 0.01);
                            assert!((ball.velocity.0 + 45.0).abs() < 0.01);
                        }
                    }
                    std::hint::black_box(report);
                }
                times.sort_unstable();
                eprintln!(
                    "ccd-first-tick count={count} irrelevant={irrelevant} ccd={ccd} median_us={:.2}",
                    times[times.len() / 2] as f64 / 1000.0
                );
            }
        }
    }
    for count in [32, 64, 128] {
        let mut times = Vec::new();
        for _ in 0..101 {
            let mut w = fixture::ricochet_world(count, 128, BodyStorage::Ordered);
            let start = Instant::now();
            let report = w.step(fixture::H).unwrap();
            times.push(start.elapsed().as_nanos());
            assert_eq!(report.swept_contacts, 2 * count as u64);
            assert_eq!(report.geometry.ccd_budget_fallbacks, 0);
            for i in 0..count {
                let id = w.body_id(10_000 + i as u64).unwrap();
                let ball = w.body(id).unwrap();
                assert!(ball.position.0.abs() < 0.01);
                assert!((ball.velocity.0 - 24.0).abs() < 0.01);
            }
            std::hint::black_box(report);
        }
        times.sort_unstable();
        eprintln!(
            "ccd-double-first-tick count={count} irrelevant=128 median_us={:.2}",
            times[times.len() / 2] as f64 / 1000.0
        );
    }
}
