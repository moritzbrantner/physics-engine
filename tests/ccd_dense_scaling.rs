//! Dense 60 Hz CCD workloads complement the independent, sparse lanes in
//! `ccd_scheduler_scaling`. This file measures the *whole step*, not only the
//! sweep kernel. Timings are advisory; observable behavior is checked below.

use physics_engine::{
    approximate::{BodyStorage, ContactEvent, World},
    work::{self, Counter},
};

#[path = "support/ccd_dense.rs"]
mod dense_fixture;
use dense_fixture::{Case, H, IRRELEVANT_STATICS, SEPARATION, world};

impl Case {
    fn label(self) -> &'static str {
        match self {
            Self::CoMoving => "co-moving-aabb-overlap",
            Self::Converging => "converging-true-impacts",
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
struct Sample {
    states: Vec<([u32; 3], [u32; 3])>,
    contacts: Vec<ContactEvent>,
    pair_tests: u64,
    narrow_tests: u64,
    sweep_queries: u64,
    tree_nodes: u64,
    heap_pushes: u64,
    heap_stale_pops: u64,
    impacts: u64,
    fallbacks: u64,
}

fn states(w: &World, count: usize) -> Vec<([u32; 3], [u32; 3])> {
    (0..count)
        .map(|i| {
            let body = w.body(w.body_id(10_000 + i as u64).unwrap()).unwrap();
            (
                [
                    body.position.0.to_bits(),
                    body.position.1.to_bits(),
                    body.position.2.to_bits(),
                ],
                [
                    body.velocity.0.to_bits(),
                    body.velocity.1.to_bits(),
                    body.velocity.2.to_bits(),
                ],
            )
        })
        .collect()
}

fn sample(case: Case, count: usize, storage: BodyStorage) -> Sample {
    let mut w = world(case, count, storage);
    work::reset();
    let r = w.step(H).unwrap();
    Sample {
        states: states(&w, count),
        contacts: r.contact_events,
        pair_tests: r.pair_tests,
        narrow_tests: r.narrow_tests,
        sweep_queries: r.geometry.sweep_queries,
        tree_nodes: work::get(Counter::TreeNodes),
        heap_pushes: work::get(Counter::CcdHeapPushes),
        heap_stale_pops: work::get(Counter::CcdHeapStalePops),
        impacts: r.swept_contacts,
        fallbacks: r.geometry.ccd_budget_fallbacks,
    }
}

fn verify_co_motion(s: &Sample, count: usize) {
    assert_eq!(s.impacts, 0, "spurious CCD collision: {s:?}");
    assert_eq!(s.fallbacks, 0, "unexpected CCD fallback: {s:?}");
    assert!(s.contacts.is_empty(), "spurious contact event: {s:?}");
    let distance = (count as f32 * SEPARATION + 1.0) * 60.0 * H as f32;
    let speed = (count as f32 * SEPARATION + 1.0) * 60.0;
    for (i, (p, v)) in s.states.iter().enumerate() {
        let expected = i as f32 * SEPARATION + distance;
        assert!((f32::from_bits(p[0]) - expected).abs() < 0.01);
        assert_eq!(f32::from_bits(p[1]), 0.0);
        assert_eq!(f32::from_bits(p[2]), 0.0);
        assert!((f32::from_bits(v[0]) - speed).abs() < 0.01);
        assert_eq!(f32::from_bits(v[1]), 0.0);
        assert_eq!(f32::from_bits(v[2]), 0.0);
    }
}

fn verify_converging(s: &Sample) {
    assert!(s.impacts > 0, "missing real CCD impacts: {s:?}");
    for (p, v) in &s.states {
        assert!(p.iter().all(|x| f32::from_bits(*x).is_finite()));
        assert!(v.iter().all(|x| f32::from_bits(*x).is_finite()));
    }
}

#[test]
fn dense_co_moving_sweeps_never_create_collisions() {
    for count in [8, 16, 32, 64, 128] {
        let ordered = sample(Case::CoMoving, count, BodyStorage::Ordered);
        let indexed = sample(Case::CoMoving, count, BodyStorage::Indexed);
        verify_co_motion(&ordered, count);
        assert_eq!(ordered, indexed, "storage divergence at n={count}");
    }
}

#[test]
fn converging_sweeps_produce_deterministic_real_impacts() {
    // The 64-body case exercises repeated near-imminent rescheduling and CCD
    // budget exhaustion without permitting a change in admitted-event count.
    for count in [8, 16, 32, 64] {
        let ordered = sample(Case::Converging, count, BodyStorage::Ordered);
        let indexed = sample(Case::Converging, count, BodyStorage::Indexed);
        verify_converging(&ordered);
        if count == 64 {
            assert_eq!(ordered.impacts, 268, "dense event chronology changed");
            assert_eq!(ordered.fallbacks, 64, "dense budget behavior changed");
        }
        assert_eq!(ordered, indexed, "storage divergence at n={count}");
    }
}

/// Advisory end-to-end *first-step* timings, not a portable CI speed gate.
/// Setup, validation and counter inspection are excluded from the timed region.
/// Run alongside the sparse control:
/// `cargo test --release --locked --test ccd_scheduler_scaling -- --ignored --nocapture`
#[test]
#[ignore = "advisory CPU timings; run explicitly with --ignored --nocapture"]
fn print_dense_first_tick_timings() {
    use std::time::Instant;

    const SAMPLES: usize = 31;
    eprintln!(
        "ccd-dense-env os={} arch={} profile={} work_counters={} samples={SAMPLES} statics={IRRELEVANT_STATICS}",
        std::env::consts::OS,
        std::env::consts::ARCH,
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        },
        cfg!(feature = "work-counters"),
    );
    for (case, counts) in [
        (Case::CoMoving, &[32, 64, 128][..]),
        (Case::Converging, &[8, 16, 32, 64][..]),
    ] {
        for &count in counts {
            let evidence = sample(case, count, BodyStorage::Ordered);
            match case {
                Case::CoMoving => verify_co_motion(&evidence, count),
                Case::Converging => verify_converging(&evidence),
            }

            let mut timings = Vec::with_capacity(SAMPLES);
            for _ in 0..SAMPLES {
                let mut w = world(case, count, BodyStorage::Ordered);
                let start = Instant::now();
                let report = w.step(H).unwrap();
                timings.push(start.elapsed().as_nanos());
                assert_eq!(report.swept_contacts, evidence.impacts);
                assert_eq!(report.geometry.ccd_budget_fallbacks, evidence.fallbacks);
                std::hint::black_box(report);
            }
            timings.sort_unstable();
            let median_us = timings[SAMPLES / 2] as f64 / 1000.0;
            let p95_us = timings[(SAMPLES * 95).div_ceil(100) - 1] as f64 / 1000.0;
            let swept_aabb_pairs = count * (count - 1) / 2;
            eprintln!(
                "ccd-dense case={} n={count} potential_swept_aabb_pairs={swept_aabb_pairs} pairs={} narrows={} sweeps={} tree_nodes={} heap_pushes={} heap_stale_pops={} impacts={} fallbacks={} median_us={median_us:.2} p95_us={p95_us:.2}",
                case.label(),
                evidence.pair_tests,
                evidence.narrow_tests,
                evidence.sweep_queries,
                evidence.tree_nodes,
                evidence.heap_pushes,
                evidence.heap_stale_pops,
                evidence.impacts,
                evidence.fallbacks,
            );
        }
    }
}
