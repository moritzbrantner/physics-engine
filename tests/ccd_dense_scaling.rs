//! Dense 60 Hz CCD workloads complement the independent, sparse lanes in
//! `ccd_scheduler_scaling`. This file measures the *whole step*, not only the
//! sweep kernel. Timings are advisory; observable behavior is checked below.

use physics_engine::{
    approximate::{Body, BodyStorage, Config, ContactEvent, Shape, Vector as V, World},
    work::{self, Counter},
};

const H: f64 = 1.0 / 60.0;
const RADIUS: f32 = 0.1;
const SEPARATION: f32 = 0.3;
const IRRELEVANT_STATICS: usize = 128;

#[derive(Clone, Copy)]
enum Case {
    CoMoving,
    Converging,
}

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
    impacts: u64,
    fallbacks: u64,
}

fn world(case: Case, count: usize, storage: BodyStorage) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: 8,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();

    // Match the sparse CCD baseline's unrelated-static count. Nothing in this
    // distant row can intersect a moving body's swept bounds.
    for i in 0..IRRELEVANT_STATICS {
        w.add_body(Body::keyed(
            2 + i as u64,
            Shape::Box(V(0.2, 0.2, 0.2)),
            V(10_000.0 + i as f32, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    }

    // Co-motion: every sphere starts separated, and all relative velocities
    // are exactly zero. A sufficiently long common displacement makes every
    // pair's *swept AABBs* overlap despite zero possible physical impacts.
    // Convergence: initially separated spheres aim through the same center in
    // one tick, forcing real, dense dynamic/dynamic CCD interactions.
    let ring_radius = 0.75 + count as f32 * 0.04;
    for i in 0..count {
        let (position, velocity) = match case {
            Case::CoMoving => (
                V(i as f32 * SEPARATION, 0.0, 0.0),
                V((count as f32 * SEPARATION + 1.0) * 60.0, 0.0, 0.0),
            ),
            Case::Converging => {
                let angle = std::f32::consts::TAU * i as f32 / count as f32;
                let (sin, cos) = angle.sin_cos();
                let speed = (ring_radius + 0.5) * 60.0;
                (
                    V(ring_radius * cos, ring_radius * sin, 0.0),
                    V(-speed * cos, -speed * sin, 0.0),
                )
            }
        };
        let mut body = Body::keyed(10_000 + i as u64, Shape::Sphere(RADIUS), position, 1.0);
        body.velocity = velocity;
        body.restitution = 1.0;
        body.friction = 0.0;
        body.ccd = true;
        body.sleep_allowed = false;
        w.add_body(body).unwrap();
    }
    w
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
    // The 64-body case exhausts every CCD budget on #484 and stresses
    // near-imminent event scheduling under both storage layouts.
    for count in [8, 16, 32, 64] {
        let ordered = sample(Case::Converging, count, BodyStorage::Ordered);
        let indexed = sample(Case::Converging, count, BodyStorage::Indexed);
        verify_converging(&ordered);
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
                "ccd-dense case={} n={count} potential_swept_aabb_pairs={swept_aabb_pairs} pairs={} narrows={} sweeps={} tree_nodes={} impacts={} fallbacks={} median_us={median_us:.2} p95_us={p95_us:.2}",
                case.label(),
                evidence.pair_tests,
                evidence.narrow_tests,
                evidence.sweep_queries,
                evidence.tree_nodes,
                evidence.impacts,
                evidence.fallbacks,
            );
        }
    }
}
