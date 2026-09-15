use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    rotational_sweep_candidate_pairs, sampled_rotating_recontact_search,
};

const SIZES: [u64; 5] = [16, 32, 64, 100, 128];
const TRIALS: usize = 3;

#[derive(Clone, Copy, Debug)]
struct Measurement {
    bodies: u64,
    candidate_pairs: usize,
    median_ns: u128,
}

fn stationary_box(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid scaling benchmark body")
    .with_rotation_locked()
}

fn touching_chain(body_count: u64) -> Vec<RigidBox3d> {
    (0..body_count)
        .map(|id| {
            stationary_box(
                id,
                Vec3i::new(i32::try_from(id).expect("benchmark id fits i32") * 2, 0, 0),
            )
        })
        .collect()
}

fn dense_overlap(body_count: u64) -> Vec<RigidBox3d> {
    (0..body_count)
        .map(|id| stationary_box(id, Vec3i::ZERO))
        .collect()
}

fn config() -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
        16,
        0,
    )
}

fn measure(boxes: &[RigidBox3d], search: RotatingContactSearchConfig3d) -> Measurement {
    let candidate_pairs = rotational_sweep_candidate_pairs(boxes, search.free_flight)
        .expect("candidate-pair construction")
        .len();

    black_box(
        sampled_rotating_recontact_search(black_box(boxes), search)
            .expect("recontact scaling warmup"),
    );

    let mut trials = Vec::with_capacity(TRIALS);
    for _ in 0..TRIALS {
        let started = Instant::now();
        black_box(
            sampled_rotating_recontact_search(black_box(boxes), search)
                .expect("recontact scaling benchmark"),
        );
        trials.push(started.elapsed().as_nanos());
    }
    trials.sort_unstable();

    Measurement {
        bodies: u64::try_from(boxes.len()).expect("benchmark body count fits u64"),
        candidate_pairs,
        median_ns: trials[TRIALS / 2],
    }
}

fn print_series(name: &str, measurements: &[Measurement]) {
    eprintln!("recontact_scaling,pattern,bodies,candidate_pairs,median_ns,ns_per_pair,time_ratio,empirical_exponent");
    for (index, measurement) in measurements.iter().enumerate() {
        let ns_per_pair = if measurement.candidate_pairs == 0 {
            0.0
        } else {
            measurement.median_ns as f64 / measurement.candidate_pairs as f64
        };
        let (time_ratio, exponent) = if let Some(previous) = index.checked_sub(1).and_then(|previous| measurements.get(previous)) {
            let time_ratio = measurement.median_ns as f64 / previous.median_ns.max(1) as f64;
            let body_ratio = measurement.bodies as f64 / previous.bodies as f64;
            let exponent = if time_ratio > 0.0 && body_ratio > 1.0 {
                time_ratio.ln() / body_ratio.ln()
            } else {
                0.0
            };
            (time_ratio, exponent)
        } else {
            (1.0, 0.0)
        };
        eprintln!(
            "recontact_scaling,{name},{},{},{},{ns_per_pair:.2},{time_ratio:.3},{exponent:.3}",
            measurement.bodies,
            measurement.candidate_pairs,
            measurement.median_ns,
        );
    }
}

#[test]
fn scaling_fixtures_expose_linear_and_quadratic_candidate_shapes() {
    let search = config();
    for body_count in SIZES {
        let chain = touching_chain(body_count);
        let chain_pairs = rotational_sweep_candidate_pairs(&chain, search.free_flight)
            .expect("touching-chain candidate pairs");
        assert!(
            chain_pairs.len() >= usize::try_from(body_count.saturating_sub(1)).unwrap(),
            "touching chain must retain at least the adjacent contacts"
        );
        assert!(
            chain_pairs.len() <= usize::try_from(body_count * 2).unwrap(),
            "touching chain must remain sparse enough to diagnose near-linear scaling"
        );

        let dense = dense_overlap(body_count);
        let dense_pairs = rotational_sweep_candidate_pairs(&dense, search.free_flight)
            .expect("dense-overlap candidate pairs");
        let expected_dense_pairs = usize::try_from(body_count * (body_count - 1) / 2).unwrap();
        assert_eq!(
            dense_pairs.len(),
            expected_dense_pairs,
            "all-overlap fixture must really exercise the quadratic candidate-pair boundary"
        );
    }
}

#[test]
#[ignore = "scaling microbenchmark; run explicitly in release mode"]
fn rotating_recontact_scaling_matrix() {
    let search = config();

    let chain: Vec<_> = SIZES
        .into_iter()
        .map(|body_count| measure(&touching_chain(body_count), search))
        .collect();
    let dense: Vec<_> = SIZES
        .into_iter()
        .map(|body_count| measure(&dense_overlap(body_count), search))
        .collect();

    print_series("touching_chain", &chain);
    print_series("dense_overlap", &dense);

    let hundred = dense
        .iter()
        .find(|measurement| measurement.bodies == 100)
        .expect("100-body diagnostic is present");
    assert_eq!(hundred.candidate_pairs, 4_950);
}
