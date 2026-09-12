use std::{collections::BTreeMap, error::Error, hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ObbContactSeed3d, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, Vec3i,
    obb_contact_seed, rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_contact_search,
};

#[derive(Clone, Copy)]
struct LegacyBracket {
    lower_numerator: u32,
    upper_numerator: u32,
    denominator: u32,
    upper_contact: ObbContactSeed3d,
}

fn dynamic(
    id: u64,
    position: Vec3i,
    velocity: Vec3i,
    angular_velocity: AngularVelocity3d,
) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
    )
    .expect("valid dynamic benchmark box")
}

fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed benchmark box")
}

fn config(sample_count: u16, refinement_steps: u8) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
        sample_count,
        refinement_steps,
    )
}

fn dense_shared_body_scene(body_count: u64) -> Vec<RigidBox3d> {
    (0..body_count)
        .map(|id| {
            dynamic(
                id,
                Vec3i::new(i32::try_from(id).expect("benchmark id fits i32") * 10, 0, 0),
                Vec3i::new(1_000, 0, 0),
                AngularVelocity3d::new(370_000, -210_000, 490_000),
            )
        })
        .collect()
}

fn over_cap_chain_scene(body_count: u64) -> Vec<RigidBox3d> {
    (0..body_count)
        .map(|id| {
            dynamic(
                id,
                Vec3i::new(i32::try_from(id).expect("benchmark id fits i32") * 3, 0, 0),
                Vec3i::ZERO,
                AngularVelocity3d::default(),
            )
            .with_rotation_locked()
        })
        .collect()
}

fn legacy_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    let pairs = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    let mut best = None;

    for pair in pairs {
        let left = by_id.get(&pair.left).expect("broad phase left body exists");
        let right = by_id
            .get(&pair.right)
            .expect("broad phase right body exists");
        let Some(hit) = legacy_search_pair(left, right, pair, config)? else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current).is_lt()) {
            best = Some(hit);
        }
    }
    Ok(best)
}

fn legacy_search_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    if let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? {
        return Ok(Some(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        }));
    }

    let denominator = u32::from(config.sample_count);
    for numerator in 1..=denominator {
        let sampled_left =
            sample_rigid_box_free_flight(left, config.free_flight, numerator, denominator)?;
        let sampled_right =
            sample_rigid_box_free_flight(right, config.free_flight, numerator, denominator)?;
        let Some(contact) =
            obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
        else {
            continue;
        };
        return Ok(Some(legacy_refine(
            left,
            right,
            pair,
            config,
            LegacyBracket {
                lower_numerator: numerator - 1,
                upper_numerator: numerator,
                denominator,
                upper_contact: contact,
            },
        )?));
    }
    Ok(None)
}

fn legacy_refine(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    mut bracket: LegacyBracket,
) -> Result<RotatingContactSearchHit3d, Box<dyn Error>> {
    for _ in 0..config.refinement_steps {
        let midpoint_numerator = bracket.lower_numerator + bracket.upper_numerator;
        bracket.denominator *= 2;
        bracket.lower_numerator *= 2;
        bracket.upper_numerator *= 2;

        let sampled_left = sample_rigid_box_free_flight(
            left,
            config.free_flight,
            midpoint_numerator,
            bracket.denominator,
        )?;
        let sampled_right = sample_rigid_box_free_flight(
            right,
            config.free_flight,
            midpoint_numerator,
            bracket.denominator,
        )?;
        if let Some(contact) =
            obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())?
        {
            bracket.upper_numerator = midpoint_numerator;
            bracket.upper_contact = contact;
        } else {
            bracket.lower_numerator = midpoint_numerator;
        }
    }

    Ok(RotatingContactSearchHit3d {
        time: canonical_time(bracket.upper_numerator, bracket.denominator),
        pair,
        contact: bracket.upper_contact,
    })
}

fn canonical_time(numerator: u32, denominator: u32) -> SampledContactTime3d {
    let divisor = gcd(u64::from(numerator), u64::from(denominator));
    let divisor = u32::try_from(divisor).expect("u32 gcd fits u32");
    SampledContactTime3d {
        numerator: numerator / divisor,
        denominator: denominator / divisor,
    }
}

fn compare_hits(
    left: RotatingContactSearchHit3d,
    right: RotatingContactSearchHit3d,
) -> std::cmp::Ordering {
    let left_scaled = u64::from(left.time.numerator) * u64::from(right.time.denominator);
    let right_scaled = u64::from(right.time.numerator) * u64::from(left.time.denominator);
    left_scaled
        .cmp(&right_scaled)
        .then_with(|| left.pair.cmp(&right.pair))
}

fn gcd(mut left: u64, mut right: u64) -> u64 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[test]
fn cached_coarse_search_matches_the_prior_pair_by_pair_algorithm() {
    let hit_scene = [
        dynamic(
            10,
            Vec3i::new(-20, 0, 0),
            Vec3i::new(40, 0, 0),
            AngularVelocity3d::new(0, 0, 310_000),
        ),
        fixed(3, Vec3i::ZERO),
        fixed(7, Vec3i::new(30, 0, 0)),
    ];
    let hit_config = config(32, 4);
    assert_eq!(
        sampled_rotating_contact_search(&hit_scene, hit_config).expect("optimized hit search"),
        legacy_search(&hit_scene, hit_config).expect("legacy hit search")
    );

    let shared_contact_scene = [
        fixed(8, Vec3i::ZERO),
        dynamic(
            11,
            Vec3i::new(-10, 0, 0),
            Vec3i::new(20, 0, 0),
            AngularVelocity3d::default(),
        ),
        fixed(3, Vec3i::ZERO),
    ];
    let shared_contact_config = config(8, 2);
    let optimized_shared =
        sampled_rotating_contact_search(&shared_contact_scene, shared_contact_config)
            .expect("optimized shared-contact search")
            .expect("shared moving body contacts both fixed candidates");
    let legacy_shared = legacy_search(&shared_contact_scene, shared_contact_config)
        .expect("legacy shared-contact search")
        .expect("legacy shared moving body contacts both fixed candidates");
    assert_eq!(optimized_shared, legacy_shared);
    assert_eq!(
        optimized_shared.pair,
        RotationalSweepPair3d {
            left: BodyId(3),
            right: BodyId(11),
        }
    );

    let dense_scene = dense_shared_body_scene(48);
    let dense_config = config(32, 3);
    assert_eq!(
        sampled_rotating_contact_search(&dense_scene, dense_config)
            .expect("optimized dense search"),
        legacy_search(&dense_scene, dense_config).expect("legacy dense search")
    );

    // 65 bodies × 64 coarse fractions requires 4,160 distinct body/fraction keys. Adjacent
    // broad-phase candidates share bodies, so this necessarily exercises cache reuse and then the
    // streaming fallback after the 4,096-entry cache cap is reached.
    let over_cap_scene = over_cap_chain_scene(65);
    let over_cap_config = config(64, 0);
    assert_eq!(
        sampled_rotating_contact_search(&over_cap_scene, over_cap_config)
            .expect("optimized over-cap search"),
        legacy_search(&over_cap_scene, over_cap_config).expect("legacy over-cap search")
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn rotating_contact_coarse_cache_benchmark() {
    let boxes = dense_shared_body_scene(64);
    let search = config(64, 2);
    let iterations = 8;

    let expected = legacy_search(&boxes, search).expect("legacy search");
    assert_eq!(
        sampled_rotating_contact_search(&boxes, search).expect("optimized search"),
        expected
    );

    black_box(legacy_search(black_box(&boxes), search).expect("legacy warmup"));
    black_box(
        sampled_rotating_contact_search(black_box(&boxes), search).expect("optimized warmup"),
    );

    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(legacy_search(black_box(&boxes), search).expect("legacy benchmark"));
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_contact_search(black_box(&boxes), search)
                .expect("optimized benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "rotating contact coarse cache 64 bodies × {iterations}: prior_pair_sampling={legacy_elapsed:?}, cached_sampling={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn rotating_contact_cache_over_cap_benchmark() {
    let boxes = over_cap_chain_scene(96);
    let search = config(64, 0);
    let iterations = 40;

    let expected = legacy_search(&boxes, search).expect("legacy over-cap search");
    assert_eq!(
        sampled_rotating_contact_search(&boxes, search).expect("optimized over-cap search"),
        expected
    );

    black_box(legacy_search(black_box(&boxes), search).expect("legacy over-cap warmup"));
    black_box(
        sampled_rotating_contact_search(black_box(&boxes), search)
            .expect("optimized over-cap warmup"),
    );

    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(legacy_search(black_box(&boxes), search).expect("legacy over-cap benchmark"));
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_contact_search(black_box(&boxes), search)
                .expect("optimized over-cap benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "rotating contact over-cap cache 96 bodies × {iterations}: prior_pair_sampling={legacy_elapsed:?}, capped_cache={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}
