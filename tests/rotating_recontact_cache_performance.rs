use std::{cmp::Ordering, collections::BTreeMap, error::Error, hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ObbContactSeed3d, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, Vec3i,
    obb_contact_seed, rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_recontact_search,
};

#[derive(Clone, Copy)]
struct LegacyBracket {
    clear_numerator: u32,
    contact_numerator: u32,
    denominator: u32,
    contact: ObbContactSeed3d,
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

fn config(
    gravity: Vec3i,
    sample_count: u16,
    refinement_steps: u8,
) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(gravity, 1, 1),
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

fn legacy_recontact_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    let pairs = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    if pairs.is_empty() {
        return Ok(None);
    }

    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<BodyId, &RigidBox3d>>();
    let mut best = None;
    for pair in pairs {
        let left = by_id.get(&pair.left).expect("broad phase left body exists");
        let right = by_id
            .get(&pair.right)
            .expect("broad phase right body exists");
        let Some(hit) = legacy_search_pair(left, right, pair, config)? else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
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
    let initially_contacting =
        obb_contact_seed(left.oriented_box(), right.oriented_box())?.is_some();
    let denominator = u32::from(config.sample_count);
    let mut last_clear = if initially_contacting { None } else { Some(0) };

    for numerator in 1..=denominator {
        let sampled_left =
            sample_rigid_box_free_flight(left, config.free_flight, numerator, denominator)?;
        let sampled_right =
            sample_rigid_box_free_flight(right, config.free_flight, numerator, denominator)?;
        match obb_contact_seed(sampled_left.oriented_box(), sampled_right.oriented_box())? {
            Some(contact) => {
                let Some(clear_numerator) = last_clear else {
                    continue;
                };
                return Ok(Some(legacy_refine_contact_bracket(
                    left,
                    right,
                    pair,
                    config,
                    LegacyBracket {
                        clear_numerator,
                        contact_numerator: numerator,
                        denominator,
                        contact,
                    },
                )?));
            }
            None => last_clear = Some(numerator),
        }
    }
    Ok(None)
}

fn legacy_refine_contact_bracket(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    mut bracket: LegacyBracket,
) -> Result<RotatingContactSearchHit3d, Box<dyn Error>> {
    for _ in 0..config.refinement_steps {
        let midpoint_numerator = bracket.clear_numerator + bracket.contact_numerator;
        bracket.denominator *= 2;
        bracket.clear_numerator *= 2;
        bracket.contact_numerator *= 2;

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
            bracket.contact_numerator = midpoint_numerator;
            bracket.contact = contact;
        } else {
            bracket.clear_numerator = midpoint_numerator;
        }
    }

    Ok(RotatingContactSearchHit3d {
        time: canonical_time(bracket.contact_numerator, bracket.denominator),
        pair,
        contact: bracket.contact,
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

fn compare_hits(left: RotatingContactSearchHit3d, right: RotatingContactSearchHit3d) -> Ordering {
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
fn cached_recontact_search_matches_the_prior_pair_by_pair_algorithm() {
    let touching = dynamic(
        1,
        Vec3i::new(-2, 0, 0),
        Vec3i::new(-8, 0, 0),
        AngularVelocity3d::default(),
    );
    let obstacle = fixed(2, Vec3i::ZERO);
    let recontact_config = config(Vec3i::new(16, 0, 0), 8, 3);
    let recontact_scene = [touching, obstacle];
    assert_eq!(
        sampled_rotating_recontact_search(&recontact_scene, recontact_config)
            .expect("optimized clear/recontact search"),
        legacy_recontact_search(&recontact_scene, recontact_config)
            .expect("legacy clear/recontact search")
    );

    let shared_recontact_scene = [
        fixed(8, Vec3i::ZERO),
        dynamic(
            11,
            Vec3i::new(-2, 0, 0),
            Vec3i::new(-8, 0, 0),
            AngularVelocity3d::default(),
        ),
        fixed(3, Vec3i::ZERO),
    ];
    let shared_recontact_config = config(Vec3i::new(16, 0, 0), 8, 3);
    let optimized_shared =
        sampled_rotating_recontact_search(&shared_recontact_scene, shared_recontact_config)
            .expect("optimized shared recontact search")
            .expect("shared moving body clears and recontacts both fixed candidates");
    let legacy_shared = legacy_recontact_search(&shared_recontact_scene, shared_recontact_config)
        .expect("legacy shared recontact search")
        .expect("legacy shared moving body clears and recontacts both fixed candidates");
    assert_eq!(optimized_shared, legacy_shared);
    assert_eq!(
        optimized_shared.pair,
        RotationalSweepPair3d {
            left: BodyId(3),
            right: BodyId(11),
        }
    );

    let dense_scene = dense_shared_body_scene(48);
    let dense_config = config(Vec3i::ZERO, 32, 3);
    assert_eq!(
        sampled_rotating_recontact_search(&dense_scene, dense_config)
            .expect("optimized dense recontact search"),
        legacy_recontact_search(&dense_scene, dense_config).expect("legacy dense recontact search")
    );

    let over_cap_scene = over_cap_chain_scene(65);
    let over_cap_config = config(Vec3i::ZERO, 64, 0);
    assert_eq!(
        sampled_rotating_recontact_search(&over_cap_scene, over_cap_config)
            .expect("optimized over-cap recontact search"),
        legacy_recontact_search(&over_cap_scene, over_cap_config)
            .expect("legacy over-cap recontact search")
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn rotating_recontact_coarse_cache_benchmark() {
    let boxes = dense_shared_body_scene(64);
    let search = config(Vec3i::ZERO, 64, 2);
    let iterations = 8;

    let expected = legacy_recontact_search(&boxes, search).expect("legacy search");
    assert_eq!(
        sampled_rotating_recontact_search(&boxes, search).expect("optimized search"),
        expected
    );

    black_box(legacy_recontact_search(black_box(&boxes), search).expect("legacy warmup"));
    black_box(
        sampled_rotating_recontact_search(black_box(&boxes), search).expect("optimized warmup"),
    );

    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(legacy_recontact_search(black_box(&boxes), search).expect("legacy benchmark"));
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_recontact_search(black_box(&boxes), search)
                .expect("optimized benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "rotating recontact coarse cache 64 bodies × {iterations}: prior_pair_sampling={legacy_elapsed:?}, cached_sampling={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn rotating_recontact_cache_over_cap_benchmark() {
    let boxes = over_cap_chain_scene(96);
    let search = config(Vec3i::ZERO, 64, 0);
    let iterations = 40;

    let expected = legacy_recontact_search(&boxes, search).expect("legacy over-cap search");
    assert_eq!(
        sampled_rotating_recontact_search(&boxes, search).expect("optimized over-cap search"),
        expected
    );

    black_box(legacy_recontact_search(black_box(&boxes), search).expect("legacy over-cap warmup"));
    black_box(
        sampled_rotating_recontact_search(black_box(&boxes), search)
            .expect("optimized over-cap warmup"),
    );

    let legacy_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            legacy_recontact_search(black_box(&boxes), search).expect("legacy over-cap benchmark"),
        );
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_recontact_search(black_box(&boxes), search)
                .expect("optimized over-cap benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "rotating recontact over-cap cache 96 bodies × {iterations}: prior_pair_sampling={legacy_elapsed:?}, capped_cache={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}
