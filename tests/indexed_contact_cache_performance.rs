use std::{cmp::Ordering, collections::BTreeMap, error::Error, hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ObbContactSeed3d, Orientation3d, OrientedBox3d,
    RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, Vec3i,
    obb_contact_seed, rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_contact_search,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

#[derive(Default)]
struct PriorCoarseSampleCache3d {
    samples: BTreeMap<(BodyId, u32, u32), OrientedBox3d>,
}

impl PriorCoarseSampleCache3d {
    fn sample(
        &mut self,
        rigid_box: &RigidBox3d,
        config: RigidBoxFreeFlightConfig3d,
        numerator: u32,
        denominator: u32,
    ) -> Result<OrientedBox3d, Box<dyn Error>> {
        let key = (rigid_box.body().id(), numerator, denominator);
        if let Some(sampled) = self.samples.get(&key).copied() {
            return Ok(sampled);
        }

        let sampled =
            sample_rigid_box_free_flight(rigid_box, config, numerator, denominator)?.oriented_box();
        if self.samples.len() < MAX_CACHED_COARSE_SAMPLES {
            self.samples.insert(key, sampled);
        }
        Ok(sampled)
    }
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

fn config(sample_count: u16) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
        sample_count,
        0,
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

fn prior_cached_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    let pairs = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    let mut coarse_samples = PriorCoarseSampleCache3d::default();
    let mut best = None;

    for pair in pairs {
        let left = by_id.get(&pair.left).expect("broad phase left body exists");
        let right = by_id
            .get(&pair.right)
            .expect("broad phase right body exists");
        let Some(hit) = prior_search_pair(left, right, pair, config, &mut coarse_samples)? else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
            best = Some(hit);
        }
    }
    Ok(best)
}

fn prior_search_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    coarse_samples: &mut PriorCoarseSampleCache3d,
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
            coarse_samples.sample(left, config.free_flight, numerator, denominator)?;
        let sampled_right =
            coarse_samples.sample(right, config.free_flight, numerator, denominator)?;
        let Some(contact) = obb_contact_seed(sampled_left, sampled_right)? else {
            continue;
        };
        return Ok(Some(RotatingContactSearchHit3d {
            time: canonical_time(numerator, denominator),
            pair,
            contact,
        }));
    }
    Ok(None)
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
fn indexed_contact_cache_matches_prior_btree_cache() {
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
    let hit_config = config(32);
    assert_eq!(
        sampled_rotating_contact_search(&hit_scene, hit_config).expect("indexed hit search"),
        prior_cached_search(&hit_scene, hit_config).expect("prior cached hit search")
    );

    let tie_scene = [
        fixed(8, Vec3i::ZERO),
        dynamic(
            11,
            Vec3i::new(-10, 0, 0),
            Vec3i::new(20, 0, 0),
            AngularVelocity3d::default(),
        ),
        fixed(3, Vec3i::ZERO),
    ];
    let tie_config = config(8);
    assert_eq!(
        sampled_rotating_contact_search(&tie_scene, tie_config).expect("indexed tie search"),
        prior_cached_search(&tie_scene, tie_config).expect("prior cached tie search")
    );

    let dense_scene = dense_shared_body_scene(48);
    let dense_config = config(32);
    assert_eq!(
        sampled_rotating_contact_search(&dense_scene, dense_config).expect("indexed dense search"),
        prior_cached_search(&dense_scene, dense_config).expect("prior cached dense search")
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn indexed_contact_cache_benchmark() {
    let boxes = dense_shared_body_scene(64);
    let search = config(64);
    let iterations = 12;

    let expected = prior_cached_search(&boxes, search).expect("prior cached correctness run");
    assert_eq!(
        sampled_rotating_contact_search(&boxes, search).expect("indexed correctness run"),
        expected
    );

    black_box(prior_cached_search(black_box(&boxes), search).expect("prior cached warmup"));
    black_box(sampled_rotating_contact_search(black_box(&boxes), search).expect("indexed warmup"));

    let prior_start = Instant::now();
    for _ in 0..iterations {
        black_box(prior_cached_search(black_box(&boxes), search).expect("prior cached benchmark"));
    }
    let prior_elapsed = prior_start.elapsed();

    let indexed_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_contact_search(black_box(&boxes), search).expect("indexed benchmark"),
        );
    }
    let indexed_elapsed = indexed_start.elapsed();
    let speedup = prior_elapsed.as_secs_f64() / indexed_elapsed.as_secs_f64();

    eprintln!(
        "indexed rotating contact cache 64 bodies × {iterations}: prior_btree_cache={prior_elapsed:?}, indexed_cache={indexed_elapsed:?}, speedup={speedup:.2}x"
    );
}
