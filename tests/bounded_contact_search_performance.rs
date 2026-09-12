use std::{cmp::Ordering, collections::BTreeMap, error::Error, hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, OrientedBox3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d,
    RotatingContactSearchHit3d, RotationalSweepPair3d, SampledContactTime3d, Vec3i,
    obb_contact_seed, rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_contact_search,
};

const MAX_CACHED_COARSE_SAMPLES: usize = 4_096;

struct PriorIndexedCache {
    samples_by_body: Vec<Vec<OrientedBox3d>>,
    cached_entries: usize,
    denominator: u32,
}

impl PriorIndexedCache {
    fn new(body_count: usize, denominator: u32) -> Self {
        Self {
            samples_by_body: vec![Vec::new(); body_count],
            cached_entries: 0,
            denominator,
        }
    }

    fn sample(
        &mut self,
        body_index: usize,
        rigid_box: &RigidBox3d,
        config: RigidBoxFreeFlightConfig3d,
        numerator: u32,
    ) -> Result<OrientedBox3d, Box<dyn Error>> {
        let sample_index = usize::try_from(numerator - 1).expect("coarse numerator fits usize");
        if let Some(sampled) = self.samples_by_body[body_index].get(sample_index).copied() {
            return Ok(sampled);
        }

        let sampled = sample_rigid_box_free_flight(
            rigid_box,
            config,
            numerator,
            self.denominator,
        )?
        .oriented_box();
        if self.cached_entries < MAX_CACHED_COARSE_SAMPLES
            && sample_index == self.samples_by_body[body_index].len()
        {
            self.samples_by_body[body_index].push(sampled);
            self.cached_entries += 1;
        }
        Ok(sampled)
    }
}

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
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

fn config() -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d::new(
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
        64,
        0,
    )
}

fn early_hit_many_later_scene(obstacle_count: u64) -> Vec<RigidBox3d> {
    let mut boxes = Vec::with_capacity(usize::try_from(obstacle_count + 1).expect("small scene"));
    boxes.push(dynamic(
        10_000,
        Vec3i::new(-1_000, 0, 0),
        Vec3i::new(2_000, 0, 0),
    ));
    for offset in 0..obstacle_count {
        boxes.push(fixed(
            offset + 1,
            Vec3i::new(
                -800 + i32::try_from(offset).expect("benchmark offset fits i32") * 8,
                0,
                0,
            ),
        ));
    }
    boxes
}

fn prior_full_search(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    let pairs = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    let by_id = boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<_, _>>();
    let denominator = u32::from(config.sample_count);
    let mut samples = PriorIndexedCache::new(boxes.len(), denominator);
    let mut best = None;

    for pair in pairs {
        let left_index = *by_id.get(&pair.left).expect("candidate left body exists");
        let right_index = *by_id.get(&pair.right).expect("candidate right body exists");
        let left = &boxes[left_index];
        let right = &boxes[right_index];

        let hit = if let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? {
            Some(RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair,
                contact,
            })
        } else {
            prior_search_pair(
                left_index,
                left,
                right_index,
                right,
                pair,
                config,
                &mut samples,
            )?
        };

        let Some(hit) = hit else {
            continue;
        };
        if best.is_none_or(|current| compare_hits(hit, current) == Ordering::Less) {
            best = Some(hit);
        }
    }
    Ok(best)
}

fn prior_search_pair(
    left_index: usize,
    left: &RigidBox3d,
    right_index: usize,
    right: &RigidBox3d,
    pair: RotationalSweepPair3d,
    config: RotatingContactSearchConfig3d,
    samples: &mut PriorIndexedCache,
) -> Result<Option<RotatingContactSearchHit3d>, Box<dyn Error>> {
    let denominator = u32::from(config.sample_count);
    for numerator in 1..=denominator {
        let sampled_left = samples.sample(left_index, left, config.free_flight, numerator)?;
        let sampled_right = samples.sample(right_index, right, config.free_flight, numerator)?;
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
fn bounded_search_matches_full_prior_search() {
    let boxes = early_hit_many_later_scene(96);
    let search = config();
    assert_eq!(
        sampled_rotating_contact_search(&boxes, search).expect("bounded search"),
        prior_full_search(&boxes, search).expect("prior full search")
    );
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn best_hit_coarse_bound_benchmark() {
    let boxes = early_hit_many_later_scene(128);
    let search = config();
    let iterations = 24;

    let expected = prior_full_search(&boxes, search).expect("prior correctness run");
    assert_eq!(
        sampled_rotating_contact_search(&boxes, search).expect("bounded correctness run"),
        expected
    );

    black_box(prior_full_search(black_box(&boxes), search).expect("prior warmup"));
    black_box(sampled_rotating_contact_search(black_box(&boxes), search).expect("bounded warmup"));

    let prior_start = Instant::now();
    for _ in 0..iterations {
        black_box(prior_full_search(black_box(&boxes), search).expect("prior benchmark"));
    }
    let prior_elapsed = prior_start.elapsed();

    let bounded_start = Instant::now();
    for _ in 0..iterations {
        black_box(
            sampled_rotating_contact_search(black_box(&boxes), search).expect("bounded benchmark"),
        );
    }
    let bounded_elapsed = bounded_start.elapsed();
    let speedup = prior_elapsed.as_secs_f64() / bounded_elapsed.as_secs_f64();

    eprintln!(
        "best-hit bounded contact search 128 obstacles × {iterations}: prior_full_scan={prior_elapsed:?}, bounded_scan={bounded_elapsed:?}, speedup={speedup:.2}x"
    );
}
