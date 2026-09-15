use std::{collections::BTreeMap, hint::black_box, time::Instant};

type Axis3 = [i128; 3];
type Delta3 = [i128; 3];

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct DeltaGroup {
    sum: Delta3,
    count: u32,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct PriorBTreeAccumulator {
    groups: BTreeMap<Axis3, DeltaGroup>,
}

impl PriorBTreeAccumulator {
    fn accumulate(&mut self, axis: Axis3, delta: Delta3) {
        let group = self.groups.entry(axis).or_default();
        for (target, value) in group.sum.iter_mut().zip(delta) {
            *target = target
                .checked_add(value)
                .expect("benchmark delta fits i128");
        }
        group.count = group
            .count
            .checked_add(1)
            .expect("benchmark count fits u32");
    }

    fn combined(&self) -> (Delta3, u32) {
        combine_groups(self.groups.values().copied())
    }

    fn clear(&mut self) {
        self.groups.clear();
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct CompactVecAccumulator {
    groups: Vec<(Axis3, DeltaGroup)>,
}

impl CompactVecAccumulator {
    fn accumulate(&mut self, axis: Axis3, delta: Delta3) {
        let group = match self.groups.binary_search_by_key(&axis, |(key, _)| *key) {
            Ok(index) => &mut self.groups[index].1,
            Err(index) => {
                self.groups.insert(index, (axis, DeltaGroup::default()));
                &mut self.groups[index].1
            }
        };
        for (target, value) in group.sum.iter_mut().zip(delta) {
            *target = target
                .checked_add(value)
                .expect("benchmark delta fits i128");
        }
        group.count = group
            .count
            .checked_add(1)
            .expect("benchmark count fits u32");
    }

    fn combined(&self) -> (Delta3, u32) {
        combine_groups(self.groups.iter().map(|(_, group)| *group))
    }

    fn clear(&mut self) {
        self.groups.clear();
    }
}

fn combine_groups(groups: impl Iterator<Item = DeltaGroup>) -> (Delta3, u32) {
    let mut combined = [0_i128; 3];
    let mut group_count = 0_u32;
    for group in groups {
        let divisor = i128::from(group.count);
        for (target, value) in combined.iter_mut().zip(group.sum) {
            *target = target
                .checked_add(value / divisor)
                .expect("benchmark combined delta fits i128");
        }
        group_count += 1;
    }
    (combined, group_count)
}

const AXES: [Axis3; 6] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
];

fn feed_prior(accumulators: &mut [PriorBTreeAccumulator], pass: usize) -> u64 {
    let mut checksum = 0_u64;
    for active in 0..12_usize {
        let body_index = (active * 317 + pass * 13) % accumulators.len();
        let accumulator = &mut accumulators[body_index];
        let first_axis = AXES[(active + pass) % AXES.len()];
        let second_axis = AXES[(active * 5 + pass + 1) % AXES.len()];
        accumulator.accumulate(first_axis, [31 + active as i128, -7, 13]);
        accumulator.accumulate(first_axis, [11, 5 + pass as i128, -3]);
        accumulator.accumulate(second_axis, [-17, 19, 23 + active as i128]);
        let (combined, groups) = accumulator.combined();
        checksum = checksum.wrapping_add(combined[0].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(combined[1].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(combined[2].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(u64::from(groups));
    }
    checksum
}

fn feed_compact(accumulators: &mut [CompactVecAccumulator], pass: usize) -> u64 {
    let mut checksum = 0_u64;
    for active in 0..12_usize {
        let body_index = (active * 317 + pass * 13) % accumulators.len();
        let accumulator = &mut accumulators[body_index];
        let first_axis = AXES[(active + pass) % AXES.len()];
        let second_axis = AXES[(active * 5 + pass + 1) % AXES.len()];
        accumulator.accumulate(first_axis, [31 + active as i128, -7, 13]);
        accumulator.accumulate(first_axis, [11, 5 + pass as i128, -3]);
        accumulator.accumulate(second_axis, [-17, 19, 23 + active as i128]);
        let (combined, groups) = accumulator.combined();
        checksum = checksum.wrapping_add(combined[0].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(combined[1].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(combined[2].unsigned_abs() as u64);
        checksum = checksum.wrapping_add(u64::from(groups));
    }
    checksum
}

fn run_prior(accumulators: &mut [PriorBTreeAccumulator], passes: usize) -> u64 {
    let mut checksum = 0_u64;
    for pass in 0..passes {
        for accumulator in &mut *accumulators {
            accumulator.clear();
        }
        checksum = checksum.wrapping_add(feed_prior(accumulators, pass));
    }
    checksum
}

fn run_compact(accumulators: &mut [CompactVecAccumulator], passes: usize) -> u64 {
    let mut checksum = 0_u64;
    for pass in 0..passes {
        for accumulator in &mut *accumulators {
            accumulator.clear();
        }
        checksum = checksum.wrapping_add(feed_compact(accumulators, pass));
    }
    checksum
}

#[test]
fn compact_accumulator_matches_btree_grouping_and_order() {
    let mut prior = vec![PriorBTreeAccumulator::default(); 4_096];
    let mut compact = vec![CompactVecAccumulator::default(); 4_096];
    let prior_checksum = run_prior(&mut prior, 8);
    let compact_checksum = run_compact(&mut compact, 8);
    assert_eq!(compact_checksum, prior_checksum);

    for (prior, compact) in prior.iter().zip(&compact) {
        let prior_groups = prior.groups.iter().map(|(axis, group)| (*axis, *group));
        assert!(prior_groups.eq(compact.groups.iter().copied()));
        assert_eq!(compact.combined(), prior.combined());
    }
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn compact_solver_accumulator_benchmark() {
    let body_count = 4_096;
    let passes = 8;
    let iterations = 2_000;
    let mut prior = vec![PriorBTreeAccumulator::default(); body_count];
    let mut compact = vec![CompactVecAccumulator::default(); body_count];

    assert_eq!(
        run_compact(&mut compact, passes),
        run_prior(&mut prior, passes)
    );
    black_box(run_prior(black_box(&mut prior), passes));
    black_box(run_compact(black_box(&mut compact), passes));

    let prior_start = Instant::now();
    let mut prior_checksum = 0_u64;
    for _ in 0..iterations {
        prior_checksum = prior_checksum.wrapping_add(run_prior(black_box(&mut prior), passes));
    }
    let prior_elapsed = prior_start.elapsed();

    let compact_start = Instant::now();
    let mut compact_checksum = 0_u64;
    for _ in 0..iterations {
        compact_checksum =
            compact_checksum.wrapping_add(run_compact(black_box(&mut compact), passes));
    }
    let compact_elapsed = compact_start.elapsed();
    assert_eq!(compact_checksum, prior_checksum);

    let speedup = prior_elapsed.as_secs_f64() / compact_elapsed.as_secs_f64();
    eprintln!(
        "compact solver accumulator {body_count} bodies × {passes} passes × {iterations}: prior_btree={prior_elapsed:?}, compact_vec={compact_elapsed:?}, speedup={speedup:.2}x"
    );
}
