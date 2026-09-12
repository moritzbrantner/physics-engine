use std::{collections::BTreeMap, hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, BodyKind, Orientation3d, RigidBody, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RotatingContactFrontier3d, RotatingContactResponse3d,
    RotatingContactResponseError3d, RotatingContactSearchConfig3d, Vec3i,
    earliest_rotating_contact_frontier, resolve_obb_contact, resolve_rotating_contact_frontier,
};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct BodyDelta3d {
    position: [i128; 3],
    linear_velocity: [i128; 3],
    angular_velocity: [i128; 3],
}

impl BodyDelta3d {
    fn between(before: &RigidBox3d, after: &RigidBox3d) -> Self {
        let before_body = before.body();
        let after_body = after.body();
        let before_angular = before.angular().angular_velocity;
        let after_angular = after.angular().angular_velocity;
        Self {
            position: [
                i128::from(after_body.position().x) - i128::from(before_body.position().x),
                i128::from(after_body.position().y) - i128::from(before_body.position().y),
                i128::from(after_body.position().z) - i128::from(before_body.position().z),
            ],
            linear_velocity: [
                i128::from(after_body.velocity().x) - i128::from(before_body.velocity().x),
                i128::from(after_body.velocity().y) - i128::from(before_body.velocity().y),
                i128::from(after_body.velocity().z) - i128::from(before_body.velocity().z),
            ],
            angular_velocity: [
                i128::from(after_angular.x) - i128::from(before_angular.x),
                i128::from(after_angular.y) - i128::from(before_angular.y),
                i128::from(after_angular.z) - i128::from(before_angular.z),
            ],
        }
    }

    fn is_zero(self) -> bool {
        self.position == [0; 3] && self.linear_velocity == [0; 3] && self.angular_velocity == [0; 3]
    }

    fn checked_add(
        &mut self,
        other: Self,
        id: BodyId,
    ) -> Result<(), RotatingContactResponseError3d> {
        add_vector(&mut self.position, other.position, id)?;
        add_vector(&mut self.linear_velocity, other.linear_velocity, id)?;
        add_vector(&mut self.angular_velocity, other.angular_velocity, id)
    }

    fn divided(self, divisor: u32, id: BodyId) -> Result<Self, RotatingContactResponseError3d> {
        if divisor == 0 {
            return Err(RotatingContactResponseError3d::ArithmeticOverflow(id));
        }
        let divisor = i128::from(divisor);
        Ok(Self {
            position: divide_vector(self.position, divisor, id)?,
            linear_velocity: divide_vector(self.linear_velocity, divisor, id)?,
            angular_velocity: divide_vector(self.angular_velocity, divisor, id)?,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct DeltaGroup3d {
    sum: BodyDelta3d,
    count: u32,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct BodyDeltaAccumulator3d {
    groups: BTreeMap<[i128; 3], DeltaGroup3d>,
}

impl BodyDeltaAccumulator3d {
    fn accumulate(
        &mut self,
        relative_axis: [i128; 3],
        before: &RigidBox3d,
        after: &RigidBox3d,
    ) -> Result<(), RotatingContactResponseError3d> {
        let id = before.body().id();
        let delta = BodyDelta3d::between(before, after);
        if delta.is_zero() {
            return Ok(());
        }
        let key = primitive_axis(relative_axis, id)?;
        let group = self.groups.entry(key).or_default();
        group.sum.checked_add(delta, id)?;
        group.count = group
            .count
            .checked_add(1)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
        Ok(())
    }

    fn combined(self, id: BodyId) -> Result<BodyDelta3d, RotatingContactResponseError3d> {
        let mut combined = BodyDelta3d::default();
        for group in self.groups.into_values() {
            combined.checked_add(group.sum.divided(group.count, id)?, id)?;
        }
        Ok(combined)
    }
}

fn legacy_resolve_rotating_contact_frontier(
    frontier: RotatingContactFrontier3d,
    solver_passes: u8,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    if solver_passes == 0 {
        return Err(RotatingContactResponseError3d::ZeroSolverPasses);
    }

    let indices = frontier
        .boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<_, _>>();
    let mut boxes = frontier.boxes;
    let mut passes_used = 0_u8;

    for pass in 0..solver_passes {
        let snapshot = boxes.clone();
        let mut deltas = vec![BodyDeltaAccumulator3d::default(); snapshot.len()];

        for contact in &frontier.contacts {
            let left_index = *indices.get(&contact.pair.left).ok_or(
                RotatingContactResponseError3d::MissingBody(contact.pair.left),
            )?;
            let right_index = *indices.get(&contact.pair.right).ok_or(
                RotatingContactResponseError3d::MissingBody(contact.pair.right),
            )?;
            let response = resolve_obb_contact(
                snapshot[left_index].clone(),
                snapshot[right_index].clone(),
                pass == 0,
            )?;
            let Some(resolved_contact) = response.contact else {
                continue;
            };
            deltas[left_index].accumulate(
                negate_axis(resolved_contact.axis, contact.pair.left)?,
                &snapshot[left_index],
                &response.left,
            )?;
            deltas[right_index].accumulate(
                resolved_contact.axis,
                &snapshot[right_index],
                &response.right,
            )?;
        }

        let mut combined = Vec::with_capacity(boxes.len());
        for (rigid_box, accumulator) in snapshot.iter().zip(deltas) {
            combined.push(accumulator.combined(rigid_box.body().id())?);
        }
        if combined.iter().copied().all(BodyDelta3d::is_zero) {
            break;
        }
        for (rigid_box, delta) in boxes.iter_mut().zip(combined) {
            apply_delta(rigid_box, delta)?;
        }
        passes_used = passes_used.checked_add(1).ok_or(
            RotatingContactResponseError3d::ArithmeticOverflow(
                boxes
                    .first()
                    .map_or(BodyId(0), |rigid_box| rigid_box.body().id()),
            ),
        )?;
    }

    Ok(RotatingContactResponse3d {
        boxes,
        time: frontier.time,
        contacts: frontier.contacts,
        remaining_numerator: frontier.remaining_numerator,
        passes_used,
    })
}

fn primitive_axis(
    axis: [i128; 3],
    id: BodyId,
) -> Result<[i128; 3], RotatingContactResponseError3d> {
    let divisor = axis
        .into_iter()
        .map(i128::unsigned_abs)
        .fold(0_u128, gcd_u128);
    if divisor == 0 {
        return Err(RotatingContactResponseError3d::ArithmeticOverflow(id));
    }
    let divisor = i128::try_from(divisor)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow(id))?;
    Ok([axis[0] / divisor, axis[1] / divisor, axis[2] / divisor])
}

fn gcd_u128(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn negate_axis(axis: [i128; 3], id: BodyId) -> Result<[i128; 3], RotatingContactResponseError3d> {
    Ok([
        axis[0]
            .checked_neg()
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?,
        axis[1]
            .checked_neg()
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?,
        axis[2]
            .checked_neg()
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?,
    ])
}

fn add_vector(
    target: &mut [i128; 3],
    source: [i128; 3],
    id: BodyId,
) -> Result<(), RotatingContactResponseError3d> {
    for (target, source) in target.iter_mut().zip(source) {
        *target = target
            .checked_add(source)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
    }
    Ok(())
}

fn divide_vector(
    vector: [i128; 3],
    divisor: i128,
    id: BodyId,
) -> Result<[i128; 3], RotatingContactResponseError3d> {
    Ok([
        div_round_nearest(vector[0], divisor, id)?,
        div_round_nearest(vector[1], divisor, id)?,
        div_round_nearest(vector[2], divisor, id)?,
    ])
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
    id: BodyId,
) -> Result<i128, RotatingContactResponseError3d> {
    if denominator <= 0 {
        return Err(RotatingContactResponseError3d::ArithmeticOverflow(id));
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
    Ok(adjusted / denominator)
}

fn apply_delta(
    rigid_box: &mut RigidBox3d,
    delta: BodyDelta3d,
) -> Result<(), RotatingContactResponseError3d> {
    let body = rigid_box.body();
    let id = body.id();
    let position = Vec3i::new(
        add_i32(body.position().x, delta.position[0], id)?,
        add_i32(body.position().y, delta.position[1], id)?,
        add_i32(body.position().z, delta.position[2], id)?,
    );
    let velocity = Vec3i::new(
        add_i32(body.velocity().x, delta.linear_velocity[0], id)?,
        add_i32(body.velocity().y, delta.linear_velocity[1], id)?,
        add_i32(body.velocity().z, delta.linear_velocity[2], id)?,
    );
    let angular = rigid_box.angular();
    let angular_velocity = AngularVelocity3d::new(
        add_i32(angular.angular_velocity.x, delta.angular_velocity[0], id)?,
        add_i32(angular.angular_velocity.y, delta.angular_velocity[1], id)?,
        add_i32(angular.angular_velocity.z, delta.angular_velocity[2], id)?,
    );

    let next_body = match body.kind() {
        BodyKind::Dynamic => RigidBody::dynamic(id, position, velocity, body.half_extents())
            .with_mass(body.mass_units())
            .with_material(body.material()),
        BodyKind::Fixed => {
            RigidBody::fixed(id, position, body.half_extents()).with_material(body.material())
        }
    };
    let mut next = RigidBox3d::new(
        next_body,
        AngularState3d::new(angular.orientation, angular_velocity),
    )
    .expect("legacy response must preserve valid rigid-box state");
    if rigid_box.rotation_locked() {
        next = next.with_rotation_locked();
    }
    *rigid_box = next;
    Ok(())
}

fn add_i32(current: i32, delta: i128, id: BodyId) -> Result<i32, RotatingContactResponseError3d> {
    let next = i128::from(current)
        .checked_add(delta)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
    i32::try_from(next).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow(id))
}

fn chain_frontier(count: u64) -> RotatingContactFrontier3d {
    let boxes = (0..count)
        .map(|id| {
            RigidBox3d::new(
                RigidBody::dynamic(
                    BodyId(id),
                    Vec3i::new(i32::try_from(id).expect("small chain id") * 3, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(2, 2, 2),
                ),
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            )
            .expect("valid chain body")
        })
        .collect::<Vec<_>>();
    earliest_rotating_contact_frontier(
        &boxes,
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            4,
            2,
        ),
    )
    .expect("valid chain frontier search")
    .expect("overlapping chain has a frontier")
}

fn large_sparse_frontier(body_count: u64) -> RotatingContactFrontier3d {
    let base = chain_frontier(12);
    let mut boxes = base.boxes;
    for id in 12..body_count {
        boxes.push(
            RigidBox3d::new(
                RigidBody::fixed(
                    BodyId(id),
                    Vec3i::new(
                        1_000_000 + i32::try_from(id).expect("benchmark id fits i32") * 20,
                        0,
                        0,
                    ),
                    Vec3i::new(2, 2, 2),
                ),
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            )
            .expect("valid unrelated benchmark body"),
        );
    }
    RotatingContactFrontier3d {
        boxes,
        time: base.time,
        contacts: base.contacts,
        remaining_numerator: base.remaining_numerator,
    }
}

#[test]
fn reused_solver_scratch_matches_the_prior_per_pass_allocation_algorithm() {
    let frontier = chain_frontier(24);
    let optimized =
        resolve_rotating_contact_frontier(frontier.clone(), 8).expect("optimized solver");
    let prior = legacy_resolve_rotating_contact_frontier(frontier, 8).expect("legacy solver");
    assert_eq!(optimized, prior);
}

#[test]
#[ignore = "microbenchmark; run explicitly in release mode"]
fn contact_solver_scratch_reuse_benchmark() {
    let frontier = large_sparse_frontier(4_096);
    let passes = 8;
    let iterations = 24;

    let prior = legacy_resolve_rotating_contact_frontier(frontier.clone(), passes)
        .expect("legacy correctness run");
    let optimized = resolve_rotating_contact_frontier(frontier.clone(), passes)
        .expect("optimized correctness run");
    assert_eq!(optimized, prior);
    assert!(
        optimized.passes_used > 1,
        "benchmark must exercise repeated solver passes"
    );

    black_box(
        legacy_resolve_rotating_contact_frontier(black_box(frontier.clone()), passes)
            .expect("legacy warmup"),
    );
    black_box(
        resolve_rotating_contact_frontier(black_box(frontier.clone()), passes)
            .expect("optimized warmup"),
    );

    let legacy_inputs = (0..iterations)
        .map(|_| frontier.clone())
        .collect::<Vec<_>>();
    let optimized_inputs = (0..iterations)
        .map(|_| frontier.clone())
        .collect::<Vec<_>>();

    let legacy_start = Instant::now();
    for input in legacy_inputs {
        black_box(
            legacy_resolve_rotating_contact_frontier(black_box(input), passes)
                .expect("legacy benchmark"),
        );
    }
    let legacy_elapsed = legacy_start.elapsed();

    let optimized_start = Instant::now();
    for input in optimized_inputs {
        black_box(
            resolve_rotating_contact_frontier(black_box(input), passes)
                .expect("optimized benchmark"),
        );
    }
    let optimized_elapsed = optimized_start.elapsed();
    let speedup = legacy_elapsed.as_secs_f64() / optimized_elapsed.as_secs_f64();

    eprintln!(
        "contact solver scratch reuse 4096-body frontier × {iterations}: prior_per_pass_alloc={legacy_elapsed:?}, reused_scratch={optimized_elapsed:?}, speedup={speedup:.2}x"
    );
}
