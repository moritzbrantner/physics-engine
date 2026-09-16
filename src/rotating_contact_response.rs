use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    BodyId, ObbContactResponseError3d, RigidBox3d, RigidBoxFreeFlightError3d,
    RotatingContactFrontier3d, RotatingContactSearchHit3d, SampledContactTime3d,
    resolve_obb_contact, sample_rigid_box_free_flight,
};

/// Deterministic metadata produced while resolving one sampled rotating-contact frontier.
///
/// World state is mutated through the supplied authoritative slice. The response therefore carries no
/// replacement world or body collection.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactResponse3d {
    pub time: SampledContactTime3d,
    pub contacts: Vec<RotatingContactSearchHit3d>,
    pub remaining_numerator: u32,
    pub passes_used: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactResponseError3d {
    ZeroSolverPasses,
    MissingBody(BodyId),
    Pair(ObbContactResponseError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    ArithmeticOverflow(BodyId),
}

impl fmt::Display for RotatingContactResponseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroSolverPasses => write!(
                formatter,
                "rotating contact response requires at least one solver pass"
            ),
            Self::MissingBody(id) => write!(
                formatter,
                "rotating contact response cannot find body {} in the authoritative world",
                id.0
            ),
            Self::Pair(error) => write!(formatter, "rotating pair response failed: {error}"),
            Self::FreeFlight(error) => {
                write!(formatter, "rotating frontier advance failed: {error}")
            }
            Self::ArithmeticOverflow(id) => write!(
                formatter,
                "rotating contact response arithmetic overflowed for body {}",
                id.0
            ),
        }
    }
}

impl Error for RotatingContactResponseError3d {}

impl From<ObbContactResponseError3d> for RotatingContactResponseError3d {
    fn from(value: ObbContactResponseError3d) -> Self {
        Self::Pair(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingContactResponseError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct BodyDelta3d {
    position: [i128; 3],
    linear_velocity: [i128; 3],
    angular_velocity: [i128; 3],
}

impl BodyDelta3d {
    fn between(before: &RigidBox3d, after: &RigidBox3d) -> Self {
        Self {
            position: [
                i128::from(after.body.position.x) - i128::from(before.body.position.x),
                i128::from(after.body.position.y) - i128::from(before.body.position.y),
                i128::from(after.body.position.z) - i128::from(before.body.position.z),
            ],
            linear_velocity: [
                i128::from(after.body.velocity.x) - i128::from(before.body.velocity.x),
                i128::from(after.body.velocity.y) - i128::from(before.body.velocity.y),
                i128::from(after.body.velocity.z) - i128::from(before.body.velocity.z),
            ],
            angular_velocity: [
                i128::from(after.angular.angular_velocity.x)
                    - i128::from(before.angular.angular_velocity.x),
                i128::from(after.angular.angular_velocity.y)
                    - i128::from(before.angular.angular_velocity.y),
                i128::from(after.angular.angular_velocity.z)
                    - i128::from(before.angular.angular_velocity.z),
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
    groups: Vec<([i128; 3], DeltaGroup3d)>,
}

impl BodyDeltaAccumulator3d {
    fn accumulate(
        &mut self,
        relative_axis: [i128; 3],
        before: &RigidBox3d,
        after: &RigidBox3d,
    ) -> Result<(), RotatingContactResponseError3d> {
        let id = before.body.id;
        let delta = BodyDelta3d::between(before, after);
        if delta.is_zero() {
            return Ok(());
        }
        let key = primitive_axis(relative_axis, id)?;
        let group = match self.groups.binary_search_by_key(&key, |(axis, _)| *axis) {
            Ok(index) => &mut self.groups[index].1,
            Err(index) => {
                self.groups.insert(index, (key, DeltaGroup3d::default()));
                &mut self.groups[index].1
            }
        };
        group.sum.checked_add(delta, id)?;
        group.count = group
            .count
            .checked_add(1)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
        Ok(())
    }

    fn combined(&self, id: BodyId) -> Result<BodyDelta3d, RotatingContactResponseError3d> {
        let mut combined = BodyDelta3d::default();
        for (_, group) in self.groups.iter().copied() {
            combined.checked_add(group.sum.divided(group.count, id)?, id)?;
        }
        Ok(combined)
    }

    fn clear(&mut self) {
        self.groups.clear();
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotionState3d {
    position: crate::Vec3i,
    velocity: crate::Vec3i,
    orientation: crate::Orientation3d,
    angular_velocity: crate::AngularVelocity3d,
}

impl MotionState3d {
    fn from_box(rigid_box: &RigidBox3d) -> Self {
        Self {
            position: rigid_box.body.position,
            velocity: rigid_box.body.velocity,
            orientation: rigid_box.angular.orientation,
            angular_velocity: rigid_box.angular.angular_velocity,
        }
    }

    fn apply(self, rigid_box: &mut RigidBox3d) {
        rigid_box.body.position = self.position;
        rigid_box.body.velocity = self.velocity;
        rigid_box.angular.orientation = self.orientation;
        rigid_box.angular.angular_velocity = self.angular_velocity;
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotionUpdate3d {
    world_index: usize,
    state: MotionState3d,
}

/// Reusable allocation/index storage for repeated frontier responses.
///
/// Active islands are represented by sorted world indices rather than an ordered map. Contact IDs are
/// resolved to world indices once, the small island index is found by binary search, and that resolution is
/// cached for subsequent solver passes. This keeps the active-island architecture from replacing world-copy
/// cost with per-response tree allocation/churn.
#[derive(Clone, Debug, Default)]
pub struct RotatingContactResponseScratch3d {
    indices: BTreeMap<BodyId, usize>,
    indexed_body_ids: Vec<BodyId>,
    island_world_indices: Vec<usize>,
    resolved_indices: Vec<Option<(usize, usize)>>,
    motion_updates: Vec<MotionUpdate3d>,
    island: Vec<RigidBox3d>,
    deltas: Vec<BodyDeltaAccumulator3d>,
    combined: Vec<BodyDelta3d>,
    modified_body_ids: BTreeSet<BodyId>,
}

impl RotatingContactResponseScratch3d {
    pub(crate) fn ensure_body_index(&mut self, boxes: &[RigidBox3d]) {
        let layout_matches = self.indexed_body_ids.len() == boxes.len()
            && self
                .indexed_body_ids
                .iter()
                .zip(boxes)
                .all(|(id, rigid_box)| *id == rigid_box.body().id());
        if layout_matches {
            return;
        }

        self.indices.clear();
        self.indexed_body_ids.clear();
        self.indexed_body_ids.reserve(boxes.len());
        for (index, rigid_box) in boxes.iter().enumerate() {
            let id = rigid_box.body().id();
            self.indices.insert(id, index);
            self.indexed_body_ids.push(id);
        }
    }

    pub(crate) fn indexed_box<'a>(
        &self,
        boxes: &'a [RigidBox3d],
        id: BodyId,
    ) -> Option<&'a RigidBox3d> {
        let index = *self.indices.get(&id)?;
        boxes
            .get(index)
            .filter(|rigid_box| rigid_box.body().id() == id)
    }
}

pub fn resolve_rotating_contact_frontier(
    boxes: &mut [RigidBox3d],
    frontier: &RotatingContactFrontier3d,
    solver_passes: u8,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    let mut scratch = RotatingContactResponseScratch3d::default();
    resolve_rotating_contact_frontier_with_scratch(boxes, frontier, solver_passes, &mut scratch)
}

pub fn resolve_rotating_contact_frontier_with_scratch(
    boxes: &mut [RigidBox3d],
    frontier: &RotatingContactFrontier3d,
    solver_passes: u8,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    resolve_rotating_contact_frontier_with_activity_and_scratch(
        boxes,
        frontier,
        solver_passes,
        scratch,
    )
    .map(|(response, _)| response)
}

pub(crate) fn resolve_rotating_contact_frontier_with_activity_and_scratch(
    boxes: &mut [RigidBox3d],
    frontier: &RotatingContactFrontier3d,
    solver_passes: u8,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<(RotatingContactResponse3d, Vec<BodyId>), RotatingContactResponseError3d> {
    if solver_passes == 0 {
        return Err(RotatingContactResponseError3d::ZeroSolverPasses);
    }

    scratch.ensure_body_index(boxes);
    stage_motion_updates(boxes, frontier, scratch)?;
    stage_contact_island(boxes, frontier, scratch)?;

    let RotatingContactResponseScratch3d {
        indices,
        indexed_body_ids: _,
        island_world_indices,
        resolved_indices,
        motion_updates,
        island,
        deltas,
        combined,
        modified_body_ids,
    } = scratch;

    let mut passes_used = 0_u8;
    resolved_indices.clear();
    resolved_indices.resize(frontier.contacts.len(), None);
    deltas.resize_with(island.len(), BodyDeltaAccumulator3d::default);
    deltas.truncate(island.len());
    combined.clear();
    combined.reserve(island.len());
    modified_body_ids.clear();

    for pass in 0..solver_passes {
        for accumulator in deltas.iter_mut() {
            accumulator.clear();
        }

        for (contact_index, contact) in frontier.contacts.iter().enumerate() {
            let (left_index, right_index) = match resolved_indices[contact_index] {
                Some(indices) => indices,
                None => {
                    let left_world = *indices.get(&contact.pair.left).ok_or(
                        RotatingContactResponseError3d::MissingBody(contact.pair.left),
                    )?;
                    let right_world = *indices.get(&contact.pair.right).ok_or(
                        RotatingContactResponseError3d::MissingBody(contact.pair.right),
                    )?;
                    let left_index = island_world_indices
                        .binary_search(&left_world)
                        .map_err(|_| RotatingContactResponseError3d::MissingBody(contact.pair.left))?;
                    let right_index = island_world_indices
                        .binary_search(&right_world)
                        .map_err(|_| RotatingContactResponseError3d::MissingBody(contact.pair.right))?;
                    let resolved = (left_index, right_index);
                    resolved_indices[contact_index] = Some(resolved);
                    resolved
                }
            };
            let response = resolve_obb_contact(
                island[left_index].clone(),
                island[right_index].clone(),
                pass == 0,
            )?;
            let Some(resolved_contact) = response.contact else {
                continue;
            };
            deltas[left_index].accumulate(
                negate_axis(resolved_contact.axis, contact.pair.left)?,
                &island[left_index],
                &response.left,
            )?;
            deltas[right_index].accumulate(
                resolved_contact.axis,
                &island[right_index],
                &response.right,
            )?;
        }

        combined.clear();
        for (rigid_box, accumulator) in island.iter().zip(deltas.iter()) {
            combined.push(accumulator.combined(rigid_box.body.id)?);
        }
        if combined.iter().copied().all(BodyDelta3d::is_zero) {
            break;
        }
        for (rigid_box, delta) in island.iter_mut().zip(combined.iter().copied()) {
            if !delta.is_zero() {
                modified_body_ids.insert(rigid_box.body.id);
                apply_delta(rigid_box, delta)?;
            }
        }
        passes_used = passes_used.checked_add(1).ok_or(
            RotatingContactResponseError3d::ArithmeticOverflow(
                island
                    .first()
                    .map_or(BodyId(0), |rigid_box| rigid_box.body.id),
            ),
        )?;
    }

    for update in motion_updates.iter().copied() {
        update.state.apply(
            boxes
                .get_mut(update.world_index)
                .expect("staged world index remains valid during one response"),
        );
    }
    for (island_index, rigid_box) in island.iter().enumerate() {
        let world_index = island_world_indices[island_index];
        MotionState3d::from_box(rigid_box).apply(
            boxes
                .get_mut(world_index)
                .expect("indexed world body remains valid during one response"),
        );
    }

    let modified = modified_body_ids.iter().copied().collect();
    Ok((
        RotatingContactResponse3d {
            time: frontier.time,
            contacts: frontier.contacts.clone(),
            remaining_numerator: frontier.remaining_numerator,
            passes_used,
        },
        modified,
    ))
}

fn stage_motion_updates(
    boxes: &[RigidBox3d],
    frontier: &RotatingContactFrontier3d,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<(), RotatingContactResponseError3d> {
    scratch.motion_updates.clear();
    if frontier.time == SampledContactTime3d::ZERO {
        return Ok(());
    }

    scratch.motion_updates.reserve(boxes.len());
    for (world_index, rigid_box) in boxes.iter().enumerate() {
        let sampled = sample_rigid_box_free_flight(
            rigid_box,
            frontier.free_flight,
            frontier.time.numerator,
            frontier.time.denominator,
        )?;
        let before = MotionState3d::from_box(rigid_box);
        let after = MotionState3d::from_box(&sampled);
        if before != after {
            scratch.motion_updates.push(MotionUpdate3d {
                world_index,
                state: after,
            });
        }
    }
    Ok(())
}

fn stage_contact_island(
    boxes: &[RigidBox3d],
    frontier: &RotatingContactFrontier3d,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<(), RotatingContactResponseError3d> {
    scratch.island_world_indices.clear();
    scratch
        .island_world_indices
        .reserve(frontier.contacts.len().saturating_mul(2));
    for contact in &frontier.contacts {
        scratch.island_world_indices.push(
            *scratch
                .indices
                .get(&contact.pair.left)
                .ok_or(RotatingContactResponseError3d::MissingBody(contact.pair.left))?,
        );
        scratch.island_world_indices.push(
            *scratch
                .indices
                .get(&contact.pair.right)
                .ok_or(RotatingContactResponseError3d::MissingBody(contact.pair.right))?,
        );
    }
    scratch.island_world_indices.sort_unstable();
    scratch.island_world_indices.dedup();

    scratch.island.clear();
    scratch.island.reserve(scratch.island_world_indices.len());
    for &world_index in &scratch.island_world_indices {
        let mut staged = boxes
            .get(world_index)
            .cloned()
            .expect("island world index came from current body index");
        if let Ok(update_index) = scratch
            .motion_updates
            .binary_search_by_key(&world_index, |update| update.world_index)
        {
            scratch.motion_updates[update_index]
                .state
                .apply(&mut staged);
        }
        scratch.island.push(staged);
    }
    Ok(())
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
    let id = rigid_box.body.id;
    rigid_box.body.position = crate::Vec3i::new(
        add_i32(rigid_box.body.position.x, delta.position[0], id)?,
        add_i32(rigid_box.body.position.y, delta.position[1], id)?,
        add_i32(rigid_box.body.position.z, delta.position[2], id)?,
    );
    rigid_box.body.velocity = crate::Vec3i::new(
        add_i32(rigid_box.body.velocity.x, delta.linear_velocity[0], id)?,
        add_i32(rigid_box.body.velocity.y, delta.linear_velocity[1], id)?,
        add_i32(rigid_box.body.velocity.z, delta.linear_velocity[2], id)?,
    );
    rigid_box.angular.angular_velocity = crate::AngularVelocity3d::new(
        add_i32(
            rigid_box.angular.angular_velocity.x,
            delta.angular_velocity[0],
            id,
        )?,
        add_i32(
            rigid_box.angular.angular_velocity.y,
            delta.angular_velocity[1],
            id,
        )?,
        add_i32(
            rigid_box.angular.angular_velocity.z,
            delta.angular_velocity[2],
            id,
        )?,
    );
    Ok(())
}

fn add_i32(current: i32, delta: i128, id: BodyId) -> Result<i32, RotatingContactResponseError3d> {
    let next = i128::from(current)
        .checked_add(delta)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
    i32::try_from(next).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow(id))
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
        earliest_rotating_contact_frontier,
    };

    use super::{RotatingContactResponseScratch3d, resolve_rotating_contact_frontier};

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half_extents: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half_extents),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, half_extents: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half_extents),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn config() -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1), 4, 3)
    }

    #[test]
    fn one_contact_frontier_applies_engine_native_response() {
        let extent = Vec3i::new(10, 10, 10);
        let mut boxes = vec![
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), extent),
            fixed(2, Vec3i::new(19, 0, 0), extent),
        ];
        let fixed_before = boxes[1].clone();
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("contact frontier");
        let response =
            resolve_rotating_contact_frontier(&mut boxes, &frontier, 4).expect("valid response");

        assert!(response.passes_used > 0);
        assert_eq!(boxes[0].body.velocity.x, 0);
        assert_eq!(boxes[1], fixed_before);
        assert_eq!(response.remaining_numerator, 1);
    }

    #[test]
    fn nonzero_frontier_advances_authoritative_motion_before_response() {
        let mut boxes = vec![
            dynamic(
                1,
                Vec3i::new(-10, 0, 0),
                Vec3i::new(20, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            fixed(2, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("contact frontier");
        assert!(frontier.time.numerator > 0);

        resolve_rotating_contact_frontier(&mut boxes, &frontier, 4).expect("valid response");
        assert_ne!(boxes[0].body.position, Vec3i::new(-10, 0, 0));
    }

    #[test]
    fn partitioned_wall_does_not_double_a_stopping_impulse() {
        let mut boxes = vec![
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), Vec3i::new(10, 10, 10)),
            fixed(2, Vec3i::new(19, -5, 0), Vec3i::new(10, 5, 10)),
            fixed(3, Vec3i::new(19, 5, 0), Vec3i::new(10, 5, 10)),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("partitioned wall contacts");
        assert!(frontier.contacts.len() >= 2);

        resolve_rotating_contact_frontier(&mut boxes, &frontier, 4).expect("valid response");
        assert_eq!(boxes[0].body.velocity.x, 0);
        assert!(boxes[0].body.velocity.x >= 0);
    }

    #[test]
    fn symmetric_simultaneous_projection_is_not_pair_order_owned() {
        let extent = Vec3i::new(10, 10, 10);
        let mut boxes = vec![
            fixed(1, Vec3i::new(-19, 0, 0), extent),
            dynamic(2, Vec3i::ZERO, Vec3i::ZERO, extent),
            fixed(3, Vec3i::new(19, 0, 0), extent),
        ];
        let fixed_left = boxes[0].clone();
        let fixed_right = boxes[2].clone();
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("simultaneous contacts");
        assert_eq!(frontier.contacts.len(), 2);

        resolve_rotating_contact_frontier(&mut boxes, &frontier, 4).expect("valid response");
        assert_eq!(boxes[1].body.position, Vec3i::ZERO);
        assert_eq!(boxes[0], fixed_left);
        assert_eq!(boxes[2], fixed_right);
    }

    #[test]
    fn frontier_response_is_repeatable() {
        let extent = Vec3i::new(10, 10, 10);
        let original = vec![
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), extent),
            fixed(2, Vec3i::new(19, 0, 0), extent),
        ];
        let frontier = earliest_rotating_contact_frontier(&original, config())
            .expect("valid frontier")
            .expect("contact frontier");
        let mut first_boxes = original.clone();
        let mut second_boxes = original;

        let first = resolve_rotating_contact_frontier(&mut first_boxes, &frontier, 4)
            .expect("first response");
        let second = resolve_rotating_contact_frontier(&mut second_boxes, &frontier, 4)
            .expect("second response");
        assert_eq!(first, second);
        assert_eq!(first_boxes, second_boxes);
    }

    #[test]
    fn sparse_world_stages_only_contact_island_bodies() {
        let mut boxes = vec![
            dynamic(1, Vec3i::ZERO, Vec3i::new(10, 0, 0), Vec3i::new(5, 5, 5)),
            fixed(2, Vec3i::new(9, 0, 0), Vec3i::new(5, 5, 5)),
        ];
        for id in 3..=128 {
            boxes.push(fixed(
                id,
                Vec3i::new(10_000 + i32::try_from(id).expect("small id") * 20, 0, 0),
                Vec3i::new(2, 2, 2),
            ));
        }
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("frontier query")
            .expect("contact frontier");
        let mut scratch = RotatingContactResponseScratch3d::default();
        super::resolve_rotating_contact_frontier_with_scratch(
            &mut boxes,
            &frontier,
            4,
            &mut scratch,
        )
        .expect("island response");
        assert_eq!(scratch.island.len(), 2);
    }
}

#[cfg(test)]
mod scratch_reuse_tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactFrontier3d, RotatingContactSearchHit3d,
        SampledContactTime3d, Vec3i, obb_contact_seed,
    };

    use super::{
        RotatingContactResponseScratch3d, resolve_rotating_contact_frontier,
        resolve_rotating_contact_frontier_with_scratch,
    };

    fn box3d(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    #[test]
    fn reusable_scratch_preserves_response_semantics() {
        let left = box3d(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(10, 0, 0),
            Vec3i::new(5, 5, 5),
        ));
        let right = box3d(RigidBody::fixed(
            BodyId(2),
            Vec3i::new(9, 0, 0),
            Vec3i::new(5, 5, 5),
        ));
        let contact = obb_contact_seed(left.oriented_box(), right.oriented_box())
            .expect("valid boxes")
            .expect("overlap");
        let frontier = RotatingContactFrontier3d {
            free_flight: RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
            time: SampledContactTime3d::ZERO,
            contacts: vec![RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair: crate::RotationalSweepPair3d {
                    left: BodyId(1),
                    right: BodyId(2),
                },
                contact,
            }],
            remaining_numerator: 0,
        };
        let original = vec![left, right];
        let mut expected_boxes = original.clone();
        let expected =
            resolve_rotating_contact_frontier(&mut expected_boxes, &frontier, 4).expect("response");
        let mut scratch = RotatingContactResponseScratch3d::default();
        let mut first_boxes = original.clone();
        let first = resolve_rotating_contact_frontier_with_scratch(
            &mut first_boxes,
            &frontier,
            4,
            &mut scratch,
        )
        .expect("scratch response");
        let mut second_boxes = original;
        let second = resolve_rotating_contact_frontier_with_scratch(
            &mut second_boxes,
            &frontier,
            4,
            &mut scratch,
        )
        .expect("reused scratch response");
        assert_eq!(first, expected);
        assert_eq!(second, expected);
        assert_eq!(first_boxes, expected_boxes);
        assert_eq!(second_boxes, expected_boxes);
    }
}
