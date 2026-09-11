use std::{collections::BTreeMap, error::Error, fmt};

use crate::{
    BodyId, ObbContactResponseError3d, RigidBox3d, RotatingContactFrontier3d,
    RotatingContactSearchHit3d, SampledContactTime3d, resolve_obb_contact,
};

/// Deterministic result of resolving one sampled rotating-contact frontier.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactResponse3d {
    pub boxes: Vec<RigidBox3d>,
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
                "rotating contact response cannot find body {} in the shared frontier",
                id.0
            ),
            Self::Pair(error) => write!(formatter, "rotating pair response failed: {error}"),
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

    fn divided(
        self,
        divisor: u32,
        id: BodyId,
    ) -> Result<Self, RotatingContactResponseError3d> {
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
    /// Parallel constraints that push the same body in the same direction share one response budget.
    /// This makes a wall split into two coplanar bodies equivalent to one wall rather than doubling
    /// the stopping impulse. Opposing or genuinely different normals remain separate constraints.
    groups: BTreeMap<[i128; 3], DeltaGroup3d>,
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

/// Resolves every contact in one shared sampled frontier with bounded simultaneous passes.
///
/// Every pair is evaluated from the same snapshot within a pass. Pair deltas that act on the same body
/// through the same primitive constraint direction are coupled into one averaged response budget before
/// commit; this prevents duplicate coplanar contacts from multiplying a complete stopping impulse merely
/// because one wall was partitioned into several bodies. Distinct or opposing normal directions remain
/// independent and are summed. Material restitution is admitted on the first pass only; later passes use
/// inelastic correction so numerical convergence cannot repeatedly inject bounce energy.
///
/// Orientation is deliberately frozen at the frontier while position, linear velocity and angular velocity
/// converge. Remaining-time integration belongs to the repeated-event step that consumes this response.
/// The input time still comes from sampled rotational search, so this remains sampled rotational collision
/// handling rather than analytic rotational CCD.
///
/// # Errors
///
/// Returns [`RotatingContactResponseError3d`] for zero solver passes, missing body identities, pair response
/// failures, invalid constraint axes, or checked accumulation overflow.
pub fn resolve_rotating_contact_frontier(
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
        .map(|(index, rigid_box)| (rigid_box.body.id, index))
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
            combined.push(accumulator.combined(rigid_box.body.id)?);
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
                    .map_or(BodyId(0), |rigid_box| rigid_box.body.id),
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

fn negate_axis(
    axis: [i128; 3],
    id: BodyId,
) -> Result<[i128; 3], RotatingContactResponseError3d> {
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

    use super::resolve_rotating_contact_frontier;

    fn dynamic(
        id: u64,
        position: Vec3i,
        velocity: Vec3i,
        half_extents: Vec3i,
    ) -> RigidBox3d {
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
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), extent),
            fixed(2, Vec3i::new(19, 0, 0), extent),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("contact frontier");
        let response = resolve_rotating_contact_frontier(frontier, 4).expect("valid response");

        assert!(response.passes_used > 0);
        assert_eq!(response.boxes[0].body.velocity.x, 0);
        assert_eq!(response.boxes[1], boxes[1]);
        assert_eq!(response.remaining_numerator, 1);
    }

    #[test]
    fn partitioned_wall_does_not_double_a_stopping_impulse() {
        let boxes = [
            dynamic(
                1,
                Vec3i::ZERO,
                Vec3i::new(60, 0, 0),
                Vec3i::new(10, 10, 10),
            ),
            fixed(
                2,
                Vec3i::new(19, -5, 0),
                Vec3i::new(10, 5, 10),
            ),
            fixed(3, Vec3i::new(19, 5, 0), Vec3i::new(10, 5, 10)),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("partitioned wall contacts");
        assert!(frontier.contacts.len() >= 2);

        let response = resolve_rotating_contact_frontier(frontier, 4).expect("valid response");
        assert_eq!(response.boxes[0].body.velocity.x, 0);
        assert!(response.boxes[0].body.velocity.x >= 0);
    }

    #[test]
    fn symmetric_simultaneous_projection_is_not_pair_order_owned() {
        let extent = Vec3i::new(10, 10, 10);
        let boxes = [
            fixed(1, Vec3i::new(-19, 0, 0), extent),
            dynamic(2, Vec3i::ZERO, Vec3i::ZERO, extent),
            fixed(3, Vec3i::new(19, 0, 0), extent),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("simultaneous contacts");
        assert_eq!(frontier.contacts.len(), 2);

        let response = resolve_rotating_contact_frontier(frontier, 4).expect("valid response");
        assert_eq!(response.boxes[1].body.position, Vec3i::ZERO);
        assert_eq!(response.boxes[0], boxes[0]);
        assert_eq!(response.boxes[2], boxes[2]);
    }

    #[test]
    fn frontier_response_is_repeatable() {
        let extent = Vec3i::new(10, 10, 10);
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), extent),
            fixed(2, Vec3i::new(19, 0, 0), extent),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("contact frontier");

        let first = resolve_rotating_contact_frontier(frontier.clone(), 4).expect("first response");
        let second = resolve_rotating_contact_frontier(frontier, 4).expect("second response");
        assert_eq!(first, second);
    }
}
