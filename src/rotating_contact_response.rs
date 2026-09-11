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
            Self::ZeroSolverPasses => {
                write!(formatter, "rotating contact response requires at least one solver pass")
            }
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
    fn is_zero(self) -> bool {
        self.position == [0; 3]
            && self.linear_velocity == [0; 3]
            && self.angular_velocity == [0; 3]
    }

    fn accumulate(
        &mut self,
        before: &RigidBox3d,
        after: &RigidBox3d,
    ) -> Result<(), RotatingContactResponseError3d> {
        let id = before.body.id;
        for (target, delta) in self.position.iter_mut().zip([
            i128::from(after.body.position.x) - i128::from(before.body.position.x),
            i128::from(after.body.position.y) - i128::from(before.body.position.y),
            i128::from(after.body.position.z) - i128::from(before.body.position.z),
        ]) {
            *target = target
                .checked_add(delta)
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
        }
        for (target, delta) in self.linear_velocity.iter_mut().zip([
            i128::from(after.body.velocity.x) - i128::from(before.body.velocity.x),
            i128::from(after.body.velocity.y) - i128::from(before.body.velocity.y),
            i128::from(after.body.velocity.z) - i128::from(before.body.velocity.z),
        ]) {
            *target = target
                .checked_add(delta)
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
        }
        for (target, delta) in self.angular_velocity.iter_mut().zip([
            i128::from(after.angular.angular_velocity.x)
                - i128::from(before.angular.angular_velocity.x),
            i128::from(after.angular.angular_velocity.y)
                - i128::from(before.angular.angular_velocity.y),
            i128::from(after.angular.angular_velocity.z)
                - i128::from(before.angular.angular_velocity.z),
        ]) {
            *target = target
                .checked_add(delta)
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(id))?;
        }
        Ok(())
    }
}

/// Resolves every contact in one shared sampled frontier with bounded simultaneous Jacobi-style passes.
///
/// Every pair is evaluated from the same snapshot within a pass and only accumulated deltas are committed
/// afterward. This avoids allowing deterministic pair iteration order to become hidden physical ownership
/// for simultaneous impacts. Material restitution is admitted on the first pass only; later passes use
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
/// failures, or checked accumulation overflow.
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
        let mut deltas = vec![BodyDelta3d::default(); snapshot.len()];

        for contact in &frontier.contacts {
            let left_index = *indices
                .get(&contact.pair.left)
                .ok_or(RotatingContactResponseError3d::MissingBody(contact.pair.left))?;
            let right_index = *indices
                .get(&contact.pair.right)
                .ok_or(RotatingContactResponseError3d::MissingBody(contact.pair.right))?;
            let response = resolve_obb_contact(
                snapshot[left_index].clone(),
                snapshot[right_index].clone(),
                pass == 0,
            )?;
            deltas[left_index].accumulate(&snapshot[left_index], &response.left)?;
            deltas[right_index].accumulate(&snapshot[right_index], &response.right)?;
        }

        if deltas.iter().copied().all(BodyDelta3d::is_zero) {
            break;
        }
        for (rigid_box, delta) in boxes.iter_mut().zip(deltas) {
            apply_delta(rigid_box, delta)?;
        }
        passes_used = passes_used
            .checked_add(1)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow(
                boxes
                    .first()
                    .map_or(BodyId(0), |rigid_box| rigid_box.body.id),
            ))?;
    }

    Ok(RotatingContactResponse3d {
        boxes,
        time: frontier.time,
        contacts: frontier.contacts,
        remaining_numerator: frontier.remaining_numerator,
        passes_used,
    })
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
        add_i32(
            rigid_box.body.velocity.x,
            delta.linear_velocity[0],
            id,
        )?,
        add_i32(
            rigid_box.body.velocity.y,
            delta.linear_velocity[1],
            id,
        )?,
        add_i32(
            rigid_box.body.velocity.z,
            delta.linear_velocity[2],
            id,
        )?,
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

fn add_i32(
    current: i32,
    delta: i128,
    id: BodyId,
) -> Result<i32, RotatingContactResponseError3d> {
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

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(id),
                position,
                velocity,
                Vec3i::new(10, 10, 10),
            ),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn config() -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            4,
            3,
        )
    }

    #[test]
    fn one_contact_frontier_applies_engine_native_response() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0)),
            fixed(2, Vec3i::new(19, 0, 0)),
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
    fn symmetric_simultaneous_projection_is_not_pair_order_owned() {
        let boxes = [
            fixed(1, Vec3i::new(-19, 0, 0)),
            dynamic(2, Vec3i::ZERO, Vec3i::ZERO),
            fixed(3, Vec3i::new(19, 0, 0)),
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
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0)),
            fixed(2, Vec3i::new(19, 0, 0)),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("contact frontier");

        let first = resolve_rotating_contact_frontier(frontier.clone(), 4).expect("first response");
        let second = resolve_rotating_contact_frontier(frontier, 4).expect("second response");
        assert_eq!(first, second);
    }
}
