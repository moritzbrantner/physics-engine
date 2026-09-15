use crate::ecs_world::EcsRotatingWorld3d;
use crate::{
    BodyId, RotatingContactResponseError3d, RotatingWorldError3d, Vec3i, obb_contact_seed,
};

/// Returns whether `body` currently has a contact whose normal can oppose the supplied acceleration.
///
/// Contact geometry remains engine-owned. The subject body is always evaluated as the left OBB, so the
/// SAT axis points from the subject toward the other body. A support therefore requires a strictly positive
/// dot product between that axis and the acceleration vector. Side-wall and ceiling contacts do not count as
/// support under downward gravity, while floors and sufficiently upward-facing slopes do.
///
/// This is a current-contact query; it does not predict future contact or apply impulses.
///
/// # Errors
///
/// Returns [`RotatingWorldError3d`] if the body is missing, overlap/contact geometry is invalid, or the
/// support-direction dot product overflows the deterministic integer contract.
pub fn body_has_support(
    world: &EcsRotatingWorld3d,
    body: BodyId,
    acceleration: Vec3i,
) -> Result<bool, RotatingWorldError3d> {
    if acceleration == Vec3i::ZERO {
        return Ok(false);
    }
    let subject = world
        .box_by_id(body)
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    for other_id in world.overlap_query(subject.oriented_box())? {
        if other_id == body {
            continue;
        }
        let other = world
            .box_by_id(other_id)
            .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
        let Some(contact) = obb_contact_seed(subject.oriented_box(), other.oriented_box())? else {
            continue;
        };
        if checked_dot(acceleration, contact.axis, body)? > 0 {
            return Ok(true);
        }
    }
    Ok(false)
}

fn checked_dot(vector: Vec3i, axis: [i128; 3], body: BodyId) -> Result<i128, RotatingWorldError3d> {
    [vector.x, vector.y, vector.z]
        .into_iter()
        .zip(axis)
        .try_fold(0_i128, |sum, (component, axis_component)| {
            let product = i128::from(component)
                .checked_mul(axis_component)
                .ok_or_else(|| arithmetic_overflow(body))?;
            sum.checked_add(product)
                .ok_or_else(|| arithmetic_overflow(body))
        })
}

fn arithmetic_overflow(body: BodyId) -> RotatingWorldError3d {
    RotatingWorldError3d::Response(RotatingContactResponseError3d::ArithmeticOverflow(body))
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
    };

    use super::body_has_support;

    fn rotating(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    fn world_with_subject() -> RotatingWorld3d {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::new(0, -3_600, 0),
            ..RotatingWorldConfig3d::default()
        });
        world
            .add_box(rotating(RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(0, 2, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            )))
            .expect("subject");
        world
    }

    #[test]
    fn floor_contact_supports_against_downward_gravity() {
        let mut world = world_with_subject();
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::ZERO,
                Vec3i::new(5, 1, 5),
            )))
            .expect("floor");

        assert!(
            body_has_support(&world, BodyId(1), Vec3i::new(0, -3_600, 0)).expect("support query")
        );
    }

    #[test]
    fn side_wall_contact_is_not_ground_support() {
        let mut world = world_with_subject();
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(2, 2, 0),
                Vec3i::new(1, 5, 5),
            )))
            .expect("wall");

        assert!(
            !body_has_support(&world, BodyId(1), Vec3i::new(0, -3_600, 0)).expect("support query")
        );
    }

    #[test]
    fn ceiling_contact_is_not_ground_support() {
        let mut world = world_with_subject();
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(0, 4, 0),
                Vec3i::new(5, 1, 5),
            )))
            .expect("ceiling");

        assert!(
            !body_has_support(&world, BodyId(1), Vec3i::new(0, -3_600, 0)).expect("support query")
        );
    }

    #[test]
    fn zero_acceleration_has_no_support_direction() {
        let world = world_with_subject();
        assert!(!body_has_support(&world, BodyId(1), Vec3i::ZERO).expect("support query"));
    }
}
