use crate::{
    BodyKind, ContactMode3d, ObbContactResponse3d, ObbContactResponseError3d, RigidBox3d, Vec3i,
    obb_contact_seed,
    obb_response::{resolve_physical_obb_contact, validate_pair},
};

pub(crate) fn uses_linear_response(left: &RigidBox3d, right: &RigidBox3d) -> bool {
    left.contact_mode != ContactMode3d::Physical || right.contact_mode != ContactMode3d::Physical
}

fn support_direction(body: &RigidBox3d) -> Option<Vec3i> {
    match body.contact_mode {
        ContactMode3d::Physical => None,
        ContactMode3d::LinearPush { support_direction } => Some(support_direction),
    }
}

fn dot(direction: Vec3i, axis: [i128; 3]) -> Result<i128, ObbContactResponseError3d> {
    [direction.x, direction.y, direction.z]
        .into_iter()
        .zip(axis)
        .try_fold(0_i128, |sum, (component, axis)| {
            i128::from(component)
                .checked_mul(axis)
                .and_then(|value| sum.checked_add(value))
                .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
        })
}

/// Applies the opt-in actuator policy through the existing exact normal solver. Only response-local
/// inverse mass / inertia participation changes. Original body kinds, materials, policy, and existing
/// angular state are restored before returning; no proxy enters world membership or free flight.
pub(crate) fn resolve(
    left: RigidBox3d,
    right: RigidBox3d,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {
    validate_pair(&left, &right)?;
    let Some(seed) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
        return Ok(ObbContactResponse3d {
            left,
            right,
            contact: None,
        });
    };
    let left_direction = support_direction(&left);
    let right_direction = support_direction(&right);
    let right_supports_left = match (left_direction, right_direction) {
        (Some(direction), None) => {
            dot(direction, seed.axis)? > 0 || current_shape_is_passive_support(&left, &right)?
        }
        _ => false,
    };
    let left_supports_right = match (left_direction, right_direction) {
        (None, Some(direction)) => {
            dot(direction, seed.axis)? < 0 || current_shape_is_passive_support(&right, &left)?
        }
        _ => false,
    };
    let mut left_proxy = left.clone();
    let mut right_proxy = right.clone();
    left_proxy.rotation_locked = true;
    right_proxy.rotation_locked = true;
    if right_supports_left {
        right_proxy.body.kind = BodyKind::Fixed;
    }
    if left_supports_right {
        left_proxy.body.kind = BodyKind::Fixed;
    }
    let mut response = resolve_physical_obb_contact(left_proxy, right_proxy, false)?;
    response.left.body.kind = left.body.kind;
    response.right.body.kind = right.body.kind;
    response.left.rotation_locked = left.rotation_locked;
    response.right.rotation_locked = right.rotation_locked;
    response.left.angular = left.angular;
    response.right.angular = right.angular;
    Ok(response)
}

// Cardinal support directions additionally define a passive support half-space: when every point
// of the actuator lies above the other body's centre, a shallow edge/axis tie must not turn landing
// into a horizontal kick. The ground/support query still uses actual contact normals.
fn cardinal_support_axis(body: &RigidBox3d) -> Option<(usize, i32)> {
    let direction = support_direction(body)?;
    let components = [direction.x, direction.y, direction.z];
    let mut nonzero = components
        .into_iter()
        .enumerate()
        .filter(|(_, value)| *value != 0);
    let (axis, value) = nonzero.next()?;
    if nonzero.next().is_some() {
        return None;
    }
    Some((axis, value.signum()))
}

fn current_shape_is_passive_support(
    actuator: &RigidBox3d,
    other: &RigidBox3d,
) -> Result<bool, ObbContactResponseError3d> {
    let Some((axis, sign)) = cardinal_support_axis(actuator) else {
        return Ok(false);
    };
    let centre = other.body.position.component(axis);
    let vertices = crate::oriented_box_vertices(actuator.oriented_box())?;
    Ok(vertices.iter().all(|point| {
        if sign < 0 {
            point.component(axis) > centre
        } else {
            point.component(axis) < centre
        }
    }))
}

/// A conservative envelope entirely inside the passive support half-space cannot transmit a push
/// to this sleeper under the actuator policy. Keep the sleeper as an ordinary collision-tested fixed
/// proxy, not as absent geometry. Side pushes, projectiles, and unproven/general directions retain the
/// existing conservative wake path. This uses the same half-space as response, not a sleep threshold.
pub(crate) fn sweep_is_passive_support(
    actuator: &RigidBox3d,
    envelope: crate::RotationalSweepBounds3d,
    other: &RigidBox3d,
) -> bool {
    if other.contact_mode != ContactMode3d::Physical {
        return false;
    }
    let Some((axis, sign)) = cardinal_support_axis(actuator) else {
        return false;
    };
    let centre = i64::from(other.body.position.component(axis));
    if sign < 0 {
        envelope.minimum[axis] > centre
    } else {
        envelope.maximum[axis] < centre
    }
}

#[cfg(test)]
mod tests {
    use super::sweep_is_passive_support;
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotationalSweepBounds3d, Vec3i,
    };

    fn body(id: u64) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .unwrap()
    }

    #[test]
    fn only_proven_passive_half_spaces_skip_waking() {
        let support = body(2);
        for axis in 0..3 {
            for sign in [-1, 1] {
                let mut components = [0; 3];
                components[axis] = sign;
                let actuator = body(1).with_linear_push(Vec3i::new(
                    components[0],
                    components[1],
                    components[2],
                ));
                let mut envelope = RotationalSweepBounds3d {
                    minimum: [-2; 3],
                    maximum: [2; 3],
                };
                assert!(!sweep_is_passive_support(&actuator, envelope, &support));
                if sign < 0 {
                    envelope.minimum[axis] = 1;
                } else {
                    envelope.maximum[axis] = -1;
                }
                assert!(sweep_is_passive_support(&actuator, envelope, &support));
                if sign < 0 {
                    envelope.minimum[axis] = 0;
                } else {
                    envelope.maximum[axis] = 0;
                }
                assert!(
                    !sweep_is_passive_support(&actuator, envelope, &support),
                    "centre-level sides must remain pushable"
                );
            }
        }
        let envelope = RotationalSweepBounds3d {
            minimum: [1; 3],
            maximum: [2; 3],
        };
        for direction in [Vec3i::ZERO, Vec3i::new(-1, -1, 0)] {
            assert!(!sweep_is_passive_support(
                &body(1).with_linear_push(direction),
                envelope,
                &support
            ));
        }
        assert!(
            !sweep_is_passive_support(&body(1), envelope, &support),
            "physical impacts always use normal waking"
        );
    }
}
