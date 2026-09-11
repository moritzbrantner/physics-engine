use crate::{
    ANGULAR_VELOCITY_SCALE, AngularVelocity3d, BodyKind, MATERIAL_SCALE, ORIENTATION_SCALE,
    ObbAxisFeature3d, ObbContactResponse3d, ObbContactResponseError3d, ObbResolvedContact3d,
    Orientation3d, RigidBox3d, Vec3i, box_inertia, oriented_box_vertices,
    wide_ratio::{mul_div_round_i128, mul_div_round_u128},
};

use crate::obb_response::resolve_obb_contact as resolve_normal_obb_contact;

const RESPONSE_SCALE: i128 = 1_i128 << 50;

#[derive(Clone, Copy)]
struct TangentResponse3d {
    left_offset: [i64; 3],
    right_offset: [i64; 3],
    tangent: [i128; 3],
    contact: ObbResolvedContact3d,
    friction_milli: u16,
}

/// Resolves one engine-native OBB contact with normal response, penetration projection, and one
/// deterministic Coulomb-limited tangential friction impulse.
///
/// The normal response remains owned by the existing OBB resolver. Friction uses that resolver's exact
/// reduced contact point and SAT feature, evaluates post-normal relative contact velocity including spin,
/// and chooses the dominant slip direction from the exact oriented box edges spanning the contact tangent
/// plane. The Coulomb bound compares squared impulse-vector magnitudes, so no floating-point normalization
/// or square root enters solver truth.
///
/// Friction combines with the migrated `ecs-lab` rule: the pair uses the larger material coefficient.
/// A zero coefficient preserves the previous normal-only response bit-for-bit. Rotation-locked bodies still
/// receive translational friction impulses but never receive angular velocity changes.
///
/// This changes contact response only. Rotational collision discovery remains sampled rather than analytic
/// CCD.
///
/// # Errors
///
/// Returns [`ObbContactResponseError3d`] for the existing normal-response failures, out-of-range friction,
/// malformed contact geometry, or checked arithmetic overflow.
pub fn resolve_obb_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    allow_restitution: bool,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {
    validate_friction(&left)?;
    validate_friction(&right)?;

    let left_start = left.clone();
    let right_start = right.clone();
    let mut response = resolve_normal_obb_contact(left, right, allow_restitution)?;
    let Some(contact) = response.contact else {
        return Ok(response);
    };
    if contact.normal_impulse == [0; 3] {
        return Ok(response);
    }

    let friction_milli = left_start
        .body
        .material
        .friction_milli()
        .max(right_start.body.material.friction_milli());
    if friction_milli == 0 {
        return Ok(response);
    }

    let left_offset = vector_delta(left_start.body.position, contact.point)?;
    let right_offset = vector_delta(right_start.body.position, contact.point)?;
    let relative_velocity =
        relative_contact_velocity(&response.left, left_offset, &response.right, right_offset)?;
    let tangents = contact_tangents(&left_start, &right_start, contact)?;
    let Some(tangent) = dominant_slip_tangent(relative_velocity, tangents)? else {
        return Ok(response);
    };

    apply_tangent_response(
        &mut response,
        TangentResponse3d {
            left_offset,
            right_offset,
            tangent,
            contact,
            friction_milli,
        },
    )?;
    Ok(response)
}

fn validate_friction(rigid_box: &RigidBox3d) -> Result<(), ObbContactResponseError3d> {
    if rigid_box.body.material.friction_milli() > MATERIAL_SCALE {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(())
}

fn contact_tangents(
    left: &RigidBox3d,
    right: &RigidBox3d,
    contact: ObbResolvedContact3d,
) -> Result<[[i128; 3]; 2], ObbContactResponseError3d> {
    let left_edges = oriented_edges(left)?;
    let right_edges = oriented_edges(right)?;
    match contact.feature {
        ObbAxisFeature3d::LeftFace(axis) => face_tangents(left_edges, axis),
        ObbAxisFeature3d::RightFace(axis) => face_tangents(right_edges, axis),
        ObbAxisFeature3d::EdgeEdge {
            left_axis,
            right_axis,
        } => Ok([
            indexed_edge(left_edges, left_axis)?,
            indexed_edge(right_edges, right_axis)?,
        ]),
    }
}

fn oriented_edges(rigid_box: &RigidBox3d) -> Result<[[i128; 3]; 3], ObbContactResponseError3d> {
    let vertices = oriented_box_vertices(rigid_box.oriented_box())?;
    Ok([
        edge_delta(vertices[0], vertices[1])?,
        edge_delta(vertices[0], vertices[2])?,
        edge_delta(vertices[0], vertices[4])?,
    ])
}

fn edge_delta(left: Vec3i, right: Vec3i) -> Result<[i128; 3], ObbContactResponseError3d> {
    Ok([
        checked_sub(i128::from(right.x), i128::from(left.x))?,
        checked_sub(i128::from(right.y), i128::from(left.y))?,
        checked_sub(i128::from(right.z), i128::from(left.z))?,
    ])
}

fn face_tangents(
    edges: [[i128; 3]; 3],
    axis: u8,
) -> Result<[[i128; 3]; 2], ObbContactResponseError3d> {
    match axis {
        0 => Ok([edges[1], edges[2]]),
        1 => Ok([edges[2], edges[0]]),
        2 => Ok([edges[0], edges[1]]),
        _ => Err(ObbContactResponseError3d::ArithmeticOverflow),
    }
}

fn indexed_edge(edges: [[i128; 3]; 3], axis: u8) -> Result<[i128; 3], ObbContactResponseError3d> {
    edges
        .get(usize::from(axis))
        .copied()
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn dominant_slip_tangent(
    relative_velocity: [i128; 3],
    tangents: [[i128; 3]; 2],
) -> Result<Option<[i128; 3]>, ObbContactResponseError3d> {
    let first =
        primitive_vector(tangents[0])?.ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let second =
        primitive_vector(tangents[1])?.ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let first_slip = checked_dot(relative_velocity, first)?;
    let second_slip = checked_dot(relative_velocity, second)?;
    if first_slip == 0 && second_slip == 0 {
        return Ok(None);
    }
    if first_slip.unsigned_abs() >= second_slip.unsigned_abs() {
        Ok(Some(first))
    } else {
        Ok(Some(second))
    }
}

fn apply_tangent_response(
    response: &mut ObbContactResponse3d,
    tangent_response: TangentResponse3d,
) -> Result<(), ObbContactResponseError3d> {
    let TangentResponse3d {
        left_offset,
        right_offset,
        tangent,
        contact,
        friction_milli,
    } = tangent_response;
    let tangent_length_squared = vector_length_squared(tangent)?;
    let relative_velocity =
        relative_contact_velocity(&response.left, left_offset, &response.right, right_offset)?;
    let tangent_velocity = checked_dot(relative_velocity, tangent)?;
    if tangent_velocity == 0 {
        return Ok(());
    }

    let effective_inverse_mass = checked_add(
        body_effective_inverse_mass_scaled(
            &response.left,
            left_offset,
            tangent,
            tangent_length_squared,
        )?,
        body_effective_inverse_mass_scaled(
            &response.right,
            right_offset,
            tangent,
            tangent_length_squared,
        )?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok(());
    }

    let opposing_velocity = tangent_velocity
        .checked_neg()
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let desired_impulse = scale_ratio_vector(
        opposing_velocity,
        effective_inverse_mass,
        tangent,
        RESPONSE_SCALE,
    )?;
    let impulse = coulomb_clamp_vector(
        desired_impulse,
        tangent,
        contact.normal_impulse,
        friction_milli,
    )?;
    if impulse == [0; 3] {
        return Ok(());
    }

    apply_body_impulse(&mut response.left, left_offset, negate_axis(impulse)?)?;
    apply_body_impulse(&mut response.right, right_offset, impulse)?;
    Ok(())
}

fn vector_delta(center: Vec3i, point: Vec3i) -> Result<[i64; 3], ObbContactResponseError3d> {
    Ok([
        i64::from(point.x) - i64::from(center.x),
        i64::from(point.y) - i64::from(center.y),
        i64::from(point.z) - i64::from(center.z),
    ])
}

fn relative_contact_velocity(
    left: &RigidBox3d,
    left_offset: [i64; 3],
    right: &RigidBox3d,
    right_offset: [i64; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let left = contact_velocity(left, left_offset)?;
    let right = contact_velocity(right, right_offset)?;
    Ok([
        checked_sub(right[0], left[0])?,
        checked_sub(right[1], left[1])?,
        checked_sub(right[2], left[2])?,
    ])
}

fn contact_velocity(
    rigid_box: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let omega = if rigid_box.rotation_locked {
        AngularVelocity3d::default()
    } else {
        rigid_box.angular.angular_velocity
    };
    let rotation_x = checked_sub(
        checked_mul(i128::from(omega.y), i128::from(offset[2]))?,
        checked_mul(i128::from(omega.z), i128::from(offset[1]))?,
    )?;
    let rotation_y = checked_sub(
        checked_mul(i128::from(omega.z), i128::from(offset[0]))?,
        checked_mul(i128::from(omega.x), i128::from(offset[2]))?,
    )?;
    let rotation_z = checked_sub(
        checked_mul(i128::from(omega.x), i128::from(offset[1]))?,
        checked_mul(i128::from(omega.y), i128::from(offset[0]))?,
    )?;
    let scale = i128::from(ANGULAR_VELOCITY_SCALE);
    Ok([
        checked_add(
            i128::from(rigid_box.body.velocity.x),
            div_round_nearest(rotation_x, scale)?,
        )?,
        checked_add(
            i128::from(rigid_box.body.velocity.y),
            div_round_nearest(rotation_y, scale)?,
        )?,
        checked_add(
            i128::from(rigid_box.body.velocity.z),
            div_round_nearest(rotation_z, scale)?,
        )?,
    ])
}

fn primitive_vector(vector: [i128; 3]) -> Result<Option<[i128; 3]>, ObbContactResponseError3d> {
    let mut divisor = 0_u128;
    for component in vector {
        divisor = gcd(divisor, component.unsigned_abs());
    }
    if divisor == 0 {
        return Ok(None);
    }
    let divisor =
        i128::try_from(divisor).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    Ok(Some([
        vector[0] / divisor,
        vector[1] / divisor,
        vector[2] / divisor,
    ]))
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn vector_length_squared(vector: [i128; 3]) -> Result<u128, ObbContactResponseError3d> {
    vector.into_iter().try_fold(0_u128, |sum, component| {
        let magnitude = component.unsigned_abs();
        let square = magnitude
            .checked_mul(magnitude)
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        sum.checked_add(square)
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
    })
}

fn scale_ratio_vector(
    numerator: i128,
    denominator: i128,
    axis: [i128; 3],
    axis_scale: i128,
) -> Result<[i128; 3], ObbContactResponseError3d> {
    if denominator <= 0 || axis_scale <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let mut impulse = [0_i128; 3];
    for (target, component) in impulse.iter_mut().zip(axis) {
        let scaled_component = checked_mul(component, axis_scale)?;
        *target = mul_div_round_i128(numerator, scaled_component, denominator)?;
    }
    Ok(impulse)
}

fn coulomb_clamp_vector(
    desired_impulse: [i128; 3],
    tangent: [i128; 3],
    normal_impulse: [i128; 3],
    friction_milli: u16,
) -> Result<[i128; 3], ObbContactResponseError3d> {
    if desired_impulse == [0; 3] || normal_impulse == [0; 3] || friction_milli == 0 {
        return Ok([0; 3]);
    }
    let normal_length_squared = vector_length_squared(normal_impulse)?;
    if within_coulomb_bound_vector(desired_impulse, normal_length_squared, friction_milli)? {
        return Ok(desired_impulse);
    }

    let pivot = dominant_vector_axis(tangent)?;
    let oriented_tangent = if tangent[pivot] < 0 {
        negate_axis(tangent)?
    } else {
        tangent
    };
    let pivot_denominator = oriented_tangent[pivot];
    if pivot_denominator <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let desired_pivot = desired_impulse[pivot];
    if desired_pivot == 0 {
        return Ok([0; 3]);
    }
    let negative = desired_pivot < 0;
    let mut low = 0_u128;
    let mut high = desired_pivot.unsigned_abs();
    while low < high {
        let distance = high
            .checked_sub(low)
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        let midpoint = low
            .checked_add(distance.div_ceil(2))
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        let candidate = tangent_impulse_from_pivot(
            midpoint,
            negative,
            oriented_tangent,
            pivot,
            pivot_denominator,
        )?;
        if within_coulomb_bound_vector(candidate, normal_length_squared, friction_milli)? {
            low = midpoint;
        } else {
            high = midpoint - 1;
        }
    }

    tangent_impulse_from_pivot(low, negative, oriented_tangent, pivot, pivot_denominator)
}

fn tangent_impulse_from_pivot(
    pivot_magnitude: u128,
    negative: bool,
    oriented_tangent: [i128; 3],
    pivot: usize,
    pivot_denominator: i128,
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let pivot_magnitude = i128::try_from(pivot_magnitude)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    let pivot_impulse = if negative {
        pivot_magnitude
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?
    } else {
        pivot_magnitude
    };
    let mut impulse = [0_i128; 3];
    for index in 0..3 {
        impulse[index] = if index == pivot {
            pivot_impulse
        } else {
            mul_div_round_i128(pivot_impulse, oriented_tangent[index], pivot_denominator)?
        };
    }
    Ok(impulse)
}

fn dominant_vector_axis(vector: [i128; 3]) -> Result<usize, ObbContactResponseError3d> {
    let mut best = 0_usize;
    let mut magnitude = vector[0].unsigned_abs();
    for (index, component) in vector.into_iter().enumerate().skip(1) {
        let candidate = component.unsigned_abs();
        if candidate > magnitude {
            best = index;
            magnitude = candidate;
        }
    }
    if magnitude == 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(best)
}

fn within_coulomb_bound_vector(
    tangent_impulse: [i128; 3],
    normal_length_squared: u128,
    friction_milli: u16,
) -> Result<bool, ObbContactResponseError3d> {
    let material_scale = u128::from(MATERIAL_SCALE);
    let friction = u128::from(friction_milli);
    let tangent_length_squared = vector_length_squared(tangent_impulse)?;
    let left = tangent_length_squared
        .checked_mul(material_scale)
        .and_then(|value| value.checked_mul(material_scale))
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let right = normal_length_squared
        .checked_mul(friction)
        .and_then(|value| value.checked_mul(friction))
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    Ok(left <= right)
}

fn body_effective_inverse_mass_scaled(
    rigid_box: &RigidBox3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<i128, ObbContactResponseError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(0);
    }
    let translational = i128::try_from(mul_div_round_u128(
        axis_length_squared,
        RESPONSE_SCALE as u128,
        u128::from(rigid_box.body.mass_units),
    )?)
    .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    if rigid_box.rotation_locked {
        return Ok(translational);
    }

    let angular_impulse = cross_i64_i128(contact_offset, axis)?;
    let local = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let scale = (RESPONSE_SCALE as u128)
        .checked_mul(u128::from(inertia.denominator))
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let mut rotational = 0_i128;
    for (index, component) in local.into_iter().enumerate() {
        let squared = checked_mul(component, component)?;
        let term = i128::try_from(mul_div_round_u128(
            squared as u128,
            scale,
            inertia.principal_numerators[index],
        )?)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
        rotational = checked_add(rotational, term)?;
    }
    checked_add(translational, rotational)
}

fn apply_body_impulse(
    rigid_box: &mut RigidBox3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), ObbContactResponseError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(());
    }
    let mass_units = rigid_box.body.mass_units;
    rigid_box.body.velocity = Vec3i::new(
        add_linear_impulse_axis(rigid_box.body.velocity.x, impulse[0], mass_units)?,
        add_linear_impulse_axis(rigid_box.body.velocity.y, impulse[1], mass_units)?,
        add_linear_impulse_axis(rigid_box.body.velocity.z, impulse[2], mass_units)?,
    );
    if rigid_box.rotation_locked {
        return Ok(());
    }

    let angular_impulse = cross_i64_i128(contact_offset, impulse)?;
    let local_impulse = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let angular_scale = i128::from(inertia.denominator)
        .checked_mul(i128::from(ANGULAR_VELOCITY_SCALE))
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let mut local_delta = [0_i128; 3];
    for (index, (target, component)) in local_delta.iter_mut().zip(local_impulse).enumerate() {
        let principal = i128::try_from(inertia.principal_numerators[index])
            .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
        *target = mul_div_round_i128(component, angular_scale, principal)?;
    }
    let world_delta = rotate_forward(rigid_box.angular.orientation, local_delta)?;
    rigid_box.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(rigid_box.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(rigid_box.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(rigid_box.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn add_linear_impulse_axis(
    current: i32,
    impulse: i128,
    mass_units: u32,
) -> Result<i32, ObbContactResponseError3d> {
    let delta = div_round_nearest(impulse, i128::from(mass_units))?;
    to_i32(checked_add(i128::from(current), delta)?)
}

fn add_angular_axis(current: i32, delta: i128) -> Result<i32, ObbContactResponseError3d> {
    to_i32(checked_add(i128::from(current), delta)?)
}

fn cross_i64_i128(
    left: [i64; 3],
    right: [i128; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let left = left.map(i128::from);
    Ok([
        cross_component(left[1], right[2], left[2], right[1])?,
        cross_component(left[2], right[0], left[0], right[2])?,
        cross_component(left[0], right[1], left[1], right[0])?,
    ])
}

fn cross_component(
    left_a: i128,
    right_a: i128,
    left_b: i128,
    right_b: i128,
) -> Result<i128, ObbContactResponseError3d> {
    checked_sub(checked_mul(left_a, right_a)?, checked_mul(left_b, right_b)?)
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let matrix = rotation_matrix(orientation.normalized()?)?;
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_forward(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(
    orientation: Orientation3d,
) -> Result<[[i128; 3]; 3], ObbContactResponseError3d> {
    let x = i128::from(orientation.x);
    let y = i128::from(orientation.y);
    let z = i128::from(orientation.z);
    let w = i128::from(orientation.w);
    let scale = i128::from(ORIENTATION_SCALE);
    let xx = checked_mul(x, x)?;
    let yy = checked_mul(y, y)?;
    let zz = checked_mul(z, z)?;
    let xy = checked_mul(x, y)?;
    let xz = checked_mul(x, z)?;
    let yz = checked_mul(y, z)?;
    let xw = checked_mul(x, w)?;
    let yw = checked_mul(y, w)?;
    let zw = checked_mul(z, w)?;
    Ok([
        [
            checked_sub(scale, scaled_twice(checked_add(yy, zz)?, scale)?)?,
            scaled_twice(checked_sub(xy, zw)?, scale)?,
            scaled_twice(checked_add(xz, yw)?, scale)?,
        ],
        [
            scaled_twice(checked_add(xy, zw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, zz)?, scale)?)?,
            scaled_twice(checked_sub(yz, xw)?, scale)?,
        ],
        [
            scaled_twice(checked_sub(xz, yw)?, scale)?,
            scaled_twice(checked_add(yz, xw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, yy)?, scale)?)?,
        ],
    ])
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i128; 3],
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        *target = div_round_nearest(
            checked_add(
                checked_add(
                    checked_mul(row[0], vector[0])?,
                    checked_mul(row[1], vector[1])?,
                )?,
                checked_mul(row[2], vector[2])?,
            )?,
            scale,
        )?;
    }
    Ok(output)
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, ObbContactResponseError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, ObbContactResponseError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn checked_mul(left: i128, right: i128) -> Result<i128, ObbContactResponseError3d> {
    left.checked_mul(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, ObbContactResponseError3d> {
    left.checked_add(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, ObbContactResponseError3d> {
    left.checked_sub(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn negate_axis(axis: [i128; 3]) -> Result<[i128; 3], ObbContactResponseError3d> {
    Ok([
        axis[0]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        axis[1]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        axis[2]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
    ])
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, ObbContactResponseError3d> {
    if denominator <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn to_i32(value: i128) -> Result<i32, ObbContactResponseError3d> {
    i32::try_from(value).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material,
        ObbContactResponseError3d, Orientation3d, RigidBody, RigidBox3d, Vec3i,
    };

    use super::{
        RESPONSE_SCALE, mul_div_round_i128, resolve_normal_obb_contact, resolve_obb_contact,
        scale_ratio_vector,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, friction_milli: u16) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(10, 10, 10))
                .with_material(Material::new(0).with_friction(friction_milli)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    #[test]
    fn zero_friction_preserves_normal_only_response() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 40), 0);
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO, 0);
        let expected = resolve_normal_obb_contact(left.clone(), right.clone(), true)
            .expect("valid normal-only response");
        let actual = resolve_obb_contact(left, right, true).expect("valid friction wrapper");

        assert_eq!(actual, expected);
    }

    #[test]
    fn sliding_face_impact_reduces_slip_and_generates_spin() {
        let left_start = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 40), MATERIAL_SCALE);
        let right_start = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO, 0);
        let response = resolve_obb_contact(left_start.clone(), right_start.clone(), true)
            .expect("valid frictional response");
        let contact = response.contact.expect("overlap should contact");
        let left_offset = super::vector_delta(left_start.body.position, contact.point)
            .expect("valid left offset");
        let right_offset = super::vector_delta(right_start.body.position, contact.point)
            .expect("valid right offset");
        let relative = super::relative_contact_velocity(
            &response.left,
            left_offset,
            &response.right,
            right_offset,
        )
        .expect("valid relative velocity");

        assert_ne!(contact.normal_impulse, [0; 3]);
        assert!(relative[2].unsigned_abs() < 40);
        assert_ne!(response.left.angular.angular_velocity.y, 0);
        assert_ne!(response.right.angular.angular_velocity.y, 0);
    }

    #[test]
    fn separating_overlap_does_not_inject_friction() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(-20, 0, 40), MATERIAL_SCALE);
        let right = dynamic(
            2,
            Vec3i::new(19, 0, 0),
            Vec3i::new(20, 0, 0),
            MATERIAL_SCALE,
        );
        let response = resolve_obb_contact(left.clone(), right.clone(), true)
            .expect("valid separating overlap");

        assert_eq!(
            response.contact.expect("contact evidence").normal_impulse,
            [0; 3]
        );
        assert_eq!(response.left.body.velocity, left.body.velocity);
        assert_eq!(response.right.body.velocity, right.body.velocity);
    }

    #[test]
    fn rotation_lock_blocks_friction_spin_but_not_linear_friction() {
        let left =
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 40), MATERIAL_SCALE).with_rotation_locked();
        let right = RigidBox3d::new(
            RigidBody::fixed(BodyId(2), Vec3i::new(19, 0, 0), Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed wall");
        let response =
            resolve_obb_contact(left, right, true).expect("valid locked friction response");

        assert!(response.left.rotation_locked());
        assert!(response.left.angular.angular_velocity.is_zero());
        assert!(response.left.body.velocity.z.unsigned_abs() < 40);
    }

    #[test]
    fn fractional_scalar_impulse_survives_rotated_tangent_scaling() {
        let tangent = [-110, 167, 0];
        let denominator = RESPONSE_SCALE
            .checked_mul(3)
            .expect("test denominator fits");
        assert_eq!(
            mul_div_round_i128(1, RESPONSE_SCALE, denominator)
                .expect("legacy scalar calculation is representable"),
            0
        );
        assert_eq!(
            scale_ratio_vector(1, denominator, tangent, RESPONSE_SCALE)
                .expect("vector ratio remains representable"),
            [-37, 56, 0]
        );
    }

    #[test]
    fn invalid_friction_fails_closed() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), MATERIAL_SCALE + 1);
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO, 0);
        let error =
            resolve_obb_contact(left, right, true).expect_err("invalid friction must fail closed");

        assert_eq!(error, ObbContactResponseError3d::ArithmeticOverflow);
    }
}
