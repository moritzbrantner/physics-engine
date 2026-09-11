use std::{error::Error, fmt};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularVelocity3d, BodyId, BodyKind, MATERIAL_SCALE,
    ORIENTATION_SCALE, ObbAxisFeature3d, ObbContactSeed3d, Orientation3d, OrientedBoxError3d,
    RigidBox3d, Vec3i, box_inertia, obb_contact_seed, oriented_box_vertices,
    wide_ratio::{WideRatioError, mul_div_round_i128, mul_div_round_u128},
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;

/// Reduced deterministic OBB contact evidence plus the vector impulse applied at that contact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ObbResolvedContact3d {
    pub point: Vec3i,
    pub axis: [i128; 3],
    pub feature: ObbAxisFeature3d,
    pub overlap_numerator: u128,
    pub axis_length_squared: u128,
    pub left_support_mask: u8,
    pub right_support_mask: u8,
    /// Impulse applied to the right body; the left body receives the equal-and-opposite vector.
    /// Keeping the vector directly avoids discarding representable impulses when a non-unit SAT axis
    /// would require a fractional scalar multiplier.
    pub normal_impulse: [i128; 3],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObbContactResponse3d {
    pub left: RigidBox3d,
    pub right: RigidBox3d,
    pub contact: Option<ObbResolvedContact3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObbContactResponseError3d {
    NonCanonicalPair(BodyId, BodyId),
    RestitutionOutOfRange(BodyId, u16),
    Angular(AngularError3d),
    Geometry(OrientedBoxError3d),
    ArithmeticOverflow,
}

impl fmt::Display for ObbContactResponseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonCanonicalPair(left, right) => write!(
                formatter,
                "OBB response expects ascending body ids, got {} then {}",
                left.0, right.0
            ),
            Self::RestitutionOutOfRange(body, value) => write!(
                formatter,
                "OBB response body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                body.0
            ),
            Self::Angular(error) => write!(formatter, "OBB angular response failed: {error}"),
            Self::Geometry(error) => write!(formatter, "OBB contact geometry failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "OBB contact response arithmetic overflowed")
            }
        }
    }
}

impl Error for ObbContactResponseError3d {}

impl From<AngularError3d> for ObbContactResponseError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<OrientedBoxError3d> for ObbContactResponseError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

impl From<WideRatioError> for ObbContactResponseError3d {
    fn from(_: WideRatioError) -> Self {
        Self::ArithmeticOverflow
    }
}

/// Resolves one engine-native overlapping OBB pair with deterministic linear/angular normal response
/// and penetration projection.
///
/// Pair order is part of the deterministic contract. The function consumes exact SAT contact evidence,
/// reduces the stable support sets to one contact point, evaluates relative velocity including rotational
/// motion at that point, and applies one equal-and-opposite normal impulse. Penetration is then projected
/// out along the SAT minimum-translation axis with deterministic mass weighting. Rotation-locked bodies
/// remain translationally dynamic but contribute no rotational velocity/inverse inertia and receive no
/// angular impulse.
///
/// `allow_restitution` exists for iterative island/frontier solvers: the first physical impact pass may
/// use material restitution, while later numerical passes can restore non-approaching constraints without
/// creating another bounce. Restitution combines using the standalone engine's existing conservative
/// `min(left, right)` material rule; this extraction does not silently change the translational solver's
/// established behavior.
///
/// A zero-depth contact tied across multiple SAT axes still needs to make forward progress when bodies are
/// approaching. The stable SAT tie-break axis is therefore used as a deterministic translation constraint,
/// but angular impulse is suppressed for that one ambiguous contact so feature ordering cannot invent an
/// arbitrary torque. Once a unique normal or a real manifold exists, ordinary angular response applies.
///
/// Tangential friction and full polygon manifold clipping are intentionally separate capabilities. This
/// function resolves contact state only; it does not discover time of impact and therefore makes no analytic
/// rotational CCD claim.
///
/// # Errors
///
/// Returns [`ObbContactResponseError3d`] for non-canonical body order, invalid material values, angular or
/// OBB geometry failures, or checked arithmetic overflow.
pub fn resolve_obb_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    allow_restitution: bool,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {
    validate_pair(&left, &right)?;
    let Some(seed) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
        return Ok(ObbContactResponse3d {
            left,
            right,
            contact: None,
        });
    };

    let left_vertices = oriented_box_vertices(left.oriented_box())?;
    let right_vertices = oriented_box_vertices(right.oriented_box())?;
    let left_support = support_centroid(&left_vertices, seed.left_support_mask)?;
    let right_support = support_centroid(&right_vertices, seed.right_support_mask)?;
    let point = reduced_contact_point(
        &left_vertices,
        seed.left_support_mask,
        left_support,
        &left,
        &right_vertices,
        seed.right_support_mask,
        right_support,
        &right,
        seed,
    )?;
    let left_offset = vector_delta(left.body.position, point)?;
    let right_offset = vector_delta(right.body.position, point)?;
    let left_velocity = contact_velocity(&left, left_offset)?;
    let right_velocity = contact_velocity(&right, right_offset)?;
    let relative_velocity = [
        checked_sub(i128::from(right_velocity[0]), i128::from(left_velocity[0]))?,
        checked_sub(i128::from(right_velocity[1]), i128::from(left_velocity[1]))?,
        checked_sub(i128::from(right_velocity[2]), i128::from(left_velocity[2]))?,
    ];
    let normal_velocity = checked_dot(relative_velocity, seed.axis)?;
    let tied_zero_depth = seed.overlap_numerator == 0 && seed.minimum_axis_ties > 1;

    let mut resolved_left = left;
    let mut resolved_right = right;
    let normal_impulse = if normal_velocity < 0
        && (resolved_left.body.kind == BodyKind::Dynamic
            || resolved_right.body.kind == BodyKind::Dynamic)
    {
        let impulse = normal_impulse_vector(
            &resolved_left,
            left_offset,
            &resolved_right,
            right_offset,
            seed.axis,
            seed.axis_length_squared,
            normal_velocity,
            allow_restitution,
            !tied_zero_depth,
        )?;
        if impulse != [0; 3] {
            apply_body_impulse(
                &mut resolved_left,
                left_offset,
                negate_axis(impulse)?,
                !tied_zero_depth,
            )?;
            apply_body_impulse(&mut resolved_right, right_offset, impulse, !tied_zero_depth)?;
        }
        impulse
    } else {
        [0; 3]
    };

    if seed.overlap_numerator > 0
        && !(resolved_left.body.kind == BodyKind::Fixed
            && resolved_right.body.kind == BodyKind::Fixed)
    {
        let correction = minimum_translation_vector(
            seed.axis,
            seed.overlap_numerator,
            seed.axis_length_squared,
        )?;
        project_pair(&mut resolved_left, &mut resolved_right, correction)?;
    }

    Ok(ObbContactResponse3d {
        left: resolved_left,
        right: resolved_right,
        contact: Some(ObbResolvedContact3d {
            point,
            axis: seed.axis,
            feature: seed.feature,
            overlap_numerator: seed.overlap_numerator,
            axis_length_squared: seed.axis_length_squared,
            left_support_mask: seed.left_support_mask,
            right_support_mask: seed.right_support_mask,
            normal_impulse,
        }),
    })
}

fn validate_pair(left: &RigidBox3d, right: &RigidBox3d) -> Result<(), ObbContactResponseError3d> {
    if left.body.id >= right.body.id {
        return Err(ObbContactResponseError3d::NonCanonicalPair(
            left.body.id,
            right.body.id,
        ));
    }
    for body in [&left.body, &right.body] {
        let restitution = body.material.restitution_milli();
        if restitution > MATERIAL_SCALE {
            return Err(ObbContactResponseError3d::RestitutionOutOfRange(
                body.id,
                restitution,
            ));
        }
    }
    Ok(())
}

fn support_centroid(vertices: &[Vec3i; 8], mask: u8) -> Result<Vec3i, ObbContactResponseError3d> {
    let mut sum = [0_i128; 3];
    let mut count = 0_i128;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = checked_add(count, 1)?;
        sum[0] = checked_add(sum[0], i128::from(vertex.x))?;
        sum[1] = checked_add(sum[1], i128::from(vertex.y))?;
        sum[2] = checked_add(sum[2], i128::from(vertex.z))?;
    }
    if count == 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(Vec3i::new(
        to_i32(div_round_nearest(sum[0], count)?)?,
        to_i32(div_round_nearest(sum[1], count)?)?,
        to_i32(div_round_nearest(sum[2], count)?)?,
    ))
}

#[allow(clippy::too_many_arguments)]
fn reduced_contact_point(
    left_vertices: &[Vec3i; 8],
    left_mask: u8,
    left_support: Vec3i,
    left: &RigidBox3d,
    right_vertices: &[Vec3i; 8],
    right_mask: u8,
    right_support: Vec3i,
    right: &RigidBox3d,
    seed: ObbContactSeed3d,
) -> Result<Vec3i, ObbContactResponseError3d> {
    if let Some(point) = axis_aligned_face_overlap_centroid(
        left_vertices,
        left_mask,
        left_support,
        right_vertices,
        right_mask,
        right_support,
        seed.axis,
    )? {
        return Ok(point);
    }

    let left_spread = support_spread_squared(left_vertices, left_mask, left_support)?;
    let right_spread = support_spread_squared(right_vertices, right_mask, right_support)?;
    let anchor = match left_spread.cmp(&right_spread) {
        std::cmp::Ordering::Less => left_support,
        std::cmp::Ordering::Greater => right_support,
        std::cmp::Ordering::Equal => {
            if left.body.id < right.body.id {
                left_support
            } else {
                right_support
            }
        }
    };
    project_to_support_midplane(
        anchor,
        left_support,
        right_support,
        seed.axis,
        seed.axis_length_squared,
    )
}

fn axis_aligned_face_overlap_centroid(
    left_vertices: &[Vec3i; 8],
    left_mask: u8,
    left_support: Vec3i,
    right_vertices: &[Vec3i; 8],
    right_mask: u8,
    right_support: Vec3i,
    axis: [i128; 3],
) -> Result<Option<Vec3i>, ObbContactResponseError3d> {
    if left_mask.count_ones() != 4 || right_mask.count_ones() != 4 {
        return Ok(None);
    }
    let Some(normal_axis) = coordinate_axis(axis) else {
        return Ok(None);
    };
    let tangent_axes = match normal_axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => return Ok(None),
    };
    if !support_is_axis_aligned_rectangle(left_vertices, left_mask, tangent_axes)
        || !support_is_axis_aligned_rectangle(right_vertices, right_mask, tangent_axes)
    {
        return Ok(None);
    }

    let mut coordinate = [0_i32; 3];
    coordinate[normal_axis] = midpoint_axis(
        component(left_support, normal_axis),
        component(right_support, normal_axis),
    )?;
    for tangent_axis in tangent_axes {
        let (left_minimum, left_maximum) =
            support_interval(left_vertices, left_mask, tangent_axis)?;
        let (right_minimum, right_maximum) =
            support_interval(right_vertices, right_mask, tangent_axis)?;
        let overlap_minimum = left_minimum.max(right_minimum);
        let overlap_maximum = left_maximum.min(right_maximum);
        if overlap_minimum > overlap_maximum {
            return Ok(None);
        }
        coordinate[tangent_axis] = midpoint_axis(overlap_minimum, overlap_maximum)?;
    }
    Ok(Some(Vec3i::new(
        coordinate[0],
        coordinate[1],
        coordinate[2],
    )))
}

fn coordinate_axis(axis: [i128; 3]) -> Option<usize> {
    let mut found = None;
    for (index, component) in axis.into_iter().enumerate() {
        if component == 0 {
            continue;
        }
        if found.is_some() {
            return None;
        }
        found = Some(index);
    }
    found
}

fn support_is_axis_aligned_rectangle(
    vertices: &[Vec3i; 8],
    mask: u8,
    tangent_axes: [usize; 2],
) -> bool {
    tangent_axes.into_iter().all(|axis| {
        let mut unique = [0_i32; 4];
        let mut unique_count = 0_usize;
        for (index, vertex) in vertices.iter().enumerate() {
            if mask & (1_u8 << index) == 0 {
                continue;
            }
            let value = component(*vertex, axis);
            if unique[..unique_count].contains(&value) {
                continue;
            }
            if unique_count == unique.len() {
                return false;
            }
            unique[unique_count] = value;
            unique_count += 1;
        }
        unique_count == 2
    })
}

fn support_interval(
    vertices: &[Vec3i; 8],
    mask: u8,
    axis: usize,
) -> Result<(i32, i32), ObbContactResponseError3d> {
    let mut minimum = None;
    let mut maximum = None;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        let value = component(*vertex, axis);
        minimum = Some(minimum.map_or(value, |current: i32| current.min(value)));
        maximum = Some(maximum.map_or(value, |current: i32| current.max(value)));
    }
    match (minimum, maximum) {
        (Some(minimum), Some(maximum)) => Ok((minimum, maximum)),
        _ => Err(ObbContactResponseError3d::ArithmeticOverflow),
    }
}

fn support_spread_squared(
    vertices: &[Vec3i; 8],
    mask: u8,
    center: Vec3i,
) -> Result<u128, ObbContactResponseError3d> {
    let mut total = 0_u128;
    let mut count = 0_u8;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = count
            .checked_add(1)
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        for value in [
            i128::from(vertex.x) - i128::from(center.x),
            i128::from(vertex.y) - i128::from(center.y),
            i128::from(vertex.z) - i128::from(center.z),
        ] {
            let magnitude = value.unsigned_abs();
            total = total
                .checked_add(
                    magnitude
                        .checked_mul(magnitude)
                        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
                )
                .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        }
    }
    if count == 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(total)
}

fn project_to_support_midplane(
    anchor: Vec3i,
    left_support: Vec3i,
    right_support: Vec3i,
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<Vec3i, ObbContactResponseError3d> {
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    if length_squared <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let target_projection = div_round_nearest(
        checked_add(dot_vec(left_support, axis)?, dot_vec(right_support, axis)?)?,
        2,
    )?;
    let projection_delta = checked_sub(target_projection, dot_vec(anchor, axis)?)?;
    let mut coordinate = [
        i128::from(anchor.x),
        i128::from(anchor.y),
        i128::from(anchor.z),
    ];
    for index in 0..3 {
        coordinate[index] = checked_add(
            coordinate[index],
            div_round_nearest(checked_mul(projection_delta, axis[index])?, length_squared)?,
        )?;
    }
    Ok(Vec3i::new(
        to_i32(coordinate[0])?,
        to_i32(coordinate[1])?,
        to_i32(coordinate[2])?,
    ))
}

fn midpoint_axis(left: i32, right: i32) -> Result<i32, ObbContactResponseError3d> {
    to_i32(div_round_nearest(
        checked_add(i128::from(left), i128::from(right))?,
        2,
    )?)
}

fn vector_delta(center: Vec3i, point: Vec3i) -> Result<[i64; 3], ObbContactResponseError3d> {
    Ok([
        i64::from(point.x) - i64::from(center.x),
        i64::from(point.y) - i64::from(center.y),
        i64::from(point.z) - i64::from(center.z),
    ])
}

fn contact_velocity(
    rigid_box: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i64; 3], ObbContactResponseError3d> {
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
        add_linear_rotation(rigid_box.body.velocity.x, rotation_x, scale)?,
        add_linear_rotation(rigid_box.body.velocity.y, rotation_y, scale)?,
        add_linear_rotation(rigid_box.body.velocity.z, rotation_z, scale)?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
    scale: i128,
) -> Result<i64, ObbContactResponseError3d> {
    let rotational = i64::try_from(div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

#[allow(clippy::too_many_arguments)]
fn normal_impulse_vector(
    left: &RigidBox3d,
    left_offset: [i64; 3],
    right: &RigidBox3d,
    right_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
    normal_velocity: i128,
    allow_restitution: bool,
    include_angular: bool,
) -> Result<[i128; 3], ObbContactResponseError3d> {
    let effective_inverse_mass = checked_add(
        body_effective_inverse_mass_scaled(
            left,
            left_offset,
            axis,
            axis_length_squared,
            include_angular,
        )?,
        body_effective_inverse_mass_scaled(
            right,
            right_offset,
            axis,
            axis_length_squared,
            include_angular,
        )?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok([0; 3]);
    }

    let restitution = if allow_restitution {
        left.body
            .material
            .restitution_milli()
            .min(right.body.material.restitution_milli())
    } else {
        0
    };
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    let bounce_scale = checked_add(i128::from(MATERIAL_SCALE), i128::from(restitution))?;
    let numerator = checked_mul(closing_speed, bounce_scale)?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    scale_ratio_vector(numerator, denominator, axis, RESPONSE_SCALE)
}

fn scale_ratio_vector(
    numerator: i128,
    denominator: i128,
    axis: [i128; 3],
    axis_scale: i128,
) -> Result<[i128; 3], ObbContactResponseError3d> {
    if numerator < 0 || denominator <= 0 || axis_scale <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let mut impulse = [0_i128; 3];
    for (target, component) in impulse.iter_mut().zip(axis) {
        let scaled_component = checked_mul(component, axis_scale)?;
        *target = mul_div_round_i128(numerator, scaled_component, denominator)?;
    }
    Ok(impulse)
}

fn body_effective_inverse_mass_scaled(
    rigid_box: &RigidBox3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
    include_angular: bool,
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
    if !include_angular || rigid_box.rotation_locked {
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
    include_angular: bool,
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
    if !include_angular || rigid_box.rotation_locked {
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

fn minimum_translation_vector(
    axis: [i128; 3],
    overlap_numerator: u128,
    axis_length_squared: u128,
) -> Result<[i64; 3], ObbContactResponseError3d> {
    let overlap = i128::try_from(overlap_numerator)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    if overlap <= 0 || length_squared <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let mut correction = [0_i128; 3];
    for (target, component) in correction.iter_mut().zip(axis) {
        *target = div_round_nearest(checked_mul(overlap, component)?, length_squared)?;
    }
    let achieved = checked_dot(correction, axis)?;
    if achieved < overlap {
        let dominant = dominant_axis(axis)?;
        let magnitude = axis[dominant]
            .checked_abs()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        let residual = checked_sub(overlap, achieved)?;
        let extra = div_ceil_positive(residual, magnitude)?;
        correction[dominant] = checked_add(
            correction[dominant],
            if axis[dominant] < 0 { -extra } else { extra },
        )?;
    }
    Ok([
        i64::try_from(correction[0]).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?,
        i64::try_from(correction[1]).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?,
        i64::try_from(correction[2]).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?,
    ])
}

fn dominant_axis(axis: [i128; 3]) -> Result<usize, ObbContactResponseError3d> {
    let mut best = 0_usize;
    let mut magnitude = axis[0]
        .checked_abs()
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    for (index, component) in axis.into_iter().enumerate().skip(1) {
        let candidate = component
            .checked_abs()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
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

fn project_pair(
    left: &mut RigidBox3d,
    right: &mut RigidBox3d,
    correction: [i64; 3],
) -> Result<(), ObbContactResponseError3d> {
    match (left.body.kind, right.body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => Ok(()),
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            right.body.position = offset_position(right.body.position, correction)?;
            Ok(())
        }
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            left.body.position =
                offset_position(left.body.position, negate_i64_vector(correction)?)?;
            Ok(())
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let total_mass = u64::from(left.body.mass_units)
                .checked_add(u64::from(right.body.mass_units))
                .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
            let mut left_move = [0_i64; 3];
            let mut right_move = [0_i64; 3];
            for axis in 0..3 {
                let weighted = checked_mul(
                    i128::from(correction[axis]),
                    i128::from(right.body.mass_units),
                )?;
                let share = div_round_nearest(weighted, i128::from(total_mass))?;
                left_move[axis] = i64::try_from(
                    share
                        .checked_neg()
                        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
                )
                .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
                right_move[axis] = correction[axis]
                    .checked_add(left_move[axis])
                    .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
            }
            left.body.position = offset_position(left.body.position, left_move)?;
            right.body.position = offset_position(right.body.position, right_move)?;
            Ok(())
        }
    }
}

fn offset_position(position: Vec3i, delta: [i64; 3]) -> Result<Vec3i, ObbContactResponseError3d> {
    Ok(Vec3i::new(
        to_i32(checked_add(i128::from(position.x), i128::from(delta[0]))?)?,
        to_i32(checked_add(i128::from(position.y), i128::from(delta[1]))?)?,
        to_i32(checked_add(i128::from(position.z), i128::from(delta[2]))?)?,
    ))
}

fn negate_i64_vector(vector: [i64; 3]) -> Result<[i64; 3], ObbContactResponseError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
    ])
}

fn negate_axis(vector: [i128; 3]) -> Result<[i128; 3], ObbContactResponseError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
    ])
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

fn component(vector: Vec3i, axis: usize) -> i32 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        2 => vector.z,
        _ => 0,
    }
}

fn dot_vec(vector: Vec3i, axis: [i128; 3]) -> Result<i128, ObbContactResponseError3d> {
    checked_dot(
        [
            i128::from(vector.x),
            i128::from(vector.y),
            i128::from(vector.z),
        ],
        axis,
    )
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

fn div_ceil_positive(
    numerator: i128,
    denominator: i128,
) -> Result<i128, ObbContactResponseError3d> {
    if numerator < 0 || denominator <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    numerator
        .checked_add(denominator - 1)
        .map(|value| value / denominator)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn to_i32(value: i128) -> Result<i32, ObbContactResponseError3d> {
    i32::try_from(value).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, Vec3i,
    };

    use super::{ObbContactResponseError3d, resolve_obb_contact, scale_ratio_vector};

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    #[test]
    fn centered_face_impact_equalizes_velocity_without_spin() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0));
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO);
        let response = resolve_obb_contact(left, right, true).expect("valid response");
        let contact = response.contact.expect("contact");

        assert_eq!(contact.axis, [1, 0, 0]);
        assert_eq!(contact.normal_impulse, [30, 0, 0]);
        assert_eq!(response.left.body.velocity.x, 30);
        assert_eq!(response.right.body.velocity.x, 30);
        assert_eq!(
            response.left.angular.angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(
            response.right.angular.angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(
            response.right.body.position.x - response.left.body.position.x,
            20
        );
    }

    #[test]
    fn off_center_face_impact_generates_spin() {
        let left = dynamic(1, Vec3i::new(0, 6, 0), Vec3i::new(90, 0, 0));
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO);
        let response = resolve_obb_contact(left, right, true).expect("valid response");

        assert_ne!(response.left.angular.angular_velocity.z, 0);
        assert_ne!(response.right.angular.angular_velocity.z, 0);
    }

    #[test]
    fn rotation_locked_body_stays_upright_but_transmits_linear_impulse() {
        let left = dynamic(1, Vec3i::new(0, 6, 0), Vec3i::new(90, 0, 0))
            .with_rotation_locked();
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO);
        let response = resolve_obb_contact(left, right, true).expect("valid locked response");

        assert!(response.left.rotation_locked());
        assert!(response.left.angular.angular_velocity.is_zero());
        assert_eq!(response.left.angular.orientation, Orientation3d::IDENTITY);
        assert_ne!(response.left.body.velocity.x, 90);
        assert_ne!(response.right.body.velocity.x, 0);
        assert_ne!(response.right.angular.angular_velocity.z, 0);
    }

    #[test]
    fn tied_zero_depth_corner_impact_makes_progress_without_invented_torque() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(10, 10, 0));
        let right = RigidBox3d::new(
            RigidBody::fixed(BodyId(2), Vec3i::new(20, 20, 0), Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box");
        let response = resolve_obb_contact(left, right, true).expect("valid tied response");
        let contact = response.contact.expect("corner contact");

        assert_eq!(contact.overlap_numerator, 0);
        assert_ne!(contact.normal_impulse, [0; 3]);
        assert_ne!(response.left.body.velocity, Vec3i::new(10, 10, 0));
        assert_eq!(
            response.left.angular.angular_velocity,
            AngularVelocity3d::default()
        );
    }

    #[test]
    fn fractional_scalar_impulse_survives_non_unit_axis_scaling() {
        assert_eq!(
            scale_ratio_vector(2, 5, [2, 1, 0], 1).expect("representable vector"),
            [1, 0, 0]
        );
    }

    #[test]
    fn large_inertia_still_produces_representable_angular_response() {
        let half = Vec3i::new(100_000_000, 100_000_000, 100_000_000);
        let left = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(0, 50_000_000, 0),
                Vec3i::new(1_000_000, 0, 0),
                half,
            ),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid large dynamic box");
        let right = RigidBox3d::new(
            RigidBody::fixed(BodyId(2), Vec3i::new(199_999_999, 0, 0), half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid large fixed box");
        let response = resolve_obb_contact(left, right, true).expect("valid large response");

        assert_ne!(response.left.angular.angular_velocity.z, 0);
    }

    #[test]
    fn fixed_wall_remains_unchanged() {
        let left = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(1),
                Vec3i::ZERO,
                Vec3i::new(60, 0, 0),
                Vec3i::new(10, 10, 10),
            )
            .with_material(Material::new(MATERIAL_SCALE)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box");
        let wall = RigidBox3d::new(
            RigidBody::fixed(BodyId(2), Vec3i::new(19, 0, 0), Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid wall");
        let expected_wall = wall.clone();
        let response = resolve_obb_contact(left, wall, true).expect("valid wall response");

        assert_eq!(response.left.body.velocity.x, 0);
        assert_eq!(response.right, expected_wall);
    }

    #[test]
    fn later_solver_pass_can_disable_restitution() {
        let left = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(1),
                Vec3i::ZERO,
                Vec3i::new(60, 0, 0),
                Vec3i::new(10, 10, 10),
            )
            .with_material(Material::new(MATERIAL_SCALE)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box");
        let wall = RigidBox3d::new(
            RigidBody::fixed(BodyId(2), Vec3i::new(19, 0, 0), Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid wall");
        let response = resolve_obb_contact(left, wall, false).expect("valid inelastic pass");

        assert_eq!(response.left.body.velocity.x, 0);
    }

    #[test]
    fn non_canonical_pair_fails_closed() {
        let error = resolve_obb_contact(
            dynamic(2, Vec3i::ZERO, Vec3i::ZERO),
            dynamic(1, Vec3i::new(19, 0, 0), Vec3i::ZERO),
            true,
        )
        .expect_err("order must be explicit");

        assert_eq!(
            error,
            ObbContactResponseError3d::NonCanonicalPair(BodyId(2), BodyId(1))
        );
    }
}
