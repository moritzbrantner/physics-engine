use std::{error::Error, fmt};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularVelocity3d, BodyId, BodyKind, MATERIAL_SCALE,
    Material, ObbContactSeed3d, ORIENTATION_SCALE, Orientation3d, OrientedBoxError3d, RigidBox3d,
    Vec3i, box_inertia, obb_contact_seed, oriented_box_vertices,
};

const RESPONSE_SCALE: i128 = 1_i128 << 50;

/// Deterministic normal-response evidence for one rotating OBB contact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingBoxContactResponse3d {
    pub point: Vec3i,
    pub seed: ObbContactSeed3d,
    /// Scalar multiplier for the primitive SAT axis. The impulse vector is
    /// `normal_impulse_units * seed.axis`, applied with opposite signs to the two bodies.
    pub normal_impulse_units: i64,
}

/// Result of resolving one rotating-box pair without advancing time.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingBoxPairResponse3d {
    pub left: RigidBox3d,
    pub right: RigidBox3d,
    pub contact: Option<RotatingBoxContactResponse3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactResponseError3d {
    SameBody(BodyId),
    RestitutionOutOfRange(BodyId, u16),
    EmptySupportMask,
    Angular(AngularError3d),
    Geometry(OrientedBoxError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RotatingContactResponseError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SameBody(id) => write!(
                formatter,
                "rotating OBB response requires two distinct bodies, got {} twice",
                id.0
            ),
            Self::RestitutionOutOfRange(id, value) => write!(
                formatter,
                "rotating OBB response body {} has restitution {value}, expected 0..={MATERIAL_SCALE}",
                id.0
            ),
            Self::EmptySupportMask => write!(
                formatter,
                "rotating OBB response received an empty support feature"
            ),
            Self::Angular(error) => write!(formatter, "rotating OBB angular response failed: {error}"),
            Self::Geometry(error) => write!(formatter, "rotating OBB contact geometry failed: {error}"),
            Self::ArithmeticOverflow => write!(formatter, "rotating OBB response arithmetic overflowed"),
        }
    }
}

impl Error for RotatingContactResponseError3d {}

impl From<AngularError3d> for RotatingContactResponseError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<OrientedBoxError3d> for RotatingContactResponseError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

#[derive(Clone, Copy)]
struct SupportFeature3d {
    vertices: [Vec3i; 8],
    mask: u8,
    centroid: Vec3i,
    kind: BodyKind,
}

#[derive(Clone, Copy)]
struct ContactFrame3d<'a> {
    body: &'a RigidBox3d,
    offset: [i64; 3],
}

/// Resolves one overlapping rotating OBB pair with one deterministic normal impulse.
///
/// Contact geometry is recomputed from the current pair state. Support features are reduced to one stable
/// contact point, relative velocity includes angular motion, and an equal-and-opposite normal impulse is
/// applied using translational mass plus body-space box inertia. Axis-aligned face contacts use the actual
/// shared rectangular support patch; dynamic↔fixed contacts otherwise anchor on the tighter support feature
/// instead of the fixed body's potentially huge face center.
///
/// Exact zero-depth contacts tied across multiple SAT axes are observation-only. With no unique normal,
/// applying the first stable tied axis would turn deterministic feature ordering into arbitrary torque.
/// Penetrating contacts and unique-axis zero-depth impacts retain ordinary normal response.
///
/// This function does **not** advance position/orientation, iterate a contact island, apply friction, or
/// perform rotational CCD. It is the response primitive consumed by later frontier/repeated-event solver
/// layers.
///
/// # Errors
///
/// Returns [`RotatingContactResponseError3d`] for duplicate identity, invalid restitution, invalid geometry,
/// angular arithmetic failures, or checked integer overflow.
pub fn resolve_rotating_box_pair(
    left: &RigidBox3d,
    right: &RigidBox3d,
) -> Result<RotatingBoxPairResponse3d, RotatingContactResponseError3d> {
    validate_body(left)?;
    validate_body(right)?;
    if left.body().id() == right.body().id() {
        return Err(RotatingContactResponseError3d::SameBody(left.body().id()));
    }

    let Some(seed) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
        return Ok(RotatingBoxPairResponse3d {
            left: left.clone(),
            right: right.clone(),
            contact: None,
        });
    };

    let left_vertices = oriented_box_vertices(left.oriented_box())?;
    let right_vertices = oriented_box_vertices(right.oriented_box())?;
    let left_support = support_feature(left, left_vertices, seed.left_support_mask)?;
    let right_support = support_feature(right, right_vertices, seed.right_support_mask)?;
    let point = reduced_contact_point(left_support, right_support, seed)?;
    let left_offset = position_delta(left.body().position(), point)?;
    let right_offset = position_delta(right.body().position(), point)?;

    let left_velocity = contact_velocity(left, left_offset)?;
    let right_velocity = contact_velocity(right, right_offset)?;
    let relative_velocity = [
        checked_sub(i128::from(right_velocity[0]), i128::from(left_velocity[0]))?,
        checked_sub(i128::from(right_velocity[1]), i128::from(left_velocity[1]))?,
        checked_sub(i128::from(right_velocity[2]), i128::from(left_velocity[2]))?,
    ];
    let normal_velocity = checked_dot(relative_velocity, seed.axis)?;
    let ambiguous_zero_depth_touch = seed.overlap_numerator == 0 && seed.minimum_axis_ties > 1;

    let mut next_left = left.clone();
    let mut next_right = right.clone();
    let normal_impulse_units = if !ambiguous_zero_depth_touch
        && normal_velocity < 0
        && (left.body().kind() == BodyKind::Dynamic || right.body().kind() == BodyKind::Dynamic)
    {
        let impulse = normal_impulse(
            ContactFrame3d {
                body: left,
                offset: left_offset,
            },
            ContactFrame3d {
                body: right,
                offset: right_offset,
            },
            seed,
            normal_velocity,
        )?;
        if impulse > 0 {
            let left_impulse = scale_axis(seed.axis, -i128::from(impulse))?;
            let right_impulse = scale_axis(seed.axis, i128::from(impulse))?;
            apply_body_impulse(&mut next_left, left_offset, left_impulse)?;
            apply_body_impulse(&mut next_right, right_offset, right_impulse)?;
        }
        impulse
    } else {
        0
    };

    Ok(RotatingBoxPairResponse3d {
        left: next_left,
        right: next_right,
        contact: Some(RotatingBoxContactResponse3d {
            point,
            seed,
            normal_impulse_units,
        }),
    })
}

fn validate_body(body: &RigidBox3d) -> Result<(), RotatingContactResponseError3d> {
    let restitution = body.body().material().restitution_milli();
    if restitution > MATERIAL_SCALE {
        return Err(RotatingContactResponseError3d::RestitutionOutOfRange(
            body.body().id(),
            restitution,
        ));
    }
    Ok(())
}

fn support_feature(
    body: &RigidBox3d,
    vertices: [Vec3i; 8],
    mask: u8,
) -> Result<SupportFeature3d, RotatingContactResponseError3d> {
    Ok(SupportFeature3d {
        vertices,
        mask,
        centroid: support_centroid(&vertices, mask)?,
        kind: body.body().kind(),
    })
}

fn support_centroid(
    vertices: &[Vec3i; 8],
    mask: u8,
) -> Result<Vec3i, RotatingContactResponseError3d> {
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
        return Err(RotatingContactResponseError3d::EmptySupportMask);
    }
    Ok(Vec3i::new(
        i32::try_from(div_round_nearest(sum[0], count)?)
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
        i32::try_from(div_round_nearest(sum[1], count)?)
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
        i32::try_from(div_round_nearest(sum[2], count)?)
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
    ))
}

fn reduced_contact_point(
    left: SupportFeature3d,
    right: SupportFeature3d,
    seed: ObbContactSeed3d,
) -> Result<Vec3i, RotatingContactResponseError3d> {
    if let Some(point) = axis_aligned_face_overlap_centroid(left, right, seed.axis)? {
        return Ok(point);
    }
    if left.kind == right.kind {
        return midpoint(left.centroid, right.centroid);
    }

    let left_spread = support_spread_squared(&left.vertices, left.mask, left.centroid)?;
    let right_spread = support_spread_squared(&right.vertices, right.mask, right.centroid)?;
    let anchor = match left_spread.cmp(&right_spread) {
        std::cmp::Ordering::Less => left.centroid,
        std::cmp::Ordering::Greater => right.centroid,
        std::cmp::Ordering::Equal => return midpoint(left.centroid, right.centroid),
    };
    project_to_support_midplane(
        anchor,
        left.centroid,
        right.centroid,
        seed.axis,
        seed.axis_length_squared,
    )
}

fn axis_aligned_face_overlap_centroid(
    left: SupportFeature3d,
    right: SupportFeature3d,
    axis: [i128; 3],
) -> Result<Option<Vec3i>, RotatingContactResponseError3d> {
    if left.mask.count_ones() != 4 || right.mask.count_ones() != 4 {
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
    if !support_is_axis_aligned_rectangle(&left.vertices, left.mask, tangent_axes)
        || !support_is_axis_aligned_rectangle(&right.vertices, right.mask, tangent_axes)
    {
        return Ok(None);
    }

    let mut coordinate = [0_i32; 3];
    coordinate[normal_axis] = midpoint_axis(
        left.centroid.component(normal_axis),
        right.centroid.component(normal_axis),
    )?;
    for tangent_axis in tangent_axes {
        let (left_minimum, left_maximum) =
            support_interval(&left.vertices, left.mask, tangent_axis)?;
        let (right_minimum, right_maximum) =
            support_interval(&right.vertices, right.mask, tangent_axis)?;
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
            let value = vertex.component(axis);
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
) -> Result<(i32, i32), RotatingContactResponseError3d> {
    let mut minimum = None;
    let mut maximum = None;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        let value = vertex.component(axis);
        minimum = Some(minimum.map_or(value, |current: i32| current.min(value)));
        maximum = Some(maximum.map_or(value, |current: i32| current.max(value)));
    }
    match (minimum, maximum) {
        (Some(minimum), Some(maximum)) => Ok((minimum, maximum)),
        _ => Err(RotatingContactResponseError3d::EmptySupportMask),
    }
}

fn support_spread_squared(
    vertices: &[Vec3i; 8],
    mask: u8,
    center: Vec3i,
) -> Result<u128, RotatingContactResponseError3d> {
    let mut total = 0_u128;
    let mut count = 0_u8;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = count
            .checked_add(1)
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
        for component in [
            i128::from(vertex.x) - i128::from(center.x),
            i128::from(vertex.y) - i128::from(center.y),
            i128::from(vertex.z) - i128::from(center.z),
        ] {
            let magnitude = component.unsigned_abs();
            total = total
                .checked_add(
                    magnitude
                        .checked_mul(magnitude)
                        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?,
                )
                .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
        }
    }
    if count == 0 {
        return Err(RotatingContactResponseError3d::EmptySupportMask);
    }
    Ok(total)
}

fn project_to_support_midplane(
    anchor: Vec3i,
    left_support: Vec3i,
    right_support: Vec3i,
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<Vec3i, RotatingContactResponseError3d> {
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?;
    if length_squared <= 0 {
        return Err(RotatingContactResponseError3d::ArithmeticOverflow);
    }
    let target_projection = div_round_nearest(
        checked_add(
            dot_position(left_support, axis)?,
            dot_position(right_support, axis)?,
        )?,
        2,
    )?;
    let projection_delta = checked_sub(target_projection, dot_position(anchor, axis)?)?;
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
        i32::try_from(coordinate[0])
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
        i32::try_from(coordinate[1])
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
        i32::try_from(coordinate[2])
            .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?,
    ))
}

fn midpoint(left: Vec3i, right: Vec3i) -> Result<Vec3i, RotatingContactResponseError3d> {
    Ok(Vec3i::new(
        midpoint_axis(left.x, right.x)?,
        midpoint_axis(left.y, right.y)?,
        midpoint_axis(left.z, right.z)?,
    ))
}

fn midpoint_axis(left: i32, right: i32) -> Result<i32, RotatingContactResponseError3d> {
    let sum = checked_add(i128::from(left), i128::from(right))?;
    i32::try_from(div_round_nearest(sum, 2)?)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

fn position_delta(
    center: Vec3i,
    point: Vec3i,
) -> Result<[i64; 3], RotatingContactResponseError3d> {
    Ok([
        i64::from(point.x)
            .checked_sub(i64::from(center.x))
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?,
        i64::from(point.y)
            .checked_sub(i64::from(center.y))
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?,
        i64::from(point.z)
            .checked_sub(i64::from(center.z))
            .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?,
    ])
}

fn dot_position(
    position: Vec3i,
    axis: [i128; 3],
) -> Result<i128, RotatingContactResponseError3d> {
    checked_dot(
        [
            i128::from(position.x),
            i128::from(position.y),
            i128::from(position.z),
        ],
        axis,
    )
}

fn contact_velocity(
    body: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i64; 3], RotatingContactResponseError3d> {
    let omega = body.angular().angular_velocity;
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
        add_linear_rotation(body.body().velocity().x, rotation_x, scale)?,
        add_linear_rotation(body.body().velocity().y, rotation_y, scale)?,
        add_linear_rotation(body.body().velocity().z, rotation_z, scale)?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
    scale: i128,
) -> Result<i64, RotatingContactResponseError3d> {
    let rotational = i64::try_from(div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)
}

fn normal_impulse(
    left: ContactFrame3d<'_>,
    right: ContactFrame3d<'_>,
    seed: ObbContactSeed3d,
    normal_velocity: i128,
) -> Result<i64, RotatingContactResponseError3d> {
    let effective_inverse_mass = checked_add(
        body_effective_inverse_mass_scaled(left, seed)?,
        body_effective_inverse_mass_scaled(right, seed)?,
    )?;
    if effective_inverse_mass <= 0 {
        return Ok(0);
    }

    let restitution = combined_restitution(left.body.body().material(), right.body.body().material());
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
    let bounce_scale = checked_add(i128::from(MATERIAL_SCALE), i128::from(restitution))?;
    let numerator = checked_mul(checked_mul(closing_speed, bounce_scale)?, RESPONSE_SCALE)?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    i64::try_from(div_round_nearest(numerator, denominator)?)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

fn combined_restitution(left: Material, right: Material) -> u16 {
    left.restitution_milli().max(right.restitution_milli())
}

fn body_effective_inverse_mass_scaled(
    frame: ContactFrame3d<'_>,
    seed: ObbContactSeed3d,
) -> Result<i128, RotatingContactResponseError3d> {
    if frame.body.body().kind() == BodyKind::Fixed {
        return Ok(0);
    }

    let length_squared = i128::try_from(seed.axis_length_squared)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?;
    let translational = checked_mul(RESPONSE_SCALE, length_squared)?
        / i128::from(frame.body.body().mass_units());
    let angular_impulse = cross_i64_i128(frame.offset, seed.axis)?;
    let local = rotate_inverse(frame.body.angular().orientation, angular_impulse)?;
    let inertia = box_inertia(frame.body.body())?;
    let mut rotational = 0_i128;
    for (index, component) in local.into_iter().enumerate() {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[index], inertia.denominator)?;
        rotational = checked_add(
            rotational,
            checked_mul(checked_mul(component, component)?, inverse_inertia)?,
        )?;
    }
    checked_add(translational, rotational)
}

fn apply_body_impulse(
    body: &mut RigidBox3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), RotatingContactResponseError3d> {
    if body.body().kind() == BodyKind::Fixed {
        return Ok(());
    }

    body.body.velocity = Vec3i::new(
        add_linear_impulse_axis(body.body.velocity.x, impulse[0], body.body.mass_units)?,
        add_linear_impulse_axis(body.body.velocity.y, impulse[1], body.body.mass_units)?,
        add_linear_impulse_axis(body.body.velocity.z, impulse[2], body.body.mass_units)?,
    );

    let angular_impulse = cross_i64_i128(contact_offset, impulse)?;
    let local_impulse = rotate_inverse(body.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&body.body)?;
    let mut local_delta = [0_i128; 3];
    for (index, (target, component)) in local_delta.iter_mut().zip(local_impulse).enumerate() {
        let inverse_inertia =
            inverse_inertia_scaled(inertia.principal_numerators[index], inertia.denominator)?;
        let numerator = checked_mul(
            checked_mul(component, inverse_inertia)?,
            i128::from(ANGULAR_VELOCITY_SCALE),
        )?;
        *target = div_round_nearest(numerator, RESPONSE_SCALE)?;
    }
    let world_delta = rotate_forward(body.angular.orientation, local_delta)?;
    body.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(body.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(body.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(body.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn add_linear_impulse_axis(
    current: i32,
    impulse: i128,
    mass_units: u32,
) -> Result<i32, RotatingContactResponseError3d> {
    let delta = div_round_nearest(impulse, i128::from(mass_units))?;
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

fn add_angular_axis(
    current: i32,
    delta: i128,
) -> Result<i32, RotatingContactResponseError3d> {
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)
}

fn inverse_inertia_scaled(
    principal_numerator: u128,
    denominator: u32,
) -> Result<i128, RotatingContactResponseError3d> {
    if principal_numerator == 0 {
        return Err(RotatingContactResponseError3d::ArithmeticOverflow);
    }
    let principal = i128::try_from(principal_numerator)
        .map_err(|_| RotatingContactResponseError3d::ArithmeticOverflow)?;
    Ok(checked_mul(RESPONSE_SCALE, i128::from(denominator))? / principal)
}

fn cross_i64_i128(
    left: [i64; 3],
    right: [i128; 3],
) -> Result<[i128; 3], RotatingContactResponseError3d> {
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
) -> Result<i128, RotatingContactResponseError3d> {
    checked_sub(checked_mul(left_a, right_a)?, checked_mul(left_b, right_b)?)
}

fn checked_dot(
    left: [i128; 3],
    right: [i128; 3],
) -> Result<i128, RotatingContactResponseError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn scale_axis(
    axis: [i128; 3],
    scale: i128,
) -> Result<[i128; 3], RotatingContactResponseError3d> {
    Ok([
        checked_mul(axis[0], scale)?,
        checked_mul(axis[1], scale)?,
        checked_mul(axis[2], scale)?,
    ])
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], RotatingContactResponseError3d> {
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
) -> Result<[i128; 3], RotatingContactResponseError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(
    orientation: Orientation3d,
) -> Result<[[i128; 3]; 3], RotatingContactResponseError3d> {
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
) -> Result<[i128; 3], RotatingContactResponseError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let sum = checked_add(
            checked_add(
                checked_mul(row[0], vector[0])?,
                checked_mul(row[1], vector[1])?,
            )?,
            checked_mul(row[2], vector[2])?,
        )?;
        *target = div_round_nearest(sum, scale)?;
    }
    Ok(output)
}

fn scaled_twice(
    value: i128,
    scale: i128,
) -> Result<i128, RotatingContactResponseError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn checked_mul(
    left: i128,
    right: i128,
) -> Result<i128, RotatingContactResponseError3d> {
    left.checked_mul(right)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)
}

fn checked_add(
    left: i128,
    right: i128,
) -> Result<i128, RotatingContactResponseError3d> {
    left.checked_add(right)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)
}

fn checked_sub(
    left: i128,
    right: i128,
) -> Result<i128, RotatingContactResponseError3d> {
    left.checked_sub(right)
        .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, RotatingContactResponseError3d> {
    if denominator <= 0 {
        return Err(RotatingContactResponseError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RotatingContactResponseError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

#[cfg(test)]
mod tests {
    use crate::{AngularState3d, AngularVelocity3d, Material, Orientation3d, RigidBody};

    use super::*;

    fn dynamic(id: u64, center: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), center, velocity, Vec3i::new(10, 10, 10)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, center: Vec3i, half_extents: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), center, half_extents),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    #[test]
    fn separated_pair_is_idempotently_unchanged() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(20, 0, 0));
        let right = dynamic(2, Vec3i::new(30, 0, 0), Vec3i::ZERO);
        let response = resolve_rotating_box_pair(&left, &right).expect("valid separated pair");

        assert_eq!(response.contact, None);
        assert_eq!(response.left, left);
        assert_eq!(response.right, right);
    }

    #[test]
    fn centered_face_impact_exchanges_velocity_without_spin() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0));
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO);
        let response = resolve_rotating_box_pair(&left, &right).expect("valid centered impact");
        let contact = response.contact.expect("overlapping pair should contact");

        assert_eq!(contact.seed.axis, [1, 0, 0]);
        assert!(contact.normal_impulse_units > 0);
        assert_eq!(response.left.body().velocity().x, 30);
        assert_eq!(response.right.body().velocity().x, 30);
        assert_eq!(
            response.left.angular().angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(
            response.right.angular().angular_velocity,
            AngularVelocity3d::default()
        );
    }

    #[test]
    fn off_center_face_impact_generates_spin_for_both_boxes() {
        let left = dynamic(1, Vec3i::new(0, 6, 0), Vec3i::new(90, 0, 0));
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::ZERO);
        let response = resolve_rotating_box_pair(&left, &right).expect("valid off-center impact");
        let contact = response.contact.expect("overlapping pair should contact");

        assert_eq!(contact.seed.axis, [1, 0, 0]);
        assert!(contact.normal_impulse_units > 0);
        assert_ne!(response.left.angular().angular_velocity.z, 0);
        assert_ne!(response.right.angular().angular_velocity.z, 0);
        assert!(response.left.body().velocity().x < left.body().velocity().x);
        assert!(response.right.body().velocity().x > right.body().velocity().x);
    }

    #[test]
    fn fixed_wall_receives_no_velocity_or_spin() {
        let mut left = dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0));
        left.body.material = Material::new(MATERIAL_SCALE);
        let right = fixed(2, Vec3i::new(19, 0, 0), Vec3i::new(10, 10, 10));
        let response = resolve_rotating_box_pair(&left, &right).expect("valid wall impact");

        assert!(
            response
                .contact
                .expect("wall should contact")
                .normal_impulse_units
                > 0
        );
        assert_eq!(response.left.body().velocity().x, -60);
        assert_eq!(response.right, right);
    }

    #[test]
    fn wide_fixed_support_does_not_invent_torque_from_its_face_center() {
        let floor = fixed(1, Vec3i::ZERO, Vec3i::new(100, 10, 100));
        let block = dynamic(2, Vec3i::new(60, 19, 40), Vec3i::new(0, -60, 0));
        let response = resolve_rotating_box_pair(&floor, &block).expect("valid floor contact");
        let contact = response.contact.expect("floor should contact block");

        assert_eq!(contact.seed.axis, [0, 1, 0]);
        assert_eq!(contact.point.x, block.body().position().x);
        assert_eq!(contact.point.z, block.body().position().z);
        assert!(contact.normal_impulse_units > 0);
        assert_eq!(
            response.right.angular().angular_velocity,
            AngularVelocity3d::default()
        );
        assert_eq!(response.left, floor);
    }

    #[test]
    fn exact_diagonal_edge_touch_is_observation_only() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::ZERO);
        let right = dynamic(2, Vec3i::new(0, 20, 20), Vec3i::new(0, -60, 0));
        let response = resolve_rotating_box_pair(&left, &right).expect("valid exact edge touch");
        let contact = response.contact.expect("edge touch remains contact evidence");

        assert_eq!(contact.seed.overlap_numerator, 0);
        assert!(contact.seed.minimum_axis_ties > 1);
        assert_eq!(contact.normal_impulse_units, 0);
        assert_eq!(response.left, left);
        assert_eq!(response.right, right);
    }

    #[test]
    fn separating_overlap_reports_contact_without_second_impulse() {
        let left = dynamic(1, Vec3i::ZERO, Vec3i::new(-20, 0, 0));
        let right = dynamic(2, Vec3i::new(19, 0, 0), Vec3i::new(20, 0, 0));
        let response = resolve_rotating_box_pair(&left, &right).expect("valid separating overlap");
        let contact = response.contact.expect("overlap remains contact evidence");

        assert_eq!(contact.normal_impulse_units, 0);
        assert_eq!(response.left, left);
        assert_eq!(response.right, right);
    }
}
