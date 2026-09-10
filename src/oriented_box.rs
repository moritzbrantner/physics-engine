use std::{cmp::Ordering, error::Error, fmt};

use crate::{AngularError3d, ORIENTATION_SCALE, Orientation3d, Vec3i};

const MAX_SAT_AXES: usize = 15;
const CORNER_SIGNS: [[i64; 3]; 8] = [
    [-1, -1, -1],
    [1, -1, -1],
    [-1, 1, -1],
    [1, 1, -1],
    [-1, -1, 1],
    [1, -1, 1],
    [-1, 1, 1],
    [1, 1, 1],
];

/// Oriented box geometry independent of world/ECS ownership.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OrientedBox3d {
    pub center: Vec3i,
    pub half_extents: Vec3i,
    pub orientation: Orientation3d,
}

impl OrientedBox3d {
    #[must_use]
    pub const fn new(center: Vec3i, half_extents: Vec3i, orientation: Orientation3d) -> Self {
        Self {
            center,
            half_extents,
            orientation,
        }
    }
}

/// Stable SAT feature that supplied the minimum-penetration axis.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObbAxisFeature3d {
    LeftFace(u8),
    RightFace(u8),
    EdgeEdge { left_axis: u8, right_axis: u8 },
}

/// Exact bounded contact evidence produced by oriented-box SAT.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ObbContactSeed3d {
    /// Primitive SAT axis oriented from the left box toward the right box.
    pub axis: [i128; 3],
    /// Projection overlap along [`Self::axis`]. Divide by the axis length for world-space depth.
    pub overlap_numerator: u128,
    pub axis_length_squared: u128,
    /// Earliest stable feature that produced the minimum-penetration axis.
    pub feature: ObbAxisFeature3d,
    /// Bitmasks over the stable eight-vertex order returned by [`oriented_box_vertices`].
    pub left_support_mask: u8,
    pub right_support_mask: u8,
    /// Number of unique non-degenerate SAT axes that were evaluated.
    pub tested_axes: u8,
    /// Number of unique axes tied for the exact minimum normalized overlap.
    pub minimum_axis_ties: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OrientedBoxError3d {
    InvalidHalfExtents,
    DegenerateGeometry,
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for OrientedBoxError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidHalfExtents => write!(
                formatter,
                "oriented-box geometry requires strictly positive half extents"
            ),
            Self::DegenerateGeometry => write!(
                formatter,
                "oriented-box SAT geometry collapsed after deterministic quantization"
            ),
            Self::Angular(error) => write!(formatter, "oriented-box orientation failed: {error}"),
            Self::ArithmeticOverflow => write!(formatter, "oriented-box arithmetic overflowed"),
        }
    }
}

impl Error for OrientedBoxError3d {}

impl From<AngularError3d> for OrientedBoxError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AxisSeed {
    axis: [i128; 3],
    feature: ObbAxisFeature3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct EvaluatedAxis {
    seed: AxisSeed,
    overlap: u128,
    length_squared: u128,
}

/// Returns the eight quantized world-space vertices of an oriented box in stable local-corner order.
///
/// # Errors
///
/// Returns [`OrientedBoxError3d`] for invalid half extents, invalid orientation, or checked arithmetic
/// that cannot be represented by the engine's integer coordinate contract.
pub fn oriented_box_vertices(box_shape: OrientedBox3d) -> Result<[Vec3i; 8], OrientedBoxError3d> {
    let offsets = oriented_box_offsets(box_shape.half_extents, box_shape.orientation)?;
    let mut vertices = [Vec3i::ZERO; 8];
    for (vertex, offset) in vertices.iter_mut().zip(offsets) {
        let x = i64::from(box_shape.center.x)
            .checked_add(offset[0])
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
        let y = i64::from(box_shape.center.y)
            .checked_add(offset[1])
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
        let z = i64::from(box_shape.center.z)
            .checked_add(offset[2])
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
        *vertex = Vec3i::new(
            i32::try_from(x).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            i32::try_from(y).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            i32::try_from(z).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
        );
    }
    Ok(vertices)
}

/// Evaluates exact integer SAT semantics for the quantized vertices of two oriented boxes.
///
/// The query tests each box's three face normals and all nine edge-cross-edge axes, removes exact
/// duplicate axes in stable feature order, and treats touching as contact. Support masks identify the
/// tied extreme vertices on the minimum-penetration axis. This is contact geometry only: it does not
/// integrate time, clip a full manifold, apply impulses, or claim analytic rotational CCD.
///
/// # Errors
///
/// Returns [`OrientedBoxError3d`] for invalid or degenerate oriented geometry or checked arithmetic
/// overflow.
pub fn obb_contact_seed(
    left: OrientedBox3d,
    right: OrientedBox3d,
) -> Result<Option<ObbContactSeed3d>, OrientedBoxError3d> {
    let left_vertices = oriented_box_vertices(left)?;
    let right_vertices = oriented_box_vertices(right)?;
    let left_edges = box_edges(&left_vertices)?;
    let right_edges = box_edges(&right_vertices)?;
    let left_faces = face_axes(left_edges)?;
    let right_faces = face_axes(right_edges)?;

    let mut candidate_axes = Vec::with_capacity(MAX_SAT_AXES);
    for (index, direction) in left_faces.into_iter().enumerate() {
        push_required_axis(
            &mut candidate_axes,
            direction,
            ObbAxisFeature3d::LeftFace(
                u8::try_from(index).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            ),
        )?;
    }
    for (index, direction) in right_faces.into_iter().enumerate() {
        push_required_axis(
            &mut candidate_axes,
            direction,
            ObbAxisFeature3d::RightFace(
                u8::try_from(index).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            ),
        )?;
    }
    for (left_index, left_edge) in left_edges.into_iter().enumerate() {
        for (right_index, right_edge) in right_edges.into_iter().enumerate() {
            push_optional_axis(
                &mut candidate_axes,
                cross(left_edge, right_edge)?,
                ObbAxisFeature3d::EdgeEdge {
                    left_axis: u8::try_from(left_index)
                        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
                    right_axis: u8::try_from(right_index)
                        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
                },
            )?;
        }
    }

    if candidate_axes.len() < 3 || candidate_axes.len() > MAX_SAT_AXES {
        return Err(OrientedBoxError3d::DegenerateGeometry);
    }

    let mut best: Option<EvaluatedAxis> = None;
    let mut minimum_axis_ties = 0_u8;
    for seed in candidate_axes.iter().copied() {
        let left_projection = projection(&left_vertices, seed.axis)?;
        let right_projection = projection(&right_vertices, seed.axis)?;
        if left_projection.1 < right_projection.0 || right_projection.1 < left_projection.0 {
            return Ok(None);
        }

        let overlap_start = left_projection.0.max(right_projection.0);
        let overlap_end = left_projection.1.min(right_projection.1);
        let overlap = u128::try_from(
            overlap_end
                .checked_sub(overlap_start)
                .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        )
        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
        let length_squared = squared_length(seed.axis)?;
        let evaluated = EvaluatedAxis {
            seed,
            overlap,
            length_squared,
        };

        match best {
            None => {
                best = Some(evaluated);
                minimum_axis_ties = 1;
            }
            Some(current) => match compare_normalized_overlap(evaluated, current)? {
                Ordering::Less => {
                    best = Some(evaluated);
                    minimum_axis_ties = 1;
                }
                Ordering::Equal => {
                    minimum_axis_ties = minimum_axis_ties
                        .checked_add(1)
                        .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
                }
                Ordering::Greater => {}
            },
        }
    }

    let best = best.ok_or(OrientedBoxError3d::DegenerateGeometry)?;
    let contact_axis = orient_toward_right(left.center, right.center, best.seed.axis)?;
    let tested_axes =
        u8::try_from(candidate_axes.len()).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
    Ok(Some(ObbContactSeed3d {
        axis: contact_axis,
        overlap_numerator: best.overlap,
        axis_length_squared: best.length_squared,
        feature: best.seed.feature,
        left_support_mask: support_mask(&left_vertices, contact_axis, true)?,
        right_support_mask: support_mask(&right_vertices, contact_axis, false)?,
        tested_axes,
        minimum_axis_ties,
    }))
}

fn oriented_box_offsets(
    half_extents: Vec3i,
    orientation: Orientation3d,
) -> Result<[[i64; 3]; 8], OrientedBoxError3d> {
    if half_extents.x <= 0 || half_extents.y <= 0 || half_extents.z <= 0 {
        return Err(OrientedBoxError3d::InvalidHalfExtents);
    }
    let matrix = rotation_matrix(orientation.normalized()?)?;
    let basis_vectors = [
        rotate_with_matrix(matrix, [i64::from(half_extents.x), 0, 0])?,
        rotate_with_matrix(matrix, [0, i64::from(half_extents.y), 0])?,
        rotate_with_matrix(matrix, [0, 0, i64::from(half_extents.z)])?,
    ];
    let mut offsets = [[0_i64; 3]; 8];
    for (offset, signs) in offsets.iter_mut().zip(CORNER_SIGNS) {
        *offset = combine_basis_vectors(basis_vectors, signs)?;
    }
    Ok(offsets)
}

fn combine_basis_vectors(
    basis_vectors: [[i64; 3]; 3],
    signs: [i64; 3],
) -> Result<[i64; 3], OrientedBoxError3d> {
    let mut output = [0_i64; 3];
    for (basis_vector, sign) in basis_vectors.into_iter().zip(signs) {
        for (target, component) in output.iter_mut().zip(basis_vector) {
            let signed = component
                .checked_mul(sign)
                .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
            *target = target
                .checked_add(signed)
                .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
        }
    }
    Ok(output)
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], OrientedBoxError3d> {
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

fn scaled_twice(value: i128, scale: i128) -> Result<i128, OrientedBoxError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], OrientedBoxError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i64; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let first = checked_mul(row[0], i128::from(vector[0]))?;
        let second = checked_mul(row[1], i128::from(vector[1]))?;
        let third = checked_mul(row[2], i128::from(vector[2]))?;
        let sum = checked_add(checked_add(first, second)?, third)?;
        *target = i64::try_from(div_round_nearest(sum, scale)?)
            .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
    }
    Ok(output)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, OrientedBoxError3d> {
    left.checked_mul(right)
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, OrientedBoxError3d> {
    left.checked_add(right)
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, OrientedBoxError3d> {
    left.checked_sub(right)
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, OrientedBoxError3d> {
    if denominator <= 0 {
        return Err(OrientedBoxError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn box_edges(vertices: &[Vec3i; 8]) -> Result<[[i128; 3]; 3], OrientedBoxError3d> {
    Ok([
        position_delta(vertices[0], vertices[1])?,
        position_delta(vertices[0], vertices[2])?,
        position_delta(vertices[0], vertices[4])?,
    ])
}

fn face_axes(edges: [[i128; 3]; 3]) -> Result<[[i128; 3]; 3], OrientedBoxError3d> {
    Ok([
        cross(edges[1], edges[2])?,
        cross(edges[2], edges[0])?,
        cross(edges[0], edges[1])?,
    ])
}

fn position_delta(left: Vec3i, right: Vec3i) -> Result<[i128; 3], OrientedBoxError3d> {
    Ok([
        i128::from(right.x)
            .checked_sub(i128::from(left.x))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        i128::from(right.y)
            .checked_sub(i128::from(left.y))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        i128::from(right.z)
            .checked_sub(i128::from(left.z))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
    ])
}

fn cross(left: [i128; 3], right: [i128; 3]) -> Result<[i128; 3], OrientedBoxError3d> {
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
) -> Result<i128, OrientedBoxError3d> {
    left_a
        .checked_mul(right_a)
        .and_then(|first| {
            left_b
                .checked_mul(right_b)
                .and_then(|second| first.checked_sub(second))
        })
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
}

fn push_required_axis(
    candidate_axes: &mut Vec<AxisSeed>,
    direction: [i128; 3],
    feature: ObbAxisFeature3d,
) -> Result<(), OrientedBoxError3d> {
    let direction = primitive_axis(direction)?.ok_or(OrientedBoxError3d::DegenerateGeometry)?;
    push_unique_axis(candidate_axes, direction, feature)
}

fn push_optional_axis(
    candidate_axes: &mut Vec<AxisSeed>,
    direction: [i128; 3],
    feature: ObbAxisFeature3d,
) -> Result<(), OrientedBoxError3d> {
    if let Some(direction) = primitive_axis(direction)? {
        push_unique_axis(candidate_axes, direction, feature)?;
    }
    Ok(())
}

fn push_unique_axis(
    candidate_axes: &mut Vec<AxisSeed>,
    direction: [i128; 3],
    feature: ObbAxisFeature3d,
) -> Result<(), OrientedBoxError3d> {
    if candidate_axes
        .iter()
        .any(|existing| existing.axis == direction)
    {
        return Ok(());
    }
    if candidate_axes.len() >= MAX_SAT_AXES {
        return Err(OrientedBoxError3d::ArithmeticOverflow);
    }
    candidate_axes.push(AxisSeed {
        axis: direction,
        feature,
    });
    Ok(())
}

fn primitive_axis(mut axis: [i128; 3]) -> Result<Option<[i128; 3]>, OrientedBoxError3d> {
    let divisor = axis
        .into_iter()
        .map(i128::unsigned_abs)
        .filter(|component| *component != 0)
        .reduce(greatest_common_divisor)
        .unwrap_or(0);
    if divisor == 0 {
        return Ok(None);
    }
    let divisor = i128::try_from(divisor).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
    for component in &mut axis {
        *component /= divisor;
    }
    if axis
        .iter()
        .find(|component| **component != 0)
        .is_some_and(|component| *component < 0)
    {
        axis = negate_axis(axis)?;
    }
    Ok(Some(axis))
}

fn greatest_common_divisor(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn negate_axis(axis: [i128; 3]) -> Result<[i128; 3], OrientedBoxError3d> {
    Ok([
        axis[0]
            .checked_neg()
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        axis[1]
            .checked_neg()
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        axis[2]
            .checked_neg()
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
    ])
}

fn projection(
    vertices: &[Vec3i; 8],
    axis: [i128; 3],
) -> Result<(i128, i128), OrientedBoxError3d> {
    let first = dot_position(vertices[0], axis)?;
    vertices[1..]
        .iter()
        .try_fold((first, first), |range, vertex| {
            let value = dot_position(*vertex, axis)?;
            Ok((range.0.min(value), range.1.max(value)))
        })
}

fn dot_position(position: Vec3i, axis: [i128; 3]) -> Result<i128, OrientedBoxError3d> {
    checked_dot(
        [
            i128::from(position.x),
            i128::from(position.y),
            i128::from(position.z),
        ],
        axis,
    )
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, OrientedBoxError3d> {
    let first = left[0]
        .checked_mul(right[0])
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
    let second = left[1]
        .checked_mul(right[1])
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
    let third = left[2]
        .checked_mul(right[2])
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
    first
        .checked_add(second)
        .and_then(|sum| sum.checked_add(third))
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
}

fn squared_length(axis: [i128; 3]) -> Result<u128, OrientedBoxError3d> {
    axis.into_iter().try_fold(0_u128, |sum, component| {
        let component = component.unsigned_abs();
        sum.checked_add(
            component
                .checked_mul(component)
                .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        )
        .ok_or(OrientedBoxError3d::ArithmeticOverflow)
    })
}

fn compare_normalized_overlap(
    left: EvaluatedAxis,
    right: EvaluatedAxis,
) -> Result<Ordering, OrientedBoxError3d> {
    compare_squared_ratios(
        left.overlap,
        left.length_squared,
        right.overlap,
        right.length_squared,
    )
}

fn compare_squared_ratios(
    left_numerator: u128,
    left_denominator: u128,
    right_numerator: u128,
    right_denominator: u128,
) -> Result<Ordering, OrientedBoxError3d> {
    if left_denominator == 0 || right_denominator == 0 {
        return Err(OrientedBoxError3d::DegenerateGeometry);
    }

    let mut left = [left_numerator, left_numerator, right_denominator];
    let mut right = [right_numerator, right_numerator, left_denominator];
    for left_factor in &mut left {
        for right_factor in &mut right {
            let divisor = greatest_common_divisor(*left_factor, *right_factor);
            if divisor > 1 {
                *left_factor /= divisor;
                *right_factor /= divisor;
            }
        }
    }
    Ok(wide_product(left)?.cmp(&wide_product(right)?))
}

fn wide_product(factors: [u128; 3]) -> Result<[u64; 6], OrientedBoxError3d> {
    let mut product = [0_u64; 6];
    product[0] = 1;
    for factor in factors {
        product = multiply_wide(product, factor)?;
    }
    product.reverse();
    Ok(product)
}

fn multiply_wide(product: [u64; 6], factor: u128) -> Result<[u64; 6], OrientedBoxError3d> {
    let factor = [
        u64::try_from(factor & u128::from(u64::MAX))
            .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
        u64::try_from(factor >> 64).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
    ];
    let mut result = [0_u64; 6];
    for (left_index, left) in product.into_iter().enumerate() {
        let mut carry = 0_u128;
        for (right_index, right) in factor.into_iter().enumerate() {
            let index = left_index + right_index;
            if index >= result.len() {
                if (left != 0 && right != 0) || carry != 0 {
                    return Err(OrientedBoxError3d::ArithmeticOverflow);
                }
                continue;
            }
            let value = u128::from(result[index]) + u128::from(left) * u128::from(right) + carry;
            result[index] = u64::try_from(value & u128::from(u64::MAX))
                .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
            carry = value >> 64;
        }

        let mut index = left_index + factor.len();
        while carry != 0 && index < result.len() {
            let value = u128::from(result[index]) + carry;
            result[index] = u64::try_from(value & u128::from(u64::MAX))
                .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
            carry = value >> 64;
            index += 1;
        }
        if carry != 0 {
            return Err(OrientedBoxError3d::ArithmeticOverflow);
        }
    }
    Ok(result)
}

fn orient_toward_right(
    left: Vec3i,
    right: Vec3i,
    axis: [i128; 3],
) -> Result<[i128; 3], OrientedBoxError3d> {
    let delta = [
        i128::from(right.x)
            .checked_sub(i128::from(left.x))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        i128::from(right.y)
            .checked_sub(i128::from(left.y))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        i128::from(right.z)
            .checked_sub(i128::from(left.z))
            .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
    ];
    if checked_dot(delta, axis)? < 0 {
        negate_axis(axis)
    } else {
        Ok(axis)
    }
}

fn support_mask(
    vertices: &[Vec3i; 8],
    axis: [i128; 3],
    maximum: bool,
) -> Result<u8, OrientedBoxError3d> {
    let projection = projection(vertices, axis)?;
    let target = if maximum { projection.1 } else { projection.0 };
    let mut mask = 0_u8;
    for (index, vertex) in vertices.iter().enumerate() {
        if dot_position(*vertex, axis)? == target {
            mask |= 1_u8 << index;
        }
    }
    if mask == 0 {
        return Err(OrientedBoxError3d::DegenerateGeometry);
    }
    Ok(mask)
}

#[cfg(test)]
mod tests {
    use std::cmp::Ordering;

    use super::{
        ObbAxisFeature3d, OrientedBox3d, OrientedBoxError3d, compare_squared_ratios,
        obb_contact_seed, oriented_box_vertices, wide_product,
    };
    use crate::{Orientation3d, Vec3i};

    fn cube(center: Vec3i) -> OrientedBox3d {
        OrientedBox3d::new(center, Vec3i::new(1, 1, 1), Orientation3d::IDENTITY)
    }

    #[test]
    fn identity_vertices_preserve_stable_corner_order() {
        let vertices = oriented_box_vertices(cube(Vec3i::ZERO)).expect("valid cube");
        assert_eq!(vertices[0], Vec3i::new(-1, -1, -1));
        assert_eq!(vertices[1], Vec3i::new(1, -1, -1));
        assert_eq!(vertices[7], Vec3i::new(1, 1, 1));
    }

    #[test]
    fn aligned_face_touch_has_stable_four_vertex_supports() {
        let contact = obb_contact_seed(cube(Vec3i::ZERO), cube(Vec3i::new(2, 0, 0)))
            .expect("valid OBB pair")
            .expect("touching cubes should contact");

        assert_eq!(contact.axis, [1, 0, 0]);
        assert_eq!(contact.overlap_numerator, 0);
        assert_eq!(contact.axis_length_squared, 1);
        assert_eq!(contact.feature, ObbAxisFeature3d::LeftFace(0));
        assert_eq!(contact.left_support_mask, 0b1010_1010);
        assert_eq!(contact.right_support_mask, 0b0101_0101);
        assert_eq!(contact.tested_axes, 3);
        assert_eq!(contact.minimum_axis_ties, 1);
    }

    #[test]
    fn aligned_edge_and_corner_touch_preserve_minimum_axis_ties() {
        let edge = obb_contact_seed(cube(Vec3i::ZERO), cube(Vec3i::new(2, 2, 0)))
            .expect("valid OBB pair")
            .expect("edge-touching cubes should contact");
        assert_eq!(edge.minimum_axis_ties, 2);

        let corner = obb_contact_seed(cube(Vec3i::ZERO), cube(Vec3i::new(2, 2, 2)))
            .expect("valid OBB pair")
            .expect("corner-touching cubes should contact");
        assert_eq!(corner.minimum_axis_ties, 3);
    }

    #[test]
    fn separated_boxes_return_no_contact() {
        assert_eq!(
            obb_contact_seed(cube(Vec3i::ZERO), cube(Vec3i::new(3, 0, 0)))
                .expect("valid OBB pair"),
            None
        );
    }

    #[test]
    fn normalized_overlap_comparison_keeps_exact_products_wider_than_u128() {
        let numerator = 373_263_717_518_u128;
        let other_denominator = 2_687_465_820_327_019_u128;

        assert_eq!(
            wide_product([numerator, numerator, other_denominator]),
            Ok([
                0,
                0,
                0,
                1,
                1_851_327_578_378_911_237,
                16_547_754_579_345_353_708,
            ])
        );
        assert_eq!(
            compare_squared_ratios(numerator, 1, 1, other_denominator),
            Ok(Ordering::Greater)
        );
    }

    #[test]
    fn differently_rotated_boxes_use_non_aabb_sat_axes() {
        let left_orientation = Orientation3d::new(0, 0, 410_903_207, 992_008_094)
            .normalized()
            .expect("valid Z-axis orientation");
        let right_orientation = Orientation3d::new(410_903_207, 0, 0, 992_008_094)
            .normalized()
            .expect("valid X-axis orientation");
        let left = OrientedBox3d::new(Vec3i::ZERO, Vec3i::new(4, 1, 1), left_orientation);
        let right = OrientedBox3d::new(Vec3i::ZERO, Vec3i::new(4, 1, 1), right_orientation);

        let contact = obb_contact_seed(left, right)
            .expect("valid rotated OBB pair")
            .expect("co-centered boxes should overlap");
        assert!(contact.tested_axes > 3);
    }

    #[test]
    fn swapping_boxes_reorients_contact_axis() {
        let left = cube(Vec3i::ZERO);
        let right = cube(Vec3i::new(2, 0, 0));
        let forward = obb_contact_seed(left, right)
            .expect("valid OBB pair")
            .expect("touching cubes should contact");
        let reverse = obb_contact_seed(right, left)
            .expect("valid reversed OBB pair")
            .expect("touching cubes should contact");

        assert_eq!(forward.axis, reverse.axis.map(|component| -component));
        assert_eq!(forward.overlap_numerator, reverse.overlap_numerator);
        assert_eq!(forward.axis_length_squared, reverse.axis_length_squared);
    }

    #[test]
    fn invalid_dimensions_and_zero_orientation_fail_closed() {
        let invalid = OrientedBox3d::new(
            Vec3i::ZERO,
            Vec3i::new(1, 0, 1),
            Orientation3d::IDENTITY,
        );
        assert_eq!(
            obb_contact_seed(invalid, cube(Vec3i::ZERO)),
            Err(OrientedBoxError3d::InvalidHalfExtents)
        );

        let invalid_orientation = OrientedBox3d::new(
            Vec3i::ZERO,
            Vec3i::new(1, 1, 1),
            Orientation3d::new(0, 0, 0, 0),
        );
        assert!(matches!(
            obb_contact_seed(invalid_orientation, cube(Vec3i::ZERO)),
            Err(OrientedBoxError3d::Angular(_))
        ));
    }
}
