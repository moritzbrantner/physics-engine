use std::{error::Error, fmt};

use crate::{AngularError3d, OrientedBox3d};

/// Conservative axis-aligned bounds for one translating, arbitrarily rotating box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotationalSweepBounds3d {
    pub minimum: [i64; 3],
    pub maximum: [i64; 3],
}

impl RotationalSweepBounds3d {
    #[must_use]
    pub const fn contains(self, point: [i64; 3]) -> bool {
        point[0] >= self.minimum[0]
            && point[0] <= self.maximum[0]
            && point[1] >= self.minimum[1]
            && point[1] <= self.maximum[1]
            && point[2] >= self.minimum[2]
            && point[2] <= self.maximum[2]
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotationalSweepError3d {
    ShapeMismatch,
    InvalidHalfExtents,
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for RotationalSweepError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ShapeMismatch => write!(
                formatter,
                "rotational sweep endpoints must describe the same box half extents"
            ),
            Self::InvalidHalfExtents => write!(
                formatter,
                "rotational sweep requires strictly positive box half extents"
            ),
            Self::Angular(error) => {
                write!(formatter, "rotational sweep orientation failed: {error}")
            }
            Self::ArithmeticOverflow => write!(formatter, "rotational sweep bounds overflowed"),
        }
    }
}

impl Error for RotationalSweepError3d {}

impl From<AngularError3d> for RotationalSweepError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

/// Conservatively bounds a box whose center stays inside the axis-wise interval between two endpoints
/// while its orientation may change arbitrarily.
///
/// The box is enclosed by an orientation-independent sphere with radius equal to the ceiling of the
/// Euclidean norm of its half extents. Sweeping the corresponding axis-aligned cube over the endpoint
/// center interval therefore contains every possible intermediate orientation. This is broad-phase
/// evidence only: it does not prove contact, time of impact, or analytic rotational CCD.
///
/// The caller is responsible for ensuring the translational trajectory stays inside the axis-wise
/// endpoint interval. A later accelerated free-flight sampler must widen this bound when an interior
/// translational extremum can leave that interval.
///
/// # Errors
///
/// Returns [`RotationalSweepError3d`] when endpoint shapes differ, dimensions or orientations are
/// invalid, or checked bound arithmetic overflows.
pub fn rotational_sweep_bounds(
    start: OrientedBox3d,
    end: OrientedBox3d,
) -> Result<RotationalSweepBounds3d, RotationalSweepError3d> {
    validate_endpoint(start)?;
    validate_endpoint(end)?;
    if start.half_extents != end.half_extents {
        return Err(RotationalSweepError3d::ShapeMismatch);
    }

    let radius = orientation_independent_radius(start)?;
    let start_center = [
        i64::from(start.center.x),
        i64::from(start.center.y),
        i64::from(start.center.z),
    ];
    let end_center = [
        i64::from(end.center.x),
        i64::from(end.center.y),
        i64::from(end.center.z),
    ];
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        minimum[axis] = start_center[axis]
            .min(end_center[axis])
            .checked_sub(radius)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow)?;
        maximum[axis] = start_center[axis]
            .max(end_center[axis])
            .checked_add(radius)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow)?;
    }
    Ok(RotationalSweepBounds3d { minimum, maximum })
}

fn validate_endpoint(box_shape: OrientedBox3d) -> Result<(), RotationalSweepError3d> {
    if box_shape.half_extents.x <= 0
        || box_shape.half_extents.y <= 0
        || box_shape.half_extents.z <= 0
    {
        return Err(RotationalSweepError3d::InvalidHalfExtents);
    }
    box_shape.orientation.normalized()?;
    Ok(())
}

fn orientation_independent_radius(box_shape: OrientedBox3d) -> Result<i64, RotationalSweepError3d> {
    let extents = [
        box_shape.half_extents.x,
        box_shape.half_extents.y,
        box_shape.half_extents.z,
    ];
    let squared = extents
        .into_iter()
        .map(|extent| {
            let extent = u128::from(extent.unsigned_abs());
            extent * extent
        })
        .try_fold(0_u128, |sum, component| sum.checked_add(component))
        .ok_or(RotationalSweepError3d::ArithmeticOverflow)?;
    let floor = integer_sqrt(squared);
    let radius = if floor
        .checked_mul(floor)
        .is_some_and(|floor_squared| floor_squared == squared)
    {
        floor
    } else {
        floor
            .checked_add(1)
            .ok_or(RotationalSweepError3d::ArithmeticOverflow)?
    };
    i64::try_from(radius).map_err(|_| RotationalSweepError3d::ArithmeticOverflow)
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let bit_length = u128::BITS - value.leading_zeros();
    let mut estimate = 1_u128 << bit_length.div_ceil(2);
    loop {
        let next = u128::midpoint(estimate, value / estimate);
        if next >= estimate {
            return estimate;
        }
        estimate = next;
    }
}

#[cfg(test)]
mod tests {
    use crate::{ORIENTATION_SCALE, Orientation3d, OrientedBox3d, Vec3i, oriented_box_vertices};

    use super::{RotationalSweepError3d, rotational_sweep_bounds};

    fn box_at(center: Vec3i, orientation: Orientation3d) -> OrientedBox3d {
        OrientedBox3d::new(center, Vec3i::new(20, 3, 2), orientation)
    }

    fn assert_contains_vertices(bounds: super::RotationalSweepBounds3d, box_shape: OrientedBox3d) {
        for vertex in oriented_box_vertices(box_shape).expect("valid oriented box") {
            assert!(bounds.contains([
                i64::from(vertex.x),
                i64::from(vertex.y),
                i64::from(vertex.z),
            ]));
        }
    }

    #[test]
    fn sweep_contains_endpoint_vertices_for_arbitrary_rotation() {
        let start = box_at(Vec3i::new(-12, 5, 4), Orientation3d::IDENTITY);
        let end_orientation = Orientation3d::new(0, 0, ORIENTATION_SCALE / 2, ORIENTATION_SCALE)
            .normalized()
            .expect("valid endpoint orientation");
        let end = box_at(Vec3i::new(17, 9, -6), end_orientation);
        let bounds = rotational_sweep_bounds(start, end).expect("valid conservative sweep");

        assert_contains_vertices(bounds, start);
        assert_contains_vertices(bounds, end);
    }

    #[test]
    fn sweep_contains_rotated_midpoint_box() {
        let start = box_at(Vec3i::new(-20, 0, 0), Orientation3d::IDENTITY);
        let end = box_at(Vec3i::new(20, 10, 0), Orientation3d::IDENTITY);
        let midpoint_orientation = Orientation3d::new(0, 0, ORIENTATION_SCALE, ORIENTATION_SCALE)
            .normalized()
            .expect("valid midpoint orientation");
        let midpoint = box_at(Vec3i::new(0, 5, 0), midpoint_orientation);
        let bounds = rotational_sweep_bounds(start, end).expect("valid conservative sweep");

        assert_contains_vertices(bounds, midpoint);
    }

    #[test]
    fn shape_drift_fails_closed() {
        let start = box_at(Vec3i::ZERO, Orientation3d::IDENTITY);
        let end = OrientedBox3d::new(
            Vec3i::new(10, 0, 0),
            Vec3i::new(21, 3, 2),
            Orientation3d::IDENTITY,
        );

        assert_eq!(
            rotational_sweep_bounds(start, end),
            Err(RotationalSweepError3d::ShapeMismatch)
        );
    }

    #[test]
    fn invalid_shape_and_orientation_fail_closed() {
        let invalid_shape =
            OrientedBox3d::new(Vec3i::ZERO, Vec3i::new(1, 0, 1), Orientation3d::IDENTITY);
        assert_eq!(
            rotational_sweep_bounds(invalid_shape, invalid_shape),
            Err(RotationalSweepError3d::InvalidHalfExtents)
        );

        let invalid_orientation = box_at(Vec3i::ZERO, Orientation3d::new(0, 0, 0, 0));
        assert!(matches!(
            rotational_sweep_bounds(invalid_orientation, invalid_orientation),
            Err(RotationalSweepError3d::Angular(_))
        ));
    }
}
