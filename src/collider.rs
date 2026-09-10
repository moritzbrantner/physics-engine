use std::{error::Error, fmt};

use crate::Vec3i;

/// Reusable collision shape independent of ECS, rendering and world ownership.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColliderShape {
    Aabb { half_extents: Vec3i },
    Sphere { radius: i32 },
}

impl ColliderShape {
    #[must_use]
    pub const fn aabb(half_extents: Vec3i) -> Self {
        Self::Aabb { half_extents }
    }

    #[must_use]
    pub const fn sphere(radius: i32) -> Self {
        Self::Sphere { radius }
    }

    #[must_use]
    pub const fn bounding_half_extents(self) -> Vec3i {
        match self {
            Self::Aabb { half_extents } => half_extents,
            Self::Sphere { radius } => Vec3i::new(radius, radius, radius),
        }
    }
}

/// Shape instance at a world-space center.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Collider {
    pub center: Vec3i,
    pub shape: ColliderShape,
}

impl Collider {
    #[must_use]
    pub const fn new(center: Vec3i, shape: ColliderShape) -> Self {
        Self { center, shape }
    }
}

/// Exact integer contact evidence for a collider pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ColliderContact {
    /// Vector from the left collider toward the closest/contact feature on the right collider.
    /// It is deliberately not normalized so integer direction evidence remains exact.
    pub direction: [i64; 3],
    /// Squared closest-feature distance before subtracting the applicable radius.
    pub distance_squared: i128,
    /// Squared contact threshold. Contact exists when `distance_squared <= threshold_squared`.
    pub threshold_squared: i128,
}

impl ColliderContact {
    #[must_use]
    pub const fn overlaps(self) -> bool {
        self.distance_squared <= self.threshold_squared
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ColliderError {
    NegativeAabbHalfExtent,
    NegativeSphereRadius,
    ArithmeticOverflow,
}

impl fmt::Display for ColliderError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NegativeAabbHalfExtent => {
                write!(formatter, "AABB collider has a negative half extent")
            }
            Self::NegativeSphereRadius => {
                write!(formatter, "sphere collider has a negative radius")
            }
            Self::ArithmeticOverflow => write!(formatter, "collider contact arithmetic overflow"),
        }
    }
}

impl Error for ColliderError {}

/// Evaluates deterministic integer contact semantics for AABB and sphere pairs.
///
/// This is a geometry/contact query. It does not integrate time or apply impulses: the world CCD
/// solver remains the authority for time-of-impact and collision response. Touching counts as
/// contact, matching the existing AABB world contract.
///
/// # Errors
///
/// Returns [`ColliderError`] when either shape has invalid dimensions or checked contact arithmetic
/// cannot be represented.
pub fn collider_contact(left: Collider, right: Collider) -> Result<ColliderContact, ColliderError> {
    validate_shape(left.shape)?;
    validate_shape(right.shape)?;

    match (left.shape, right.shape) {
        (
            ColliderShape::Sphere {
                radius: left_radius,
            },
            ColliderShape::Sphere {
                radius: right_radius,
            },
        ) => sphere_sphere_contact(left.center, left_radius, right.center, right_radius),
        (ColliderShape::Sphere { radius }, ColliderShape::Aabb { half_extents }) => {
            sphere_aabb_contact(left.center, radius, right.center, half_extents)
        }
        (ColliderShape::Aabb { half_extents }, ColliderShape::Sphere { radius }) => {
            let contact = sphere_aabb_contact(right.center, radius, left.center, half_extents)?;
            Ok(ColliderContact {
                direction: contact.direction.map(|value| -value),
                ..contact
            })
        }
        (
            ColliderShape::Aabb {
                half_extents: left_half,
            },
            ColliderShape::Aabb {
                half_extents: right_half,
            },
        ) => aabb_aabb_contact(left.center, left_half, right.center, right_half),
    }
}

fn validate_shape(shape: ColliderShape) -> Result<(), ColliderError> {
    match shape {
        ColliderShape::Aabb { half_extents }
            if half_extents.x < 0 || half_extents.y < 0 || half_extents.z < 0 =>
        {
            Err(ColliderError::NegativeAabbHalfExtent)
        }
        ColliderShape::Sphere { radius } if radius < 0 => Err(ColliderError::NegativeSphereRadius),
        _ => Ok(()),
    }
}

fn sphere_sphere_contact(
    left: Vec3i,
    left_radius: i32,
    right: Vec3i,
    right_radius: i32,
) -> Result<ColliderContact, ColliderError> {
    let direction = delta(left, right);
    let distance_squared = squared_length(direction)?;
    let radius = i128::from(left_radius)
        .checked_add(i128::from(right_radius))
        .ok_or(ColliderError::ArithmeticOverflow)?;
    let threshold_squared = radius
        .checked_mul(radius)
        .ok_or(ColliderError::ArithmeticOverflow)?;
    Ok(ColliderContact {
        direction,
        distance_squared,
        threshold_squared,
    })
}

fn sphere_aabb_contact(
    sphere: Vec3i,
    radius: i32,
    aabb: Vec3i,
    half_extents: Vec3i,
) -> Result<ColliderContact, ColliderError> {
    let sphere_axes = axes(sphere);
    let aabb_axes = axes(aabb);
    let half_axes = axes(half_extents);
    let mut closest = [0_i64; 3];

    for axis in 0..3 {
        let minimum = aabb_axes[axis]
            .checked_sub(half_axes[axis])
            .ok_or(ColliderError::ArithmeticOverflow)?;
        let maximum = aabb_axes[axis]
            .checked_add(half_axes[axis])
            .ok_or(ColliderError::ArithmeticOverflow)?;
        closest[axis] = sphere_axes[axis].clamp(minimum, maximum);
    }

    let direction = [
        closest[0] - sphere_axes[0],
        closest[1] - sphere_axes[1],
        closest[2] - sphere_axes[2],
    ];
    let distance_squared = squared_length(direction)?;
    let radius = i128::from(radius);
    let threshold_squared = radius
        .checked_mul(radius)
        .ok_or(ColliderError::ArithmeticOverflow)?;
    Ok(ColliderContact {
        direction,
        distance_squared,
        threshold_squared,
    })
}

fn aabb_aabb_contact(
    left: Vec3i,
    left_half: Vec3i,
    right: Vec3i,
    right_half: Vec3i,
) -> Result<ColliderContact, ColliderError> {
    let center_delta = delta(left, right);
    let left_half = axes(left_half);
    let right_half = axes(right_half);
    let mut direction = [0_i64; 3];

    for axis in 0..3 {
        let center_distance = i128::from(center_delta[axis]).abs();
        let combined_half = i128::from(left_half[axis])
            .checked_add(i128::from(right_half[axis]))
            .ok_or(ColliderError::ArithmeticOverflow)?;
        let gap = center_distance.saturating_sub(combined_half).max(0);
        let gap = i64::try_from(gap).map_err(|_| ColliderError::ArithmeticOverflow)?;
        direction[axis] = match center_delta[axis].cmp(&0) {
            std::cmp::Ordering::Less => -gap,
            std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => gap,
        };
    }

    Ok(ColliderContact {
        direction,
        distance_squared: squared_length(direction)?,
        threshold_squared: 0,
    })
}

fn squared_length(vector: [i64; 3]) -> Result<i128, ColliderError> {
    vector.into_iter().try_fold(0_i128, |sum, component| {
        let component = i128::from(component);
        sum.checked_add(
            component
                .checked_mul(component)
                .ok_or(ColliderError::ArithmeticOverflow)?,
        )
        .ok_or(ColliderError::ArithmeticOverflow)
    })
}

fn delta(left: Vec3i, right: Vec3i) -> [i64; 3] {
    [
        i64::from(right.x) - i64::from(left.x),
        i64::from(right.y) - i64::from(left.y),
        i64::from(right.z) - i64::from(left.z),
    ]
}

fn axes(vector: Vec3i) -> [i64; 3] {
    [
        i64::from(vector.x),
        i64::from(vector.y),
        i64::from(vector.z),
    ]
}

#[cfg(test)]
mod tests {
    use super::{Collider, ColliderError, ColliderShape, collider_contact};
    use crate::Vec3i;

    #[test]
    fn sphere_sphere_touching_is_contact() {
        let left = Collider::new(Vec3i::ZERO, ColliderShape::sphere(3));
        let right = Collider::new(Vec3i::new(7, 0, 0), ColliderShape::sphere(4));
        let contact = collider_contact(left, right).unwrap();

        assert!(contact.overlaps());
        assert_eq!(contact.distance_squared, 49);
        assert_eq!(contact.threshold_squared, 49);
        assert_eq!(contact.direction, [7, 0, 0]);
    }

    #[test]
    fn separated_spheres_do_not_contact() {
        let left = Collider::new(Vec3i::ZERO, ColliderShape::sphere(2));
        let right = Collider::new(Vec3i::new(5, 0, 0), ColliderShape::sphere(2));

        assert!(!collider_contact(left, right).unwrap().overlaps());
    }

    #[test]
    fn sphere_aabb_corner_uses_exact_squared_distance() {
        let sphere = Collider::new(Vec3i::new(4, 4, 0), ColliderShape::sphere(3));
        let aabb = Collider::new(Vec3i::ZERO, ColliderShape::aabb(Vec3i::new(2, 2, 2)));
        let contact = collider_contact(sphere, aabb).unwrap();

        assert!(contact.overlaps());
        assert_eq!(contact.direction, [-2, -2, 0]);
        assert_eq!(contact.distance_squared, 8);
        assert_eq!(contact.threshold_squared, 9);
    }

    #[test]
    fn aabb_sphere_is_symmetric_except_for_direction() {
        let sphere = Collider::new(Vec3i::new(5, 0, 0), ColliderShape::sphere(2));
        let aabb = Collider::new(Vec3i::ZERO, ColliderShape::aabb(Vec3i::new(3, 3, 3)));
        let left = collider_contact(sphere, aabb).unwrap();
        let right = collider_contact(aabb, sphere).unwrap();

        assert_eq!(left.overlaps(), right.overlaps());
        assert_eq!(left.distance_squared, right.distance_squared);
        assert_eq!(left.threshold_squared, right.threshold_squared);
        assert_eq!(left.direction, right.direction.map(|value| -value));
    }

    #[test]
    fn aabb_direction_matches_closest_feature_distance() {
        let left = Collider::new(Vec3i::ZERO, ColliderShape::aabb(Vec3i::new(2, 2, 2)));
        let right = Collider::new(
            Vec3i::new(10, 3, 0),
            ColliderShape::aabb(Vec3i::new(2, 2, 2)),
        );
        let contact = collider_contact(left, right).unwrap();

        assert_eq!(contact.direction, [6, 0, 0]);
        assert_eq!(contact.distance_squared, 36);
        assert!(!contact.overlaps());
    }

    #[test]
    fn existing_aabb_touching_semantics_are_preserved() {
        let left = Collider::new(Vec3i::ZERO, ColliderShape::aabb(Vec3i::new(2, 2, 2)));
        let right = Collider::new(
            Vec3i::new(4, 0, 0),
            ColliderShape::aabb(Vec3i::new(2, 2, 2)),
        );

        assert!(collider_contact(left, right).unwrap().overlaps());
    }

    #[test]
    fn invalid_shapes_fail_closed() {
        let sphere = Collider::new(Vec3i::ZERO, ColliderShape::sphere(-1));
        let aabb = Collider::new(Vec3i::ZERO, ColliderShape::aabb(Vec3i::new(1, 1, 1)));

        assert_eq!(
            collider_contact(sphere, aabb),
            Err(ColliderError::NegativeSphereRadius)
        );
    }
}
