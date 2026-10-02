//! Physical scaling of reusable uniform-solid geometric products.

use super::{Real, Shape, Vector, primitive};

/// Uniform-solid products in the shape's local frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub mass: Real,
    pub volume: Real,
    /// Offset from the existing shape origin, in scene length units.
    pub local_center_of_mass: Vector,
    /// Inertia about the center of mass, in mass * scene length squared units.
    /// Rows and columns follow local X/Y/Z; off-diagonal products are included.
    pub local_inertia: [[Real; 3]; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MassPropertiesError {
    InvalidDimensions,
    InvalidMass,
    Unrepresentable,
}

impl Shape {
    /// Compute uniform volume, center of mass and the full local inertia tensor.
    ///
    /// Uses the same dimension and positive dynamic mass ranges as floating bodies:
    /// positive dimensions and mass in `[1e-6, 1e12)`, capsule half-segment in
    /// `[0, 1e12)`. Zero mass is an immovable-body policy, not a uniform-solid mass.
    /// Wedge products describe its geometric COM; they do not change the existing
    /// origin-based body pose or enable rotation for solver-owned dynamic wedges.
    pub fn mass_properties(self, mass: Real) -> Result<MassProperties, MassPropertiesError> {
        if !self.valid_dimensions() {
            return Err(MassPropertiesError::InvalidDimensions);
        }
        if !mass.is_finite() || !(1e-6..1e12).contains(&mass) {
            return Err(MassPropertiesError::InvalidMass);
        }
        let geometry =
            primitive::volume_properties(self).map_err(|_| MassPropertiesError::Unrepresentable)?;
        let moment = geometry.normalized_second_moment;
        // The kernel product stays f64; scale in f64, then narrow each result once.
        let wide = primitive::widen(mass);
        // Sum the other diagonals directly to avoid trace-minus-diagonal cancellation.
        let local_inertia = [
            [
                wide * (moment[1][1] + moment[2][2]),
                -wide * moment[0][1],
                -wide * moment[0][2],
            ],
            [
                -wide * moment[1][0],
                wide * (moment[0][0] + moment[2][2]),
                -wide * moment[1][2],
            ],
            [
                -wide * moment[2][0],
                -wide * moment[2][1],
                wide * (moment[0][0] + moment[1][1]),
            ],
        ]
        .map(|row| row.map(primitive::nearest));
        if local_inertia
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
            || (0..3).any(|axis| local_inertia[axis][axis] <= 0.0)
        {
            return Err(MassPropertiesError::Unrepresentable);
        }
        Ok(MassProperties {
            mass,
            volume: primitive::nearest(geometry.volume),
            local_center_of_mass: Vector(
                primitive::nearest(geometry.centroid[0]),
                primitive::nearest(geometry.centroid[1]),
                primitive::nearest(geometry.centroid[2]),
            ),
            local_inertia,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn public_inertia_agrees_with_existing_solver_coefficients() {
        for scale in [1e-6, 1.0, 1e6, 1e11] {
            for shape in [
                Shape::Sphere(scale),
                Shape::Box(Vector(scale, 2.0 * scale, 3.0 * scale)),
                Shape::capsule(0.0, scale),
                Shape::capsule(0.01 * scale, scale),
                Shape::capsule(3.0 * scale, scale),
            ] {
                for mass in [1e-6, 1.0, 1e11] {
                    // The public inertia tensor is `Real`; combinations whose inertia leaves the
                    // `Real` range are outside the f32 envelope.
                    if !(mass * 64.0 * scale * scale * scale).is_finite() {
                        assert_eq!(super::super::REAL_BITS, 32);
                        continue;
                    }
                    let properties = shape.mass_properties(mass).unwrap();
                    let inverse = shape.local_inverse_inertia(mass).unwrap();
                    for (axis, inverse) in [inverse.0, inverse.1, inverse.2].into_iter().enumerate()
                    {
                        let actual = properties.local_inertia[axis][axis] * inverse;
                        assert!(
                            (actual - 1.0).abs() <= (2e-14 as Real).max(32.0 * Real::EPSILON),
                            "{shape:?} mass {mass}: {actual}"
                        );
                    }
                }
            }
        }
        assert_eq!(
            Shape::wedge(Vector(1.0, 2.0, 3.0)).local_inverse_inertia(1.0),
            None
        );
        // Accepted extremes whose `mass * length^2` overflows f32 keep a representable,
        // nonzero inverse inertia (evaluated in f64, then narrowed).
        for shape in [
            Shape::capsule(1e9, 1e9),
            Shape::Box(Vector(1e9, 1e9, 1e9)),
            Shape::Sphere(1e9),
        ] {
            let inverse = shape.local_inverse_inertia(1e12).unwrap();
            for value in [inverse.0, inverse.1, inverse.2] {
                assert!(value.is_finite() && value > 0.0, "{shape:?}: {value}");
            }
        }
    }
}
