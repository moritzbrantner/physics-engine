//! Collision shapes: dimensions, bounds radii and inverse inertia.

use super::{Real, TerrainId, Vector};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box(Vector),
    Sphere(Real),
    /// Local-Y segment expanded by a spherical radius.
    Capsule {
        half_segment: Real,
        radius: Real,
    },
    /// Right triangular prism inside the local bounding box. The ramp rises toward -X.
    Wedge(Vector),
    /// Static terrain created by [`World::add_heightfield`](super::World::add_heightfield);
    /// `half_extents` bounds it around the body position.
    Heightfield {
        terrain: TerrainId,
        half_extents: Vector,
    },
    /// Static solid halfspace below the plane through the body position; the outward normal is
    /// the body's local +Y (an infinite floor with the identity orientation).
    HalfSpace,
}
impl Shape {
    pub const fn capsule(half_segment: Real, radius: Real) -> Self {
        Self::Capsule {
            half_segment,
            radius,
        }
    }
    pub const fn wedge(half_extents: Vector) -> Self {
        Self::Wedge(half_extents)
    }
    /// Terrain and halfspaces are static only and stay out of the bounds structures. They
    /// answer queries and character moves; halfspaces also give dynamic bodies rigid contacts,
    /// heightfields give dynamic boxes rigid contacts.
    pub fn query_only(self) -> bool {
        matches!(self, Self::Heightfield { .. } | Self::HalfSpace)
    }
    pub fn radius(self) -> Real {
        match self {
            Self::Box(h)
            | Self::Wedge(h)
            | Self::Heightfield {
                half_extents: h, ..
            } => h.length(),
            Self::HalfSpace => 0.0,
            Self::Sphere(r) => r,
            Self::Capsule {
                half_segment,
                radius,
            } => half_segment + radius,
        }
    }
    pub fn half_extents(self) -> Vector {
        match self {
            Self::Box(h)
            | Self::Wedge(h)
            | Self::Heightfield {
                half_extents: h, ..
            } => h,
            Self::HalfSpace => Vector::ZERO,
            Self::Sphere(r) => Vector(r, r, r),
            Self::Capsule {
                half_segment,
                radius,
            } => Vector(radius, half_segment + radius, radius),
        }
    }
    pub(super) fn valid_dimensions(self) -> bool {
        match self {
            Self::Box(h) | Self::Wedge(h) => {
                h.finite() && h.min_component() >= 1e-6 && h.max_component() < 1e12
            }
            Self::Sphere(radius) => radius.is_finite() && (1e-6..1e12).contains(&radius),
            Self::Heightfield {
                half_extents: h, ..
            } => h.finite() && h.max_component() < 1e12,
            Self::HalfSpace => true,
            Self::Capsule {
                half_segment,
                radius,
            } => {
                half_segment.is_finite()
                    && (0.0..1e12).contains(&half_segment)
                    && radius.is_finite()
                    && (1e-6..1e12).contains(&radius)
            }
        }
    }
    /// Evaluated in f64 in every build (the identity for the default build): with f32 state,
    /// `mass * length^2` can overflow for accepted inputs although the inverse inertia is
    /// representable. The narrowed result rounds to nearest.
    #[allow(clippy::unnecessary_cast)]
    pub(super) fn local_inverse_inertia(self, mass: Real) -> Option<Vector> {
        let mass = mass as f64;
        let narrow = |x: f64, y: f64, z: f64| Some(Vector(x as Real, y as Real, z as Real));
        match self {
            Self::Sphere(r) => {
                let r = r as f64;
                let inverse = 2.5 / (mass * r * r);
                narrow(inverse, inverse, inverse)
            }
            Self::Box(h) => {
                let [h0, h1, h2] = [h.0 as f64, h.1 as f64, h.2 as f64];
                narrow(
                    3.0 / (mass * (h1 * h1 + h2 * h2)),
                    3.0 / (mass * (h0 * h0 + h2 * h2)),
                    3.0 / (mass * (h0 * h0 + h1 * h1)),
                )
            }
            Self::Capsule {
                half_segment: h,
                radius: r,
            } => {
                let (h, r) = (h as f64, r as f64);
                let cylinder_volume = r * r * (2.0 * h);
                let sphere_volume = (4.0 / 3.0) * r * r * r;
                let total_volume = cylinder_volume + sphere_volume;
                let cylinder_mass = mass * cylinder_volume / total_volume;
                let sphere_mass = mass * sphere_volume / total_volume;
                let axial = cylinder_mass * r * r * 0.5 + sphere_mass * r * r * (2.0 / 5.0);
                let radial = cylinder_mass * (3.0 * r * r + 4.0 * h * h) / 12.0
                    + sphere_mass * ((2.0 / 5.0) * r * r + h * h + (3.0 / 4.0) * h * r);
                narrow(1.0 / radial, 1.0 / axial, 1.0 / radial)
            }
            // Solver-owned dynamic wedges are required to be rotation locked until a COM-centered
            // wedge inertia/pose contract is introduced. Terrain is static.
            Self::Wedge(_) | Self::Heightfield { .. } | Self::HalfSpace => None,
        }
    }
}
