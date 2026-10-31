//! Substep-local response coefficients. This caches division/shape work, not body motion.
use super::{Body, Real, Report, Vector};

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct PreparedResponse {
    pub inverse_mass: Real,
    /// Rows of the symmetric world-space inverse inertia at the prepared orientation.
    world_inverse_inertia: [Vector; 3],
    spin: bool,
}

impl PreparedResponse {
    pub fn new(body: &Body, report: &mut Report) -> Self {
        let inverse_mass = body.inverse_mass();
        if inverse_mass == 0.0 {
            return Self::default();
        }
        report.response_preparations += 1;
        if body.rotation_locked {
            return Self {
                inverse_mass,
                ..Self::default()
            };
        }
        report.inertia_preparations += 1;
        let Some(local_inverse_inertia) = body.shape.local_inverse_inertia(body.mass) else {
            return Self {
                inverse_mass,
                ..Self::default()
            };
        };
        // R diag(d) Rᵀ as rows, once per preparation: applying it is three dot products, as
        // Rapier's world-space inverse inertia (#424), instead of two quaternion rotations.
        let axes = body.orientation.axes();
        let d = local_inverse_inertia;
        let row = |i: usize| {
            axes[0] * (d.0 * axes[0].at(i))
                + axes[1] * (d.1 * axes[1].at(i))
                + axes[2] * (d.2 * axes[2].at(i))
        };
        Self {
            inverse_mass,
            world_inverse_inertia: [row(0), row(1), row(2)],
            spin: true,
        }
    }

    /// Whether impulses also change this body's angular velocity.
    #[inline]
    pub fn spins(self) -> bool {
        self.spin
    }

    /// The world-space inverse inertia at the prepared orientation applied to `v`.
    #[inline]
    pub fn inertia(self, v: Vector, report: &mut Report) -> Vector {
        if !self.spin {
            return Vector::ZERO;
        }
        report.inertia_applications += 1;
        let [x, y, z] = self.world_inverse_inertia;
        Vector(x.dot(v), y.dot(v), z.dot(v))
    }
}

#[cfg(test)]
mod tests;
