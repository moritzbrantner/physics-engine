//! Substep-local response coefficients. This caches division/shape work, not body motion.
use super::{Body, Report, Scalar, Shape, Vector};

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct PreparedResponse {
    pub inverse_mass: Scalar,
    local_inverse_inertia: Vector,
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
        Self {
            inverse_mass,
            local_inverse_inertia,
            spin: true,
        }
    }

    #[inline]
    pub fn select<const PREPARED: bool>(self, body: &Body, report: &mut Report) -> Self {
        if PREPARED {
            self
        } else {
            Self::new(body, report)
        }
    }

    #[inline]
    pub fn inertia(self, body: &Body, v: Vector, report: &mut Report) -> Vector {
        if !self.spin {
            return Vector::ZERO;
        }
        report.inertia_applications += 1;
        // Deliberately preserve the reference's quaternion operations and their evaluation order.
        // A world-space matrix or pre-scaled contact axes would regroup floating-point operations.
        // Read orientation from its sole authority; it is constant during velocity solving and
        // changes only in integration, after the final use of this substep's coefficients.
        body.orientation.rotate(
            body.orientation
                .inverse_rotate(v)
                .component_mul(self.local_inverse_inertia),
        )
    }
}

#[cfg(test)]
mod tests;
