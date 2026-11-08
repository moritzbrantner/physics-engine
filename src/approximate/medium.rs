//! Per-body fluid medium (#525): buoyancy at the submerged centroid and implicit drag toward a
//! flow, applied in the force phase of each substep. Bodies without a medium pay one branch.

use super::{Body, BodyId, Error, Real, Shape, Vector, World, primitive};

/// How much of the sleep speed the forces on a floating body may build up over the sleep time:
/// a fraction, so a body sleeps at its equilibrium depth to within a millimetre at the default
/// sleep settings rather than anywhere the forces are merely small.
pub(super) const BALANCE_MARGIN: Real = 0.1;

/// The medium a dynamic body is in: a flat surface, a density, a flow and two drag rates.
///
/// Each substep a body with a medium, `V_s` of its volume `V` below the surface
/// (`dot(x - surface_point, surface_normal) <= 0`), gets the buoyancy `-density * V_s * gravity`
/// at its submerged centroid and, with `phi = V_s / V`, the implicit drag
/// `v = flow + (v - flow) / (1 + h * linear_drag * phi)` and
/// `w = w / (1 + h * angular_drag * phi)`. See docs/world-api-contract.md.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Medium {
    /// Any point on the surface. Finite, each component's magnitude below 1e12.
    pub surface_point: Vector,
    /// Points up out of the fluid; need not be unit. Finite, length in `1e-6..=1e6`.
    pub surface_normal: Vector,
    /// Mass per cubic scene unit, in the body mass's units. Finite, in `0..1e12`.
    pub density: Real,
    /// Velocity of the fluid. Finite, each component's magnitude below 1e12.
    pub flow: Vector,
    /// Per second, at full submersion. Finite, in `0..1e6`, as `Body::linear_damping`.
    pub linear_drag: Real,
    /// Per second, at full submersion. Finite, in `0..1e6`, as `Body::angular_damping`.
    pub angular_drag: Real,
}

impl Medium {
    pub(super) fn valid(&self) -> bool {
        let normal = self.surface_normal.length();
        self.surface_point.finite()
            && self.surface_point.abs().max_component() < 1e12
            && self.surface_normal.finite()
            && normal.is_finite()
            && (1e-6..=1e6).contains(&normal)
            && (0.0..1e12).contains(&self.density)
            && self.flow.finite()
            && self.flow.abs().max_component() < 1e12
            && (0.0..1e6).contains(&self.linear_drag)
            && (0.0..1e6).contains(&self.angular_drag)
    }
}

/// A body's medium with what the substeps derive from it once.
#[derive(Clone)]
pub(super) struct BodyMedium {
    pub medium: Medium,
    /// The whole body's kernel volume.
    pub volume: f64,
    /// `density * volume / mass`: the buoyancy per unit of gravity when fully submerged.
    pub full_buoyancy: Real,
    /// Whether this substep's forces balance (see `World::apply_medium_forces`). Recomputed
    /// before every read, so it is neither compared nor checkpointed.
    pub balanced: bool,
}

impl PartialEq for BodyMedium {
    fn eq(&self, other: &Self) -> bool {
        self.medium == other.medium
    }
}

impl std::fmt::Debug for BodyMedium {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.medium.fmt(f)
    }
}

impl BodyMedium {
    /// The state of `medium` on a body of this shape and mass, `None` when out of range.
    pub(super) fn new(medium: Medium, shape: Shape, mass: Real) -> Option<Box<Self>> {
        if !medium.valid() {
            return None;
        }
        let volume = primitive::volume_properties(shape).ok()?.volume;
        Some(Box::new(Self {
            medium,
            volume,
            full_buoyancy: primitive::nearest(
                primitive::widen(medium.density) * volume / primitive::widen(mass),
            ),
            balanced: false,
        }))
    }
}

impl Body {
    /// How far a medium can move this body over the step's later substeps beyond what its
    /// velocity and gravity account for: full buoyancy, and the velocity drag can add on the way
    /// to the flow. Zero without a medium.
    #[inline]
    pub(super) fn medium_reach(&self, gravity: Vector, gain: Real, lookahead: Real) -> Real {
        match &self.medium {
            Some(m) if gain > 0.0 && self.movable() && !self.sleeping => {
                gravity.length() * m.full_buoyancy * gain
                    + (m.medium.flow - self.velocity).length() * lookahead
            }
            _ => 0.0,
        }
    }
}

impl World {
    /// Puts body `id` in `medium`, or takes it out with `None`. The medium persists until the
    /// next call. A change wakes the body's island; the same medium again changes nothing.
    /// `InvalidInput` for out-of-range values and for a medium on a body that is not dynamic.
    pub fn set_medium(&mut self, id: BodyId, medium: Option<Medium>) -> Result<(), Error> {
        let i = self.index(id)?;
        let body = &self.bodies[i];
        if body.medium.as_ref().map(|m| m.medium) == medium {
            return Ok(());
        }
        let state = match medium {
            Some(medium) if body.movable() => {
                Some(BodyMedium::new(medium, body.shape, body.mass).ok_or(Error::InvalidInput)?)
            }
            Some(_) => return Err(Error::InvalidInput),
            None => None,
        };
        self.sort_storage();
        let i = self.index(id)?;
        self.wake_island(self.bodies[i].order);
        self.bodies[i].medium = state;
        Ok(())
    }
}
