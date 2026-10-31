//! Rigid-body records and the internal body order.

use super::{CombineRule, Quaternion, Real, Shape, Vector};
use crate::{BodyId, CollisionLayers3d};

#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    /// Assigned by [`World::add_body`](super::World::add_body); [`BodyId::NONE`] before.
    pub id: BodyId,
    /// Deterministic traversal and tie-break order, assigned by
    /// [`World::add_body`](super::World::add_body).
    pub(super) order: Order,
    /// Optional game-supplied stable key (the game's entity id), unique among live bodies.
    pub key: Option<u64>,
    pub position: Vector,
    pub velocity: Vector,
    pub orientation: Quaternion,
    /// Radians per second in world coordinates.
    pub angular_velocity: Vector,
    pub shape: Shape,
    /// Zero is immovable geometry; positive mass is dynamic unless `external` is set.
    pub mass: Real,
    /// Finite coefficient in [0, 10], default 0.5 (Rapier's). Contacts combine the pair's
    /// coefficients by [`Body::friction_combine`] and clamp the accumulated tangential impulse
    /// to a disk of radius mu * normal impulse. Linear-support contacts disable friction. See
    /// docs/contact-materials.md.
    pub friction: Real,
    /// Finite coefficient in [0, 1], default zero. Contacts combine the pair's coefficients by
    /// [`Body::restitution_combine`]. Restitution requires closing speed > 1 m/s (scaled by
    /// `length_unit`), separation <= contact_slop and no retained pair history; speculative
    /// positive separation does not bounce.
    pub restitution: Real,
    /// Default [`CombineRule::Average`], as in Rapier.
    pub friction_combine: CombineRule,
    /// Default [`CombineRule::Average`], as in Rapier.
    pub restitution_combine: CombineRule,
    /// Finite, nonnegative, default zero. After forces, each substep scales the linear
    /// velocity by `1 / (1 + h * linear_damping)`, as Rapier does.
    pub linear_damping: Real,
    /// Finite, nonnegative, default zero. The same scaling for the angular velocity.
    pub angular_damping: Real,
    pub rotation_locked: bool,
    pub layers: CollisionLayers3d,
    pub external: bool,
    pub sensor: bool,
    pub sleep_allowed: bool,
    pub ccd: bool,
    pub retire_on_impact: bool,
    pub linear_support: Option<Vector>,
    pub(super) sleeping: bool,
    pub(super) quiet_time: Real,
    pub(super) force: Vector,
    pub(super) torque: Vector,
    pub(super) impulse: Vector,
    pub(super) angular_impulse: Vector,
    pub(super) cached_bounds: (Vector, Vector),
}
impl Body {
    /// A body carrying the game's stable `key` (see [`World::body_id`](super::World::body_id)).
    pub fn keyed(key: u64, shape: Shape, position: Vector, mass: Real) -> Self {
        let mut body = Self::new(shape, position, mass);
        body.key = Some(key);
        body
    }
    /// Test fixture: keyed, with the id it would have as slot `key` of a fresh world.
    #[cfg(test)]
    pub(crate) fn fixture(key: u64, shape: Shape, position: Vector, mass: Real) -> Self {
        let mut body = Self::keyed(key, shape, position, mass);
        body.id = BodyId::new(key as u32, 0);
        body.order = Order::new(Some(key), 0);
        body
    }
    pub fn new(shape: Shape, position: Vector, mass: Real) -> Self {
        Self {
            id: BodyId::NONE,
            order: Order(u128::MAX),
            key: None,
            shape,
            position,
            mass,
            velocity: Vector::ZERO,
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector::ZERO,
            friction: 0.5,
            restitution: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Average,
            linear_damping: 0.0,
            angular_damping: 0.0,
            rotation_locked: false,
            layers: CollisionLayers3d::ALL,
            external: false,
            sensor: false,
            sleep_allowed: true,
            ccd: false,
            retire_on_impact: false,
            linear_support: None,
            sleeping: false,
            quiet_time: 0.0,
            force: Vector::ZERO,
            torque: Vector::ZERO,
            impulse: Vector::ZERO,
            angular_impulse: Vector::ZERO,
            cached_bounds: (Vector::ZERO, Vector::ZERO),
        }
    }
    /// Diagnostic kinetic energy in mass-units * scene-units squared / second squared.
    /// This observation does not feed back into dynamics or the sleep policy.
    /// Angular motion with unsupported inertia returns NaN rather than invented energy.
    pub fn kinetic_energy(&self) -> Real {
        if self.mass == 0.0 {
            return 0.0;
        }
        let linear = 0.5 * self.mass * self.velocity.dot(self.velocity);
        if self.angular_velocity == Vector::ZERO {
            return linear;
        }
        let Some(inverse) = self.shape.local_inverse_inertia(self.mass) else {
            return Real::NAN;
        };
        let w = self.orientation.inverse_rotate(self.angular_velocity);
        linear + 0.5 * (w.0 * w.0 / inverse.0 + w.1 * w.1 / inverse.1 + w.2 * w.2 / inverse.2)
    }
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }
    pub(super) fn movable(&self) -> bool {
        self.mass > 0.0 && !self.external
    }
    pub(super) fn inverse_mass(&self) -> Real {
        if self.movable() && !self.sleeping {
            1.0 / self.mass
        } else {
            0.0
        }
    }
    #[cfg(test)]
    pub(super) fn inverse_inertia(&self, v: Vector) -> Vector {
        if self.inverse_mass() == 0.0 || self.rotation_locked {
            return Vector::ZERO;
        }
        let inverse = self
            .shape
            .local_inverse_inertia(self.mass)
            .unwrap_or(Vector::ZERO);
        self.orientation
            .rotate(self.orientation.inverse_rotate(v).component_mul(inverse))
    }
    pub(super) fn valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && self.shape.valid_dimensions()
            && (!matches!(self.shape, Shape::Wedge(_)) || self.rotation_locked || !self.movable())
            && (!self.shape.query_only() || (self.mass == 0.0 && !self.external))
            && self.mass.is_finite()
            && (self.mass == 0.0 || self.mass >= 1e-6)
            && self.mass < 1e12
            && self.friction.is_finite()
            && (0.0..=10.0).contains(&self.friction)
            && self.restitution.is_finite()
            && (0.0..=1.0).contains(&self.restitution)
            && (0.0..1e6).contains(&self.linear_damping)
            && (0.0..1e6).contains(&self.angular_damping)
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
    /// [`Self::valid`] for a valid body whose motion changed: only the motion terms can fail.
    pub(super) fn motion_valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
}
/// Internal body order: keyed bodies first by key, then keyless bodies by creation serial.
/// Every traversal, pair orientation and tie uses it, so results depend on game keys and the
/// world's contents, not on slot reuse or insertion history of keyed bodies.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub(crate) struct Order(pub(super) u128);
impl Ord for Order {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        crate::work::add(crate::work::Counter::OrderComparisons, 1);
        self.0.cmp(&other.0)
    }
}
impl PartialOrd for Order {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Order {
    pub(crate) fn new(key: Option<u64>, serial: u64) -> Self {
        Self(match key {
            Some(key) => u128::from(key),
            None => (1 << 64) | u128::from(serial),
        })
    }
}
