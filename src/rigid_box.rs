use std::{error::Error, fmt};

use crate::{
    AngularError3d, AngularState3d, AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBody,
    Vec3i,
};

/// Symmetric collision-layer membership and mask for one rotating body.
///
/// A pair is eligible only when each body's membership intersects the other body's mask. The default
/// deliberately preserves the historical engine behavior by allowing every layer to interact with every
/// other layer. Layers affect collision discovery only; they do not change material or response policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionLayers3d {
    memberships: u32,
    mask: u32,
}

impl CollisionLayers3d {
    pub const ALL: Self = Self::new(u32::MAX, u32::MAX);

    #[must_use]
    pub const fn new(memberships: u32, mask: u32) -> Self {
        Self { memberships, mask }
    }

    #[must_use]
    pub const fn memberships(self) -> u32 {
        self.memberships
    }

    #[must_use]
    pub const fn mask(self) -> u32 {
        self.mask
    }

    #[must_use]
    pub const fn collides_with(self, other: Self) -> bool {
        self.memberships & other.mask != 0 && other.memberships & self.mask != 0
    }
}

impl Default for CollisionLayers3d {
    fn default() -> Self {
        Self::ALL
    }
}

/// Optional response policy for externally controlled bodies. Collision discovery is unchanged.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ContactMode3d {
    #[default]
    Physical,
    /// Normal-only, inelastic linear pushing. Contacts opposing the supplied support direction
    /// resolve this body against an unchanged support, without transferring landing load or torque.
    /// Zero support direction disables one-way support. This is an actuator policy, not a claim of
    /// momentum-conserving rigid-body dynamics. Two actuators use symmetric linear response.
    LinearPush { support_direction: Vec3i },
}

/// Controls whether a resolved collision is retained as a resting/contact constraint after its impact.
///
/// This is intentionally independent from [`ContactMode3d`]. A transient body still uses the normal
/// physical response path—including restitution, linear impulse, angular impulse, and penetration
/// projection—but contacts involving it are not fed into persistent-contact stabilization. This is useful
/// for ballistic/projectile bodies that should deliver discrete impacts without becoming part of a resting
/// manifold. Collision discovery and later re-contact remain enabled.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ContactPersistence3d {
    #[default]
    Persistent,
    Transient,
}

/// Engine-native rotating cuboid state.
///
/// `RigidBody` remains the canonical translational/material state. This wrapper adds rotational state
/// without making the existing AABB `World` imply rotational collision response before that solver exists.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RigidBox3d {
    pub(crate) body: RigidBody,
    pub(crate) angular: AngularState3d,
    pub(crate) rotation_locked: bool,
    pub(crate) contact_mode: ContactMode3d,
    pub(crate) contact_persistence: ContactPersistence3d,
    pub(crate) collision_layers: CollisionLayers3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RigidBoxError3d {
    InvalidHalfExtents(BodyId),
    ZeroMass(BodyId),
    FixedAngularVelocity(BodyId),
    Angular(AngularError3d),
}

impl fmt::Display for RigidBoxError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidHalfExtents(body) => write!(
                formatter,
                "rotating rigid box {} requires strictly positive half extents",
                body.0
            ),
            Self::ZeroMass(body) => write!(
                formatter,
                "dynamic rotating rigid box {} requires non-zero mass",
                body.0
            ),
            Self::FixedAngularVelocity(body) => write!(
                formatter,
                "fixed rotating rigid box {} must have zero angular velocity",
                body.0
            ),
            Self::Angular(error) => write!(formatter, "rotating rigid-box state failed: {error}"),
        }
    }
}

impl Error for RigidBoxError3d {}

impl From<AngularError3d> for RigidBoxError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl RigidBox3d {
    /// Creates a validated rotating cuboid while preserving `RigidBody` as the translational/material
    /// authority.
    ///
    /// # Errors
    ///
    /// Returns [`RigidBoxError3d`] for degenerate box dimensions, zero dynamic mass, non-zero angular
    /// velocity on a fixed body, or an invalid orientation.
    pub fn new(body: RigidBody, angular: AngularState3d) -> Result<Self, RigidBoxError3d> {
        let half_extents = body.half_extents();
        if half_extents.x <= 0 || half_extents.y <= 0 || half_extents.z <= 0 {
            return Err(RigidBoxError3d::InvalidHalfExtents(body.id()));
        }
        if body.kind() == BodyKind::Dynamic && body.mass_units() == 0 {
            return Err(RigidBoxError3d::ZeroMass(body.id()));
        }
        if body.kind() == BodyKind::Fixed && !angular.angular_velocity.is_zero() {
            return Err(RigidBoxError3d::FixedAngularVelocity(body.id()));
        }

        let angular =
            AngularState3d::new(angular.orientation.normalized()?, angular.angular_velocity);
        Ok(Self {
            body,
            angular,
            rotation_locked: false,
            contact_mode: ContactMode3d::Physical,
            contact_persistence: ContactPersistence3d::Persistent,
            collision_layers: CollisionLayers3d::default(),
        })
    }

    /// Prevents collision response and free-flight integration from changing this box's orientation.
    ///
    /// The current orientation is preserved while angular velocity is cleared. This is intended for
    /// constrained rigid bodies such as upright character-controller proxies; translational collision
    /// response remains fully dynamic.
    #[must_use]
    pub fn with_rotation_locked(mut self) -> Self {
        self.rotation_locked = true;
        self.angular = AngularState3d::new(self.angular.orientation, AngularVelocity3d::default());
        self
    }

    /// Opts an externally controlled body into linear-only pushing and one-way support contacts.
    /// Only contacts involving this body change; ordinary objects and projectiles retain rotation.
    #[must_use]
    pub fn with_linear_push(mut self, support_direction: Vec3i) -> Self {
        self.contact_mode = ContactMode3d::LinearPush { support_direction };
        self.with_rotation_locked()
    }

    /// Marks this body as an impact-only participant in persistent-contact stabilization.
    ///
    /// Collision discovery and ordinary physical impulse response are unchanged. Only the post-impact
    /// resting/contact graph excludes pairs involving this body.
    #[must_use]
    pub const fn with_transient_contacts(mut self) -> Self {
        self.contact_persistence = ContactPersistence3d::Transient;
        self
    }

    /// Assigns collision-layer membership and the symmetric interaction mask used by collision discovery.
    #[must_use]
    pub const fn with_collision_layers(mut self, collision_layers: CollisionLayers3d) -> Self {
        self.collision_layers = collision_layers;
        self
    }

    #[must_use]
    pub const fn contact_mode(&self) -> ContactMode3d {
        self.contact_mode
    }

    #[must_use]
    pub const fn contact_persistence(&self) -> ContactPersistence3d {
        self.contact_persistence
    }

    #[must_use]
    pub const fn collision_layers(&self) -> CollisionLayers3d {
        self.collision_layers
    }

    #[must_use]
    pub const fn body(&self) -> &RigidBody {
        &self.body
    }

    #[must_use]
    pub const fn angular(&self) -> AngularState3d {
        self.angular
    }

    #[must_use]
    pub const fn rotation_locked(&self) -> bool {
        self.rotation_locked
    }

    #[must_use]
    pub fn oriented_box(&self) -> OrientedBox3d {
        OrientedBox3d::new(
            self.body.position(),
            self.body.half_extents(),
            self.angular.orientation,
        )
    }

    #[must_use]
    pub fn into_body(self) -> RigidBody {
        self.body
    }
}

#[cfg(test)]
mod tests {
    use crate::{AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, Vec3i};

    use super::{CollisionLayers3d, ContactPersistence3d, RigidBox3d, RigidBoxError3d};

    #[test]
    fn constructor_preserves_engine_body_identity_and_geometry() {
        let body = RigidBody::dynamic(
            BodyId(4),
            Vec3i::new(1, 2, 3),
            Vec3i::new(4, 5, 6),
            Vec3i::new(7, 8, 9),
        );
        let rigid_box = RigidBox3d::new(
            body.clone(),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid rotating box");

        assert_eq!(rigid_box.body(), &body);
        assert_eq!(rigid_box.oriented_box().center, body.position());
        assert_eq!(rigid_box.oriented_box().half_extents, body.half_extents());
        assert!(!rigid_box.rotation_locked());
        assert_eq!(rigid_box.collision_layers(), CollisionLayers3d::ALL);
        assert_eq!(
            rigid_box.contact_persistence(),
            ContactPersistence3d::Persistent
        );
    }

    #[test]
    fn transient_contacts_are_an_explicit_independent_policy() {
        let rigid_box = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(8),
                Vec3i::ZERO,
                Vec3i::new(20, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid rotating box")
        .with_transient_contacts();

        assert_eq!(
            rigid_box.contact_persistence(),
            ContactPersistence3d::Transient
        );
        assert_eq!(rigid_box.contact_mode(), super::ContactMode3d::Physical);
    }

    #[test]
    fn rotation_lock_preserves_pose_and_clears_spin() {
        let rigid_box = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(7),
                Vec3i::ZERO,
                Vec3i::new(1, 2, 3),
                Vec3i::new(2, 3, 4),
            ),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(100, 200, 300),
            ),
        )
        .expect("valid rotating box")
        .with_rotation_locked();

        assert!(rigid_box.rotation_locked());
        assert_eq!(rigid_box.angular().orientation, Orientation3d::IDENTITY);
        assert!(rigid_box.angular().angular_velocity.is_zero());
    }

    #[test]
    fn collision_layers_require_both_masks_to_accept_the_pair() {
        let character = CollisionLayers3d::new(0b0010, 0b0100);
        let crate_body = CollisionLayers3d::new(0b0100, 0b0010);
        let projectile = CollisionLayers3d::new(0b1000, u32::MAX);

        assert!(character.collides_with(crate_body));
        assert!(!character.collides_with(projectile));
    }

    #[test]
    fn fixed_body_rejects_angular_velocity() {
        let body = RigidBody::fixed(BodyId(5), Vec3i::ZERO, Vec3i::new(1, 1, 1));
        let result = RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::new(1, 0, 0)),
        );

        assert_eq!(
            result,
            Err(RigidBoxError3d::FixedAngularVelocity(BodyId(5)))
        );
    }

    #[test]
    fn degenerate_box_is_rejected() {
        let body = RigidBody::dynamic(BodyId(6), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 0, 1));
        assert_eq!(
            RigidBox3d::new(
                body,
                AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            ),
            Err(RigidBoxError3d::InvalidHalfExtents(BodyId(6)))
        );
    }
}
