use std::{error::Error, fmt};

use crate::{
    AngularError3d, AngularState3d, AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBody,
};

/// Engine-native rotating cuboid state.
///
/// `RigidBody` remains the canonical translational/material state. This wrapper adds rotational state
/// without making the existing AABB `World` imply rotational collision response before that solver exists.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RigidBox3d {
    pub(crate) body: RigidBody,
    pub(crate) angular: AngularState3d,
    pub(crate) rotation_locked: bool,
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
        self.angular = AngularState3d::new(
            self.angular.orientation,
            AngularVelocity3d::default(),
        );
        self
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

    use super::{RigidBox3d, RigidBoxError3d};

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
